"""Offline closed loop THROUGH THE REAL PROTOCOL OBJECTS (2026-11-09):

  ray-cast LiDAR -> occupancy map (SLAM V6 grid code; mode "slam" = full SLAM V6 estimate)
  -> Brain (src/mission/nidar_explorer_service.py)  --NIDAR_NAV_V1 JSON lines-->
  BrainFollower (gazebo/ros2/nidar_rig_driver.py, alignment + RouteFollower + LiDAR guard)
  -> body velocities -> a rig with a short velocity lag -> ground truth.

Same pieces as Pi + WSL, minus sockets/ROS: proves the frames (SLAM vs odometry, a rotated
spawn pose), the plan/look/land handshake and the guards work together. The drone in
explorer_sim.py instead flies the Explorer's plans directly.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np

from src.mission import nidar_explorer as ne
from src.mission import nidar_explorer_service as svc
from src.simulation import arena_raycast as ar
from src.simulation.arena_feed import visible_survivors

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("nidar_rig_driver", ROOT / "gazebo/ros2/nidar_rig_driver.py")
rig = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rig)

from explorer_sim import Frames, wall_clearance, wrap  # noqa: E402  (tests/ is on sys.path)

LIDAR_H = 1.0
TAU = 0.2            # rig velocity lag (s)


def wire(msgs):
    """What the TCP link does: JSON with the protocol tag, one per line, decoded on the other side."""
    return [json.loads(json.dumps(dict(m, protocol=svc.PROTOCOL))) for m in msgs]


def run(layout_path, mode="map", t_max=1800.0, dt=0.05, radius=ne.DRONE_RADIUS_M, mission_time_s=ne.MISSION_TIME_S,
        speed=0.4, odom_offset=(0.0, 0.0, 0.0), verbose=False):
    """odom_offset: the rig's odometry frame relative to the arena (x, y, yaw) -- the follower must not care."""
    layout = json.loads(Path(layout_path).read_text())
    arena = ar.load_arena(Path(layout_path))
    sp = layout["spawn_pose"]
    F = Frames(sp["x"], sp["y"], sp["yaw"])                    # SLAM frame = take-off pose
    O = Frames(*odom_offset)                                   # odometry frame
    if mode == "slam":
        from src.core import nidar_slam_v6 as s6
        slam = s6.SlamV6()
    else:
        from src.core.nidar_slam_v6 import GridLevel
        grid = GridLevel(0.1)
    brain = svc.Brain(ne.Explorer(drone_radius=radius, mission_time_s=mission_time_s))
    fol = rig.BrainFollower(speed=speed)
    x, y, yaw = sp["x"], sp["y"], sp["yaw"]
    vxw = vyw = wz_ = 0.0
    n_inc = 2 * math.pi / ar.SAMPLES
    ang = -math.pi + n_inc * np.arange(ar.SAMPLES)
    t, last_map, last_brain = 0.0, -1.0, -1.0
    to_brain, to_rig = [], []
    min_clear, min_at, seen_surv, traj = math.inf, None, set(), []
    blocked = 0
    diag = []        # brain-rate: t, true SLAM-frame pose, estimate, accepted, lost, score
    t_one = None
    g_export = None
    while t < t_max:
        r, _ = ar.cast(arena, x, y, yaw, LIDAR_H)
        ok = np.isfinite(r) & (r >= 0.15) & (r <= 10.0)
        if t - last_brain >= 0.1 - 1e-9:                      # brain loop 10 Hz (SLAM at the same rate)
            last_brain = t
            pts = np.column_stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])])
            tx, ty, tyaw = F.to_slam(x, y, yaw)
            if mode == "slam":
                est = slam.process(pts, t)
                pose, acc, score, lost = (est["x"], est["y"], est["yaw"]), est["accepted"], est["score"], est["lost"]
            else:
                c, s = math.cos(tyaw), math.sin(tyaw)
                grid.update(np.array([tx, ty]), pts @ np.array([[c, s], [-s, c]]) + np.array([tx, ty]))
                pose, acc, score, lost = (tx, ty, tyaw), True, 1.0, False
            diag.append((round(t, 2), round(tx, 3), round(ty, 3), round(tyaw, 4), round(pose[0], 3), round(pose[1], 3),
                         round(pose[2], 4), bool(acc), bool(lost), round(float(score), 3)))
            g_export = None
            if t - last_map >= 1.0:
                last_map = t
                g_export = slam.export_level().export_int8() if mode == "slam" else grid.export_int8()
            scan = {"angle_min": -math.pi, "angle_increment": n_inc, "range_min": 0.15, "range_max": 12.0,
                    "ranges": [float(v) if k else None for v, k in zip(r, ok)], "received": t}
            brain.feed_scan(scan)
            state = {"timestamp": t, "scan_timestamp": t, "status": "LOST" if lost else "TRACKING",
                     "pose": {"x_m": pose[0], "y_m": pose[1], "yaw_deg": math.degrees(pose[2])},
                     "match": {"score": score, "accepted": acc},
                     "map": {"resolution_m": 0.1, "origin_m": [-20.0, -20.0]}}
            to_rig += wire(brain.step(t, state, g_export, to_brain, connected=True))
            to_brain = []
        # rig driver at 20 Hz
        odom = O.to_slam(x, y, yaw)
        for m in to_rig:
            fol.on_message(m, odom, t)
        to_rig = []

        def ahead(direction, r=r, ok=ok):
            return rig.ahead_min([float(v) if k else math.inf for v, k in zip(r, ok)], -math.pi, n_inc, 0.15, direction)
        vx_b, vy_b, wz = fol.update(odom, t, ahead)
        out = fol.drain()
        blocked += sum(m.get("state") == "BLOCKED" for m in out)
        to_brain += wire(out)
        # body velocity (odometry frame body = arena body: yaw differs only by a constant) -> arena world
        c, s = math.cos(yaw), math.sin(yaw)
        a = min(1.0, dt / TAU)
        vxw += a * ((c * vx_b - s * vy_b) - vxw)
        vyw += a * ((s * vx_b + c * vy_b) - vyw)
        wz_ += a * (wz - wz_)
        x, y, yaw = x + vxw * dt, y + vyw * dt, wrap(yaw + wz_ * dt)
        if int(round(t / dt)) % 6 == 0:
            for s_, rng_, _ in visible_survivors(arena, x, y, yaw, max_range=ne.CAMERA_RANGE_M):
                seen_surv.add(s_["id"])
        wc = wall_clearance(layout, x, y)
        if wc < min_clear:
            min_clear, min_at = wc, (round(x, 2), round(y, 2), round(t, 1))
        traj.append((t, x, y, yaw))
        if brain.complete and fol.state == "COMPLETE":
            break
        if brain.complete or fol.state == "COMPLETE":            # the other side must follow within 3 s
            t_one = t_one if t_one is not None else t
            if t - t_one > 3.0:
                break
        t += dt
    g = layout["grid"]
    cells = covered = 0
    for cc in range(g["cells"]):
        for rr in range(g["cells"]):
            cx, cy = ar.cell_centre(layout, cc, rr)
            sx_, sy_, _ = F.to_slam(cx, cy)
            j, i = brain.ex.view.cell(sx_, sy_)
            cells += 1
            covered += bool(brain.ex.seen[j, i])
    return {"diag": diag, "time_min": t / 60.0, "complete": brain.complete and fol.state == "COMPLETE",
            "brain_events": brain.events, "rig_events": fol.events, "aligned": fol.align is not None,
            "coverage": covered / cells, "survivors_seen": sorted(seen_surv), "blocked": blocked,
            "min_wall_clearance": min_clear, "min_clear_at": min_at, "plans": brain.plan_id,
            "end_dist_to_pad": math.hypot(x - sp["x"], y - sp["y"]), "traj": traj}


if __name__ == "__main__":
    import sys
    lay = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "gazebo/layouts/nidar_airmouse_arena_s1.json")
    res = run(lay, mode=sys.argv[2] if len(sys.argv) > 2 else "map")
    res.pop("traj"); res.pop("diag")
    print(json.dumps({k: v for k, v in res.items() if not k.endswith("events")}, indent=1, default=str))
    print("\n".join(res["rig_events"][-12:]))
