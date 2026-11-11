"""Offline closed loop for the autonomous explorer (src/mission/nidar_explorer.py), 2026-11-09.

Ray-cast arena LiDAR (walls + survivor bodies) -> occupancy map built with SLAM V6's own grid code
(mode "map": true pose; mode "slam": the full SLAM V6 estimate) -> Explorer -> a kinematic drone
that flies the plans like the flight interfaces do (turn on the spot at 30 deg/s, straight legs at
0.3 m/s, a full circle at each viewpoint) -> metrics against the arena's ground truth.
Frames: SLAM frame = take-off pose (x = initial heading, y left)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from src.mission import nidar_explorer as ne
from src.simulation import arena_raycast as ar
from src.simulation.arena_feed import visible_survivors

YAW_RATE = math.radians(30.0)
SPEED = 0.4
LIDAR_H = 1.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Frames:
    def __init__(self, sx, sy, syaw):
        self.s = (sx, sy, syaw)

    def to_slam(self, x, y, yaw=0.0):
        sx, sy, syaw = self.s
        c, s = math.cos(-syaw), math.sin(-syaw)
        dx, dy = x - sx, y - sy
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - syaw)

    def to_arena(self, x, y, yaw=0.0):
        sx, sy, syaw = self.s
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * x - s * y, sy + s * x + c * y, wrap(yaw + syaw)


def wall_clearance(layout, x, y):
    half = layout["grid"]["wall_m"] / 2
    best = math.inf
    for x1, y1, x2, y2 in layout["walls"]:
        ax, ay = x2 - x1, y2 - y1
        L2 = ax * ax + ay * ay
        t = max(0.0, min(1.0, ((x - x1) * ax + (y - y1) * ay) / L2))
        best = min(best, math.hypot(x - x1 - t * ax, y - y1 - t * ay) - half)
    return best


def run(layout_path, mode="map", t_max=3600.0, dt=0.1, radius=ne.DRONE_RADIUS_M, verbose=False,
        mission_time_s=ne.MISSION_TIME_S):
    layout = json.loads(Path(layout_path).read_text())
    arena = ar.load_arena(Path(layout_path))
    sp = layout["spawn_pose"]
    F = Frames(sp["x"], sp["y"], sp["yaw"])
    if mode == "slam":
        from src.core import nidar_slam_v6 as s6
        slam = s6.SlamV6()
    else:
        from src.core.nidar_slam_v6 import GridLevel
        grid = GridLevel(0.1)
    ex = ne.Explorer(drone_radius=radius, mission_time_s=mission_time_s)
    look_q = []
    x, y, yaw = sp["x"], sp["y"], sp["yaw"]
    t = 0.0
    traj, seen_surv = [], set()
    min_clear, min_at = math.inf, None
    plan, wp_i, sub, look_left = None, 0, None, 0.0
    last_map = -1.0
    n_inc = 2 * math.pi / ar.SAMPLES
    ang = -math.pi + n_inc * np.arange(ar.SAMPLES)
    done_reason = "time limit"
    replans = 0
    while t < t_max:
        # sense
        r, _ = ar.cast(arena, x, y, yaw, LIDAR_H)
        ok = np.isfinite(r) & (r >= 0.15) & (r <= 10.0)
        pts = np.column_stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])])
        tx, ty, tyaw = F.to_slam(x, y, yaw)
        if mode == "slam":
            est = slam.process(pts, t)
            px, py, pyaw = est["x"], est["y"], est["yaw"]
        else:
            px, py, pyaw = tx, ty, tyaw
            c, s = math.cos(tyaw), math.sin(tyaw)
            grid.update(np.array([tx, ty]), pts @ np.array([[c, s], [-s, c]]) + np.array([tx, ty]))
        if t - last_map >= 1.0 or ex.view is None:
            g = slam.export_level().export_int8() if mode == "slam" else grid.export_int8()
            ex.update_map(g)
            last_map = t
        c_, s_ = math.cos(pyaw), math.sin(pyaw)
        if ex.view is not None:
            ex.add_hits(pts @ np.array([[c_, s_], [-s_, c_]]) + np.array([px, py]))
        ex.observe(px, py, pyaw)
        if plan is not None and plan.waypoints and int(round(t / dt)) % 10 == 0 \
                and not ex.plan_still_safe(px, py, plan.waypoints, wp_i):
            unsafe_replans = locals().get("unsafe_replans", 0) + 1
            plan = None                                   # stop here and replan around the new obstacle
        if int(round(t / dt)) % 3 == 0:
            for s_, rng_, _ in visible_survivors(arena, x, y, yaw, max_range=ne.CAMERA_RANGE_M):
                seen_surv.add(s_["id"])
        wc = wall_clearance(layout, x, y)
        if wc < min_clear:
            min_clear, min_at = wc, (round(x, 2), round(y, 2), round(t, 1))
        traj.append((t, x, y, yaw))
        if len(traj) > 200:
            t0_, x0_, y0_, a0_ = traj[-200]
            if plan is not None and math.hypot(x - x0_, y - y0_) < 0.02 and abs(wrap(yaw - a0_)) < math.radians(2):
                stalls = locals().get("stalls", 0) + 1
                plan = None
        # decide
        if plan is None:
            plan = ex.plan(px, py, pyaw, t)
            replans += 1
            wp_i, sub = 0, None
            if plan is None or (plan.end == "land" and not plan.waypoints):
                done_reason = plan.reason if plan else "done"
                break
            if verbose:
                print(f"t={t:6.1f} plan -> {plan.reason} goal {plan.goal} legs {len(plan.waypoints)}")
        # act (kinematic follower in the SLAM frame, applied to the true pose)
        if wp_i < len(plan.waypoints):
            gx, gy = plan.waypoints[wp_i]
            d = math.hypot(gx - px, gy - py)
            if d < 0.06:                                  # flight interfaces use 0.08 m
                wp_i += 1
            else:
                # move and turn at the same time (a quad's footprint does not change when it yaws)
                want = math.atan2(gy - py, gx - px)
                e = wrap(want - pyaw)
                yaw = wrap(yaw + max(-YAW_RATE * dt, min(YAW_RATE * dt, e)))
                stepd = min(SPEED * dt, d)
                ax_, ay_, _ = F.to_arena(px + (gx - px) / d * stepd, py + (gy - py) / d * stepd)
                bx_, by_, _ = F.to_arena(px, py)
                x += ax_ - bx_
                y += ay_ - by_
        elif plan.end == "look" and sub not in ("looking", "looked"):
            look_q = list(plan.look_yaws)
            sub = "looking"
        elif sub == "looking":
            if not look_q:
                sub = "looked"
                ex.arrived_and_looked(px, py)
                plan = None
            else:
                e = wrap(look_q[0] - pyaw)
                if abs(e) < math.radians(4.0):            # flight interfaces use 8 deg
                    look_q.pop(0)
                else:
                    yaw = wrap(yaw + max(-YAW_RATE * dt, min(YAW_RATE * dt, e)))
        elif plan.end == "land":
            done_reason = plan.reason
            break
        else:
            ex.arrived_and_looked(px, py)
            plan = None
        t += dt
    # coverage of the arena's grid cells by the camera (cell centre marked seen)
    g = layout["grid"]
    cells, covered = 0, 0
    for c in range(g["cells"]):
        for rr in range(g["cells"]):
            cx, cy = ar.cell_centre(layout, c, rr)
            sx_, sy_, _ = F.to_slam(cx, cy)
            j, i = ex.view.cell(sx_, sy_)
            cells += 1
            covered += bool(ex.seen[j, i])
    end = traj[-1]
    return {"time_min": t / 60.0, "reason": done_reason, "min_wall_clearance": min_clear, "min_clear_at": min_at,
            "coverage": covered / cells, "survivors_seen": sorted(seen_surv),
            "n_survivors": len(layout["survivors"]), "plans": replans,
            "end_dist_to_pad": math.hypot(end[1] - sp["x"], end[2] - sp["y"]), "explorer": ex.stats,
            "traj": traj}
