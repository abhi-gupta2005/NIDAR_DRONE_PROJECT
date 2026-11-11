#!/usr/bin/env python3
"""
Make a DEMO flight record for the AirMouse ground station (2026-11-10), so the app can be tried without the Pi.

Runs the offline closed loop of tests/nav_loop_sim.py (ray-cast LiDAR -> SLAM V6 -> explorer brain ->
rig follower, the same code as on the Pi and in WSL; about 2 minutes) and writes what the Pi mission
server would have sent, as a flight record:

    python3 tools/make_demo_record.py                       # -> gcs_records/demo_s1_offline/flight.jsonl.gz
    python3 gcs/airmouse_gcs.py --demo gcs_records/demo_s1_offline/flight.jsonl.gz --no-ros

Honest limits, also written into the record: there is no camera pipeline in this offline run (no YOLO, no
V18), so camera/fusion health is "no data" and a survivor is placed at its simulated position the first
time it is in the camera's field of view (70 deg, 5 m, line of sight).
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import time
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "tests")]

import nav_loop_sim as nl  # noqa: E402
from gcs.airmouse.record import RECORD_NAME, SUMMARY_NAME, Recorder  # noqa: E402
from src.mission import nidar_explorer as ne  # noqa: E402
from src.mission import nidar_explorer_service as svc  # noqa: E402
from src.mission.survivor_mapper import Grid, compose  # noqa: E402
from src.simulation import arena_raycast as ar  # noqa: E402
from src.simulation.arena_feed import visible_survivors  # noqa: E402

NOTE = ("Offline closed-loop simulation on arena s1 (simulated LiDAR -> SLAM V6 -> explorer brain -> follower). "
        "No camera pipeline in this run: survivors are placed at their simulated positions when first in "
        "camera view.")
STATE_DT = 0.5
MAP_DT = 5.0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--layout", default=str(ROOT / "gazebo/layouts/nidar_airmouse_arena_s1.json"))
    ap.add_argument("--out", default=str(ROOT / "gcs_records/demo_s1_offline"))
    ap.add_argument("--drone-name", default="ABHISHEK DRONE")
    args = ap.parse_args(argv)

    ticks, maps = [], []

    class RecBrain(svc.Brain):
        def step(self, t, state, g_export, from_rig, connected=True):
            msgs = super().step(t, state, g_export, from_rig, connected=connected)
            if state is not None and (not ticks or t - ticks[-1][0] >= STATE_DT - 1e-9):
                ticks.append((t, json.loads(json.dumps(state)), json.loads(json.dumps(self.state_json(t), default=float))))
            if g_export is not None and (not maps or t - maps[-1][0] >= MAP_DT - 1e-9):
                maps.append((t, np.asarray(g_export).copy()))
            return msgs

    svc.Brain = RecBrain
    print("running the offline closed loop (about 2 minutes)...", flush=True)
    t_wall = time.time()
    res = nl.run(args.layout, mode="slam")
    print(f"done in {time.time() - t_wall:.0f} s: complete={res['complete']} coverage={res['coverage']:.3f} "
          f"survivors={res['survivors_seen']}", flush=True)

    layout = json.loads(Path(args.layout).read_text())
    arena = ar.load_arena(Path(args.layout))
    grid = Grid.from_layout(layout)
    sp = layout["spawn_pose"]
    start = (float(sp["x"]), float(sp["y"]), float(sp["yaw"]))
    traj = np.array(res["traj"])
    seen_at = {}                                            # id -> list of times in camera view
    for t, x, y, yaw in traj[::6]:
        for s, _, _ in visible_survivors(arena, x, y, yaw, max_range=ne.CAMERA_RANGE_M):
            seen_at.setdefault(s["id"], []).append(float(t))
    surv = {s["id"]: s for s in layout["survivors"]}

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for name in (RECORD_NAME, SUMMARY_NAME):
        (out / name).unlink(missing_ok=True)
    base = time.time() - traj[-1, 0]
    rec = Recorder(out, {"t": base, "source": "demo", "drone": args.drone_name, "endpoint": "offline simulation",
                         "note": NOTE, "layout": str(Path(args.layout).relative_to(ROOT))}, state_hz=1.0 / STATE_DT)

    def arena(xy):
        ax, ay, _ = compose(start, (float(xy[0]), float(xy[1]), 0.0))
        return [round(ax, 3), round(ay, 3)]

    path = []
    acc = rej = 0
    mi = 0
    for t, slam, ex in ticks:
        while mi < len(maps) and maps[mi][0] <= t:
            g = maps[mi][1]
            known = np.argwhere(g >= 0)
            r0, c0 = known.min(axis=0) if len(known) else (0, 0)
            r1, c1 = (known.max(axis=0) + 1) if len(known) else (1, 1)
            crop = np.ascontiguousarray(g[r0:r1, c0:c1].astype(np.int8))
            rec.add_map(base + maps[mi][0], {
                "type": "map", "protocol": "NIDAR_MISSION_V1", "timestamp": base + maps[mi][0], "resolution_m": 0.1,
                "origin_m": [-20.0 + c0 * 0.1, -20.0 + r0 * 0.1], "shape": [int(crop.shape[0]), int(crop.shape[1])],
                "frame": "SLAM (start pose)", "start_pose": [start[0], start[1], math.degrees(start[2])],
                "encoding": "int8 row-major (row = +y), zlib, base64; -1 unknown, 0..100 occupied %",
                "data": base64.b64encode(zlib.compress(crop.tobytes(), 6)).decode("ascii"),
                "path": [p[:] for p in path[-2000:]]})
            mi += 1
        p = slam["pose"]
        lx, ly, lyaw = p["x_m"], p["y_m"], math.radians(p["yaw_deg"])
        ax, ay, ayaw = compose(start, (lx, ly, lyaw))
        if not path or math.hypot(ax - path[-1][0], ay - path[-1][1]) >= 0.05:
            path.append([round(ax, 3), round(ay, 3)])
        accepted = bool(slam["match"]["accepted"])
        acc, rej = acc + accepted, rej + (not accepted)
        on_pad = math.hypot(ax - start[0], ay - start[1]) < 0.6
        survivors = []
        for sid, times in seen_at.items():
            seen = [x for x in times if x <= t]
            if not seen:
                continue
            s = surv[sid]
            survivors.append({"id": sid, "status": "CONFIRMED", "grid": grid.label(s["x"], s["y"]),
                              "x_m": round(s["x"], 3), "y_m": round(s["y"], 3), "std_m": None,
                              "observations": len(seen), "inliers": None, "methods": {"sim view": len(seen)},
                              "first_seen": base + seen[0], "last_seen": base + seen[-1], "confidence": None,
                              "aliases": []})
        survivors.sort(key=lambda s: s["first_seen"])
        plan = ex.get("plan")
        state = {
            "type": "state", "protocol": "NIDAR_MISSION_V1", "timestamp": base + t,
            "pose": {"x_m": round(ax, 3), "y_m": round(ay, 3), "yaw_deg": round(math.degrees(ayaw), 2),
                     "grid": "PAD" if on_pad else grid.label(ax, ay), "age_s": 0.1,
                     "trusted": accepted and slam["status"] == "TRACKING", "slam_status": slam["status"],
                     "slam": {"x_m": round(lx, 3), "y_m": round(ly, 3), "yaw_deg": round(p["yaw_deg"], 2)}},
            "path_tail": path[-30:], "path_len": len(path), "survivors": survivors, "measurements": [],
            "camera": {"width": 640, "height": 480, "objects": []},
            "health": {"perception": {"frame": None, "age_s": None, "objects": 0, "source": None},
                       "v18": {"age_s": None, "active": 0, "fused": 0},
                       "lidar": {"age_s": 0.1, "seq": int(round(t * 10)), "points": ar.SAMPLES},
                       "slam": {"age_s": 0.1, "fps": 10.0, "score": round(float(slam["match"]["score"]), 3),
                                "accepted": acc, "rejected": rej, "status": slam["status"], "reject_streak": None},
                       "mapper": {}},
            "start_pose": [start[0], start[1], math.degrees(start[2])],
            "grid": {"pitch_m": grid.pitch_m, "wall_m": grid.wall_m, "cells": grid.cells, "bounded": True},
            "explorer": {"state": ex.get("state"), "hold_reason": ex.get("hold_reason"), "plan_id": ex.get("plan_id"),
                         "plan": None if not plan else {"waypoints": [arena(w) for w in plan["waypoints"]],
                                                        "goal": arena(plan["goal"]), "end": plan.get("end"),
                                                        "reason": plan.get("reason")},
                         "looks": (ex.get("stats") or {}).get("looks"), "radius_m": ex.get("drone_radius_m"),
                         "return_reason": ex.get("return_reason"), "events": list(ex.get("events") or [])[-10:]},
            "flight_control": "NONE (read-only)",
        }
        rec.add_state(base + t, state)
    rec.close({"note": NOTE, "coverage": res["coverage"], "end_dist_to_pad_m": res["end_dist_to_pad"]})
    size = (out / RECORD_NAME).stat().st_size
    print(f"wrote {out / RECORD_NAME} ({size / 1e6:.1f} MB, {rec.n_states} states, {len(maps)} maps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
