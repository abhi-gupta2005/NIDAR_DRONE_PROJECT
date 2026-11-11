#!/usr/bin/env python3
"""
Sweep V18's camera+LiDAR fusion over the generated arena (2026-11-02).

For every survivor, many viewpoints (stand-off x bearing offset) with clear
line of sight: ray-cast the LiDAR (arena geometry, scan plane at --height),
project the survivor with a pinhole 640 px / 70 deg camera, then run the
PRODUCTION V18 chain in-process: LidarObstacleDetector -> TrackerV3 ->
normalize_track -> fuse_objects. Reports, per posture, how often V18 fuses
the camera detection and how wrong the fused range is.

    python3 tools/arena_fusion_sweep.py --seed 1 --height 1.0

No Gazebo, no rendering: survivors are cylinders (standing 1.70 m, low
"lying" models 0.45 m). It measures the fusion LOGIC on arena geometry.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "src", ROOT / "src" / "core", ROOT / "src" / "tracking", ROOT / "src" / "perception",
          ROOT / "src" / "simulation"):
    sys.path.insert(0, str(p))

import numpy as np  # noqa: E402

from src.simulation import arena_raycast as ar  # noqa: E402
from src.simulation.arena_feed import visible_survivors, camera_packet  # noqa: E402
from src.core.lidar_gazebo_backend import ros_scan_to_lidar_scan  # noqa: E402


def load_v18():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "v18_sweep", ROOT / "src" / "perception" / "nidar_v18_headless_persistent_fusion.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def sweep(seed: int = 1, height: float = 1.0, standoffs=(1.0, 1.5, 2.0, 2.5, 3.0),
          offsets=(-25, -12, 0, 12, 25), headings=16):
    v18 = load_v18()
    from lidar_obstacle_detector import LidarObstacleDetector, ObstacleDetectorConfig
    from lidar_obstacle_tracker_v3 import LidarObstacleTrackerV3, TrackerV3Config
    arena = ar.load_arena(ar.default_layout(seed))
    walls = ar.Arena(arena.segments, arena.wall_height_m, [], arena.layout)
    rows = []
    for s in arena.survivors:
        for d in standoffs:
            for k in range(headings):
                a = 2 * math.pi * k / headings
                x, y = s["x"] + d * math.cos(a), s["y"] + d * math.sin(a)
                if ar.cast(walls, x, y, 0.0, height, samples=72)[0].min() < 0.35:
                    continue                                   # inside / touching a wall
                for off in offsets:
                    yaw = math.atan2(s["y"] - y, s["x"] - x) + math.radians(off)
                    vis = [v for v in visible_survivors(arena, x, y, yaw) if v[0]["id"] == s["id"]]
                    if not vis:
                        continue
                    r, labels = ar.cast(arena, x, y, yaw, height_m=height)
                    msg = ar.laserscan_message(r, seq=1)
                    det = LidarObstacleDetector(ObstacleDetectorConfig())
                    trk = LidarObstacleTrackerV3(TrackerV3Config())
                    t = time.time()
                    with contextlib.redirect_stdout(io.StringIO()):
                        for i in range(5):                     # static scene, let tracks confirm
                            tracks = trk.update(det.detect(ros_scan_to_lidar_scan(msg, i, t + 0.1 * i)), t + 0.1 * i)
                        tracks = [n for n in (v18.normalize_track(tr) for tr in tracks) if n]
                        pkt = camera_packet(1, vis)
                        fused = v18.fuse_objects(pkt["objects"], tracks, v18.packet_camera_width(pkt))
                    true_rng = vis[0][1]
                    surface = true_rng - (ar.SURVIVOR_SHAPES[s["posture"]]["radius_m"]
                                          if height < ar.SURVIVOR_SHAPES[s["posture"]]["height_m"] else 0.0)
                    # what the LiDAR actually hits along the survivor's bearing
                    ros_i = int(round((math.radians(-vis[0][2]) + math.pi) / (2 * math.pi / ar.SAMPLES))) % ar.SAMPLES
                    rows.append({
                        "survivor": s["id"], "posture": s["posture"], "standoff": d, "offset": off,
                        "fused": bool(fused),
                        "range": float(fused[0]["range_m"]) if fused else None,
                        "surface": surface, "ray_hit": labels[ros_i],
                    })
    return rows


def summarise(rows):
    out = {}
    for posture in sorted({r["posture"] for r in rows}):
        rs = [r for r in rows if r["posture"] == posture]
        fused = [r for r in rs if r["fused"]]
        err = [abs(r["range"] - r["surface"]) for r in fused]
        wrong = [r for r in fused if abs(r["range"] - r["surface"]) > 0.5]
        out[posture] = {
            "views": len(rs), "fused": len(fused),
            "fused_pct": 100.0 * len(fused) / max(1, len(rs)),
            "median_err_m": float(np.median(err)) if err else None,
            "p95_err_m": float(np.percentile(err, 95)) if err else None,
            "wrong_range_gt_0.5m": len(wrong),
            "ray_hits_survivor_pct": 100.0 * sum(r["ray_hit"] == r["survivor"] for r in rs) / max(1, len(rs)),
        }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--height", type=float, default=1.0, help="LiDAR scan-plane height above the floor (m)")
    args = ap.parse_args()
    rows = sweep(args.seed, args.height)
    print(f"arena seed {args.seed}, LiDAR plane {args.height:.2f} m, {len(rows)} views")
    for posture, s in summarise(rows).items():
        med = "-" if s["median_err_m"] is None else f"{s['median_err_m']:.2f}"
        p95 = "-" if s["p95_err_m"] is None else f"{s['p95_err_m']:.2f}"
        print(f"  {posture:9s} views={s['views']:4d} fused={s['fused']:4d} ({s['fused_pct']:5.1f}%) "
              f"range err median={med} m p95={p95} m  wrong(>0.5 m)={s['wrong_range_gt_0.5m']}  "
              f"LiDAR ray hits survivor={s['ray_hits_survivor_pct']:.0f}%")


if __name__ == "__main__":
    main()
