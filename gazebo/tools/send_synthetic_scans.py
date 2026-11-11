#!/usr/bin/env python3
"""
Pi-side dry run WITHOUT Gazebo and WITHOUT the Windows camera (2026-11-02).

Moves a virtual drone along a route in the generated arena and sends, at
10 Hz, to the Raspberry Pi:
  * LiDAR scans (NIDAR_LIDAR_V1, ray-cast from the arena)  -> Pi :5011
  * camera packets (NIDAR_PERCEPTION_V1, survivors in view) -> Pi :5000  (--camera)

So the real Pi chain -- receivers -> V18 (NIDAR_LIDAR_BACKEND=GAZEBO) -> SLAM
-> AirMouse server -> WSL bridge -- can be checked before Gazebo is involved.
Detections are perfect (every visible survivor, confidence 0.8): this tests
the plumbing and the fusion geometry, not YOLO.

    py -3 gazebo/tools/send_synthetic_scans.py --pi-host 10.218.70.147 --route survivor:S2 --camera
    python3 gazebo/tools/send_synthetic_scans.py --pi-host 10.218.70.147 --route rooms

Ctrl+C stops sending; the Pi then reports LiDAR STALE / CAMERA_STALE and the
bridge publishes zero.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from src.simulation import arena_raycast as ar  # noqa: E402
from src.simulation.arena_feed import visible_survivors, camera_packet, CAMERA_BELOW_LIDAR_M  # noqa: E402
from src.simulation.arena_trajectory import route_trajectory  # noqa: E402

sys.path.insert(0, str(REPO / "gazebo" / "ros2"))
from nidar_lidar_relay import LatestLineSender  # noqa: E402  (no ROS needed for this class)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi-host", default=os.environ.get("NIDAR_PI_HOST", ""))
    ap.add_argument("--lidar-port", type=int, default=5011)
    ap.add_argument("--camera-port", type=int, default=5000)
    ap.add_argument("--camera", action="store_true", help="also send camera packets to the perception receiver")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--route", default="survivor:S2", help="survey | rooms | survivor:<ID>")
    ap.add_argument("--speed", type=float, default=0.4, help="m/s along the route")
    ap.add_argument("--height", type=float, default=1.0, help="LiDAR plane height (m)")
    ap.add_argument("--hold", type=float, default=30.0, help="seconds to keep sending at the route's end")
    args = ap.parse_args(argv)
    if not args.pi_host:
        print("Give --pi-host <PI_IP> (or set NIDAR_PI_HOST)", file=sys.stderr)
        return 2

    arena = ar.load_arena(ar.default_layout(args.seed))
    # Vehicle-like motion (same follower as the Gazebo rig): smooth position, yaw <= 0.8 rad/s.
    traj = route_trajectory(arena.layout, args.route, rate_hz=10.0, speed=args.speed)
    lidar = LatestLineSender(args.pi_host, args.lidar_port, name="LIDAR")
    lidar.start()
    cam = None
    if args.camera:
        cam = LatestLineSender(args.pi_host, args.camera_port, name="CAMERA")
        cam.start()

    dt = 0.1
    seq = frame = 0
    cam_h = args.height - CAMERA_BELOW_LIDAR_M
    print(f"[SYN] arena s{args.seed} route {args.route}: {len(traj)} poses ({len(traj) * dt:.0f} s) -> "
          f"{args.pi_host} (LiDAR :{args.lidar_port}{', camera :%d' % args.camera_port if cam else ''})", flush=True)
    try:
        k = 0
        hold_until = None
        while True:
            t0 = time.time()
            _, x, y, yaw = traj[min(k, len(traj) - 1)]
            r, _ = ar.cast(arena, x, y, yaw, height_m=args.height)
            fin = np.isfinite(r)
            r = r.copy()
            r[fin] += np.random.normal(0.0, 0.01, fin.sum())        # like the Gazebo sensor noise
            seq += 1
            msg = ar.laserscan_message(r, seq=seq, stamp=seq * dt, sent=time.time())
            lidar.submit((json.dumps(msg, separators=(",", ":")) + "\n").encode())
            if cam:
                frame += 1
                vis = visible_survivors(arena, x, y, yaw)
                cam.submit((json.dumps(camera_packet(frame, vis, source="synthetic_arena",
                                                     camera_height_m=cam_h)) + "\n").encode())
                if frame % 20 == 0:
                    seen = ", ".join(f"{s['id']} {rng:.2f} m {brg:+.1f} deg" for s, rng, brg in vis) or "none"
                    print(f"[SYN] t={k * dt:5.1f} s ({x:.2f},{y:.2f}) truth in view: {seen}", flush=True)
            k += 1
            if k >= len(traj):
                hold_until = hold_until or time.time() + args.hold
                if time.time() >= hold_until:
                    break
            time.sleep(max(0.0, dt - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        if cam:
            cam.stop()
    print(f"[SYN] stopped after {seq} scans (sent {lidar.sent}, dropped {lidar.dropped}).", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
