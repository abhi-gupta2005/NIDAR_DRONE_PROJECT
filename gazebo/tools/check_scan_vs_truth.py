#!/usr/bin/env python3
"""
Compare the Gazebo LiDAR with the arena's ground truth (2026-11-02c).

Takes the rig's pose from /nidar/rig/odom, ray-casts the expected scan from
gazebo/layouts/<world>.json (src/simulation/arena_raycast.py) and compares
it beam by beam with /nidar/scan. Proves three things at once:
  * the Gazebo walls are where the layout file says (geometry),
  * the LaserScan angle convention is what lidar_gazebo_backend.py assumes
    (ROS: counter-clockwise from forward) - a mirrored or rotated scan is
    detected and reported,
  * the odometry pose is in the arena frame (or relative to the spawn pose).

Run in WSL with the rig world running (gazebo/run_rig.sh):
    source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
    python3 $NIDAR_REPO/gazebo/tools/check_scan_vs_truth.py --seed 1

Survivors are cylinders in the ray-caster but meshes in Gazebo, so beams that
hit a survivor are reported separately and not counted as wall errors.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Sequence

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from src.simulation import arena_raycast as ar  # noqa: E402


def resample_prediction(arena, x, y, yaw, height, n, angle_min, angle_inc):
    """Ray-cast exactly at the measured scan's beam angles."""
    r, labels = ar.cast(arena, x, y, yaw, height_m=height, samples=n, angle_min=angle_min)
    if abs(angle_inc - 2 * math.pi / n) > 1e-6:          # non-360 scan: cast beam by beam
        r = np.empty(n)
        labels = []
        for i in range(n):
            ri, li = ar.cast(arena, x, y, yaw, height_m=height, samples=1, angle_min=angle_min + i * angle_inc)
            r[i] = ri[0]
            labels.append(li[0])
    return np.asarray(r, dtype=float), labels


def compare(measured: Sequence[float], predicted: Sequence[float], labels: Sequence[Optional[str]],
            range_min: float = 0.15, range_max: float = 12.0, tol_m: float = 0.05,
            angle_min: float = -math.pi, angle_inc: Optional[float] = None, near_m: float = 0.35) -> Dict:
    m = np.array([np.nan if v is None else float(v) for v in measured], dtype=float)
    p = np.asarray(predicted, dtype=float)
    n = len(m)
    angle_inc = angle_inc if angle_inc is not None else 2 * math.pi / n
    m_hit = np.isfinite(m) & (m >= range_min) & (m <= range_max)
    p_hit = np.isfinite(p) & (p <= range_max)
    walls = np.array([lab in (None, "wall") for lab in labels])

    def score(mm_hit, mm):
        both = mm_hit & p_hit & walls
        err = np.abs(mm[both] - p[both]) if both.any() else np.array([])
        agree = int(np.sum(mm_hit[walls] == p_hit[walls]))
        within = int(np.sum(err <= tol_m))
        return {"both": int(both.sum()), "agree_hit": agree, "within_tol": within,
                "median_err": float(np.median(err)) if err.size else float("nan"),
                "p95_err": float(np.percentile(err, 95)) if err.size else float("nan")}

    normal = score(m_hit, m)
    # Mirror about the forward axis (angle a <-> -a). For a full scan starting at
    # -pi with n beams, beam i <-> beam (n - i) mod n.
    mirror_idx = (n - np.arange(n)) % n
    mirrored = score(m_hit[mirror_idx], m[mirror_idx])
    best = None
    for k in range(-n // 8, n // 8 + 1):
        s = score(np.roll(m_hit, k), np.roll(m, k))
        med = s["median_err"] if s["median_err"] == s["median_err"] else 1e9
        key = (s["within_tol"], -med, -abs(k))           # ties -> smallest rotation
        if best is None or key > best[0]:
            best = (key, k, s)
    best_shift = best[1]
    survivor_beams = [i for i, lab in enumerate(labels) if lab not in (None, "wall")]
    # Returns very close to the sensor that the arena (walls AND survivors) does not
    # explain: almost always the vehicle's own body in the scan plane (2026-11-02:
    # the first rig's nose block gave 29 beams at 0.15-0.17 m straight ahead).
    with np.errstate(invalid="ignore"):
        near = m_hit & (m < near_m) & ~(np.isfinite(p) & (p < m + 0.25))
    near_idx = np.nonzero(near)[0]
    return {
        "near_unexplained": int(near.sum()),
        "near_angles_deg": [round(float(np.degrees(angle_min + i * angle_inc)), 1) for i in near_idx[:12]],
        "near_ranges": [round(float(m[i]), 3) for i in near_idx[:12]],
        "beams": n, "measured_hits": int(m_hit.sum()), "predicted_hits": int(p_hit.sum()),
        "normal": normal, "mirrored": mirrored, "best_shift_beams": best_shift,
        "best_shift_deg": best_shift * 360.0 / n, "survivor_beams": len(survivor_beams),
        "survivor_measured": [float(m[i]) for i in survivor_beams[:10]],
        "survivor_predicted": [float(p[i]) for i in survivor_beams[:10]],
    }


def verdict(res: Dict) -> str:
    if res.get("near_unexplained", 0) >= 2:
        return (f"FAIL: {res['near_unexplained']} beams hit something within 0.35 m that is not in the arena "
                f"(the vehicle's own body in the scan plane?) at {res['near_angles_deg']} deg")
    nb = res["normal"]
    if nb["both"] < 20:
        return "TOO FEW WALL RETURNS here to judge - move the rig inside the maze and run again"
    frac = nb["within_tol"] / max(1, nb["both"])
    mir = res["mirrored"]["within_tol"] / max(1, res["mirrored"]["both"])
    if frac >= 0.9 and res["best_shift_beams"] == 0:
        amb = " (mirror test ambiguous at this pose)" if mir >= 0.9 else ""
        return f"PASS: {frac:.0%} of wall beams within 5 cm, no rotation, not mirrored{amb}"
    if mir > frac + 0.2:
        return "FAIL: scan looks MIRRORED (angle direction opposite to the ROS convention)"
    if res["best_shift_beams"] != 0:
        return f"FAIL: scan looks ROTATED by {res['best_shift_deg']:+.1f} deg (sensor yaw / frame offset)"
    return f"FAIL: only {frac:.0%} of wall beams within 5 cm (pose, height or geometry differs)"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--height", type=float, default=None, help="LiDAR plane height; default = odometry z")
    ap.add_argument("--spawn-height", type=float, default=1.0, help="height given to run_rig.sh")
    ap.add_argument("--scans", type=int, default=5)
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    sys.path.insert(0, str(REPO / "gazebo" / "ros2"))
    from nidar_rig_driver import OdomFrame, yaw_from_quaternion

    layout_path = ar.default_layout(args.seed)
    arena = ar.load_arena(layout_path)
    sp = arena.layout["spawn_pose"]
    frame = OdomFrame((sp["x"], sp["y"], sp["yaw"]))

    rclpy.init()
    node = Node("nidar_scan_check")
    state = {"odom": None, "scans": []}

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        state["odom"] = (p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_scan(msg):
        if state["odom"] is not None:
            state["scans"].append((msg, state["odom"]))

    node.create_subscription(Odometry, "/nidar/rig/odom", on_odom, 10)
    node.create_subscription(LaserScan, "/nidar/scan", on_scan, qos_profile_sensor_data)
    t0 = time.time()
    while len(state["scans"]) < args.scans and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not state["scans"]:
        print("[CHECK] no scan+odom received in 15 s (is run_rig.sh running?)"
              f" odom={'yes' if state['odom'] else 'NO'}")
        return 1

    msg, (ox, oy, oz, oyaw) = state["scans"][-1]
    x, y, yaw = frame.to_arena(ox, oy, oyaw)
    height = args.height if args.height is not None else (oz if not frame.relative else args.spawn_height + oz)
    n = len(msg.ranges)
    pred, labels = resample_prediction(arena, x, y, yaw, height, n, msg.angle_min, msg.angle_increment)
    res = compare(list(msg.ranges), pred, labels, msg.range_min, msg.range_max,
                  angle_min=msg.angle_min, angle_inc=msg.angle_increment)

    print("=" * 72)
    print(f"world {layout_path.stem}   scans received {len(state['scans'])}")
    print(f"odometry raw   : x={ox:.3f} y={oy:.3f} z={oz:.3f} yaw={math.degrees(oyaw):.1f} deg "
          f"-> {'RELATIVE to spawn' if frame.relative else 'arena/world frame'}")
    print(f"arena pose     : x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} deg  LiDAR plane {height:.2f} m")
    print(f"scan           : {n} beams, angle_min {math.degrees(msg.angle_min):.2f} deg, "
          f"inc {math.degrees(msg.angle_increment):.4f} deg, range {msg.range_min:.2f}-{msg.range_max:.1f} m")
    print(f"hits           : measured {res['measured_hits']}  predicted {res['predicted_hits']}")
    nb = res["normal"]
    print(f"wall beams     : {nb['both']} both-hit, {nb['within_tol']} within 5 cm, "
          f"median err {nb['median_err']:.3f} m, p95 {nb['p95_err']:.3f} m")
    mb = res["mirrored"]
    print(f"if mirrored    : {mb['within_tol']} within 5 cm (should be much lower unless the scene is symmetric)")
    print(f"best rotation  : {res['best_shift_beams']} beams ({res['best_shift_deg']:+.1f} deg)")
    for name, deg in (("forward", 0), ("left", 90), ("back", 180), ("right", -90)):
        i = int(round((math.radians(deg) - msg.angle_min) / msg.angle_increment)) % n
        mv = msg.ranges[i]
        print(f"  {name:8s} ({deg:+4d} deg): measured {mv:7.3f} m   predicted {pred[i]:7.3f} m")
    if res["survivor_beams"]:
        print(f"survivor beams : {res['survivor_beams']} (mesh vs cylinder, not scored) "
              f"measured {np.round(res['survivor_measured'][:5], 2).tolist()} "
              f"predicted {np.round(res['survivor_predicted'][:5], 2).tolist()}")
    print(f"near returns   : {res['near_unexplained']} beams < 0.35 m not explained by the arena"
          + (f" at {res['near_angles_deg']} deg, ranges {res['near_ranges']}" if res['near_unexplained'] else ""))
    print("VERDICT        :", verdict(res))
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
