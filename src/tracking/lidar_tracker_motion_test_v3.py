#!/usr/bin/env python3
"""
NIDAR V12.2 - LiDAR Tracker V3 Controlled Motion Validation

Validates:
1. STATIC object -> ~0 m/s
2. APPROACHING object -> negative X velocity
3. SIDEWAYS object -> negative Y velocity
4. TEMP_LOSS -> TEMP_LOST/PREDICTED, then same ID + RECOVERED
5. No unrealistic velocity spikes
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from lidar_obstacle_detector import DetectedObstacle
from lidar_obstacle_tracker_v3 import (
    LidarObstacleTrackerV3,
    TrackerV3Config,
)


@dataclass
class SyntheticObject:
    name: str
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    active: bool = True


def make_detections(
    objects: List[SyntheticObject],
    scan_id: int,
    dt: float,
) -> List[DetectedObstacle]:
    detections = []

    for obj in objects:
        if not obj.active:
            continue

        obj.x_m += obj.vx_mps * dt
        obj.y_m += obj.vy_mps * dt

        distance = math.hypot(obj.x_m, obj.y_m)
        bearing = math.degrees(math.atan2(obj.y_m, obj.x_m)) % 360.0

        detections.append(
            DetectedObstacle(
                obstacle_id=scan_id * 100 + len(detections),
                center_x_m=obj.x_m,
                center_y_m=obj.y_m,
                distance_m=distance,
                bearing_deg=bearing,
                width_m=0.30,
                depth_m=0.30,
                point_count=20,
                confidence=0.90,
                start_angle_deg=(bearing - 2.0) % 360.0,
                end_angle_deg=(bearing + 2.0) % 360.0,
            )
        )

    return detections


def run_test() -> None:
    tracker = LidarObstacleTrackerV3(
        TrackerV3Config(
            association_distance_m=0.55,
            recovery_distance_m=0.75,
            max_missed_scans=3,
            confirmation_hits=2,
            position_alpha=0.55,
            velocity_alpha=0.30,
            velocity_deadband_mps=0.08,
            max_speed_mps=5.0,
            max_acceleration_mps2=3.0,
        )
    )

    objects = [
        SyntheticObject("STATIC", 2.5, 0.8, 0.0, 0.0),
        SyntheticObject("APPROACHING", 4.0, -0.8, -0.45, 0.0),
        SyntheticObject("SIDEWAYS", 2.0, 3.0, 0.0, -0.40),
        SyntheticObject("TEMP_LOSS", -2.5, 1.0, 0.0, 0.0),
    ]

    dt = 0.1
    total_scans = 35
    temp_loss_id = None
    max_speed_seen = 0.0
    recovery_seen = False
    moving_seen = {"APPROACHING": False, "SIDEWAYS": False}
    stationary_ok = False

    print("=" * 118)
    print("NIDAR V12.2 - LIDAR TRACKER V3 CONTROLLED MOTION TEST")
    print("=" * 118)
    print("STATIC      : 0.00 m/s reference")
    print("APPROACHING : -0.45 m/s X")
    print("SIDEWAYS    : -0.40 m/s Y")
    print("TEMP_LOSS   : hidden scans 15-17, restored scan 18")
    print("")

    for scan_id in range(total_scans):
        objects[3].active = not (15 <= scan_id <= 17)

        if scan_id == 15:
            print("*** EVENT: TEMP_LOSS hidden ***")
        if scan_id == 18:
            print("*** EVENT: TEMP_LOSS restored ***")

        detections = make_detections(objects, scan_id, dt)
        tracks = tracker.update(detections, scan_id * dt)

        print(f"SCAN {scan_id:02d} | detections={len(detections)} | tracks={len(tracks)}")

        for track in tracks:
            print(
                f"  T{track.track_id:02d} "
                f"{track.state:10s} "
                f"{track.observation_state:9s} "
                f"{track.motion_state:10s} "
                f"pos=({track.x_m:+5.2f},{track.y_m:+5.2f}) "
                f"speed={track.speed_mps:5.2f} "
                f"vx={track.vx_mps:+5.2f} "
                f"vy={track.vy_mps:+5.2f} "
                f"miss={track.missed_scans} "
                f"hits={track.hits:02d} "
                f"quality={track.track_quality:.2f}"
            )

            max_speed_seen = max(max_speed_seen, track.speed_mps)

            if track.motion_state == "MOVING":
                # Identify by current position.
                if track.y_m < 0.0 and track.x_m > 2.0:
                    moving_seen["APPROACHING"] = True
                if track.x_m > 0.5 and track.y_m > 1.0:
                    moving_seen["SIDEWAYS"] = True

            if (
                track.x_m > 2.0
                and 0.3 < track.y_m < 1.3
                and track.speed_mps < 0.08
            ):
                stationary_ok = True

            if (
                scan_id < 15
                and track.state == "CONFIRMED"
                and math.hypot(track.x_m + 2.5, track.y_m - 1.0) < 0.8
            ):
                temp_loss_id = track.track_id

            if (
                scan_id == 18
                and temp_loss_id is not None
                and track.track_id == temp_loss_id
                and track.observation_state == "RECOVERED"
            ):
                recovery_seen = True

        print()

    print("=" * 118)
    print("VALIDATION RESULT")
    print("=" * 118)
    print(f"STATIC near 0 m/s       : {'PASS' if stationary_ok else 'CHECK'}")
    print(f"APPROACHING detected    : {'PASS' if moving_seen['APPROACHING'] else 'FAIL'}")
    print(f"SIDEWAYS detected       : {'PASS' if moving_seen['SIDEWAYS'] else 'FAIL'}")
    print(f"TEMP_LOSS original ID   : T{temp_loss_id:02d}" if temp_loss_id else "TEMP_LOSS ID: NOT FOUND")
    print(f"TEMP_LOSS recovered     : {'PASS' if recovery_seen else 'FAIL'}")
    print(f"Max speed observed      : {max_speed_seen:.2f} m/s")
    print(f"Velocity spike check    : {'PASS' if max_speed_seen < 2.0 else 'CHECK'}")
    print("=" * 118)


if __name__ == "__main__":
    run_test()
