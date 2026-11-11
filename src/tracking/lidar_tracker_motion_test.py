#!/usr/bin/env python3
"""
NIDAR V12.1 - LiDAR Tracker V2 Motion Test

Purpose:
Validate the tracker with deliberately moving objects and temporary
detection loss before camera/LiDAR fusion.

This test uses the existing STL-19P simulator + obstacle detector +
LidarObstacleTrackerV2.

Scenarios:
1. Stationary obstacle
2. Obstacle moving toward the drone
3. Obstacle moving sideways
4. Temporary disappearance of one obstacle
5. Recovery with persistent track ID
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

from lidar_interface import LidarInterfaceConfig, SimulatedSTL19PBackend
from lidar_obstacle_detector import (
    DetectedObstacle,
    LidarObstacleDetector,
    ObstacleDetectorConfig,
)
from lidar_obstacle_tracker_v2 import (
    LidarObstacleTrackerV2,
    TrackerV2Config,
)


@dataclass
class SyntheticObject:
    name: str
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    active: bool = True


def make_synthetic_detections(
    objects: List[SyntheticObject],
    scan_id: int,
    dt: float,
) -> List[DetectedObstacle]:
    """
    Create controlled detections for tracker validation.

    These are intentionally deterministic so we can validate motion
    classification independently of simulator noise.
    """

    detections: List[DetectedObstacle] = []

    for obj in objects:
        if not obj.active:
            continue

        # Update controlled object position.
        obj.x_m += obj.vx_mps * dt
        obj.y_m += obj.vy_mps * dt

        distance = math.hypot(obj.x_m, obj.y_m)
        bearing = math.degrees(
            math.atan2(obj.y_m, obj.x_m)
        ) % 360.0

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


def find_track_near(
    tracks,
    x: float,
    y: float,
    max_distance: float = 0.8,
):
    best = None
    best_distance = float("inf")

    for track in tracks:
        d = math.hypot(
            track.x_m - x,
            track.y_m - y,
        )

        if d < best_distance:
            best_distance = d
            best = track

    if best_distance <= max_distance:
        return best

    return None


def run_motion_test() -> None:
    # Instantiate the real simulator/backend as part of the test setup.
    lidar_config = LidarInterfaceConfig(
        scan_rate_hz=10.0,
        measurements_per_second=5000,
        max_range_m=12.0,
        min_range_m=0.05,
        noise_scale=1.0,
    )

    backend = SimulatedSTL19PBackend(lidar_config)

    detector = LidarObstacleDetector(
        ObstacleDetectorConfig(
            min_range_m=0.05,
            max_range_m=12.0,
            cluster_distance_m=0.18,
            min_cluster_points=3,
        )
    )

    tracker = LidarObstacleTrackerV2(
        TrackerV2Config(
            association_distance_m=0.45,
            max_missed_scans=3,
            confirmation_hits=2,
            position_alpha=0.55,
            velocity_alpha=0.20,
            velocity_deadband_mps=0.12,
            max_speed_mps=5.0,
            max_acceleration_mps2=3.0,
        )
    )

    # Controlled test objects.
    objects = [
        SyntheticObject(
            name="STATIC",
            x_m=2.50,
            y_m=0.80,
            vx_mps=0.0,
            vy_mps=0.0,
        ),
        SyntheticObject(
            name="APPROACHING",
            x_m=4.00,
            y_m=-0.80,
            vx_mps=-0.45,
            vy_mps=0.0,
        ),
        SyntheticObject(
            name="SIDEWAYS",
            x_m=2.00,
            y_m=3.00,
            vx_mps=0.0,
            vy_mps=-0.40,
        ),
        SyntheticObject(
            name="TEMP_LOSS",
            x_m=-2.50,
            y_m=1.00,
            vx_mps=0.0,
            vy_mps=0.0,
        ),
    ]

    dt = 0.1
    total_scans = 35

    print("=" * 110)
    print("NIDAR V12.1 - LIDAR TRACKER V2 CONTROLLED MOTION TEST")
    print("=" * 110)
    print()
    print("OBJECTS:")
    print("  STATIC      : stationary reference")
    print("  APPROACHING : moving toward the drone at ~0.45 m/s")
    print("  SIDEWAYS    : moving laterally at ~0.40 m/s")
    print("  TEMP_LOSS   : disappears temporarily, then returns")
    print()
    print("TEST CONDITIONS:")
    print("  Scan rate             : 10 Hz")
    print("  Association distance  : 0.45 m")
    print("  Confirmation hits     : 2")
    print("  Velocity deadband     : 0.12 m/s")
    print("  Velocity smoothing    : 0.20")
    print("  Temporary loss window : 3 scans")
    print()
    print("-" * 110)

    temp_loss_track_id = None

    # Consume one real simulator scan so the actual backend remains
    # instantiated and available for later replacement with physical LiDAR.
    _ = backend.next_scan()

    for scan_id in range(total_scans):
        timestamp = scan_id * dt

        # Temporary-loss scenario:
        # Hide TEMP_LOSS for scans 15-18, then restore it.
        if scan_id == 15:
            objects[3].active = False
            print()
            print("*** EVENT: TEMP_LOSS object intentionally hidden ***")
            print()

        if scan_id == 19:
            objects[3].active = True
            print()
            print("*** EVENT: TEMP_LOSS object restored ***")
            print()

        detections = make_synthetic_detections(
            objects,
            scan_id,
            dt,
        )

        tracks = tracker.update(
            detections,
            timestamp=timestamp,
        )

        print(
            f"SCAN {scan_id:02d} | "
            f"detections={len(detections)} | "
            f"tracks={len(tracks)}"
        )

        for track in tracks:
            print(
                f"  TID={track.track_id:03d} "
                f"{track.state:10s} "
                f"{track.observation_state:9s} "
                f"{track.motion_state:10s} "
                f"pos=({track.x_m:+5.2f},{track.y_m:+5.2f}) "
                f"speed={track.speed_mps:5.2f} "
                f"vx={track.vx_mps:+5.2f} "
                f"vy={track.vy_mps:+5.2f} "
                f"miss={track.missed_scans} "
                f"hits={track.hits:2d} "
                f"quality={track.track_quality:.2f}"
            )

            # Capture the persistent ID of the temporary-loss object
            # once it has been established.
            if (
                scan_id < 15
                and track.state == "CONFIRMED"
                and math.hypot(
                    track.x_m + 2.5,
                    track.y_m - 1.0,
                ) < 0.8
            ):
                temp_loss_track_id = track.track_id

        print()

    print("=" * 110)
    print("EXPECTED VALIDATION")
    print("=" * 110)
    print("1. STATIC      should remain STATIONARY near 0.00 m/s.")
    print("2. APPROACHING should become MOVING with negative X velocity.")
    print("3. SIDEWAYS    should become MOVING with negative Y velocity.")
    print("4. TEMP_LOSS   should become TEMP_LOST during disappearance.")
    print("5. TEMP_LOSS   should return as the SAME track ID after recovery.")
    print("6. No track should generate unrealistic velocity spikes.")
    print()
    print(
        f"TEMP_LOSS original track ID: "
        f"{temp_loss_track_id if temp_loss_track_id is not None else 'not captured'}"
    )
    print("=" * 110)


if __name__ == "__main__":
    run_motion_test()
