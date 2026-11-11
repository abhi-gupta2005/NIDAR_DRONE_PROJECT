#!/usr/bin/env python3
"""
NIDAR V12.1 - Persistent LiDAR Obstacle Tracker

Input:
    DetectedObstacle objects from lidar_obstacle_detector.py

Output:
    Persistent TrackedObstacle objects with stable IDs.

Purpose:
    Keep the same identity for an obstacle across successive LiDAR scans.
    This is required before camera + LiDAR sensor fusion.

This is a lightweight nearest-neighbour tracker suitable for the current
simulation and as a first baseline for the physical STL-19P.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from lidar_interface import LidarInterfaceConfig, SimulatedSTL19PBackend
from lidar_obstacle_detector import (
    DetectedObstacle,
    LidarObstacleDetector,
    ObstacleDetectorConfig,
)


@dataclass
class TrackedObstacle:
    track_id: int
    x_m: float
    y_m: float
    distance_m: float
    bearing_deg: float

    width_m: float
    depth_m: float
    confidence: float

    vx_mps: float
    vy_mps: float
    speed_mps: float

    age_scans: int
    missed_scans: int
    hits: int
    state: str

    point_count: int

    def to_dict(self) -> dict:
        return {
            "track_id": self.track_id,
            "x_m": round(self.x_m, 4),
            "y_m": round(self.y_m, 4),
            "distance_m": round(self.distance_m, 4),
            "bearing_deg": round(self.bearing_deg, 3),
            "width_m": round(self.width_m, 4),
            "depth_m": round(self.depth_m, 4),
            "confidence": round(self.confidence, 4),
            "vx_mps": round(self.vx_mps, 4),
            "vy_mps": round(self.vy_mps, 4),
            "speed_mps": round(self.speed_mps, 4),
            "age_scans": self.age_scans,
            "missed_scans": self.missed_scans,
            "hits": self.hits,
            "state": self.state,
            "point_count": self.point_count,
        }


@dataclass
class TrackerConfig:
    # Maximum position difference for associating a detection to a track.
    association_distance_m: float = 0.45

    # Track is considered temporarily lost after this many missed scans.
    max_missed_scans: int = 3

    # New detections need this many hits to become CONFIRMED.
    confirmation_hits: int = 2

    # Position smoothing. Higher = more responsive, lower = smoother.
    position_alpha: float = 0.55

    # Velocity smoothing.
    velocity_alpha: float = 0.45

    # Prevent unrealistically large velocity values from simulator noise.
    max_speed_mps: float = 8.0


class LidarObstacleTracker:
    """Nearest-neighbour persistent tracker in local LiDAR XY coordinates."""

    def __init__(self, config: Optional[TrackerConfig] = None):
        self.config = config or TrackerConfig()
        self.tracks: Dict[int, TrackedObstacle] = {}
        self.next_track_id = 1
        self.previous_timestamp: Optional[float] = None

    @staticmethod
    def _distance(a: TrackedObstacle, d: DetectedObstacle) -> float:
        return math.hypot(a.x_m - d.center_x_m, a.y_m - d.center_y_m)

    def _new_track(self, detection: DetectedObstacle) -> TrackedObstacle:
        track = TrackedObstacle(
            track_id=self.next_track_id,
            x_m=detection.center_x_m,
            y_m=detection.center_y_m,
            distance_m=detection.distance_m,
            bearing_deg=detection.bearing_deg,
            width_m=detection.width_m,
            depth_m=detection.depth_m,
            confidence=detection.confidence,
            vx_mps=0.0,
            vy_mps=0.0,
            speed_mps=0.0,
            age_scans=1,
            missed_scans=0,
            hits=1,
            state=(
                "CONFIRMED"
                if self.config.confirmation_hits <= 1
                else "TENTATIVE"
            ),
            point_count=detection.point_count,
        )

        self.next_track_id += 1
        return track

    def _update_track(
        self,
        track: TrackedObstacle,
        detection: DetectedObstacle,
        dt: float,
    ) -> None:
        old_x = track.x_m
        old_y = track.y_m

        alpha = self.config.position_alpha

        new_x = (
            alpha * detection.center_x_m
            + (1.0 - alpha) * old_x
        )
        new_y = (
            alpha * detection.center_y_m
            + (1.0 - alpha) * old_y
        )

        if dt > 1e-6:
            raw_vx = (new_x - old_x) / dt
            raw_vy = (new_y - old_y) / dt

            raw_speed = math.hypot(raw_vx, raw_vy)

            if raw_speed > self.config.max_speed_mps:
                scale = self.config.max_speed_mps / raw_speed
                raw_vx *= scale
                raw_vy *= scale

            va = self.config.velocity_alpha
            track.vx_mps = va * raw_vx + (1.0 - va) * track.vx_mps
            track.vy_mps = va * raw_vy + (1.0 - va) * track.vy_mps

        track.x_m = new_x
        track.y_m = new_y

        track.distance_m = math.hypot(new_x, new_y)

        bearing = math.degrees(math.atan2(new_y, new_x))
        track.bearing_deg = bearing % 360.0

        track.width_m = (
            alpha * detection.width_m
            + (1.0 - alpha) * track.width_m
        )
        track.depth_m = (
            alpha * detection.depth_m
            + (1.0 - alpha) * track.depth_m
        )
        track.confidence = (
            alpha * detection.confidence
            + (1.0 - alpha) * track.confidence
        )

        track.speed_mps = math.hypot(track.vx_mps, track.vy_mps)
        track.point_count = detection.point_count

        track.age_scans += 1
        track.missed_scans = 0
        track.hits += 1

        if track.hits >= self.config.confirmation_hits:
            track.state = "CONFIRMED"

    def _predict_track(self, track: TrackedObstacle, dt: float) -> None:
        track.x_m += track.vx_mps * dt
        track.y_m += track.vy_mps * dt

        track.distance_m = math.hypot(track.x_m, track.y_m)

        bearing = math.degrees(math.atan2(track.y_m, track.x_m))
        track.bearing_deg = bearing % 360.0

        track.missed_scans += 1
        track.age_scans += 1

        # Slowly reduce confidence while the object is not observed.
        track.confidence *= 0.82

        if track.missed_scans > 0:
            track.state = "TEMP_LOST"

    def update(
        self,
        detections: List[DetectedObstacle],
        timestamp: float,
    ) -> List[TrackedObstacle]:
        """
        Associate current detections with existing tracks and update them.

        Each detection is used at most once.
        """
        if self.previous_timestamp is None:
            dt = 0.1
        else:
            dt = max(1e-3, timestamp - self.previous_timestamp)

        self.previous_timestamp = timestamp

        # Predict every existing track before association.
        existing_ids = list(self.tracks.keys())
        for track_id in existing_ids:
            self._predict_track(self.tracks[track_id], dt)

        unmatched_tracks = set(self.tracks.keys())
        unmatched_detections = set(range(len(detections)))

        # Greedy nearest-neighbour association.
        candidates: List[Tuple[float, int, int]] = []

        for track_id in self.tracks:
            track = self.tracks[track_id]

            for detection_index, detection in enumerate(detections):
                distance = self._distance(track, detection)

                if distance <= self.config.association_distance_m:
                    candidates.append(
                        (distance, track_id, detection_index)
                    )

        candidates.sort(key=lambda item: item[0])

        for _, track_id, detection_index in candidates:
            if track_id not in unmatched_tracks:
                continue
            if detection_index not in unmatched_detections:
                continue

            self._update_track(
                self.tracks[track_id],
                detections[detection_index],
                dt,
            )

            unmatched_tracks.remove(track_id)
            unmatched_detections.remove(detection_index)

        # Every unmatched detection starts a new track.
        for detection_index in sorted(unmatched_detections):
            track = self._new_track(detections[detection_index])
            self.tracks[track.track_id] = track

        # Remove tracks that have been missing too long.
        to_delete = [
            track_id
            for track_id, track in self.tracks.items()
            if track.missed_scans > self.config.max_missed_scans
        ]

        for track_id in to_delete:
            del self.tracks[track_id]

        # Return oldest/most established tracks first, then distance.
        result = list(self.tracks.values())
        result.sort(key=lambda t: (-t.hits, t.distance_m))

        return result


def print_tracks(tracks: List[TrackedObstacle]) -> None:
    print(f"Persistent tracks: {len(tracks)}")

    for track in tracks:
        print(
            f"  TID={track.track_id:03d} "
            f"{track.state:10s} "
            f"dist={track.distance_m:5.2f} m "
            f"bearing={track.bearing_deg:6.1f}° "
            f"xy=({track.x_m:6.2f},{track.y_m:6.2f}) "
            f"speed={track.speed_mps:5.2f} m/s "
            f"miss={track.missed_scans} "
            f"hits={track.hits:2d} "
            f"conf={track.confidence:.2f}"
        )


def run_self_test(scans: int = 15) -> None:
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

    tracker = LidarObstacleTracker(
        TrackerConfig(
            association_distance_m=0.45,
            max_missed_scans=3,
            confirmation_hits=2,
            position_alpha=0.55,
            velocity_alpha=0.45,
        )
    )

    print("=" * 76)
    print("NIDAR V12.1 - PERSISTENT LiDAR OBSTACLE TRACKER SELF TEST")
    print("=" * 76)
    print("Association distance : 0.45 m")
    print("Confirmation hits    : 2")
    print("Max missed scans     : 3")
    print("Position smoothing   : 0.55")
    print()

    for _ in range(scans):
        scan = backend.next_scan()

        detections = detector.detect(scan)
        tracks = tracker.update(
            detections,
            timestamp=scan.timestamp,
        )

        print(
            f"scan={scan.scan_id:04d} "
            f"detections={len(detections):2d} "
            f"tracks={len(tracks):2d}"
        )

        print_tracks(tracks)
        print()

    print("=" * 76)
    print("Persistent tracker test complete.")
    print("=" * 76)


if __name__ == "__main__":
    run_self_test()
