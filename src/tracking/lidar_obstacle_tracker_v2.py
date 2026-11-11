#!/usr/bin/env python3
"""
NIDAR V12.1 - LiDAR Obstacle Tracker V2

Refinement over lidar_obstacle_tracker.py:
- stable persistent IDs
- filtered velocity estimation
- stationary / moving classification
- measured / predicted state
- confidence decay during temporary loss
- track quality score
- velocity deadband to suppress noise
- bounded acceleration/velocity changes

This remains independent of the camera and flight controller.
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
class TrackedObstacleV2:
    track_id: int
    x_m: float
    y_m: float
    distance_m: float
    bearing_deg: float

    width_m: float
    depth_m: float

    vx_mps: float
    vy_mps: float
    speed_mps: float

    confidence: float
    track_quality: float

    motion_state: str
    observation_state: str
    state: str

    age_scans: int
    hits: int
    missed_scans: int
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
            "vx_mps": round(self.vx_mps, 4),
            "vy_mps": round(self.vy_mps, 4),
            "speed_mps": round(self.speed_mps, 4),
            "confidence": round(self.confidence, 4),
            "track_quality": round(self.track_quality, 4),
            "motion_state": self.motion_state,
            "observation_state": self.observation_state,
            "state": self.state,
            "age_scans": self.age_scans,
            "hits": self.hits,
            "missed_scans": self.missed_scans,
            "point_count": self.point_count,
        }


@dataclass
class TrackerV2Config:
    association_distance_m: float = 0.45
    max_missed_scans: int = 3
    confirmation_hits: int = 2

    position_alpha: float = 0.55
    velocity_alpha: float = 0.20

    # Velocities below this are treated as stationary/noise.
    velocity_deadband_mps: float = 0.12

    # Prevent noise spikes from producing unrealistic motion.
    max_speed_mps: float = 5.0

    # Maximum allowed velocity change per second.
    max_acceleration_mps2: float = 3.0

    # Confidence behavior.
    confidence_decay_prediction: float = 0.82

    # Track-quality weights.
    quality_hit_weight: float = 0.55
    quality_confidence_weight: float = 0.30
    quality_age_weight: float = 0.15


class LidarObstacleTrackerV2:
    """Persistent nearest-neighbour tracker with motion classification."""

    def __init__(self, config: Optional[TrackerV2Config] = None):
        self.config = config or TrackerV2Config()
        self.tracks: Dict[int, TrackedObstacleV2] = {}
        self.next_track_id = 1
        self.previous_timestamp: Optional[float] = None

    @staticmethod
    def _distance(track: TrackedObstacleV2, detection: DetectedObstacle) -> float:
        return math.hypot(
            track.x_m - detection.center_x_m,
            track.y_m - detection.center_y_m,
        )

    def _new_track(self, detection: DetectedObstacle) -> TrackedObstacleV2:
        state = (
            "CONFIRMED"
            if self.config.confirmation_hits <= 1
            else "TENTATIVE"
        )

        track = TrackedObstacleV2(
            track_id=self.next_track_id,
            x_m=detection.center_x_m,
            y_m=detection.center_y_m,
            distance_m=detection.distance_m,
            bearing_deg=detection.bearing_deg,
            width_m=detection.width_m,
            depth_m=detection.depth_m,
            vx_mps=0.0,
            vy_mps=0.0,
            speed_mps=0.0,
            confidence=detection.confidence,
            track_quality=0.0,
            motion_state="STATIONARY",
            observation_state="MEASURED",
            state=state,
            age_scans=1,
            hits=1,
            missed_scans=0,
            point_count=detection.point_count,
        )

        self.next_track_id += 1
        return track

    def _update_velocity(
        self,
        track: TrackedObstacleV2,
        new_x: float,
        new_y: float,
        dt: float,
    ) -> None:
        if dt <= 1e-6:
            return

        raw_vx = (new_x - track.x_m) / dt
        raw_vy = (new_y - track.y_m) / dt

        raw_speed = math.hypot(raw_vx, raw_vy)

        # Limit extreme measurement noise.
        if raw_speed > self.config.max_speed_mps:
            scale = self.config.max_speed_mps / raw_speed
            raw_vx *= scale
            raw_vy *= scale

        # Limit acceleration relative to previous filtered velocity.
        dvx = raw_vx - track.vx_mps
        dvy = raw_vy - track.vy_mps
        max_dv = self.config.max_acceleration_mps2 * dt
        dv = math.hypot(dvx, dvy)

        if dv > max_dv and dv > 1e-9:
            scale = max_dv / dv
            raw_vx = track.vx_mps + dvx * scale
            raw_vy = track.vy_mps + dvy * scale

        a = self.config.velocity_alpha

        track.vx_mps = (
            a * raw_vx + (1.0 - a) * track.vx_mps
        )
        track.vy_mps = (
            a * raw_vy + (1.0 - a) * track.vy_mps
        )

        speed = math.hypot(track.vx_mps, track.vy_mps)

        # Deadband: small movement is treated as measurement noise.
        if speed < self.config.velocity_deadband_mps:
            track.vx_mps = 0.0
            track.vy_mps = 0.0
            speed = 0.0

        track.speed_mps = speed

        if speed == 0.0:
            track.motion_state = "STATIONARY"
        else:
            track.motion_state = "MOVING"

    def _calculate_quality(self, track: TrackedObstacleV2) -> float:
        hit_quality = min(
            1.0,
            track.hits / 10.0,
        )

        age_quality = min(
            1.0,
            track.age_scans / 10.0,
        )

        quality = (
            self.config.quality_hit_weight * hit_quality
            + self.config.quality_confidence_weight * track.confidence
            + self.config.quality_age_weight * age_quality
        )

        if track.state == "TEMP_LOST":
            quality *= max(
                0.0,
                1.0 - 0.20 * track.missed_scans,
            )

        return float(np.clip(quality, 0.0, 1.0))

    def _update_track(
        self,
        track: TrackedObstacleV2,
        detection: DetectedObstacle,
        dt: float,
    ) -> None:
        alpha = self.config.position_alpha

        old_x = track.x_m
        old_y = track.y_m

        new_x = (
            alpha * detection.center_x_m
            + (1.0 - alpha) * old_x
        )
        new_y = (
            alpha * detection.center_y_m
            + (1.0 - alpha) * old_y
        )

        self._update_velocity(track, new_x, new_y, dt)

        track.x_m = new_x
        track.y_m = new_y

        track.distance_m = math.hypot(new_x, new_y)
        track.bearing_deg = (
            math.degrees(math.atan2(new_y, new_x)) % 360.0
        )

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

        track.point_count = detection.point_count
        track.age_scans += 1
        track.hits += 1
        track.missed_scans = 0

        track.observation_state = "MEASURED"

        if track.hits >= self.config.confirmation_hits:
            track.state = "CONFIRMED"

        track.track_quality = self._calculate_quality(track)

    def _predict_track(
        self,
        track: TrackedObstacleV2,
        dt: float,
    ) -> None:
        track.x_m += track.vx_mps * dt
        track.y_m += track.vy_mps * dt

        track.distance_m = math.hypot(track.x_m, track.y_m)
        track.bearing_deg = (
            math.degrees(math.atan2(track.y_m, track.x_m)) % 360.0
        )

        track.missed_scans += 1
        track.age_scans += 1

        track.observation_state = "PREDICTED"
        track.state = "TEMP_LOST"

        track.confidence *= self.config.confidence_decay_prediction

        track.track_quality = self._calculate_quality(track)

    def update(
        self,
        detections: List[DetectedObstacle],
        timestamp: float,
    ) -> List[TrackedObstacleV2]:

        if self.previous_timestamp is None:
            dt = 0.1
        else:
            dt = max(
                1e-3,
                min(0.5, timestamp - self.previous_timestamp),
            )

        self.previous_timestamp = timestamp

        # Predict existing tracks first.
        for track in list(self.tracks.values()):
            self._predict_track(track, dt)

        unmatched_tracks = set(self.tracks.keys())
        unmatched_detections = set(range(len(detections)))

        candidates: List[Tuple[float, int, int]] = []

        for track_id, track in self.tracks.items():
            for index, detection in enumerate(detections):
                distance = self._distance(track, detection)

                if distance <= self.config.association_distance_m:
                    candidates.append(
                        (distance, track_id, index)
                    )

        candidates.sort(key=lambda c: c[0])

        for _, track_id, index in candidates:
            if track_id not in unmatched_tracks:
                continue

            if index not in unmatched_detections:
                continue

            self._update_track(
                self.tracks[track_id],
                detections[index],
                dt,
            )

            unmatched_tracks.remove(track_id)
            unmatched_detections.remove(index)

        # New detections become new tracks.
        for index in sorted(unmatched_detections):
            track = self._new_track(detections[index])
            track.track_quality = self._calculate_quality(track)
            self.tracks[track.track_id] = track

        # Delete tracks that exceed the prediction budget.
        expired = [
            track_id
            for track_id, track in self.tracks.items()
            if track.missed_scans > self.config.max_missed_scans
        ]

        for track_id in expired:
            del self.tracks[track_id]

        result = list(self.tracks.values())

        result.sort(
            key=lambda t: (
                t.state != "CONFIRMED",
                -t.track_quality,
                t.distance_m,
            )
        )

        return result


def print_tracks(tracks: List[TrackedObstacleV2]) -> None:
    print(f"Persistent tracks: {len(tracks)}")

    for track in tracks:
        print(
            f"  TID={track.track_id:03d} "
            f"{track.state:10s} "
            f"{track.observation_state:9s} "
            f"{track.motion_state:10s} "
            f"dist={track.distance_m:5.2f} m "
            f"bearing={track.bearing_deg:6.1f}° "
            f"speed={track.speed_mps:5.2f} m/s "
            f"miss={track.missed_scans} "
            f"hits={track.hits:2d} "
            f"conf={track.confidence:.2f} "
            f"quality={track.track_quality:.2f}"
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

    print("=" * 86)
    print("NIDAR V12.1 - LiDAR TRACKER V2 SELF TEST")
    print("=" * 86)
    print("Association distance : 0.45 m")
    print("Confirmation hits    : 2")
    print("Velocity smoothing   : 0.20")
    print("Velocity deadband    : 0.12 m/s")
    print("Max speed            : 5.00 m/s")
    print("Max acceleration     : 3.00 m/s²")
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

    print("=" * 86)
    print("LiDAR Tracker V2 test complete.")
    print("=" * 86)


if __name__ == "__main__":
    run_self_test()
