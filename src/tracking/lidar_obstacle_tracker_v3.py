#!/usr/bin/env python3
"""
NIDAR V12.2 - LiDAR Obstacle Tracker V3

Fixes V2 motion/recovery issues:
- Velocity is calculated from LAST MEASURED position, never predicted position.
- Prediction is kept separate from measurement history.
- TEMP_LOST tracks remain eligible for recovery.
- Returning detections recover the SAME track ID.
- Explicit RECOVERED observation state.
- Motion classification uses filtered measured velocity.
- Velocity deadband and acceleration limiting retained.
- Confidence/quality handling retained.

Designed to remain compatible with lidar_obstacle_detector.py.
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
class TrackedObstacleV3:
    track_id: int

    # Current output position. This may be measured or predicted.
    x_m: float
    y_m: float

    # Last actual sensor measurement.
    last_measured_x_m: float
    last_measured_y_m: float

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

    recovery_count: int = 0

    def to_dict(self) -> dict:
        return {
            "track_id": self.track_id,
            "x_m": round(self.x_m, 4),
            "y_m": round(self.y_m, 4),
            "last_measured_x_m": round(self.last_measured_x_m, 4),
            "last_measured_y_m": round(self.last_measured_y_m, 4),
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
            "recovery_count": self.recovery_count,
        }


@dataclass
class TrackerV3Config:
    association_distance_m: float = 0.55
    recovery_distance_m: float = 0.75

    max_missed_scans: int = 3
    confirmation_hits: int = 2

    position_alpha: float = 0.55
    velocity_alpha: float = 0.30

    velocity_deadband_mps: float = 0.08
    max_speed_mps: float = 5.0
    max_acceleration_mps2: float = 3.0

    confidence_decay_prediction: float = 0.82

    quality_hit_weight: float = 0.55
    quality_confidence_weight: float = 0.30
    quality_age_weight: float = 0.15


class LidarObstacleTrackerV3:
    """
    Persistent nearest-neighbour tracker.

    Important design rule:
    velocity MUST be calculated from two real measurements.

        previous measured position
                    ↓
             new measured position
                    ↓
                raw velocity

    Predicted positions are never used as the velocity reference.
    """

    def __init__(self, config: Optional[TrackerV3Config] = None):
        self.config = config or TrackerV3Config()

        self.tracks: Dict[int, TrackedObstacleV3] = {}
        self.next_track_id = 1
        self.previous_timestamp: Optional[float] = None

    @staticmethod
    def _distance_xy(
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    def _distance_to_detection(
        self,
        track: TrackedObstacleV3,
        detection: DetectedObstacle,
    ) -> float:
        # Associate against predicted/current position.
        return self._distance_xy(
            track.x_m,
            track.y_m,
            detection.center_x_m,
            detection.center_y_m,
        )

    def _new_track(
        self,
        detection: DetectedObstacle,
    ) -> TrackedObstacleV3:

        state = (
            "CONFIRMED"
            if self.config.confirmation_hits <= 1
            else "TENTATIVE"
        )

        track = TrackedObstacleV3(
            track_id=self.next_track_id,

            x_m=detection.center_x_m,
            y_m=detection.center_y_m,

            last_measured_x_m=detection.center_x_m,
            last_measured_y_m=detection.center_y_m,

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

            recovery_count=0,
        )

        self.next_track_id += 1
        return track

    def _update_velocity_from_measurements(
        self,
        track: TrackedObstacleV3,
        measured_x: float,
        measured_y: float,
        dt: float,
    ) -> None:
        """
        Calculate velocity ONLY from consecutive measured positions.

        This is the critical V3 fix.
        """

        if dt <= 1e-6:
            return

        raw_vx = (
            measured_x - track.last_measured_x_m
        ) / dt

        raw_vy = (
            measured_y - track.last_measured_y_m
        ) / dt

        raw_speed = math.hypot(raw_vx, raw_vy)

        # Limit impossible measurement spikes.
        if raw_speed > self.config.max_speed_mps:
            scale = self.config.max_speed_mps / raw_speed
            raw_vx *= scale
            raw_vy *= scale

        # Limit acceleration changes.
        dvx = raw_vx - track.vx_mps
        dvy = raw_vy - track.vy_mps
        dv = math.hypot(dvx, dvy)

        max_dv = self.config.max_acceleration_mps2 * dt

        if dv > max_dv and dv > 1e-9:
            scale = max_dv / dv

            raw_vx = (
                track.vx_mps
                + dvx * scale
            )

            raw_vy = (
                track.vy_mps
                + dvy * scale
            )

        alpha = self.config.velocity_alpha

        track.vx_mps = (
            alpha * raw_vx
            + (1.0 - alpha) * track.vx_mps
        )

        track.vy_mps = (
            alpha * raw_vy
            + (1.0 - alpha) * track.vy_mps
        )

        speed = math.hypot(
            track.vx_mps,
            track.vy_mps,
        )

        # Suppress low-level sensor jitter.
        if speed < self.config.velocity_deadband_mps:
            track.vx_mps = 0.0
            track.vy_mps = 0.0
            speed = 0.0

        track.speed_mps = speed

        if speed == 0.0:
            track.motion_state = "STATIONARY"
        else:
            track.motion_state = "MOVING"

    def _calculate_quality(
        self,
        track: TrackedObstacleV3,
    ) -> float:

        hit_quality = min(
            1.0,
            track.hits / 10.0,
        )

        age_quality = min(
            1.0,
            track.age_scans / 10.0,
        )

        quality = (
            self.config.quality_hit_weight
            * hit_quality
            + self.config.quality_confidence_weight
            * track.confidence
            + self.config.quality_age_weight
            * age_quality
        )

        if track.state == "TEMP_LOST":
            quality *= max(
                0.0,
                1.0 - 0.20 * track.missed_scans,
            )

        return float(
            np.clip(quality, 0.0, 1.0)
        )

    def _update_measured_track(
        self,
        track: TrackedObstacleV3,
        detection: DetectedObstacle,
        dt: float,
    ) -> None:

        # Save old measured position BEFORE replacing it.
        old_measured_x = track.last_measured_x_m
        old_measured_y = track.last_measured_y_m

        measured_x = detection.center_x_m
        measured_y = detection.center_y_m

        # Velocity uses measured-to-measured displacement.
        self._update_velocity_from_measurements(
            track,
            measured_x,
            measured_y,
            dt,
        )

        alpha = self.config.position_alpha

        # Smooth displayed position.
        track.x_m = (
            alpha * measured_x
            + (1.0 - alpha) * track.x_m
        )

        track.y_m = (
            alpha * measured_y
            + (1.0 - alpha) * track.y_m
        )

        # Store the NEW measured position separately.
        track.last_measured_x_m = measured_x
        track.last_measured_y_m = measured_y

        track.distance_m = math.hypot(
            track.x_m,
            track.y_m,
        )

        track.bearing_deg = (
            math.degrees(
                math.atan2(track.y_m, track.x_m)
            ) % 360.0
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

        was_lost = track.state == "TEMP_LOST"

        track.observation_state = (
            "RECOVERED"
            if was_lost
            else "MEASURED"
        )

        if was_lost:
            track.recovery_count += 1

        if track.hits >= self.config.confirmation_hits:
            track.state = "CONFIRMED"

        track.track_quality = self._calculate_quality(
            track
        )

    def _predict_track(
        self,
        track: TrackedObstacleV3,
        dt: float,
    ) -> None:

        # Prediction affects output position only.
        # It NEVER modifies last_measured_x/y.
        track.x_m += track.vx_mps * dt
        track.y_m += track.vy_mps * dt

        track.distance_m = math.hypot(
            track.x_m,
            track.y_m,
        )

        track.bearing_deg = (
            math.degrees(
                math.atan2(track.y_m, track.x_m)
            ) % 360.0
        )

        track.missed_scans += 1
        track.age_scans += 1

        track.observation_state = "PREDICTED"

        # A track only becomes TEMP_LOST when a complete association
        # cycle finishes without a measurement. Do not mark it lost
        # during the prediction step because a measurement may arrive
        # in the same scan.
        track.confidence *= (
            self.config.confidence_decay_prediction
        )

        track.track_quality = self._calculate_quality(
            track
        )

    def _association_gate(
        self,
        track: TrackedObstacleV3,
    ) -> float:
        """
        Give TEMP_LOST tracks a slightly larger recovery gate.
        """

        if track.state == "TEMP_LOST":
            return self.config.recovery_distance_m

        return self.config.association_distance_m

    def update(
        self,
        detections: List[DetectedObstacle],
        timestamp: float,
    ) -> List[TrackedObstacleV3]:

        if self.previous_timestamp is None:
            dt = 0.1
        else:
            dt = max(
                1e-3,
                min(
                    0.5,
                    timestamp - self.previous_timestamp,
                ),
            )

        self.previous_timestamp = timestamp

        # ---------------------------------------------------------
        # STEP 1: Predict current tracks.
        # ---------------------------------------------------------
        for track in list(self.tracks.values()):
            self._predict_track(
                track,
                dt,
            )

        unmatched_tracks = set(
            self.tracks.keys()
        )

        unmatched_detections = set(
            range(len(detections))
        )

        candidates: List[
            Tuple[float, int, int]
        ] = []

        # ---------------------------------------------------------
        # STEP 2: Build association candidates.
        # ---------------------------------------------------------
        for track_id, track in self.tracks.items():

            gate = self._association_gate(track)

            for index, detection in enumerate(detections):

                distance = self._distance_to_detection(
                    track,
                    detection,
                )

                if distance <= gate:
                    candidates.append(
                        (
                            distance,
                            track_id,
                            index,
                        )
                    )

        candidates.sort(
            key=lambda c: c[0]
        )

        # ---------------------------------------------------------
        # STEP 3: Greedy nearest-neighbour association.
        # ---------------------------------------------------------
        for _, track_id, index in candidates:

            if track_id not in unmatched_tracks:
                continue

            if index not in unmatched_detections:
                continue

            self._update_measured_track(
                self.tracks[track_id],
                detections[index],
                dt,
            )

            unmatched_tracks.remove(
                track_id
            )

            unmatched_detections.remove(
                index
            )

        # ---------------------------------------------------------
        # STEP 4: Tracks that received NO measurement this scan are
        # genuinely temporarily lost.
        # ---------------------------------------------------------
        for track_id in unmatched_tracks:
            track = self.tracks[track_id]

            # It was predicted above and never associated with a
            # measurement during this update cycle.
            track.state = "TEMP_LOST"
            track.observation_state = "PREDICTED"
            track.track_quality = self._calculate_quality(track)

        # ---------------------------------------------------------
        # STEP 5: Create new tracks for unmatched detections.
        # ---------------------------------------------------------
        for index in sorted(
            unmatched_detections
        ):
            track = self._new_track(
                detections[index]
            )

            track.track_quality = (
                self._calculate_quality(track)
            )

            self.tracks[
                track.track_id
            ] = track

        # ---------------------------------------------------------
        # STEP 6: Remove tracks after prediction budget.
        # ---------------------------------------------------------
        expired = [
            track_id
            for track_id, track
            in self.tracks.items()
            if track.missed_scans
            > self.config.max_missed_scans
        ]

        for track_id in expired:
            del self.tracks[track_id]

        result = list(
            self.tracks.values()
        )

        result.sort(
            key=lambda t: (
                t.state != "CONFIRMED",
                -t.track_quality,
                t.distance_m,
            )
        )

        return result


def print_tracks(
    tracks: List[TrackedObstacleV3],
) -> None:

    print(
        f"Persistent tracks: {len(tracks)}"
    )

    for track in tracks:

        print(
            f"  TID={track.track_id:03d} "
            f"{track.state:10s} "
            f"{track.observation_state:9s} "
            f"{track.motion_state:10s} "
            f"dist={track.distance_m:5.2f} m "
            f"bearing={track.bearing_deg:6.1f}° "
            f"speed={track.speed_mps:5.2f} m/s "
            f"vx={track.vx_mps:+5.2f} "
            f"vy={track.vy_mps:+5.2f} "
            f"miss={track.missed_scans} "
            f"hits={track.hits:2d} "
            f"recoveries={track.recovery_count} "
            f"quality={track.track_quality:.2f}"
        )


def run_self_test(
    scans: int = 15,
) -> None:

    lidar_config = LidarInterfaceConfig(
        scan_rate_hz=10.0,
        measurements_per_second=5000,
        max_range_m=12.0,
        min_range_m=0.05,
        noise_scale=1.0,
    )

    backend = SimulatedSTL19PBackend(
        lidar_config
    )

    detector = LidarObstacleDetector(
        ObstacleDetectorConfig(
            min_range_m=0.05,
            max_range_m=12.0,
            cluster_distance_m=0.18,
            min_cluster_points=3,
        )
    )

    tracker = LidarObstacleTrackerV3(
        TrackerV3Config()
    )

    print("=" * 100)
    print(
        "NIDAR V12.2 - LiDAR TRACKER V3 SELF TEST"
    )
    print("=" * 100)
    print(
        "Velocity source       : MEASURED → MEASURED"
    )
    print(
        "Association gate      : 0.55 m"
    )
    print(
        "Recovery gate         : 0.75 m"
    )
    print(
        "Confirmation hits     : 2"
    )
    print(
        "Velocity smoothing    : 0.30"
    )
    print(
        "Velocity deadband     : 0.08 m/s"
    )
    print(
        "Max speed             : 5.00 m/s"
    )
    print(
        "Max acceleration      : 3.00 m/s²"
    )
    print()

    for _ in range(scans):

        scan = backend.next_scan()

        detections = detector.detect(
            scan
        )

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

    print("=" * 100)
    print(
        "LiDAR Tracker V3 self-test complete."
    )
    print("=" * 100)


if __name__ == "__main__":
    run_self_test()
