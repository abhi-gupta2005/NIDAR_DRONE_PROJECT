#!/usr/bin/env python3
"""
NIDAR V13.0 - Camera + LiDAR Fusion Foundation

Purpose:
    Associate NIDAR camera detections with persistent STL-19P LiDAR tracks.

This first version deliberately uses a 2-D bearing association model:
    camera bbox center -> camera bearing
    LiDAR track bearing -> LiDAR bearing
    nearest compatible track -> fused object

It does NOT send flight-controller commands and does NOT claim calibrated
3-D position. Camera/LiDAR extrinsic and intrinsic calibration is the next
step after this association layer is validated.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Iterable, Optional


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class CameraCalibration:
    # Horizontal field of view of the NIDAR camera.
    hfov_deg: float = 70.0

    # Camera optical center as a fraction of image width.
    cx_norm: float = 0.5

    # Camera and LiDAR yaw offset.
    # Positive means camera bearing is rotated relative to LiDAR.
    yaw_offset_deg: float = 0.0


@dataclass
class FusionConfig:
    # Maximum camera/LiDAR bearing disagreement for association.
    max_bearing_error_deg: float = 8.0

    # If camera class and LiDAR track are compatible, allow association.
    # LiDAR itself is class-agnostic.
    use_class_gate: bool = False

    # Do not fuse tracks that are too stale.
    max_lidar_missed_scans: int = 3

    # Confidence weights.
    bearing_weight: float = 0.65
    camera_confidence_weight: float = 0.35


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------

@dataclass
class CameraDetection:
    stable_id: int
    class_name: str
    confidence: float
    frame: int
    timestamp: float
    image_width: int
    image_height: int

    # Bounding box in pixels: x1, y1, x2, y2.
    x1: float
    y1: float
    x2: float
    y2: float

    # Optional state information from NIDAR V11.
    state: str = "UNKNOWN"
    spatial_status: str = "OBSERVED"

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) * 0.5

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) * 0.5


@dataclass
class LidarTrackInput:
    track_id: int
    x_m: float
    y_m: float
    distance_m: float
    bearing_deg: float
    speed_mps: float = 0.0
    state: str = "CONFIRMED"
    observation_state: str = "MEASURED"
    motion_state: str = "STATIONARY"
    confidence: float = 1.0
    track_quality: float = 1.0
    missed_scans: int = 0

    @property
    def active(self) -> bool:
        return self.missed_scans <= 3


@dataclass
class FusedObject:
    fusion_id: str
    camera_id: int
    lidar_id: int

    class_name: str
    camera_confidence: float
    lidar_confidence: float
    fusion_confidence: float

    bearing_camera_deg: float
    bearing_lidar_deg: float
    bearing_error_deg: float

    distance_m: float
    x_m: float
    y_m: float
    speed_mps: float

    camera_state: str
    lidar_state: str
    lidar_observation: str
    motion_state: str


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def camera_pixel_to_bearing(
    center_x: float,
    image_width: int,
    calibration: CameraCalibration,
) -> float:
    """
    Convert horizontal image position to an approximate bearing.

    Assumes a pinhole-style horizontal FOV approximation. This is an
    association-stage estimate, not a calibrated 3-D reconstruction.
    """
    if image_width <= 0:
        raise ValueError("image_width must be positive")

    normalized = (
        center_x / float(image_width)
        - calibration.cx_norm
    )

    bearing = normalized * calibration.hfov_deg
    bearing += calibration.yaw_offset_deg

    return normalize_angle_deg(bearing)


def fusion_confidence(
    bearing_error_deg: float,
    camera_confidence: float,
    lidar_confidence: float,
    config: FusionConfig,
) -> float:
    bearing_score = max(
        0.0,
        1.0
        - bearing_error_deg / config.max_bearing_error_deg,
    )

    sensor_score = (
        config.camera_confidence_weight
        * max(0.0, min(1.0, camera_confidence))
        + (1.0 - config.camera_confidence_weight)
        * max(0.0, min(1.0, lidar_confidence))
    )

    score = (
        config.bearing_weight * bearing_score
        + (1.0 - config.bearing_weight) * sensor_score
    )

    return max(0.0, min(1.0, score))


# ---------------------------------------------------------------------------
# Fusion engine
# ---------------------------------------------------------------------------

class CameraLidarFusion:
    def __init__(
        self,
        calibration: Optional[CameraCalibration] = None,
        config: Optional[FusionConfig] = None,
    ):
        self.calibration = calibration or CameraCalibration()
        self.config = config or FusionConfig()

    def associate(
        self,
        camera_objects: Iterable[CameraDetection],
        lidar_tracks: Iterable[LidarTrackInput],
    ) -> list[FusedObject]:

        cameras = list(camera_objects)
        lidars = [
            track
            for track in lidar_tracks
            if track.active
            and track.state in ("CONFIRMED", "TEMP_LOST")
        ]

        candidates = []

        for cam in cameras:
            cam_bearing = camera_pixel_to_bearing(
                cam.center_x,
                cam.image_width,
                self.calibration,
            )

            for lidar in lidars:
                error = abs(
                    normalize_angle_deg(
                        cam_bearing - lidar.bearing_deg
                    )
                )

                if error > self.config.max_bearing_error_deg:
                    continue

                score = fusion_confidence(
                    error,
                    cam.confidence,
                    lidar.confidence * lidar.track_quality,
                    self.config,
                )

                candidates.append(
                    (
                        score,
                        error,
                        cam,
                        lidar,
                        cam_bearing,
                    )
                )

        # Greedy one-to-one association.
        candidates.sort(
            key=lambda item: (
                item[0],
                -item[1],
            ),
            reverse=True,
        )

        used_camera = set()
        used_lidar = set()
        fused = []

        for (
            score,
            error,
            cam,
            lidar,
            cam_bearing,
        ) in candidates:

            if cam.stable_id in used_camera:
                continue

            if lidar.track_id in used_lidar:
                continue

            used_camera.add(cam.stable_id)
            used_lidar.add(lidar.track_id)

            fused.append(
                FusedObject(
                    fusion_id=(
                        f"C{cam.stable_id:03d}"
                        f"-L{lidar.track_id:03d}"
                    ),
                    camera_id=cam.stable_id,
                    lidar_id=lidar.track_id,
                    class_name=cam.class_name,
                    camera_confidence=cam.confidence,
                    lidar_confidence=(
                        lidar.confidence
                        * lidar.track_quality
                    ),
                    fusion_confidence=score,
                    bearing_camera_deg=cam_bearing,
                    bearing_lidar_deg=lidar.bearing_deg,
                    bearing_error_deg=error,
                    distance_m=lidar.distance_m,
                    x_m=lidar.x_m,
                    y_m=lidar.y_m,
                    speed_mps=lidar.speed_mps,
                    camera_state=cam.state,
                    lidar_state=lidar.state,
                    lidar_observation=lidar.observation_state,
                    motion_state=lidar.motion_state,
                )
            )

        return fused


# ---------------------------------------------------------------------------
# JSON helpers for the current NIDAR V11 Pi receiver output
# ---------------------------------------------------------------------------

def camera_detection_from_nidar_record(
    record: dict[str, Any],
    image_width: int,
    image_height: int,
    frame: int,
    timestamp: float,
) -> Optional[CameraDetection]:

    # V11 records can contain center/predicted-center fields. For fusion,
    # we require a real bounding box. If a future packet adds bbox fields,
    # this adapter can be extended without changing the fusion engine.
    bbox = record.get("bbox") or record.get("bounding_box")

    if not bbox:
        return None

    if isinstance(bbox, dict):
        x1 = bbox.get("x1", bbox.get("left"))
        y1 = bbox.get("y1", bbox.get("top"))
        x2 = bbox.get("x2", bbox.get("right"))
        y2 = bbox.get("y2", bbox.get("bottom"))
    elif isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
        x1, y1, x2, y2 = bbox[:4]
    else:
        return None

    if None in (x1, y1, x2, y2):
        return None

    return CameraDetection(
        stable_id=int(record.get("stable_id", record.get("tracker_id", record.get("id")))),
        class_name=str(
            record.get("class_name", record.get("class", "unknown"))
        ),
        confidence=float(record.get("confidence", 0.0)),
        frame=frame,
        timestamp=timestamp,
        image_width=image_width,
        image_height=image_height,
        x1=float(x1),
        y1=float(y1),
        x2=float(x2),
        y2=float(y2),
        state=str(record.get("state", "UNKNOWN")),
        spatial_status=str(
            record.get("spatial_status", "OBSERVED")
        ),
    )


# ---------------------------------------------------------------------------
# Controlled fusion self-test
# ---------------------------------------------------------------------------

def run_self_test() -> None:
    calibration = CameraCalibration(
        hfov_deg=70.0,
        cx_norm=0.5,
        yaw_offset_deg=0.0,
    )

    config = FusionConfig(
        max_bearing_error_deg=8.0,
    )

    engine = CameraLidarFusion(
        calibration,
        config,
    )

    # 1280 px camera:
    # x=640 -> 0°
    # x=320 -> -17.5°
    # x=960 -> +17.5°
    cameras = [
        CameraDetection(
            stable_id=1,
            class_name="person",
            confidence=0.91,
            frame=100,
            timestamp=time.time(),
            image_width=1280,
            image_height=720,
            x1=600,
            y1=180,
            x2=680,
            y2=500,
            state="CONFIRMED",
        ),
        CameraDetection(
            stable_id=2,
            class_name="chair",
            confidence=0.84,
            frame=100,
            timestamp=time.time(),
            image_width=1280,
            image_height=720,
            x1=900,
            y1=200,
            x2=1020,
            y2=600,
            state="CONFIRMED",
        ),
    ]

    lidars = [
        LidarTrackInput(
            track_id=11,
            x_m=2.00,
            y_m=0.00,
            distance_m=2.00,
            bearing_deg=0.0,
            speed_mps=0.40,
            motion_state="MOVING",
            confidence=0.95,
            track_quality=0.94,
        ),
        LidarTrackInput(
            track_id=12,
            x_m=2.80,
            y_m=1.00,
            distance_m=2.97,
            bearing_deg=19.7,
            speed_mps=0.0,
            motion_state="STATIONARY",
            confidence=0.90,
            track_quality=0.91,
        ),
    ]

    fused = engine.associate(
        cameras,
        lidars,
    )

    print("=" * 78)
    print("NIDAR V13.0 - CAMERA + LIDAR FUSION SELF-TEST")
    print("=" * 78)
    print(
        f"Camera objects : {len(cameras)}"
    )
    print(
        f"LiDAR tracks   : {len(lidars)}"
    )
    print(
        f"Fused objects  : {len(fused)}"
    )
    print()

    for obj in fused:
        print(
            f"{obj.fusion_id} | "
            f"{obj.class_name:8s} | "
            f"bearing cam={obj.bearing_camera_deg:+6.2f}° | "
            f"lidar={obj.bearing_lidar_deg:+6.2f}° | "
            f"error={obj.bearing_error_deg:4.2f}° | "
            f"range={obj.distance_m:4.2f}m | "
            f"speed={obj.speed_mps:4.2f}m/s | "
            f"confidence={obj.fusion_confidence:.2f}"
        )

    print()
    print(
        "ASSOCIATION TEST : "
        + ("PASS" if len(fused) == 2 else "FAIL")
    )
    print(
        "ONE-TO-ONE TEST  : "
        + (
            "PASS"
            if len(
                {(x.camera_id, x.lidar_id) for x in fused}
            ) == len(fused)
            else "FAIL"
        )
    )
    print("=" * 78)


if __name__ == "__main__":
    run_self_test()
