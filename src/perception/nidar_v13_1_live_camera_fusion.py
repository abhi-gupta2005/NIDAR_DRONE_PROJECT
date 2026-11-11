#!/usr/bin/env python3
"""
NIDAR V13.1 - Live Camera -> Fusion Integration Harness

Reads the actual NIDAR perception packet produced by the existing
Mac -> Raspberry Pi TCP receiver:

    ~/drone_project/nidar_latest_perception.json

This version validates the LIVE CAMERA DATA PATH and performs bearing
association against a controlled/synthetic LiDAR layer.

IMPORTANT:
- Camera data is real NIDAR packet data.
- LiDAR tracks in this V13.1 harness are synthetic test tracks.
- The synthetic LiDAR range is NOT a physical measurement.
- No flight-controller commands are sent.

Why synthetic LiDAR here?
The current Pi has the STL-19P simulator, while the physical sensor
has not yet been connected. This lets us validate camera packet parsing,
camera bearing calculation, fusion IDs, confidence, and one-to-one
association before physical LiDAR integration.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any

from nidar_camera_lidar_fusion_v1 import (
    CameraCalibration,
    CameraDetection,
    CameraLidarFusion,
    FusionConfig,
    LidarTrackInput,
)


PERCEPTION_FILE = Path.home() / "drone_project" / "nidar_latest_perception.json"

# Match the current Mac NIDAR camera stream resolution.
DEFAULT_IMAGE_WIDTH = 1280
DEFAULT_IMAGE_HEIGHT = 720

# Approximate initial camera FOV. This MUST be calibrated later.
CAMERA_HFOV_DEG = 70.0

# Synthetic LiDAR range for integration testing only.
BASE_SYNTHETIC_RANGE_M = 2.5


def first_value(data: dict[str, Any], *keys, default=None):
    for key in keys:
        if key in data and data[key] is not None:
            return data[key]
    return default


def extract_objects(packet: dict[str, Any]) -> list[dict[str, Any]]:
    """
    Accept the common NIDAR packet layouts without requiring the Mac
    sender to be rewritten immediately.
    """
    for key in ("objects", "detections", "tracks", "perception"):
        value = packet.get(key)

        if isinstance(value, list):
            return value

        if isinstance(value, dict):
            for nested_key in ("objects", "detections", "tracks"):
                nested = value.get(nested_key)
                if isinstance(nested, list):
                    return nested

    return []


def center_x_from_record(
    obj: dict[str, Any],
    image_width: int,
) -> float | None:
    # Preferred: explicit center_x.
    value = first_value(
        obj,
        "center_x",
        "cx",
        "image_center_x",
    )

    if value is not None:
        return float(value)

    # Alternative: nested center.
    center = obj.get("center")
    if isinstance(center, dict):
        value = first_value(
            center,
            "x",
            "cx",
        )
        if value is not None:
            return float(value)

    if isinstance(center, (list, tuple)) and len(center) >= 1:
        return float(center[0])

    # Fallback: derive center from bbox.
    bbox = obj.get("bbox") or obj.get("bounding_box")

    if isinstance(bbox, dict):
        x1 = first_value(bbox, "x1", "left")
        x2 = first_value(bbox, "x2", "right")
        if x1 is not None and x2 is not None:
            return (float(x1) + float(x2)) * 0.5

    if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
        return (float(bbox[0]) + float(bbox[2])) * 0.5

    # Last-resort support for normalized center.
    normalized = first_value(
        obj,
        "center_x_norm",
        "cx_norm",
    )

    if normalized is not None:
        return float(normalized) * image_width

    return None


def build_camera_detections(
    packet: dict[str, Any],
) -> list[CameraDetection]:
    image_width = int(
        first_value(
            packet,
            "camera_width",
            "image_width",
            default=DEFAULT_IMAGE_WIDTH,
        )
    )

    image_height = int(
        first_value(
            packet,
            "camera_height",
            "image_height",
            default=DEFAULT_IMAGE_HEIGHT,
        )
    )

    frame = int(
        first_value(
            packet,
            "frame",
            "frame_id",
            default=0,
        )
    )

    timestamp = float(
        first_value(
            packet,
            "timestamp",
            "time",
            default=time.time(),
        )
    )

    detections: list[CameraDetection] = []

    for obj in extract_objects(packet):
        stable_id = first_value(
            obj,
            "stable_id",
            "id",
            "track_id",
        )

        if stable_id is None:
            continue

        center_x = center_x_from_record(
            obj,
            image_width,
        )

        if center_x is None:
            continue

        class_name = str(
            first_value(
                obj,
                "class_name",
                "class",
                "label",
                default="unknown",
            )
        )

        confidence = float(
            first_value(
                obj,
                "confidence",
                "conf",
                default=0.0,
            )
        )

        state = str(
            first_value(
                obj,
                "state",
                default="UNKNOWN",
            )
        )

        # A small synthetic vertical extent is enough for this stage.
        # We only use center X for bearing association.
        detections.append(
            CameraDetection(
                stable_id=int(stable_id),
                class_name=class_name,
                confidence=confidence,
                frame=frame,
                timestamp=timestamp,
                image_width=image_width,
                image_height=image_height,
                x1=center_x - 20.0,
                y1=image_height * 0.35,
                x2=center_x + 20.0,
                y2=image_height * 0.65,
                state=state,
            )
        )

    return detections


def make_synthetic_lidar_tracks(
    cameras: list[CameraDetection],
) -> list[LidarTrackInput]:
    """
    Create a controlled LiDAR counterpart for each camera object.

    This deliberately uses the camera-derived bearing so that we can
    test the fusion plumbing before physical LiDAR is available.
    """
    tracks: list[LidarTrackInput] = []

    calibration = CameraCalibration(
        hfov_deg=CAMERA_HFOV_DEG,
        cx_norm=0.5,
        yaw_offset_deg=0.0,
    )

    from nidar_camera_lidar_fusion_v1 import camera_pixel_to_bearing

    for index, camera in enumerate(cameras):
        bearing = camera_pixel_to_bearing(
            camera.center_x,
            camera.image_width,
            calibration,
        )

        # Give different objects different deterministic ranges.
        distance = (
            BASE_SYNTHETIC_RANGE_M
            + 0.35 * (index % 5)
        )

        radians = math.radians(bearing)

        x = distance * math.cos(radians)
        y = distance * math.sin(radians)

        tracks.append(
            LidarTrackInput(
                track_id=100 + int(camera.stable_id),
                x_m=x,
                y_m=y,
                distance_m=distance,
                bearing_deg=bearing,
                speed_mps=0.0,
                state="CONFIRMED",
                observation_state="MEASURED",
                motion_state="STATIONARY",
                confidence=0.95,
                track_quality=0.90,
                missed_scans=0,
            )
        )

    return tracks


def run_once() -> bool:
    if not PERCEPTION_FILE.exists():
        print()
        print("ERROR: NIDAR perception file was not found:")
        print(f"  {PERCEPTION_FILE}")
        print()
        print("Start the Pi NIDAR receiver and Mac perception stream first.")
        return False

    try:
        packet = json.loads(
            PERCEPTION_FILE.read_text(
                encoding="utf-8"
            )
        )
    except Exception as exc:
        print(f"ERROR reading perception JSON: {exc}")
        return False

    cameras = build_camera_detections(packet)

    fusion = CameraLidarFusion(
        calibration=CameraCalibration(
            hfov_deg=CAMERA_HFOV_DEG,
            cx_norm=0.5,
            yaw_offset_deg=0.0,
        ),
        config=FusionConfig(
            max_bearing_error_deg=8.0,
        ),
    )

    lidar = make_synthetic_lidar_tracks(cameras)

    fused = fusion.associate(
        cameras,
        lidar,
    )

    frame = first_value(
        packet,
        "frame",
        "frame_id",
        default=0,
    )

    timestamp = first_value(
        packet,
        "timestamp",
        "time",
        default=0.0,
    )

    print("\033[2J\033[H", end="")
    print("=" * 88)
    print("NIDAR V13.1 - LIVE CAMERA → FUSION INTEGRATION")
    print("=" * 88)
    print(f"Source packet : {PERCEPTION_FILE}")
    print(f"Frame         : {frame}")
    print(f"Timestamp     : {timestamp}")
    print(f"Camera objects: {len(cameras)}")
    print(f"LiDAR tracks  : {len(lidar)}  [SYNTHETIC TEST DATA]")
    print(f"Fused objects : {len(fused)}")
    print("-" * 88)

    if not cameras:
        print("No camera objects with usable center coordinates were found.")
        print()
        print("The current packet may only contain spatial center fields.")
        print("If so, V13.1 will add explicit camera bbox fields to the Mac sender.")
        print("=" * 88)
        return False

    for obj in fused:
        print(
            f"{obj.fusion_id:10s} | "
            f"{obj.class_name:12s} | "
            f"CamID={obj.camera_id:3d} | "
            f"LiDAR={obj.lidar_id:3d} | "
            f"bearing={obj.bearing_camera_deg:+6.2f}° | "
            f"range={obj.distance_m:4.2f}m | "
            f"conf={obj.fusion_confidence:.2f}"
        )

    print("-" * 88)

    one_to_one = (
        len(fused)
        == len(
            {
                (obj.camera_id, obj.lidar_id)
                for obj in fused
            }
        )
    )

    print(
        "CAMERA PACKET READ : "
        + ("PASS" if cameras else "FAIL")
    )
    print(
        "FUSION ASSOCIATION  : "
        + ("PASS" if fused else "WAITING")
    )
    print(
        "ONE-TO-ONE MAPPING  : "
        + ("PASS" if one_to_one else "FAIL")
    )

    print()
    print(
        "NOTE: LiDAR range is synthetic in V13.1. "
        "Physical STL-19P data comes next."
    )
    print("=" * 88)

    return bool(cameras and fused and one_to_one)


def main():
    print("Watching live NIDAR perception packet...")
    print("Press Ctrl+C to stop.")
    time.sleep(1.0)

    last_signature = None

    while True:
        try:
            if PERCEPTION_FILE.exists():
                stat = PERCEPTION_FILE.stat()
                signature = (
                    stat.st_mtime_ns,
                    stat.st_size,
                )

                if signature != last_signature:
                    last_signature = signature
                    run_once()

            time.sleep(0.10)

        except KeyboardInterrupt:
            print("\nStopped.")
            break


if __name__ == "__main__":
    main()
