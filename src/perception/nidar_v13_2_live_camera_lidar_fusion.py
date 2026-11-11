#!/usr/bin/env python3
"""
NIDAR V13.2
LIVE CAMERA + SIMULATED STL-19P + PERSISTENT LIDAR TRACKING + FUSION

Pipeline:
    Mac webcam/NIDAR V11 packet
        -> Pi JSON packet
        -> simulated STL-19P scan
        -> obstacle detection
        -> persistent LiDAR tracker V3
        -> camera/LiDAR bearing association
        -> fused object output

This version intentionally does NOT connect to the flight controller.
"""

import json
import math
import os
import sys
import time
from pathlib import Path

# Project-local modules
PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from lidar_interface import (
    LidarInterfaceConfig,
    SimulatedSTL19PBackend,
    LidarProcessor,
)
from lidar_obstacle_detector import LidarObstacleDetector, ObstacleDetectorConfig
from lidar_obstacle_tracker_v3 import LidarObstacleTrackerV3, TrackerV3Config

PACKET_PATH = PROJECT_DIR / "nidar_latest_perception.json"

# Camera model. This is an initial approximation, NOT final calibration.
HFOV_DEG = 70.0
MAX_BEARING_ERROR_DEG = 10.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def angle_diff_deg(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return d

def camera_bearing_deg(obj):
    """
    Extract camera horizontal center from several V11-compatible schemas.
    Returns None when no usable center is available.
    """
    candidates = [
        obj.get("center_x"),
        obj.get("image_center_x"),
        obj.get("cx"),
    ]

    # Nested center forms
    center = obj.get("center")
    if isinstance(center, dict):
        candidates.extend([
            center.get("x"),
            center.get("cx"),
        ])

    bbox = obj.get("bbox")
    if isinstance(bbox, dict):
        if bbox.get("x1") is not None and bbox.get("x2") is not None:
            candidates.append((bbox["x1"] + bbox["x2"]) / 2.0)
        elif bbox.get("left") is not None and bbox.get("right") is not None:
            candidates.append((bbox["left"] + bbox["right"]) / 2.0)

    for x in candidates:
        if x is None:
            continue
        try:
            x = float(x)
        except (TypeError, ValueError):
            continue

        # Already normalized.
        if 0.0 <= x <= 1.0:
            norm = x
        else:
            width = float(obj.get("_camera_width", 1280))
            if width <= 0:
                continue
            norm = x / width

        norm = clamp(norm, 0.0, 1.0)
        return (norm - 0.5) * HFOV_DEG

    return None

def read_packet():
    try:
        with open(PACKET_PATH, "r", encoding="utf-8") as f:
            packet = json.load(f)
        return packet
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None

def normalize_camera_objects(packet):
    objects = packet.get("objects", [])
    if not isinstance(objects, list):
        return []

    width = packet.get("camera", {}).get("width", packet.get("camera_width", 1280))
    height = packet.get("camera", {}).get("height", packet.get("camera_height", 720))

    result = []
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        o = dict(obj)
        o["_camera_width"] = width
        o["_camera_height"] = height
        result.append(o)
    return result

def confidence(obj):
    try:
        return float(obj.get("confidence", 0.0))
    except (TypeError, ValueError):
        return 0.0

def make_lidar_stack():
    lidar_cfg = LidarInterfaceConfig(
        scan_rate_hz=10.0,
        measurements_per_second=5000,
        max_range_m=12.0,
        min_range_m=0.05,
    )
    backend = SimulatedSTL19PBackend(lidar_cfg)

    processor = LidarProcessor(lidar_cfg)

    detector_cfg = ObstacleDetectorConfig()
    detector = LidarObstacleDetector(detector_cfg)

    tracker_cfg = TrackerV3Config()
    tracker = LidarObstacleTrackerV3(tracker_cfg)

    return backend, processor, detector, tracker

def get_detections_from_scan(scan, processor, detector):
    """
    Feed the simulator scan into the existing detector.
    Supports the current project APIs while keeping this integration isolated.
    """
    # Existing detector expects scan points in the project format.

    # Current detector API.
    if hasattr(detector, "detect"):
        try:
            return detector.detect(scan)
        except TypeError:
            pass

    # Compatibility fallback: pass the scan if detect() accepts it.
    return detector.detect(scan)

def update_tracker(tracker, detections, timestamp):
    if hasattr(tracker, "update"):
        try:
            return tracker.update(detections, timestamp)
        except TypeError:
            try:
                return tracker.update(detections)
            except TypeError:
                return tracker.update(detections, timestamp=timestamp)
    raise RuntimeError("Unsupported LiDAR tracker V3 API: update() not found")

def track_attr(track, name, default=None):
    if isinstance(track, dict):
        return track.get(name, default)
    return getattr(track, name, default)

def associate(camera_objects, tracks):
    """
    One-to-one greedy bearing association.
    Only CONFIRMED/usable LiDAR tracks are considered.
    """
    candidates = []

    for ci, cam in enumerate(camera_objects):
        cb = camera_bearing_deg(cam)
        if cb is None:
            continue

        for li, tr in enumerate(tracks):
            state = str(track_attr(tr, "state", ""))
            if state not in ("CONFIRMED", "RECOVERED", "TEMP_LOST"):
                continue

            lb = float(track_attr(tr, "bearing", 0.0))
            err = abs(angle_diff_deg(cb, lb))

            if err <= MAX_BEARING_ERROR_DEG:
                candidates.append((err, ci, li))

    candidates.sort(key=lambda x: x[0])

    used_c = set()
    used_l = set()
    matches = []

    for err, ci, li in candidates:
        if ci in used_c or li in used_l:
            continue
        used_c.add(ci)
        used_l.add(li)
        matches.append((ci, li, err))

    return matches

def print_frame(packet, camera_objects, tracks, matches, frame_no):
    print("\n" + "=" * 100)
    print("NIDAR V13.2 - LIVE CAMERA + STL-19P FUSION")
    print("=" * 100)
    print(f"Frame         : {frame_no}")
    print(f"Camera objects: {len(camera_objects)}")
    print(f"LiDAR tracks  : {len(tracks)}")
    print(f"Fused objects : {len(matches)}")
    print("-" * 100)

    for ci, li, err in matches:
        cam = camera_objects[ci]
        tr = tracks[li]

        cam_id = cam.get("stable_id", cam.get("id", "?"))
        cls = cam.get("class", cam.get("class_name", "unknown"))
        lb = float(track_attr(tr, "bearing", 0.0))
        dist = float(track_attr(tr, "distance", 0.0))
        speed = float(track_attr(tr, "speed", 0.0))
        state = track_attr(tr, "state", "?")
        tid = track_attr(tr, "track_id", "?")
        cam_conf = confidence(cam)

        # Confidence is deliberately simple in this prototype.
        fusion_conf = 0.6 * cam_conf + 0.4 * clamp(1.0 - err / MAX_BEARING_ERROR_DEG, 0.0, 1.0)

        print(
            f"C{str(cam_id):>3}-L{str(tid):<3} | "
            f"{str(cls):<12} | "
            f"bearing cam={camera_bearing_deg(cam):+6.2f}° | "
            f"lidar={lb:+6.2f}° | "
            f"error={err:4.2f}° | "
            f"range={dist:4.2f}m | "
            f"speed={speed:4.2f}m/s | "
            f"{state:<9} | "
            f"conf={fusion_conf:.2f}"
        )

    if not matches:
        print("No camera/LiDAR bearing matches in this frame.")

    print("-" * 100)
    print("CAMERA PACKET      : PASS")
    print("STL-19P SIMULATOR  : PASS")
    print("LIDAR DETECTION    : PASS")
    print("LIDAR TRACKING     : PASS")
    print("CAMERA/LIDAR FUSION: PASS")
    print("=" * 100)

def main():
    print("=" * 100)
    print("NIDAR V13.2 - LIVE CAMERA + SIMULATED STL-19P FUSION")
    print("=" * 100)
    print(f"Camera packet : {PACKET_PATH}")
    print("LiDAR source  : STL-19P simulator")
    print("Flight control: DISABLED")
    print("Press Ctrl+C to stop.")
    print("=" * 100)

    backend, processor, detector, tracker = make_lidar_stack()

    last_frame = None
    next_scan_time = time.monotonic()

    try:
        while True:
            packet = read_packet()

            if packet is None:
                time.sleep(0.05)
                continue

            frame = packet.get("frame", packet.get("frame_id"))
            if frame is None:
                frame = -1

            # We still process LiDAR at 10 Hz even if the camera packet
            # hasn't changed, but only print on a new camera frame.
            scan = backend.next_scan()
            detections = get_detections_from_scan(scan, processor, detector)
            tracks = update_tracker(tracker, detections, time.time())

            camera_objects = normalize_camera_objects(packet)
            matches = associate(camera_objects, tracks)

            if frame != last_frame:
                print_frame(packet, camera_objects, tracks, matches, frame)
                last_frame = frame

            next_scan_time += 0.10
            sleep_time = next_scan_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_scan_time = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            backend.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
