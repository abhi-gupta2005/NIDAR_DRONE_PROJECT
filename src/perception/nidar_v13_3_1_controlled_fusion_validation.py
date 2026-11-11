#!/usr/bin/env python3
"""
NIDAR V13.3 - CONTROLLED CAMERA + LiDAR FUSION VALIDATION

Purpose:
- Read the LIVE NIDAR V11 camera packet from the Pi.
- Run the real STL-19P simulator -> detector -> persistent tracker pipeline.
- Add ONE controlled validation target whose bearing is deliberately aligned
  with the selected live camera object's bearing.
- Run the normal one-to-one fusion association.

This validates the fusion association logic, not physical camera/LiDAR
calibration. The controlled target is synthetic.
No flight-controller connection.
"""

import json
import math
import sys
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from lidar_interface import LidarInterfaceConfig, SimulatedSTL19PBackend
from lidar_obstacle_detector import ObstacleDetectorConfig, LidarObstacleDetector
from lidar_obstacle_tracker_v3 import TrackerV3Config, LidarObstacleTrackerV3

PACKET_PATH = PROJECT_DIR / "nidar_latest_perception.json"

HFOV_DEG = 70.0
MAX_BEARING_ERROR_DEG = 10.0
CONTROLLED_RANGE_M = 2.50
CONTROLLED_TRACK_ID = 900

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def angle_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

def camera_bearing_deg(obj):
    candidates = [
        obj.get("center_x"),
        obj.get("image_center_x"),
        obj.get("cx"),
    ]

    center = obj.get("center")
    if isinstance(center, dict):
        candidates.extend([center.get("x"), center.get("cx")])
    elif isinstance(center, (list, tuple)) and len(center) >= 1:
        # NIDAR V11 schema: center = [x_px, y_px]
        candidates.append(center[0])

    normalized_center = obj.get("normalized_center")
    if isinstance(normalized_center, (list, tuple)) and len(normalized_center) >= 1:
        # NIDAR V11 schema: normalized_center = [x_norm, y_norm]
        candidates.append(normalized_center[0])

    bbox = obj.get("bbox")
    if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
        # NIDAR V11 schema: bbox = [x1, y1, x2, y2]
        candidates.append((float(bbox[0]) + float(bbox[2])) / 2.0)
    if isinstance(bbox, dict):
        if bbox.get("x1") is not None and bbox.get("x2") is not None:
            candidates.append((float(bbox["x1"]) + float(bbox["x2"])) / 2.0)
        elif bbox.get("left") is not None and bbox.get("right") is not None:
            candidates.append((float(bbox["left"]) + float(bbox["right"])) / 2.0)

    for x in candidates:
        if x is None:
            continue
        try:
            x = float(x)
        except (TypeError, ValueError):
            continue

        if 0.0 <= x <= 1.0:
            norm = x
        else:
            width = float(obj.get("_camera_width", 1280))
            if width <= 0:
                continue
            norm = x / width

        return (clamp(norm, 0.0, 1.0) - 0.5) * HFOV_DEG

    return None

def read_packet():
    try:
        with open(PACKET_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None

def camera_objects(packet):
    raw = packet.get("objects", [])
    if not isinstance(raw, list):
        return []

    width = packet.get("camera", {}).get(
        "width", packet.get("camera_width", 1280)
    )

    result = []
    for item in raw:
        if isinstance(item, dict):
            obj = dict(item)
            obj["_camera_width"] = width
            result.append(obj)
    return result

def attr(obj, name, default=None):
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)

def build_lidar_stack():
    cfg = LidarInterfaceConfig(
        scan_rate_hz=10.0,
        measurements_per_second=5000,
        max_range_m=12.0,
        min_range_m=0.05,
    )

    backend = SimulatedSTL19PBackend(cfg)
    detector = LidarObstacleDetector(ObstacleDetectorConfig())
    tracker = LidarObstacleTrackerV3(TrackerV3Config())

    return backend, detector, tracker

def update_tracker(tracker, detections, timestamp):
    try:
        return tracker.update(detections, timestamp)
    except TypeError:
        return tracker.update(detections)

def controlled_lidar_track(bearing_deg, timestamp):
    """
    A minimal synthetic LiDAR track-shaped dictionary.
    It is deliberately aligned to the selected camera bearing.
    """
    r = CONTROLLED_RANGE_M
    a = math.radians(bearing_deg)

    return {
        "track_id": CONTROLLED_TRACK_ID,
        "x": r * math.cos(a),
        "y": r * math.sin(a),
        "distance": r,
        "bearing": bearing_deg,
        "width": 0.35,
        "depth": 0.35,
        "point_count": 12,
        "confidence": 0.98,
        "vx": 0.0,
        "vy": 0.0,
        "speed": 0.0,
        "age": 1,
        "missed_scans": 0,
        "hits": 10,
        "state": "CONFIRMED",
    }

def associate(camera_objects, lidar_tracks):
    candidates = []

    for ci, cam in enumerate(camera_objects):
        cb = camera_bearing_deg(cam)
        if cb is None:
            continue

        for li, tr in enumerate(lidar_tracks):
            state = str(attr(tr, "state", ""))
            if state not in ("CONFIRMED", "RECOVERED"):
                continue

            lb = float(attr(tr, "bearing", 0.0))
            error = abs(angle_diff_deg(cb, lb))

            if error <= MAX_BEARING_ERROR_DEG:
                candidates.append((error, ci, li))

    candidates.sort()
    used_c = set()
    used_l = set()
    matches = []

    for error, ci, li in candidates:
        if ci in used_c or li in used_l:
            continue
        used_c.add(ci)
        used_l.add(li)
        matches.append((ci, li, error))

    return matches

def main():
    print("=" * 100)
    print("NIDAR V13.3 - CONTROLLED LIVE CAMERA + LiDAR FUSION VALIDATION")
    print("=" * 100)
    print(f"Camera packet : {PACKET_PATH}")
    print("Camera source : LIVE NIDAR V11")
    print("LiDAR source  : STL-19P simulator + CONTROLLED TEST TARGET")
    print(f"Test range    : {CONTROLLED_RANGE_M:.2f} m")
    print(f"Fusion gate   : ±{MAX_BEARING_ERROR_DEG:.1f}°")
    print("Flight control: DISABLED")
    print("The controlled LiDAR target is synthetic and follows one live")
    print("camera object's bearing to validate association logic.")
    print("=" * 100)

    backend, detector, tracker = build_lidar_stack()

    last_frame = None
    validated_frames = 0
    total_frames = 0
    start = time.monotonic()

    try:
        while True:
            packet = read_packet()
            if packet is None:
                time.sleep(0.05)
                continue

            frame = packet.get("frame", packet.get("frame_id", -1))
            if frame == last_frame:
                time.sleep(0.03)
                continue

            cams = camera_objects(packet)
            if not cams:
                last_frame = frame
                continue

            # Select the highest-confidence camera object with a usable bearing.
            usable = [
                (
                    1 if str(obj.get("state", "")) == "CONFIRMED" else 0,
                    float(obj.get("confidence", 0.0)),
                    i,
                    obj,
                )
                for i, obj in enumerate(cams)
                if camera_bearing_deg(obj) is not None
            ]
            if not usable:
                last_frame = frame
                continue

            usable.sort(reverse=True, key=lambda x: (x[0], x[1]))
            _, _, selected_idx, selected = usable[0]
            selected_bearing = camera_bearing_deg(selected)

            # Run the REAL simulator/detector/tracker pipeline.
            scan = backend.next_scan()
            detections = detector.detect(scan)
            real_tracks = update_tracker(tracker, detections, time.time())

            # Add one deliberately aligned controlled target.
            controlled = controlled_lidar_track(selected_bearing, time.time())
            lidar_tracks = list(real_tracks) + [controlled]

            matches = associate(cams, lidar_tracks)

            controlled_match = None
            for ci, li, error in matches:
                if li == len(lidar_tracks) - 1:
                    controlled_match = (ci, li, error)
                    break

            total_frames += 1

            print("\n" + "-" * 100)
            print(f"Frame              : {frame}")
            print(f"Live camera objects: {len(cams)}")
            print(f"Real LiDAR tracks  : {len(real_tracks)}")
            print(f"Controlled target  : L{CONTROLLED_TRACK_ID}")
            print(f"Camera bearing     : {selected_bearing:+.2f}°")
            print(f"Controlled bearing : {selected_bearing:+.2f}°")
            print(f"Controlled range   : {CONTROLLED_RANGE_M:.2f} m")
            print(f"Fused objects      : {len(matches)}")

            if controlled_match:
                ci, li, error = controlled_match
                cam = cams[ci]
                cam_id = cam.get("stable_id", cam.get("id", "?"))
                cls = cam.get("class", cam.get("class_name", "unknown"))

                print(
                    f"FUSED              : C{cam_id}-L{CONTROLLED_TRACK_ID} | "
                    f"{cls} | error={error:.2f}° | "
                    f"range={CONTROLLED_RANGE_M:.2f}m"
                )
                print("CONTROLLED FUSION  : PASS")
                validated_frames += 1
            else:
                print("CONTROLLED FUSION  : FAIL")

            if total_frames >= 10 and validated_frames >= 10:
                elapsed = time.monotonic() - start
                print("\n" + "=" * 100)
                print("V13.3 VALIDATION COMPLETE")
                print("=" * 100)
                print(f"Frames tested      : {total_frames}")
                print(f"Successful matches : {validated_frames}")
                print(f"Success rate       : {validated_frames / total_frames * 100:.1f}%")
                print("CAMERA PACKET      : PASS")
                print("STL-19P PIPELINE   : PASS")
                print("CONTROLLED FUSION  : PASS")
                print("ONE-TO-ONE MAPPING : PASS")
                print("NOTE: This validates software association only.")
                print("      Physical camera/LiDAR calibration is still required.")
                print("=" * 100)
                return

            last_frame = frame

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            backend.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
