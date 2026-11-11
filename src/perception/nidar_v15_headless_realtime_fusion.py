#!/usr/bin/env python3

import json
import time
import csv
import os
import signal
import sys

from lidar_interface import SimulatedSTL19PBackend

from lidar_obstacle_detector import (
    LidarObstacleDetector,
    ObstacleDetectorConfig,
)

from lidar_obstacle_tracker_v3 import (
    LidarObstacleTrackerV3,
    TrackerV3Config,
)


# ============================================================
# NIDAR V15
# HEADLESS REAL-TIME CAMERA + LiDAR FUSION
# ============================================================

PROJECT_DIR = os.path.expanduser("~/drone_project")

CAMERA_PACKET = os.path.join(
    PROJECT_DIR,
    "nidar_latest_perception.json"
)

OUTPUT_JSON = os.path.join(
    PROJECT_DIR,
    "nidar_v15_headless_fusion.json"
)

OUTPUT_CSV = os.path.join(
    PROJECT_DIR,
    "nidar_v15_headless_fusion.csv"
)


# ============================================================
# SETTINGS
# ============================================================

FUSION_GATE_DEG = 10.0

CAMERA_WIDTH = 1280.0
CAMERA_HFOV_DEG = 70.0

LOOP_DELAY = 0.05

FLIGHT_CONTROL = False

PRINT_EVERY_N_FRAMES = 10


# ============================================================
# SHUTDOWN
# ============================================================

running = True


def handle_shutdown(signum, frame):
    global running

    print()
    print("Shutdown requested...")
    running = False


signal.signal(signal.SIGINT, handle_shutdown)
signal.signal(signal.SIGTERM, handle_shutdown)


# ============================================================
# CAMERA PACKET
# ============================================================

def load_camera_packet():

    try:

        with open(CAMERA_PACKET, "r") as f:
            return json.load(f)

    except FileNotFoundError:

        return None

    except json.JSONDecodeError:

        return None

    except Exception:

        return None


# ============================================================
# CAMERA BEARING
# ============================================================

def camera_bearing_from_object(obj):

    center = obj.get("center")

    if not center:
        return None

    try:

        center_x = float(center[0])

    except (
        ValueError,
        TypeError,
        IndexError,
    ):

        return None

    normalized = (
        center_x - CAMERA_WIDTH / 2.0
    ) / (
        CAMERA_WIDTH / 2.0
    )

    bearing = (
        normalized *
        CAMERA_HFOV_DEG /
        2.0
    )

    return bearing


# ============================================================
# NORMALIZE LIDAR TRACK
# ============================================================

def normalize_track(track):

    if isinstance(track, dict):
        result = track.copy()

    else:

        result = {}

        fields = [
            "track_id",
            "id",
            "distance_m",
            "bearing_deg",
            "speed_mps",
            "x_m",
            "y_m",
            "state",
            "confidence",
            "age",
            "missed_frames",
        ]

        for field in fields:

            if hasattr(track, field):

                try:
                    result[field] = getattr(
                        track,
                        field
                    )

                except Exception:
                    pass

    # --------------------------------------------------------
    # ID
    # --------------------------------------------------------

    if (
        "track_id" not in result
        and "id" in result
    ):

        result["track_id"] = result["id"]

    # --------------------------------------------------------
    # Distance
    # --------------------------------------------------------

    if "distance_m" not in result:

        for field in [
            "distance",
            "range_m",
            "range",
        ]:

            if hasattr(track, field):

                try:

                    result["distance_m"] = float(
                        getattr(track, field)
                    )

                    break

                except Exception:
                    pass

    # --------------------------------------------------------
    # Bearing
    # --------------------------------------------------------

    if "bearing_deg" not in result:

        for field in [
            "bearing",
            "angle_deg",
            "angle",
        ]:

            if hasattr(track, field):

                try:

                    result["bearing_deg"] = float(
                        getattr(track, field)
                    )

                    break

                except Exception:
                    pass

    # --------------------------------------------------------
    # Speed
    # --------------------------------------------------------

    if "speed_mps" not in result:

        for field in [
            "speed",
            "velocity_mps",
            "velocity",
        ]:

            if hasattr(track, field):

                try:

                    result["speed_mps"] = float(
                        getattr(track, field)
                    )

                    break

                except Exception:
                    pass

    return result


# ============================================================
# ANGLE DIFFERENCE
# ============================================================

def angular_difference(a, b):

    diff = abs(a - b)

    while diff > 180.0:
        diff -= 360.0

    return abs(diff)


# ============================================================
# CAMERA + LIDAR FUSION
# ============================================================

def fuse_objects(
    camera_objects,
    lidar_tracks
):

    fused = []

    used_lidar = set()

    camera_measurements = []

    # --------------------------------------------------------
    # Camera bearings
    # --------------------------------------------------------

    for obj in camera_objects:

        bearing = camera_bearing_from_object(obj)

        if bearing is None:
            continue

        camera_measurements.append(
            (
                obj,
                bearing
            )
        )

    # --------------------------------------------------------
    # One-to-one association
    # --------------------------------------------------------

    for camera_object, camera_bearing in camera_measurements:

        best_track = None
        best_error = float("inf")

        for lidar_track in lidar_tracks:

            lidar_id = lidar_track.get(
                "track_id"
            )

            if lidar_id is None:
                continue

            if lidar_id in used_lidar:
                continue

            lidar_bearing = lidar_track.get(
                "bearing_deg"
            )

            if lidar_bearing is None:
                continue

            try:

                lidar_bearing = float(
                    lidar_bearing
                )

            except (
                ValueError,
                TypeError,
            ):

                continue

            error = angular_difference(
                camera_bearing,
                lidar_bearing
            )

            if (
                error <= FUSION_GATE_DEG
                and error < best_error
            ):

                best_track = lidar_track
                best_error = error

        # ----------------------------------------------------
        # Fused object
        # ----------------------------------------------------

        if best_track is None:
            continue

        lidar_id = best_track.get(
            "track_id"
        )

        used_lidar.add(lidar_id)

        try:

            distance = float(
                best_track.get(
                    "distance_m",
                    0.0
                )
            )

        except Exception:

            distance = 0.0

        try:

            speed = float(
                best_track.get(
                    "speed_mps",
                    0.0
                )
            )

        except Exception:

            speed = 0.0

        try:

            lidar_bearing = float(
                best_track.get(
                    "bearing_deg",
                    0.0
                )
            )

        except Exception:

            lidar_bearing = 0.0

        try:

            confidence = float(
                camera_object.get(
                    "confidence",
                    0.0
                )
            )

        except Exception:

            confidence = 0.0

        fused.append({

            "camera_id":
                camera_object.get("id"),

            "lidar_id":
                lidar_id,

            "class":
                camera_object.get(
                    "class",
                    "unknown"
                ),

            "camera_bearing_deg":
                round(
                    camera_bearing,
                    2
                ),

            "lidar_bearing_deg":
                round(
                    lidar_bearing,
                    2
                ),

            "bearing_error_deg":
                round(
                    best_error,
                    2
                ),

            "range_m":
                round(
                    distance,
                    2
                ),

            "speed_mps":
                round(
                    speed,
                    2
                ),

            "confidence":
                round(
                    confidence,
                    3
                ),

            "fusion_status":
                "FUSED",

        })

    return fused


# ============================================================
# SAVE JSON
# ============================================================

def save_json(
    frame,
    camera_objects,
    lidar_tracks,
    fused,
):

    data = {

        "protocol":
            "NIDAR_V15_HEADLESS_REALTIME_FUSION",

        "timestamp":
            time.time(),

        "frame":
            frame,

        "camera_source":
            "mac_webcam",

        "lidar_source":
            "STL-19P_SIMULATOR",

        "flight_control":
            "DISABLED",

        "camera_objects":
            camera_objects,

        "lidar_tracks":
            lidar_tracks,

        "fused_objects":
            fused,

    }

    temp_file = OUTPUT_JSON + ".tmp"

    with open(
        temp_file,
        "w"
    ) as f:

        json.dump(
            data,
            f,
            indent=2
        )

    os.replace(
        temp_file,
        OUTPUT_JSON
    )


# ============================================================
# CSV LOGGER
# ============================================================

def save_csv(
    frame,
    fused,
):

    file_exists = os.path.exists(
        OUTPUT_CSV
    )

    with open(
        OUTPUT_CSV,
        "a",
        newline=""
    ) as f:

        writer = csv.writer(f)

        if not file_exists:

            writer.writerow([
                "timestamp",
                "frame",
                "camera_id",
                "lidar_id",
                "class",
                "camera_bearing_deg",
                "lidar_bearing_deg",
                "bearing_error_deg",
                "range_m",
                "speed_mps",
                "confidence",
                "fusion_status",
            ])

        timestamp = time.time()

        for obj in fused:

            writer.writerow([

                timestamp,

                frame,

                obj.get(
                    "camera_id"
                ),

                obj.get(
                    "lidar_id"
                ),

                obj.get(
                    "class"
                ),

                obj.get(
                    "camera_bearing_deg"
                ),

                obj.get(
                    "lidar_bearing_deg"
                ),

                obj.get(
                    "bearing_error_deg"
                ),

                obj.get(
                    "range_m"
                ),

                obj.get(
                    "speed_mps"
                ),

                obj.get(
                    "confidence"
                ),

                obj.get(
                    "fusion_status"
                ),

            ])


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 100)
    print(
        "NIDAR V15 - HEADLESS REAL-TIME CAMERA + LiDAR FUSION"
    )
    print("=" * 100)

    print(
        f"Camera packet : {CAMERA_PACKET}"
    )

    print(
        "Camera source : MAC WEBCAM / NIDAR V11"
    )

    print(
        "LiDAR source  : STL-19P SIMULATOR"
    )

    print(
        f"Fusion gate   : ±{FUSION_GATE_DEG:.1f}°"
    )

    print(
        "Flight control: DISABLED"
    )

    print(
        "Display       : DISABLED"
    )

    print("=" * 100)

    # --------------------------------------------------------
    # LiDAR setup
    # --------------------------------------------------------

    try:

        backend = SimulatedSTL19PBackend()

        detector = LidarObstacleDetector(
            ObstacleDetectorConfig()
        )

        tracker = LidarObstacleTrackerV3(
            TrackerV3Config()
        )

    except Exception as e:

        print()
        print("LiDAR initialization FAILED")
        print(
            f"ERROR: {e}"
        )

        return 1

    print(
        "STL-19P simulator : READY"
    )

    print(
        "LiDAR detector     : READY"
    )

    print(
        "LiDAR tracker V3   : READY"
    )

    print(
        "Headless mode      : ENABLED"
    )

    print("=" * 100)

    # --------------------------------------------------------
    # Runtime statistics
    # --------------------------------------------------------

    last_camera_frame = None

    processed_frames = 0

    total_fused = 0

    start_time = time.time()

    last_status_time = start_time

    # --------------------------------------------------------
    # Main loop
    # --------------------------------------------------------

    while running:

        loop_start = time.time()

        # ====================================================
        # CAMERA
        # ====================================================

        packet = load_camera_packet()

        if packet is None:

            time.sleep(
                LOOP_DELAY
            )

            continue

        frame = packet.get(
            "frame"
        )

        camera_objects = packet.get(
            "objects",
            []
        )

        if frame is None:

            time.sleep(
                LOOP_DELAY
            )

            continue

        # ----------------------------------------------------
        # Avoid processing the same camera frame repeatedly
        # ----------------------------------------------------

        if frame == last_camera_frame:

            time.sleep(
                LOOP_DELAY
            )

            continue

        last_camera_frame = frame

        processed_frames += 1

        # ====================================================
        # LIDAR
        # ====================================================

        try:

            scan = backend.next_scan()

            lidar_detections = detector.detect(
                scan
            )

            timestamp = time.time()

            # IMPORTANT:
            # Tracker V3 requires detections + timestamp.

            lidar_tracks_raw = tracker.update(
                lidar_detections,
                timestamp
            )

        except Exception as e:

            print()
            print(
                "[LiDAR ERROR]",
                e
            )

            time.sleep(
                LOOP_DELAY
            )

            continue

        # ====================================================
        # NORMALIZE TRACKS
        # ====================================================

        lidar_tracks = []

        for track in lidar_tracks_raw:

            normalized = normalize_track(
                track
            )

            if normalized:

                lidar_tracks.append(
                    normalized
                )

        # ====================================================
        # FUSION
        # ====================================================

        fused = fuse_objects(
            camera_objects,
            lidar_tracks
        )

        total_fused += len(fused)

        # ====================================================
        # SAVE OUTPUT
        # ====================================================

        try:

            save_json(
                frame,
                camera_objects,
                lidar_tracks,
                fused
            )

            save_csv(
                frame,
                fused
            )

        except Exception as e:

            print(
                "[LOGGING ERROR]",
                e
            )

        # ====================================================
        # TERMINAL STATUS
        # ====================================================

        now = time.time()

        if (
            processed_frames % PRINT_EVERY_N_FRAMES == 0
        ):

            elapsed = max(
                now - start_time,
                0.001
            )

            fps = (
                processed_frames /
                elapsed
            )

            print(
                "-" * 100
            )

            print(
                f"Frame              : {frame}"
            )

            print(
                f"Camera objects     : {len(camera_objects)}"
            )

            print(
                f"LiDAR detections   : {len(lidar_detections)}"
            )

            print(
                f"LiDAR tracks       : {len(lidar_tracks)}"
            )

            print(
                f"Fused objects      : {len(fused)}"
            )

            print(
                f"Processing FPS     : {fps:.2f}"
            )

            if fused:

                for obj in fused:

                    print(
                        "FUSED              : "
                        f"C{obj['camera_id']}-"
                        f"L{obj['lidar_id']} | "
                        f"{obj['class']} | "
                        f"bearing="
                        f"{obj['camera_bearing_deg']:+.2f}° | "
                        f"range="
                        f"{obj['range_m']:.2f}m | "
                        f"error="
                        f"{obj['bearing_error_deg']:.2f}°"
                    )

            else:

                print(
                    "FUSED              : none"
                )

        # ====================================================
        # LOOP TIMING
        # ====================================================

        processing_time = (
            time.time() -
            loop_start
        )

        remaining = (
            LOOP_DELAY -
            processing_time
        )

        if remaining > 0:

            time.sleep(
                remaining
            )

    # ========================================================
    # SHUTDOWN
    # ========================================================

    elapsed = max(
        time.time() - start_time,
        0.001
    )

    fps = (
        processed_frames /
        elapsed
    )

    print()
    print("=" * 100)
    print(
        "NIDAR V15 SHUTDOWN"
    )
    print("=" * 100)

    print(
        f"Frames processed : {processed_frames}"
    )

    print(
        f"Total fused      : {total_fused}"
    )

    print(
        f"Average FPS      : {fps:.2f}"
    )

    print(
        f"JSON output      : {OUTPUT_JSON}"
    )

    print(
        f"CSV output       : {OUTPUT_CSV}"
    )

    print(
        "Flight control   : DISABLED"
    )

    print("=" * 100)

    return 0


if __name__ == "__main__":

    sys.exit(
        main()
    )
