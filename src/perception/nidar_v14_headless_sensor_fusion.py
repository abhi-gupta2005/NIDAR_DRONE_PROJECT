#!/usr/bin/env python3

import json
import time
import csv
import os
from datetime import datetime

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
# NIDAR V14 HEADLESS SENSOR FUSION
# ============================================================
#
# Camera:
#   Mac webcam -> NIDAR V11 -> TCP -> Raspberry Pi
#
# LiDAR:
#   STL-19P simulator
#
# Processing:
#   LiDAR detection
#   LiDAR tracking V3
#   Camera/LiDAR bearing association
#
# Output:
#   Terminal
#   JSON
#   CSV
#
# GUI:
#   DISABLED
#
# Flight control:
#   DISABLED
#
# ============================================================


# ============================================================
# PATHS
# ============================================================

PROJECT_DIR = os.path.expanduser(
    "~/drone_project"
)

CAMERA_PACKET = os.path.join(
    PROJECT_DIR,
    "nidar_latest_perception.json"
)

OUTPUT_JSON = os.path.join(
    PROJECT_DIR,
    "nidar_v14_headless_fusion.json"
)

OUTPUT_CSV = os.path.join(
    PROJECT_DIR,
    "nidar_v14_headless_fusion.csv"
)


# ============================================================
# FUSION SETTINGS
# ============================================================

FUSION_GATE_DEG = 10.0

LOOP_DELAY = 0.05

CAMERA_WIDTH = 1280.0

# Approximate horizontal camera field of view.
#
# This is only a software bearing model.
# Physical camera/LiDAR calibration will be done later.
CAMERA_HFOV_DEG = 70.0


# ============================================================
# CAMERA PACKET
# ============================================================

def load_camera_packet():

    try:

        with open(
            CAMERA_PACKET,
            "r"
        ) as f:

            return json.load(f)

    except FileNotFoundError:

        return None

    except json.JSONDecodeError:

        return None


# ============================================================
# CAMERA BEARING
# ============================================================

def camera_bearing_from_object(obj):

    center = obj.get(
        "center"
    )

    if not center:

        return None

    try:

        center_x = float(
            center[0]
        )

    except (
        ValueError,
        TypeError,
        IndexError
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
# LIDAR SETUP
# ============================================================

def create_lidar_pipeline():

    backend = SimulatedSTL19PBackend()

    detector = LidarObstacleDetector(
        ObstacleDetectorConfig()
    )

    tracker = LidarObstacleTrackerV3(
        TrackerV3Config()
    )

    return (
        backend,
        detector,
        tracker
    )


# ============================================================
# NORMALIZE LIDAR TRACK
# ============================================================

def normalize_track(track):

    # Some versions may return dictionaries.

    if isinstance(
        track,
        dict
    ):

        return track.copy()

    result = {}

    possible_fields = [

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

    for field in possible_fields:

        if hasattr(
            track,
            field
        ):

            try:

                result[field] = getattr(
                    track,
                    field
                )

            except Exception:

                pass

    # --------------------------------------------------------
    # ID fallback
    # --------------------------------------------------------

    if (
        "track_id" not in result
        and "id" in result
    ):

        result["track_id"] = result["id"]

    # --------------------------------------------------------
    # Distance fallbacks
    # --------------------------------------------------------

    if "distance_m" not in result:

        for field in [
            "distance",
            "range_m",
            "range",
        ]:

            if hasattr(
                track,
                field
            ):

                try:

                    result["distance_m"] = (
                        float(
                            getattr(
                                track,
                                field
                            )
                        )
                    )

                    break

                except Exception:

                    pass

    # --------------------------------------------------------
    # Bearing fallback
    # --------------------------------------------------------

    if "bearing_deg" not in result:

        for field in [
            "bearing",
            "angle_deg",
            "angle",
        ]:

            if hasattr(
                track,
                field
            ):

                try:

                    result["bearing_deg"] = (
                        float(
                            getattr(
                                track,
                                field
                            )
                        )
                    )

                    break

                except Exception:

                    pass

    # --------------------------------------------------------
    # Speed fallback
    # --------------------------------------------------------

    if "speed_mps" not in result:

        for field in [
            "speed",
            "velocity_mps",
            "velocity",
        ]:

            if hasattr(
                track,
                field
            ):

                try:

                    result["speed_mps"] = (
                        float(
                            getattr(
                                track,
                                field
                            )
                        )
                    )

                    break

                except Exception:

                    pass

    return result


# ============================================================
# CAMERA + LIDAR FUSION
# ============================================================

def fuse_objects(
    camera_objects,
    lidar_tracks
):

    fused = []

    used_lidar = set()

    # --------------------------------------------------------
    # Calculate camera bearings
    # --------------------------------------------------------

    camera_measurements = []

    for camera_object in camera_objects:

        bearing = camera_bearing_from_object(
            camera_object
        )

        if bearing is None:

            continue

        camera_measurements.append(
            (
                camera_object,
                bearing
            )
        )

    # --------------------------------------------------------
    # One-to-one association
    # --------------------------------------------------------

    for camera_object, camera_bearing in camera_measurements:

        best_track = None

        best_error = float(
            "inf"
        )

        for lidar_track in lidar_tracks:

            lidar_id = lidar_track.get(
                "track_id"
            )

            if lidar_id is None:

                continue

            # Prevent one LiDAR track from
            # being assigned to multiple cameras.

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
                TypeError
            ):

                continue

            error = abs(
                camera_bearing -
                lidar_bearing
            )

            if (
                error <= FUSION_GATE_DEG
                and error < best_error
            ):

                best_track = lidar_track

                best_error = error

        # ----------------------------------------------------
        # Fusion result
        # ----------------------------------------------------

        if best_track is None:

            continue

        lidar_id = best_track.get(
            "track_id"
        )

        used_lidar.add(
            lidar_id
        )

        distance = best_track.get(
            "distance_m",
            0.0
        )

        speed = best_track.get(
            "speed_mps",
            0.0
        )

        try:

            distance = float(
                distance
            )

        except Exception:

            distance = 0.0

        try:

            speed = float(
                speed
            )

        except Exception:

            speed = 0.0

        confidence = camera_object.get(
            "confidence",
            0.0
        )

        try:

            confidence = float(
                confidence
            )

        except Exception:

            confidence = 0.0

        fused.append({

            "camera_id":
                camera_object.get(
                    "id"
                ),

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
                    float(
                        best_track.get(
                            "bearing_deg",
                            0.0
                        )
                    ),
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
    fused
):

    data = {

        "protocol":
            "NIDAR_V14_HEADLESS_FUSION",

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

    with open(
        OUTPUT_JSON,
        "w"
    ) as f:

        json.dump(
            data,
            f,
            indent=2
        )


# ============================================================
# SAVE CSV
# ============================================================

def append_csv(
    fused
):

    file_exists = os.path.exists(
        OUTPUT_CSV
    )

    with open(
        OUTPUT_CSV,
        "a",
        newline=""
    ) as f:

        writer = csv.writer(
            f
        )

        if not file_exists:

            writer.writerow([

                "timestamp",

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

        for obj in fused:

            writer.writerow([

                datetime.now().isoformat(),

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
# TERMINAL HEADER
# ============================================================

def print_header():

    print(
        "=" * 100
    )

    print(
        "NIDAR V14 - HEADLESS CAMERA + LiDAR SENSOR FUSION"
    )

    print(
        "=" * 100
    )

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
        "Display       : DISABLED / HEADLESS"
    )

    print(
        "=" * 100
    )


# ============================================================
# PRINT FRAME
# ============================================================

def print_frame(
    frame,
    camera_objects,
    lidar_tracks,
    fused
):

    print()

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
        f"LiDAR tracks       : {len(lidar_tracks)}"
    )

    print(
        f"Fused objects      : {len(fused)}"
    )

    print(
        "-" * 100
    )

    # --------------------------------------------------------
    # Camera objects
    # --------------------------------------------------------

    if camera_objects:

        print(
            "CAMERA OBJECTS"
        )

        for obj in camera_objects:

            camera_id = obj.get(
                "id",
                "?"
            )

            object_class = obj.get(
                "class",
                "unknown"
            )

            confidence = obj.get(
                "confidence",
                0.0
            )

            bearing = camera_bearing_from_object(
                obj
            )

            if bearing is None:

                bearing_text = "N/A"

            else:

                bearing_text = (
                    f"{bearing:+.2f}°"
                )

            print(

                f"  C{camera_id:<4} | "
                f"{object_class:<12} | "
                f"bearing={bearing_text} | "
                f"conf={float(confidence):.2f}"

            )

    else:

        print(
            "CAMERA OBJECTS: none"
        )

    print()

    # --------------------------------------------------------
    # LiDAR tracks
    # --------------------------------------------------------

    if lidar_tracks:

        print(
            "LiDAR TRACKS"
        )

        for track in lidar_tracks:

            track_id = track.get(
                "track_id",
                "?"
            )

            bearing = track.get(
                "bearing_deg",
                0.0
            )

            distance = track.get(
                "distance_m",
                0.0
            )

            speed = track.get(
                "speed_mps",
                0.0
            )

            try:
                bearing = float(
                    bearing
                )
            except Exception:
                bearing = 0.0

            try:
                distance = float(
                    distance
                )
            except Exception:
                distance = 0.0

            try:
                speed = float(
                    speed
                )
            except Exception:
                speed = 0.0

            print(

                f"  L{track_id:<4} | "
                f"bearing={bearing:+.2f}° | "
                f"range={distance:.2f}m | "
                f"speed={speed:.2f}m/s"

            )

    else:

        print(
            "LiDAR TRACKS: none"
        )

    print()

    # --------------------------------------------------------
    # Fusion
    # --------------------------------------------------------

    if fused:

        print(
            "FUSED OBJECTS"
        )

        for obj in fused:

            print(

                f"  C{obj['camera_id']}"
                f"-L{obj['lidar_id']} | "
                f"{obj['class']:<12} | "
                f"bearing="
                f"{obj['camera_bearing_deg']:+.2f}° | "
                f"range="
                f"{obj['range_m']:.2f}m | "
                f"speed="
                f"{obj['speed_mps']:.2f}m/s | "
                f"error="
                f"{obj['bearing_error_deg']:.2f}° | "
                f"conf="
                f"{obj['confidence']:.2f}"

            )

    else:

        print(
            "No camera/LiDAR matches inside "
            "the fusion gate."
        )

    print(
        "-" * 100
    )

    print(
        "CAMERA PACKET      : PASS"
    )

    print(
        "STL-19P SIMULATOR  : PASS"
    )

    print(
        "LIDAR DETECTION    : PASS"
    )

    print(
        "LIDAR TRACKING     : PASS"
    )

    if fused:

        print(
            "CAMERA/LIDAR FUSION: PASS"
        )

    else:

        print(
            "CAMERA/LIDAR FUSION: WAITING"
        )

    print(
        "-" * 100
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print_header()

    # --------------------------------------------------------
    # Create LiDAR pipeline
    # --------------------------------------------------------

    try:

        (
            backend,
            detector,
            tracker
        ) = create_lidar_pipeline()

    except Exception as e:

        print()
        print(
            "ERROR INITIALIZING LiDAR PIPELINE"
        )

        print(
            repr(e)
        )

        return

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

    print(
        "=" * 100
    )

    last_frame = None

    try:

        while True:

            # ------------------------------------------------
            # Read latest camera packet
            # ------------------------------------------------

            packet = load_camera_packet()

            if packet is None:

                print(
                    "\rWaiting for camera packet...",
                    end="",
                    flush=True
                )

                time.sleep(
                    0.5
                )

                continue

            frame = packet.get(
                "frame",
                0
            )

            # ------------------------------------------------
            # Only process a new camera frame
            # ------------------------------------------------

            if frame == last_frame:

                time.sleep(
                    LOOP_DELAY
                )

                continue

            last_frame = frame

            camera_objects = packet.get(
                "objects",
                []
            )

            # ------------------------------------------------
            # Get LiDAR scan
            # ------------------------------------------------

            try:

                scan = backend.next_scan()

            except Exception as e:

                print()
                print(
                    "LiDAR scan error:",
                    repr(e)
                )

                time.sleep(
                    LOOP_DELAY
                )

                continue

            # ------------------------------------------------
            # LiDAR detection
            #
            # IMPORTANT:
            # detector expects LidarScan,
            # NOT scan.to_points().
            # ------------------------------------------------

            try:

                lidar_detections = detector.detect(
                    scan
                )

            except Exception as e:

                print()
                print(
                    "LiDAR detection error:",
                    repr(e)
                )

                time.sleep(
                    LOOP_DELAY
                )

                continue

            # ------------------------------------------------
            # LiDAR tracking
            #
            # Tracker V3 requires:
            #
            # tracker.update(
            #     detections,
            #     timestamp
            # )
            #
            # ------------------------------------------------

            try:

                lidar_tracks = tracker.update(
                    lidar_detections,
                    scan.timestamp
                )

            except Exception as e:

                print()
                print(
                    "LiDAR tracking error:",
                    repr(e)
                )

                time.sleep(
                    LOOP_DELAY
                )

                continue

            # ------------------------------------------------
            # Normalize tracks
            # ------------------------------------------------

            normalized_tracks = []

            for track in lidar_tracks:

                normalized_tracks.append(
                    normalize_track(
                        track
                    )
                )

            # ------------------------------------------------
            # Camera + LiDAR fusion
            # ------------------------------------------------

            fused = fuse_objects(
                camera_objects,
                normalized_tracks
            )

            # ------------------------------------------------
            # Save outputs
            # ------------------------------------------------

            try:

                save_json(
                    frame,
                    camera_objects,
                    normalized_tracks,
                    fused
                )

                append_csv(
                    fused
                )

            except Exception as e:

                print()
                print(
                    "Output save error:",
                    repr(e)
                )

            # ------------------------------------------------
            # Terminal
            # ------------------------------------------------

            print_frame(
                frame,
                camera_objects,
                normalized_tracks,
                fused
            )

            time.sleep(
                LOOP_DELAY
            )

    except KeyboardInterrupt:

        print()

        print(
            "=" * 100
        )

        print(
            "NIDAR V14 HEADLESS FUSION STOPPED"
        )

        print(
            "=" * 100
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()
