#!/usr/bin/env python3

import csv
import json
import math
import os
import time
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
# NIDAR V15 - HEADLESS SENSOR FUSION
# ============================================================
#
# Camera:
#   Mac webcam -> NIDAR V11 -> Raspberry Pi JSON
#
# LiDAR:
#   STL-19P simulator
#
# Processing:
#   Camera perception
#   LiDAR detection
#   LiDAR tracking V3
#   Camera/LiDAR association
#   Fused object state
#
# Display:
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
    "nidar_v15_fused_state.json"
)

OUTPUT_CSV = os.path.join(
    PROJECT_DIR,
    "nidar_v15_fused_objects.csv"
)


# ============================================================
# CAMERA CONFIGURATION
# ============================================================

CAMERA_WIDTH = 1280.0
CAMERA_HEIGHT = 720.0

# Software camera horizontal FOV.
#
# This is NOT physical calibration.
# It will be calibrated when the real camera/LiDAR
# system is assembled.
CAMERA_HFOV_DEG = 70.0


# ============================================================
# FUSION CONFIGURATION
# ============================================================

FUSION_GATE_DEG = 10.0

LOOP_DELAY = 0.03

# Maximum age allowed for camera packet.
# This is only used for health reporting.
MAX_CAMERA_AGE_SEC = 2.0


# ============================================================
# UTILITY
# ============================================================

def safe_float(
    value,
    default=0.0
):

    try:
        return float(value)

    except (
        TypeError,
        ValueError,
    ):

        return default


# ============================================================
# ANGLE NORMALIZATION
# ============================================================

def normalize_angle_deg(
    angle
):

    """
    Convert angle to [-180, +180).
    """

    angle = safe_float(
        angle
    )

    return (
        (angle + 180.0) % 360.0
    ) - 180.0


# ============================================================
# ANGULAR ERROR
# ============================================================

def angular_error_deg(
    a,
    b
):

    """
    Smallest angular difference between
    two bearings.
    """

    a = normalize_angle_deg(
        a
    )

    b = normalize_angle_deg(
        b
    )

    return abs(
        normalize_angle_deg(
            a - b
        )
    )


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

    except (
        FileNotFoundError,
        json.JSONDecodeError,
    ):

        return None


# ============================================================
# CAMERA BEARING
# ============================================================

def camera_bearing_deg(
    obj
):

    center = obj.get(
        "center"
    )

    if not center:

        return None

    if len(center) < 1:

        return None

    center_x = safe_float(
        center[0],
        None
    )

    if center_x is None:

        return None

    normalized_x = (
        center_x -
        CAMERA_WIDTH / 2.0
    ) / (
        CAMERA_WIDTH / 2.0
    )

    bearing = (
        normalized_x *
        CAMERA_HFOV_DEG /
        2.0
    )

    return normalize_angle_deg(
        bearing
    )


# ============================================================
# LIDAR PIPELINE
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
        tracker,
    )


# ============================================================
# TRACK NORMALIZATION
# ============================================================

def normalize_track(
    track
):

    """
    Convert tracker output into the common
    V15 representation.
    """

    if isinstance(
        track,
        dict
    ):

        raw = track.copy()

    else:

        raw = {}

        fields = [

            "track_id",
            "id",

            "distance_m",
            "distance",
            "range_m",
            "range",

            "bearing_deg",
            "bearing",
            "angle_deg",
            "angle",

            "speed_mps",
            "speed",
            "velocity_mps",
            "velocity",

            "x_m",
            "y_m",

            "state",
            "confidence",

            "age",
            "missed_frames",

        ]

        for field in fields:

            if hasattr(
                track,
                field
            ):

                try:

                    raw[field] = getattr(
                        track,
                        field
                    )

                except Exception:

                    pass

    # --------------------------------------------------------
    # Track ID
    # --------------------------------------------------------

    track_id = raw.get(
        "track_id"
    )

    if track_id is None:

        track_id = raw.get(
            "id"
        )

    # --------------------------------------------------------
    # Distance
    # --------------------------------------------------------

    distance = raw.get(
        "distance_m"
    )

    if distance is None:

        distance = raw.get(
            "distance"
        )

    if distance is None:

        distance = raw.get(
            "range_m"
        )

    if distance is None:

        distance = raw.get(
            "range"
        )

    distance = safe_float(
        distance
    )

    # --------------------------------------------------------
    # Bearing
    # --------------------------------------------------------

    bearing = raw.get(
        "bearing_deg"
    )

    if bearing is None:

        bearing = raw.get(
            "bearing"
        )

    if bearing is None:

        bearing = raw.get(
            "angle_deg"
        )

    if bearing is None:

        bearing = raw.get(
            "angle"
        )

    bearing = normalize_angle_deg(
        bearing
    )

    # --------------------------------------------------------
    # Speed
    # --------------------------------------------------------

    speed = raw.get(
        "speed_mps"
    )

    if speed is None:

        speed = raw.get(
            "speed"
        )

    if speed is None:

        speed = raw.get(
            "velocity_mps"
        )

    if speed is None:

        speed = raw.get(
            "velocity"
        )

    speed = safe_float(
        speed
    )

    # --------------------------------------------------------
    # X/Y
    # --------------------------------------------------------

    x = raw.get(
        "x_m"
    )

    y = raw.get(
        "y_m"
    )

    x = safe_float(
        x
    )

    y = safe_float(
        y
    )

    # If X/Y are not provided, calculate from
    # range and bearing.
    #
    # Bearing convention:
    #   0° = forward
    #   +90° = right
    #   -90° = left
    #
    if (
        "x_m" not in raw
        or "y_m" not in raw
    ):

        angle_rad = math.radians(
            bearing
        )

        x = (
            distance *
            math.cos(
                angle_rad
            )
        )

        y = (
            distance *
            math.sin(
                angle_rad
            )
        )

    # --------------------------------------------------------
    # State
    # --------------------------------------------------------

    state = raw.get(
        "state",
        "UNKNOWN"
    )

    # --------------------------------------------------------
    # Confidence
    # --------------------------------------------------------

    confidence = safe_float(
        raw.get(
            "confidence",
            0.0
        )
    )

    confidence = max(
        0.0,
        min(
            1.0,
            confidence
        )
    )

    return {

        "track_id":
            track_id,

        "distance_m":
            distance,

        "bearing_deg":
            bearing,

        "speed_mps":
            speed,

        "x_m":
            x,

        "y_m":
            y,

        "state":
            state,

        "confidence":
            confidence,

        "age":
            raw.get(
                "age"
            ),

        "missed_frames":
            raw.get(
                "missed_frames"
            ),

    }


# ============================================================
# FUSION CONFIDENCE
# ============================================================

def calculate_fusion_confidence(
    camera_confidence,
    lidar_confidence,
    bearing_error
):

    camera_confidence = max(
        0.0,
        min(
            1.0,
            safe_float(
                camera_confidence
            )
        )
    )

    lidar_confidence = max(
        0.0,
        min(
            1.0,
            safe_float(
                lidar_confidence
            )
        )
    )

    # Bearing agreement:
    #
    # 0° error = 1.0
    # gate error = 0.0
    #
    bearing_score = max(
        0.0,
        1.0 -
        (
            bearing_error /
            FUSION_GATE_DEG
        )
    )

    fusion_confidence = (
        0.40 *
        camera_confidence
        +
        0.30 *
        lidar_confidence
        +
        0.30 *
        bearing_score
    )

    return max(
        0.0,
        min(
            1.0,
            fusion_confidence
        )
    )


# ============================================================
# CAMERA/LIDAR FUSION
# ============================================================

def fuse_objects(
    camera_objects,
    lidar_tracks
):

    fused = []

    used_lidar_ids = set()

    # --------------------------------------------------------
    # Prepare camera measurements
    # --------------------------------------------------------

    camera_measurements = []

    for camera_object in camera_objects:

        bearing = camera_bearing_deg(
            camera_object
        )

        if bearing is None:

            continue

        camera_measurements.append({

            "object":
                camera_object,

            "bearing_deg":
                bearing,

        })

    # --------------------------------------------------------
    # Associate each camera object
    # with the closest available LiDAR track.
    # --------------------------------------------------------

    for measurement in camera_measurements:

        camera_object = measurement[
            "object"
        ]

        cam_bearing = measurement[
            "bearing_deg"
        ]

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

            if lidar_id in used_lidar_ids:

                continue

            lidar_bearing = lidar_track.get(
                "bearing_deg"
            )

            error = angular_error_deg(
                cam_bearing,
                lidar_bearing
            )

            if (
                error <= FUSION_GATE_DEG
                and error < best_error
            ):

                best_track = (
                    lidar_track
                )

                best_error = error

        # ----------------------------------------------------
        # No association
        # ----------------------------------------------------

        if best_track is None:

            continue

        lidar_id = best_track.get(
            "track_id"
        )

        used_lidar_ids.add(
            lidar_id
        )

        # ----------------------------------------------------
        # Camera information
        # ----------------------------------------------------

        camera_id = camera_object.get(
            "id"
        )

        object_class = camera_object.get(
            "class",
            "unknown"
        )

        camera_confidence = safe_float(
            camera_object.get(
                "confidence",
                0.0
            )
        )

        # ----------------------------------------------------
        # LiDAR information
        # ----------------------------------------------------

        lidar_bearing = normalize_angle_deg(
            best_track.get(
                "bearing_deg",
                0.0
            )
        )

        range_m = safe_float(
            best_track.get(
                "distance_m",
                0.0
            )
        )

        speed_mps = safe_float(
            best_track.get(
                "speed_mps",
                0.0
            )
        )

        x_m = safe_float(
            best_track.get(
                "x_m",
                0.0
            )
        )

        y_m = safe_float(
            best_track.get(
                "y_m",
                0.0
            )
        )

        lidar_confidence = safe_float(
            best_track.get(
                "confidence",
                0.0
            )
        )

        # ----------------------------------------------------
        # Fusion confidence
        # ----------------------------------------------------

        fusion_confidence = (
            calculate_fusion_confidence(
                camera_confidence,
                lidar_confidence,
                best_error
            )
        )

        # ----------------------------------------------------
        # Fused state
        # ----------------------------------------------------

        fused.append({

            "fused_id":
                f"C{camera_id}-L{lidar_id}",

            "camera_id":
                camera_id,

            "lidar_id":
                lidar_id,

            "class":
                object_class,

            "camera_bearing_deg":
                round(
                    cam_bearing,
                    3
                ),

            "lidar_bearing_deg":
                round(
                    lidar_bearing,
                    3
                ),

            "bearing_error_deg":
                round(
                    best_error,
                    3
                ),

            "range_m":
                round(
                    range_m,
                    3
                ),

            "x_m":
                round(
                    x_m,
                    3
                ),

            "y_m":
                round(
                    y_m,
                    3
                ),

            "speed_mps":
                round(
                    speed_mps,
                    3
                ),

            "camera_confidence":
                round(
                    camera_confidence,
                    3
                ),

            "lidar_confidence":
                round(
                    lidar_confidence,
                    3
                ),

            "fusion_confidence":
                round(
                    fusion_confidence,
                    3
                ),

            "lidar_state":
                best_track.get(
                    "state",
                    "UNKNOWN"
                ),

            "fusion_status":
                "FUSED",

        })

    return fused


# ============================================================
# SENSOR HEALTH
# ============================================================

def build_sensor_health(
    packet,
    scan,
    lidar_tracks,
    fused
):

    now = time.time()

    packet_timestamp = safe_float(
        packet.get(
            "timestamp",
            now
        ),
        now
    )

    camera_age = max(
        0.0,
        now -
        packet_timestamp
    )

    return {

        "camera":

            "OK"
            if camera_age <= MAX_CAMERA_AGE_SEC
            else "STALE",

        "lidar":

            "OK"
            if scan is not None
            else "ERROR",

        "lidar_detector":

            "OK",

        "lidar_tracker":

            "OK"
            if lidar_tracks is not None
            else "ERROR",

        "fusion":

            "OK"
            if len(fused) > 0
            else "NO_MATCH",

        "headless":

            "ENABLED",

        "flight_control":

            "DISABLED",

        "camera_packet_age_sec":

            round(
                camera_age,
                3
            ),

    }


# ============================================================
# SAVE V15 JSON
# ============================================================

def save_json(
    packet,
    scan,
    camera_objects,
    lidar_tracks,
    fused,
    health
):

    output = {

        "protocol":
            "NIDAR_V15_FUSED_STATE",

        "timestamp":
            time.time(),

        "frame":
            packet.get(
                "frame",
                0
            ),

        "camera_source":
            "mac_webcam",

        "lidar_source":
            "STL-19P_SIMULATOR",

        "display":
            "DISABLED",

        "flight_control":
            "DISABLED",

        "fusion_gate_deg":
            FUSION_GATE_DEG,

        "sensor_health":
            health,

        "camera_objects":
            camera_objects,

        "lidar_tracks":
            lidar_tracks,

        "fused_objects":
            fused,

    }

    if scan is not None:

        output["lidar_scan"] = {

            "scan_id":
                getattr(
                    scan,
                    "scan_id",
                    None
                ),

            "timestamp":
                safe_float(
                    getattr(
                        scan,
                        "timestamp",
                        0.0
                    )
                ),

            "scan_rate_hz":
                safe_float(
                    getattr(
                        scan,
                        "scan_rate_hz",
                        0.0
                    )
                ),

            "measurements_per_second":
                safe_float(
                    getattr(
                        scan,
                        "measurements_per_second",
                        0.0
                    )
                ),

            "valid_count":
                getattr(
                    scan,
                    "valid_count",
                    None
                ),

            "total_count":
                getattr(
                    scan,
                    "total_count",
                    None
                ),

        }

    with open(
        OUTPUT_JSON,
        "w"
    ) as f:

        json.dump(
            output,
            f,
            indent=2
        )


# ============================================================
# CSV
# ============================================================

def append_csv(
    fused,
    frame
):

    exists = os.path.exists(
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

        if not exists:

            writer.writerow([

                "timestamp",
                "frame",

                "fused_id",

                "camera_id",
                "lidar_id",

                "class",

                "camera_bearing_deg",
                "lidar_bearing_deg",
                "bearing_error_deg",

                "range_m",

                "x_m",
                "y_m",

                "speed_mps",

                "camera_confidence",
                "lidar_confidence",
                "fusion_confidence",

                "lidar_state",
                "fusion_status",

            ])

        for obj in fused:

            writer.writerow([

                datetime.now().isoformat(),

                frame,

                obj.get(
                    "fused_id"
                ),

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
                    "x_m"
                ),

                obj.get(
                    "y_m"
                ),

                obj.get(
                    "speed_mps"
                ),

                obj.get(
                    "camera_confidence"
                ),

                obj.get(
                    "lidar_confidence"
                ),

                obj.get(
                    "fusion_confidence"
                ),

                obj.get(
                    "lidar_state"
                ),

                obj.get(
                    "fusion_status"
                ),

            ])


# ============================================================
# TERMINAL OUTPUT
# ============================================================

def print_header():

    print(
        "=" * 100
    )

    print(
        "NIDAR V15 - HEADLESS CAMERA + LiDAR FUSION"
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
        "Output JSON   : nidar_v15_fused_state.json"
    )

    print(
        "Output CSV    : nidar_v15_fused_objects.csv"
    )

    print(
        "Display       : DISABLED / HEADLESS"
    )

    print(
        "Flight control: DISABLED"
    )

    print(
        "=" * 100
    )


# ============================================================
# FRAME OUTPUT
# ============================================================

def print_frame(
    frame,
    camera_objects,
    lidar_tracks,
    fused,
    health
):

    print()

    print(
        "-" * 100
    )

    print(
        f"FRAME {frame}"
    )

    print(
        f"Camera objects : {len(camera_objects)}"
    )

    print(
        f"LiDAR tracks   : {len(lidar_tracks)}"
    )

    print(
        f"Fused objects  : {len(fused)}"
    )

    print(
        "-" * 100
    )

    # --------------------------------------------------------
    # Sensor health
    # --------------------------------------------------------

    print(
        "SENSOR STATUS"
    )

    print(
        f"  Camera : {health['camera']}"
    )

    print(
        f"  LiDAR  : {health['lidar']}"
    )

    print(
        f"  Tracker: {health['lidar_tracker']}"
    )

    print(
        f"  Fusion : {health['fusion']}"
    )

    # --------------------------------------------------------
    # Camera
    # --------------------------------------------------------

    if camera_objects:

        print()

        print(
            "CAMERA OBJECTS"
        )

        for obj in camera_objects:

            object_id = obj.get(
                "id",
                "?"
            )

            object_class = obj.get(
                "class",
                "unknown"
            )

            confidence = safe_float(
                obj.get(
                    "confidence",
                    0.0
                )
            )

            bearing = camera_bearing_deg(
                obj
            )

            if bearing is None:

                bearing_text = "N/A"

            else:

                bearing_text = (
                    f"{bearing:+.2f}°"
                )

            print(

                f"  C{object_id} | "
                f"{object_class:<12} | "
                f"bearing={bearing_text} | "
                f"confidence={confidence:.2f}"

            )

    # --------------------------------------------------------
    # LiDAR
    # --------------------------------------------------------

    if lidar_tracks:

        print()

        print(
            "LiDAR TRACKS"
        )

        for track in lidar_tracks:

            track_id = track.get(
                "track_id",
                "?"
            )

            bearing = safe_float(
                track.get(
                    "bearing_deg",
                    0.0
                )
            )

            distance = safe_float(
                track.get(
                    "distance_m",
                    0.0
                )
            )

            x = safe_float(
                track.get(
                    "x_m",
                    0.0
                )
            )

            y = safe_float(
                track.get(
                    "y_m",
                    0.0
                )
            )

            confidence = safe_float(
                track.get(
                    "confidence",
                    0.0
                )
            )

            print(

                f"  L{track_id} | "
                f"bearing={bearing:+.2f}° | "
                f"range={distance:.2f}m | "
                f"X={x:+.2f}m | "
                f"Y={y:+.2f}m | "
                f"conf={confidence:.2f}"

            )

    # --------------------------------------------------------
    # Fused
    # --------------------------------------------------------

    print()

    if fused:

        print(
            "FUSED OBJECTS"
        )

        for obj in fused:

            print(

                f"  {obj['fused_id']:<10} | "
                f"{obj['class']:<12} | "
                f"bearing="
                f"{obj['camera_bearing_deg']:+.2f}°/"
                f"{obj['lidar_bearing_deg']:+.2f}° | "
                f"error="
                f"{obj['bearing_error_deg']:.2f}° | "
                f"range="
                f"{obj['range_m']:.2f}m | "
                f"XY=("
                f"{obj['x_m']:+.2f},"
                f"{obj['y_m']:+.2f})m | "
                f"speed="
                f"{obj['speed_mps']:.2f}m/s | "
                f"fusion="
                f"{obj['fusion_confidence']:.2f}"

            )

    else:

        print(
            "FUSED OBJECTS: none in current frame"
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
    # LiDAR pipeline
    # --------------------------------------------------------

    try:

        (
            backend,
            detector,
            tracker,
        ) = create_lidar_pipeline()

    except Exception as e:

        print()
        print(
            "FAILED TO INITIALIZE LiDAR PIPELINE"
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
        "V15 fusion engine  : READY"
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
            # Camera packet
            # ------------------------------------------------

            packet = load_camera_packet()

            if packet is None:

                print(
                    "\rWaiting for NIDAR camera packet...",
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
            # New frame only
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
            # LiDAR scan
            # ------------------------------------------------

            try:

                scan = backend.next_scan()

            except Exception as e:

                print()

                print(
                    "LiDAR scan ERROR:",
                    repr(e)
                )

                time.sleep(
                    LOOP_DELAY
                )

                continue

            # ------------------------------------------------
            # Detection
            # ------------------------------------------------

            try:

                lidar_detections = detector.detect(
                    scan
                )

            except Exception as e:

                print()

                print(
                    "LiDAR detection ERROR:",
                    repr(e)
                )

                time.sleep(
                    LOOP_DELAY
                )

                continue

            # ------------------------------------------------
            # Tracking
            #
            # IMPORTANT:
            # Tracker V3 requires timestamp.
            # ------------------------------------------------

            try:

                lidar_tracks_raw = tracker.update(
                    lidar_detections,
                    scan.timestamp
                )

            except Exception as e:

                print()

                print(
                    "LiDAR tracking ERROR:",
                    repr(e)
                )

                time.sleep(
                    LOOP_DELAY
                )

                continue

            # ------------------------------------------------
            # Normalize tracks
            # ------------------------------------------------

            lidar_tracks = []

            for track in lidar_tracks_raw:

                lidar_tracks.append(
                    normalize_track(
                        track
                    )
                )

            # ------------------------------------------------
            # Fusion
            # ------------------------------------------------

            fused = fuse_objects(
                camera_objects,
                lidar_tracks
            )

            # ------------------------------------------------
            # Health
            # ------------------------------------------------

            health = build_sensor_health(
                packet,
                scan,
                lidar_tracks,
                fused
            )

            # ------------------------------------------------
            # Save
            # ------------------------------------------------

            try:

                save_json(
                    packet,
                    scan,
                    camera_objects,
                    lidar_tracks,
                    fused,
                    health
                )

                append_csv(
                    fused,
                    frame
                )

            except Exception as e:

                print()

                print(
                    "OUTPUT ERROR:",
                    repr(e)
                )

            # ------------------------------------------------
            # Display terminal
            # ------------------------------------------------

            print_frame(
                frame,
                camera_objects,
                lidar_tracks,
                fused,
                health
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
            "NIDAR V15 STOPPED"
        )

        print(
            "Flight control remained DISABLED."
        )

        print(
            "=" * 100
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()
