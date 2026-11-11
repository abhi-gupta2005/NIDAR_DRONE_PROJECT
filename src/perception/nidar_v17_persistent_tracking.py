#!/usr/bin/env python3

import csv
import json
import math
import os
import signal
import sys
import time

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
# NIDAR V17
# PERSISTENT CAMERA + LiDAR FUSION
#
# Camera:
#   Mac webcam -> NIDAR V11 -> JSON packet
#
# LiDAR:
#   STL-19P simulator
#
# Fusion:
#   Camera bearing + LiDAR bearing
#
# Tracking:
#   Stable object ID
#   ACTIVE
#   TEMP_LOST
#   PREDICTED
#   EXPIRED
#
# Display:
#   DISABLED
#
# Flight control:
#   DISABLED
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
    "nidar_v17_persistent_fusion.json"
)

OUTPUT_CSV = os.path.join(
    PROJECT_DIR,
    "nidar_v17_persistent_fusion.csv"
)


# ============================================================
# CONFIGURATION
# ============================================================

FUSION_GATE_DEG = 10.0

CAMERA_WIDTH = 1280.0

CAMERA_HFOV_DEG = 70.0

LOOP_DELAY = 0.05

PRINT_EVERY_N_FRAMES = 10

# Time during which an object remains
# temporarily lost.
TEMP_LOST_TIMEOUT_SEC = 1.5

# Time after which a lost object is expired.
EXPIRE_TIMEOUT_SEC = 3.0

# Position smoothing.
POSITION_ALPHA = 0.35

# Velocity smoothing.
VELOCITY_ALPHA = 0.30

FLIGHT_CONTROL = False


# ============================================================
# GLOBAL SHUTDOWN
# ============================================================

running = True


def handle_shutdown(signum, frame):

    global running

    print()
    print("Shutdown requested...")

    running = False


signal.signal(
    signal.SIGINT,
    handle_shutdown
)

signal.signal(
    signal.SIGTERM,
    handle_shutdown
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

    except Exception:

        return None


# ============================================================
# ANGULAR DIFFERENCE
# ============================================================

def angular_difference(
    a,
    b
):

    diff = abs(
        a - b
    )

    while diff > 180.0:

        diff -= 360.0

    return abs(
        diff
    )


# ============================================================
# CAMERA BEARING
# ============================================================

def camera_bearing_from_object(
    obj
):

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
        IndexError,
    ):

        return None

    normalized = (
        center_x -
        CAMERA_WIDTH / 2.0
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
# NORMALIZE LiDAR TRACK
# ============================================================

def normalize_track(
    track
):

    # --------------------------------------------------------
    # Dictionary track
    # --------------------------------------------------------

    if isinstance(
        track,
        dict
    ):

        result = track.copy()

    # --------------------------------------------------------
    # Object/class track
    # --------------------------------------------------------

    else:

        result = {}

        fields = [

            "track_id",

            "id",

            "distance_m",

            "range_m",

            "distance",

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
        ]

        for field in fields:

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
    # Track ID
    # --------------------------------------------------------

    if (
        "track_id" not in result
        and
        "id" in result
    ):

        result["track_id"] = (
            result["id"]
        )

    # --------------------------------------------------------
    # Distance
    # --------------------------------------------------------

    if "distance_m" not in result:

        for field in [
            "range_m",
            "distance",
        ]:

            if field in result:

                result["distance_m"] = (
                    result[field]
                )

                break

    # --------------------------------------------------------
    # Bearing
    # --------------------------------------------------------

    if "bearing_deg" not in result:

        for field in [
            "bearing",
            "angle_deg",
            "angle",
        ]:

            if field in result:

                result["bearing_deg"] = (
                    result[field]
                )

                break

    # --------------------------------------------------------
    # Speed
    # --------------------------------------------------------

    if "speed_mps" not in result:

        for field in [
            "speed",
            "velocity_mps",
            "velocity",
        ]:

            if field in result:

                result["speed_mps"] = (
                    result[field]
                )

                break

    return result


# ============================================================
# CAMERA + LiDAR FUSION
# ============================================================

def fuse_objects(
    camera_objects,
    lidar_tracks
):

    fused = []

    used_lidar = set()

    camera_measurements = []

    # --------------------------------------------------------
    # Calculate camera bearing
    # --------------------------------------------------------

    for obj in camera_objects:

        bearing = (
            camera_bearing_from_object(
                obj
            )
        )

        if bearing is None:

            continue

        camera_measurements.append(
            (
                obj,
                bearing
            )
        )

    # --------------------------------------------------------
    # Match camera objects with LiDAR
    # --------------------------------------------------------

    for (
        camera_object,
        camera_bearing
    ) in camera_measurements:

        best_track = None

        best_error = float(
            "inf"
        )

        for lidar_track in lidar_tracks:

            lidar_id = (
                lidar_track.get(
                    "track_id"
                )
            )

            if lidar_id is None:

                continue

            if lidar_id in used_lidar:

                continue

            try:

                lidar_bearing = float(
                    lidar_track.get(
                        "bearing_deg",
                        0.0
                    )
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
                and
                error < best_error
            ):

                best_track = (
                    lidar_track
                )

                best_error = (
                    error
                )

        # ----------------------------------------------------
        # No match
        # ----------------------------------------------------

        if best_track is None:

            continue

        lidar_id = (
            best_track.get(
                "track_id"
            )
        )

        used_lidar.add(
            lidar_id
        )

        # ----------------------------------------------------
        # Range
        # ----------------------------------------------------

        try:

            distance = float(
                best_track.get(
                    "distance_m",
                    0.0
                )
            )

        except Exception:

            distance = 0.0

        # ----------------------------------------------------
        # LiDAR bearing
        # ----------------------------------------------------

        try:

            lidar_bearing = float(
                best_track.get(
                    "bearing_deg",
                    0.0
                )
            )

        except Exception:

            lidar_bearing = 0.0

        # ----------------------------------------------------
        # Speed
        # ----------------------------------------------------

        try:

            lidar_speed = float(
                best_track.get(
                    "speed_mps",
                    0.0
                )
            )

        except Exception:

            lidar_speed = 0.0

        # ----------------------------------------------------
        # Camera confidence
        # ----------------------------------------------------

        try:

            camera_confidence = float(
                camera_object.get(
                    "confidence",
                    0.0
                )
            )

        except Exception:

            camera_confidence = 0.0

        # ----------------------------------------------------
        # Fusion quality
        # ----------------------------------------------------

        fusion_quality = max(
            0.0,
            1.0 -
            (
                best_error /
                FUSION_GATE_DEG
            )
        )

        fusion_confidence = (
            0.60 *
            camera_confidence
            +
            0.40 *
            fusion_quality
        )

        # ----------------------------------------------------
        # Fused object
        # ----------------------------------------------------

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
                    distance,
                    3
                ),

            "speed_mps":
                round(
                    lidar_speed,
                    3
                ),

            "confidence":
                round(
                    camera_confidence,
                    3
                ),

            "fusion_confidence":
                round(
                    fusion_confidence,
                    3
                ),

            "fusion_status":
                "FUSED",
        })

    return fused


# ============================================================
# PERSISTENT TARGET MANAGER
# ============================================================

class PersistentTargetManager:

    def __init__(
        self
    ):

        self.next_id = 1

        self.objects = {}


    # ========================================================
    # CREATE STABLE ID
    # ========================================================

    def create_id(
        self
    ):

        value = (
            self.next_id
        )

        self.next_id += 1

        return value


    # ========================================================
    # FIND EXISTING OBJECT
    # ========================================================

    def find_match(
        self,
        camera_id,
        lidar_id,
        class_name
    ):

        candidates = []

        now = time.time()

        for (
            object_id,
            obj
        ) in self.objects.items():

            if (
                obj.get("state")
                ==
                "EXPIRED"
            ):

                continue

            previous_camera = (
                obj.get(
                    "camera_id"
                )
            )

            previous_lidar = (
                obj.get(
                    "lidar_id"
                )
            )

            previous_class = (
                obj.get(
                    "class"
                )
            )

            camera_match = (
                camera_id is not None
                and
                previous_camera
                == camera_id
            )

            lidar_match = (
                lidar_id is not None
                and
                previous_lidar
                == lidar_id
            )

            class_match = (
                previous_class
                ==
                class_name
            )

            # ------------------------------------------------
            # Exact camera + LiDAR
            # ------------------------------------------------

            if (
                camera_match
                and
                lidar_match
            ):

                candidates.append(
                    (
                        object_id,
                        0
                    )
                )

                continue

            # ------------------------------------------------
            # Camera match
            # ------------------------------------------------

            if camera_match:

                candidates.append(
                    (
                        object_id,
                        1
                    )
                )

                continue

            # ------------------------------------------------
            # LiDAR match
            # ------------------------------------------------

            if lidar_match:

                candidates.append(
                    (
                        object_id,
                        2
                    )

                )

                continue

            # ------------------------------------------------
            # Same class, very recent
            # ------------------------------------------------

            if class_match:

                last_seen = (
                    obj.get(
                        "last_seen_time",
                        0.0
                    )
                )

                age = (
                    now -
                    last_seen
                )

                if age < 0.5:

                    candidates.append(
                        (
                            object_id,
                            3
                        )
                    )

        if not candidates:

            return None

        candidates.sort(
            key=lambda item: item[1]
        )

        return candidates[0][0]


    # ========================================================
    # UPDATE
    # ========================================================

    def update(
        self,
        fused_objects
    ):

        now = time.time()

        output = []

        active_ids = set()

        # ====================================================
        # UPDATE FUSED OBJECTS
        # ====================================================

        for fused in fused_objects:

            camera_id = (
                fused.get(
                    "camera_id"
                )
            )

            lidar_id = (
                fused.get(
                    "lidar_id"
                )
            )

            class_name = (
                fused.get(
                    "class",
                    "unknown"
                )
            )

            object_id = (
                self.find_match(
                    camera_id,
                    lidar_id,
                    class_name
                )
            )

            # ------------------------------------------------
            # New object
            # ------------------------------------------------

            if object_id is None:

                object_id = (
                    self.create_id()
                )

                self.objects[
                    object_id
                ] = {

                    "stable_id":
                        object_id,

                    "created_time":
                        now,

                    "first_seen_time":
                        now,

                    "last_seen_time":
                        now,

                    "state":
                        "ACTIVE",

                    "hits":
                        0,

                    "lost_count":
                        0,

                    "prediction_count":
                        0,

                    "x_m":
                        None,

                    "y_m":
                        None,

                    "range_m":
                        None,

                    "bearing_deg":
                        None,

                    "speed_mps":
                        0.0,
                }

            obj = self.objects[
                object_id
            ]

            # ------------------------------------------------
            # Measurements
            # ------------------------------------------------

            try:

                measured_range = float(
                    fused.get(
                        "range_m",
                        0.0
                    )
                )

            except Exception:

                measured_range = 0.0

            try:

                measured_bearing = float(
                    fused.get(
                        "camera_bearing_deg",
                        0.0
                    )
                )

            except Exception:

                measured_bearing = 0.0

            # ------------------------------------------------
            # Polar -> Cartesian
            #
            # X = horizontal
            # Y = forward
            # ------------------------------------------------

            angle_rad = math.radians(
                measured_bearing
            )

            measured_x = (
                measured_range *
                math.sin(
                    angle_rad
                )
            )

            measured_y = (
                measured_range *
                math.cos(
                    angle_rad
                )
            )

            # ------------------------------------------------
            # Previous position
            # ------------------------------------------------

            old_x = obj.get(
                "x_m"
            )

            old_y = obj.get(
                "y_m"
            )

            old_time = obj.get(
                "last_seen_time"
            )

            # ------------------------------------------------
            # Position smoothing
            # ------------------------------------------------

            if (
                old_x is not None
                and
                old_y is not None
            ):

                x_m = (
                    (
                        1.0 -
                        POSITION_ALPHA
                    )
                    *
                    old_x
                    +
                    POSITION_ALPHA
                    *
                    measured_x
                )

                y_m = (
                    (
                        1.0 -
                        POSITION_ALPHA
                    )
                    *
                    old_y
                    +
                    POSITION_ALPHA
                    *
                    measured_y
                )

            else:

                x_m = measured_x

                y_m = measured_y

            # ------------------------------------------------
            # Position velocity
            # ------------------------------------------------

            calculated_speed = 0.0

            if (
                old_x is not None
                and
                old_y is not None
                and
                old_time is not None
            ):

                dt = (
                    now -
                    old_time
                )

                if dt > 0.001:

                    dx = (
                        x_m -
                        old_x
                    )

                    dy = (
                        y_m -
                        old_y
                    )

                    calculated_speed = (
                        math.sqrt(
                            dx * dx
                            +
                            dy * dy
                        )
                        /
                        dt
                    )

            # ------------------------------------------------
            # LiDAR speed
            # ------------------------------------------------

            try:

                lidar_speed = float(
                    fused.get(
                        "speed_mps",
                        0.0
                    )
                )

            except Exception:

                lidar_speed = 0.0

            # ------------------------------------------------
            # Combine speed
            # ------------------------------------------------

            if (
                calculated_speed
                <
                20.0
            ):

                speed_mps = (
                    (
                        1.0 -
                        VELOCITY_ALPHA
                    )
                    *
                    lidar_speed
                    +
                    VELOCITY_ALPHA
                    *
                    calculated_speed
                )

            else:

                speed_mps = (
                    lidar_speed
                )

            # ------------------------------------------------
            # Update persistent state
            # ------------------------------------------------

            obj[
                "camera_id"
            ] = camera_id

            obj[
                "lidar_id"
            ] = lidar_id

            obj[
                "class"
            ] = class_name

            obj[
                "range_m"
            ] = round(
                measured_range,
                3
            )

            obj[
                "bearing_deg"
            ] = round(
                measured_bearing,
                3
            )

            obj[
                "x_m"
            ] = round(
                x_m,
                3
            )

            obj[
                "y_m"
            ] = round(
                y_m,
                3
            )

            obj[
                "z_m"
            ] = None

            obj[
                "speed_mps"
            ] = round(
                speed_mps,
                3
            )

            obj[
                "bearing_error_deg"
            ] = fused.get(
                "bearing_error_deg",
                0.0
            )

            obj[
                "camera_confidence"
            ] = fused.get(
                "confidence",
                0.0
            )

            obj[
                "fusion_confidence"
            ] = fused.get(
                "fusion_confidence",
                0.0
            )

            obj[
                "last_seen_time"
            ] = now

            obj[
                "state"
            ] = "ACTIVE"

            obj[
                "hits"
            ] = (
                obj.get(
                    "hits",
                    0
                )
                +
                1
            )

            obj[
                "lost_count"
            ] = 0

            active_ids.add(
                object_id
            )

            output.append(
                obj.copy()
            )

        # ====================================================
        # HANDLE LOST OBJECTS
        # ====================================================

        for (
            object_id,
            obj
        ) in self.objects.items():

            if object_id in active_ids:

                continue

            if (
                obj.get("state")
                ==
                "EXPIRED"
            ):

                continue

            last_seen = (
                obj.get(
                    "last_seen_time",
                    0.0
                )
            )

            age = (
                now -
                last_seen
            )

            # ------------------------------------------------
            # TEMPORARILY LOST
            # ------------------------------------------------

            if (
                age
                <=
                TEMP_LOST_TIMEOUT_SEC
            ):

                obj[
                    "state"
                ] = "TEMP_LOST"

                obj[
                    "lost_count"
                ] = (
                    obj.get(
                        "lost_count",
                        0
                    )
                    +
                    1
                )

                output.append(
                    obj.copy()
                )

            # ------------------------------------------------
            # PREDICTED
            # ------------------------------------------------

            elif (
                age
                <=
                EXPIRE_TIMEOUT_SEC
            ):

                obj[
                    "state"
                ] = "PREDICTED"

                obj[
                    "prediction_count"
                ] = (
                    obj.get(
                        "prediction_count",
                        0
                    )
                    +
                    1
                )

                # --------------------------------------------
                # Keep last valid position.
                #
                # We intentionally DO NOT invent a
                # movement direction because scalar LiDAR
                # speed does not provide a reliable velocity
                # vector.
                # --------------------------------------------

                output.append(
                    obj.copy()
                )

            # ------------------------------------------------
            # EXPIRED
            # ------------------------------------------------

            else:

                obj[
                    "state"
                ] = "EXPIRED"

        return output


# ============================================================
# SAVE JSON
# ============================================================

def save_json(
    frame,
    camera_objects,
    lidar_tracks,
    fused_objects,
    persistent_objects
):

    data = {

        "protocol":
            "NIDAR_V17_PERSISTENT_FUSION",

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

        "headless":
            True,

        "camera_objects":
            camera_objects,

        "lidar_tracks":
            lidar_tracks,

        "fused_objects":
            fused_objects,

        "persistent_objects":
            persistent_objects,
    }

    temp_file = (
        OUTPUT_JSON
        +
        ".tmp"
    )

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
# SAVE CSV
# ============================================================

def save_csv(
    frame,
    objects
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

                "frame",

                "stable_id",

                "camera_id",

                "lidar_id",

                "class",

                "state",

                "range_m",

                "bearing_deg",

                "x_m",

                "y_m",

                "z_m",

                "speed_mps",

                "bearing_error_deg",

                "camera_confidence",

                "fusion_confidence",

                "hits",

                "lost_count",

                "prediction_count",
            ])

        timestamp = time.time()

        for obj in objects:

            writer.writerow([

                timestamp,

                frame,

                obj.get(
                    "stable_id"
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
                    "state"
                ),

                obj.get(
                    "range_m"
                ),

                obj.get(
                    "bearing_deg"
                ),

                obj.get(
                    "x_m"
                ),

                obj.get(
                    "y_m"
                ),

                obj.get(
                    "z_m"
                ),

                obj.get(
                    "speed_mps"
                ),

                obj.get(
                    "bearing_error_deg"
                ),

                obj.get(
                    "camera_confidence"
                ),

                obj.get(
                    "fusion_confidence"
                ),

                obj.get(
                    "hits"
                ),

                obj.get(
                    "lost_count"
                ),

                obj.get(
                    "prediction_count"
                ),
            ])


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 100)

    print(
        "NIDAR V17 - PERSISTENT TARGET TRACKING + PREDICTION"
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
        f"Temp lost     : "
        f"{TEMP_LOST_TIMEOUT_SEC:.1f}s"
    )

    print(
        f"Expire        : "
        f"{EXPIRE_TIMEOUT_SEC:.1f}s"
    )

    print(
        "Flight control: DISABLED"
    )

    print(
        "Display       : DISABLED"
    )

    print("=" * 100)

    # ========================================================
    # INITIALIZE LiDAR
    # ========================================================

    try:

        backend = (
            SimulatedSTL19PBackend()
        )

        detector = (
            LidarObstacleDetector(
                ObstacleDetectorConfig()
            )
        )

        tracker = (
            LidarObstacleTrackerV3(
                TrackerV3Config()
            )
        )

    except Exception as e:

        print()
        print(
            "LiDAR initialization FAILED"
        )

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
        "Persistent manager : READY"
    )

    print(
        "Prediction layer   : READY"
    )

    print(
        "Headless mode      : ENABLED"
    )

    print("=" * 100)

    manager = (
        PersistentTargetManager()
    )

    last_camera_frame = None

    processed_frames = 0

    total_fused = 0

    total_active = 0

    total_temp_lost = 0

    total_predicted = 0

    start_time = time.time()

    # ========================================================
    # MAIN LOOP
    # ========================================================

    while running:

        loop_start = time.time()

        # ----------------------------------------------------
        # CAMERA
        # ----------------------------------------------------

        packet = (
            load_camera_packet()
        )

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
        # Avoid duplicate camera frames
        # ----------------------------------------------------

        if (
            frame
            ==
            last_camera_frame
        ):

            time.sleep(
                LOOP_DELAY
            )

            continue

        last_camera_frame = frame

        processed_frames += 1

        # ====================================================
        # LiDAR
        # ====================================================

        try:

            scan = (
                backend.next_scan()
            )

            lidar_detections = (
                detector.detect(
                    scan
                )
            )

            timestamp = time.time()

            lidar_tracks_raw = (
                tracker.update(
                    lidar_detections,
                    timestamp
                )
            )

        except Exception as e:

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

        for track in (
            lidar_tracks_raw
        ):

            normalized = (
                normalize_track(
                    track
                )
            )

            if normalized:

                lidar_tracks.append(
                    normalized
                )

        # ====================================================
        # CAMERA + LiDAR FUSION
        # ====================================================

        fused_objects = (
            fuse_objects(
                camera_objects,
                lidar_tracks
            )
        )

        total_fused += len(
            fused_objects
        )

        # ====================================================
        # PERSISTENT TRACKING
        # ====================================================

        persistent_objects = (
            manager.update(
                fused_objects
            )
        )

        # ====================================================
        # STATISTICS
        # ====================================================

        for obj in (
            persistent_objects
        ):

            state = obj.get(
                "state"
            )

            if state == "ACTIVE":

                total_active += 1

            elif state == "TEMP_LOST":

                total_temp_lost += 1

            elif state == "PREDICTED":

                total_predicted += 1

        # ====================================================
        # SAVE JSON
        # ====================================================

        try:

            save_json(

                frame,

                camera_objects,

                lidar_tracks,

                fused_objects,

                persistent_objects,
            )

        except Exception as e:

            print(
                "[JSON SAVE ERROR]",
                e
            )

        # ====================================================
        # SAVE CSV
        # ====================================================

        try:

            save_csv(
                frame,
                persistent_objects
            )

        except Exception as e:

            print(
                "[CSV SAVE ERROR]",
                e
            )

        # ====================================================
        # TERMINAL OUTPUT
        # ====================================================

        if (
            processed_frames
            %
            PRINT_EVERY_N_FRAMES
            ==
            0
        ):

            elapsed = max(
                time.time()
                -
                start_time,
                0.001
            )

            fps = (
                processed_frames
                /
                elapsed
            )

            print(
                "-" * 100
            )

            print(
                f"Frame              : "
                f"{frame}"
            )

            print(
                f"Camera objects     : "
                f"{len(camera_objects)}"
            )

            print(
                f"LiDAR detections   : "
                f"{len(lidar_detections)}"
            )

            print(
                f"LiDAR tracks       : "
                f"{len(lidar_tracks)}"
            )

            print(
                f"Fused objects      : "
                f"{len(fused_objects)}"
            )

            print(
                f"Persistent objects : "
                f"{len(persistent_objects)}"
            )

            print(
                f"Processing FPS     : "
                f"{fps:.2f}"
            )

            # ------------------------------------------------
            # Object information
            # ------------------------------------------------

            if persistent_objects:

                for obj in (
                    persistent_objects
                ):

                    print(
                        "OBJECT             : "
                        f"S{obj.get('stable_id')} | "
                        f"{obj.get('class')} | "
                        f"state="
                        f"{obj.get('state')} | "
                        f"range="
                        f"{obj.get('range_m', 0.0):.2f}m | "
                        f"bearing="
                        f"{obj.get('bearing_deg', 0.0):+.2f}° | "
                        f"X="
                        f"{obj.get('x_m', 0.0):+.2f}m | "
                        f"Y="
                        f"{obj.get('y_m', 0.0):+.2f}m | "
                        f"speed="
                        f"{obj.get('speed_mps', 0.0):.2f}m/s | "
                        f"conf="
                        f"{obj.get('fusion_confidence', 0.0):.2f}"
                    )

            else:

                print(
                    "OBJECT             : none"
                )

        # ====================================================
        # LOOP TIMING
        # ====================================================

        processing_time = (
            time.time()
            -
            loop_start
        )

        remaining = (
            LOOP_DELAY
            -
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
        time.time()
        -
        start_time,
        0.001
    )

    fps = (
        processed_frames
        /
        elapsed
    )

    print()

    print("=" * 100)

    print(
        "NIDAR V17 SHUTDOWN"
    )

    print("=" * 100)

    print(
        f"Frames processed : "
        f"{processed_frames}"
    )

    print(
        f"Total fused      : "
        f"{total_fused}"
    )

    print(
        f"Active reports   : "
        f"{total_active}"
    )

    print(
        f"Temp lost reports: "
        f"{total_temp_lost}"
    )

    print(
        f"Predicted reports: "
        f"{total_predicted}"
    )

    print(
        f"Average FPS      : "
        f"{fps:.2f}"
    )

    print(
        f"JSON output      : "
        f"{OUTPUT_JSON}"
    )

    print(
        f"CSV output       : "
        f"{OUTPUT_CSV}"
    )

    print(
        "Flight control   : DISABLED"
    )

    print(
        "Display          : DISABLED"
    )

    print("=" * 100)

    return 0


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    sys.exit(
        main()
    )
