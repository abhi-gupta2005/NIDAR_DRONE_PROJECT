#!/usr/bin/env python3

import json
import time
import csv
import os
import signal
import sys
import math


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
# NIDAR V16
# STABLE OBJECT CAMERA + LiDAR FUSION
# HEADLESS
# ============================================================

PROJECT_DIR = os.path.expanduser("~/drone_project")

CAMERA_PACKET = os.path.join(
    PROJECT_DIR,
    "nidar_latest_perception.json"
)

OUTPUT_JSON = os.path.join(
    PROJECT_DIR,
    "nidar_v16_stable_fusion.json"
)

OUTPUT_CSV = os.path.join(
    PROJECT_DIR,
    "nidar_v16_stable_fusion.csv"
)


# ============================================================
# CONFIGURATION
# ============================================================

FUSION_GATE_DEG = 10.0

CAMERA_WIDTH = 1280.0
CAMERA_HFOV_DEG = 70.0

LOOP_DELAY = 0.05

FLIGHT_CONTROL = False

PRINT_EVERY_N_FRAMES = 10

# How many seconds a fused object can remain alive
# after temporary camera/LiDAR loss.
OBJECT_TIMEOUT_SEC = 1.5


# ============================================================
# GLOBAL SHUTDOWN
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
# STABLE FUSED OBJECT MANAGER
# ============================================================

class StableFusionManager:

    def __init__(self):

        self.next_id = 1

        self.objects = {}

        self.last_update = {}

    # --------------------------------------------------------
    # Generate stable ID
    # --------------------------------------------------------

    def new_id(self):

        value = self.next_id

        self.next_id += 1

        return value

    # --------------------------------------------------------
    # Find existing fused object
    # --------------------------------------------------------

    def find_match(
        self,
        camera_id,
        lidar_id,
        fused
    ):

        # First priority:
        # exact camera + LiDAR pair

        for object_id, previous in self.objects.items():

            if (
                previous.get("camera_id")
                == camera_id
                and
                previous.get("lidar_id")
                == lidar_id
            ):

                return object_id

        # Second priority:
        # same camera ID

        for object_id, previous in self.objects.items():

            if (
                previous.get("camera_id")
                == camera_id
            ):

                old_time = previous.get(
                    "last_seen_time",
                    0.0
                )

                if (
                    time.time() - old_time
                    <= OBJECT_TIMEOUT_SEC
                ):

                    return object_id

        # Third priority:
        # same LiDAR track

        for object_id, previous in self.objects.items():

            if (
                previous.get("lidar_id")
                == lidar_id
            ):

                old_time = previous.get(
                    "last_seen_time",
                    0.0
                )

                if (
                    time.time() - old_time
                    <= OBJECT_TIMEOUT_SEC
                ):

                    return object_id

        return None

    # --------------------------------------------------------
    # Update object
    # --------------------------------------------------------

    def update(self, fused):

        now = time.time()

        result = []

        for item in fused:

            camera_id = item.get(
                "camera_id"
            )

            lidar_id = item.get(
                "lidar_id"
            )

            object_id = self.find_match(
                camera_id,
                lidar_id,
                item
            )

            if object_id is None:

                object_id = self.new_id()

                self.objects[object_id] = {
                    "stable_id":
                        object_id,

                    "first_seen_time":
                        now,

                    "last_seen_time":
                        now,

                    "state":
                        "ACTIVE",

                    "hits":
                        0,
                }

            previous = self.objects[
                object_id
            ]

            # ------------------------------------------------
            # Current measurements
            # ------------------------------------------------

            try:

                range_m = float(
                    item.get(
                        "range_m",
                        0.0
                    )
                )

            except Exception:

                range_m = 0.0

            try:

                bearing_deg = float(
                    item.get(
                        "camera_bearing_deg",
                        0.0
                    )
                )

            except Exception:

                bearing_deg = 0.0

            # ------------------------------------------------
            # Coordinate system
            #
            # X = right/left direction
            # Y = forward direction
            # Z = currently unknown
            # ------------------------------------------------

            angle_rad = math.radians(
                bearing_deg
            )

            x_m = (
                range_m *
                math.sin(angle_rad)
            )

            y_m = (
                range_m *
                math.cos(angle_rad)
            )

            # ------------------------------------------------
            # Estimate velocity
            # ------------------------------------------------

            previous_x = previous.get(
                "x_m"
            )

            previous_y = previous.get(
                "y_m"
            )

            previous_time = previous.get(
                "last_seen_time"
            )

            speed_mps = float(
                item.get(
                    "speed_mps",
                    0.0
                )
            )

            if (
                previous_x is not None
                and previous_y is not None
                and previous_time is not None
            ):

                dt = now - previous_time

                if dt > 0.001:

                    dx = x_m - previous_x
                    dy = y_m - previous_y

                    calculated_speed = (
                        math.sqrt(
                            dx * dx +
                            dy * dy
                        ) / dt
                    )

                    # Prevent unrealistic simulator spikes
                    if calculated_speed < 20.0:

                        speed_mps = (
                            0.7 * speed_mps
                            +
                            0.3 * calculated_speed
                        )

            # ------------------------------------------------
            # Confidence
            # ------------------------------------------------

            try:

                camera_confidence = float(
                    item.get(
                        "confidence",
                        0.0
                    )
                )

            except Exception:

                camera_confidence = 0.0

            try:

                bearing_error = abs(
                    float(
                        item.get(
                            "bearing_error_deg",
                            999.0
                        )
                    )
                )

            except Exception:

                bearing_error = 999.0

            # Smaller bearing error = higher confidence
            fusion_quality = max(
                0.0,
                1.0 -
                (
                    bearing_error /
                    FUSION_GATE_DEG
                )
            )

            fusion_confidence = (
                0.6 *
                camera_confidence
                +
                0.4 *
                fusion_quality
            )

            # ------------------------------------------------
            # Update persistent object
            # ------------------------------------------------

            previous["stable_id"] = object_id

            previous["camera_id"] = camera_id

            previous["lidar_id"] = lidar_id

            previous["class"] = item.get(
                "class",
                "unknown"
            )

            previous["range_m"] = round(
                range_m,
                3
            )

            previous["bearing_deg"] = round(
                bearing_deg,
                3
            )

            previous["x_m"] = round(
                x_m,
                3
            )

            previous["y_m"] = round(
                y_m,
                3
            )

            previous["z_m"] = None

            previous["speed_mps"] = round(
                speed_mps,
                3
            )

            previous["bearing_error_deg"] = round(
                bearing_error,
                3
            )

            previous["camera_confidence"] = round(
                camera_confidence,
                3
            )

            previous["fusion_confidence"] = round(
                fusion_confidence,
                3
            )

            previous["last_seen_time"] = now

            previous["hits"] = (
                previous.get(
                    "hits",
                    0
                )
                + 1
            )

            previous["state"] = "ACTIVE"

            previous["fusion_status"] = "FUSED"

            result.append(
                previous.copy()
            )

        # ----------------------------------------------------
        # Mark old objects as LOST
        # ----------------------------------------------------

        for object_id, previous in self.objects.items():

            if object_id in [
                item["stable_id"]
                for item in result
            ]:

                continue

            last_seen = previous.get(
                "last_seen_time",
                0.0
            )

            age = (
                now -
                last_seen
            )

            if age <= OBJECT_TIMEOUT_SEC:

                previous["state"] = "LOST"

            else:

                previous["state"] = "EXPIRED"

        return result


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

def normalize_track(track):

    if isinstance(
        track,
        dict
    ):

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

    if (
        "track_id" not in result
        and
        "id" in result
    ):

        result["track_id"] = result["id"]

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

                    result[
                        "distance_m"
                    ] = float(
                        getattr(
                            track,
                            field
                        )
                    )

                    break

                except Exception:

                    pass

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

                    result[
                        "bearing_deg"
                    ] = float(
                        getattr(
                            track,
                            field
                        )
                    )

                    break

                except Exception:

                    pass

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

                    result[
                        "speed_mps"
                    ] = float(
                        getattr(
                            track,
                            field
                        )
                    )

                    break

                except Exception:

                    pass

    return result


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
    # Calculate camera bearings
    # --------------------------------------------------------

    for obj in camera_objects:

        bearing = camera_bearing_from_object(
            obj
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
    # Match each camera object
    # with closest LiDAR bearing
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
                and
                error < best_error
            ):

                best_track = lidar_track

                best_error = error

        if best_track is None:

            continue

        lidar_id = best_track.get(
            "track_id"
        )

        used_lidar.add(
            lidar_id
        )

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
                    speed,
                    3
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
# JSON OUTPUT
# ============================================================

def save_json(
    frame,
    camera_objects,
    lidar_tracks,
    fused_objects,
    stable_objects,
):

    data = {

        "protocol":
            "NIDAR_V16_STABLE_OBJECT_FUSION",

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

        "stable_objects":
            stable_objects,
    }

    temp_file = (
        OUTPUT_JSON +
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
# CSV OUTPUT
# ============================================================

def save_csv(
    frame,
    stable_objects
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
            ])

        timestamp = time.time()

        for obj in stable_objects:

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
            ])


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 100)

    print(
        "NIDAR V16 - STABLE OBJECT CAMERA + LiDAR FUSION"
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
        "Coordinate    : X=horizontal, Y=forward, Z=unknown"
    )

    print(
        "Flight control: DISABLED"
    )

    print(
        "Display       : DISABLED"
    )

    print("=" * 100)

    # --------------------------------------------------------
    # LiDAR initialization
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
        "Stable fusion      : READY"
    )

    print(
        "Headless mode      : ENABLED"
    )

    print("=" * 100)

    # --------------------------------------------------------
    # Stable manager
    # --------------------------------------------------------

    manager = StableFusionManager()

    # --------------------------------------------------------
    # Runtime
    # --------------------------------------------------------

    last_camera_frame = None

    processed_frames = 0

    total_fused = 0

    start_time = time.time()

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
        # Do not process same camera frame twice
        # ----------------------------------------------------

        if frame == last_camera_frame:

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

            scan = backend.next_scan()

            lidar_detections = detector.detect(
                scan
            )

            timestamp = time.time()

            lidar_tracks_raw = tracker.update(
                lidar_detections,
                timestamp
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
        # NORMALIZE LiDAR TRACKS
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
        # BASIC FUSION
        # ====================================================

        fused_objects = fuse_objects(
            camera_objects,
            lidar_tracks
        )

        # ====================================================
        # STABLE FUSION
        # ====================================================

        stable_objects = manager.update(
            fused_objects
        )

        total_fused += len(
            fused_objects
        )

        # ====================================================
        # SAVE
        # ====================================================

        try:

            save_json(
                frame,
                camera_objects,
                lidar_tracks,
                fused_objects,
                stable_objects
            )

            save_csv(
                frame,
                stable_objects
            )

        except Exception as e:

            print(
                "[LOGGING ERROR]",
                e
            )

        # ====================================================
        # TERMINAL OUTPUT
        # ====================================================

        if (
            processed_frames %
            PRINT_EVERY_N_FRAMES
            == 0
        ):

            elapsed = max(
                time.time() -
                start_time,
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
                f"Stable objects     : "
                f"{len(stable_objects)}"
            )

            print(
                f"Processing FPS     : "
                f"{fps:.2f}"
            )

            if stable_objects:

                for obj in stable_objects:

                    print(
                        "OBJECT             : "
                        f"S{obj.get('stable_id')} | "
                        f"{obj.get('class')} | "
                        f"state="
                        f"{obj.get('state')} | "
                        f"range="
                        f"{obj.get('range_m'):.2f}m | "
                        f"bearing="
                        f"{obj.get('bearing_deg'):+.2f}° | "
                        f"X="
                        f"{obj.get('x_m'):+.2f}m | "
                        f"Y="
                        f"{obj.get('y_m'):+.2f}m | "
                        f"speed="
                        f"{obj.get('speed_mps'):.2f}m/s | "
                        f"conf="
                        f"{obj.get('fusion_confidence'):.2f}"
                    )

            else:

                print(
                    "OBJECT             : none"
                )

        # ====================================================
        # TIMING
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
        time.time() -
        start_time,
        0.001
    )

    fps = (
        processed_frames /
        elapsed
    )

    print()

    print("=" * 100)

    print(
        "NIDAR V16 SHUTDOWN"
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


if __name__ == "__main__":

    sys.exit(
        main()
    )
