#!/usr/bin/env python3

import csv
import json
import math
import os
import time
from collections import defaultdict, deque
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
# NIDAR V16
# HEADLESS REAL-TIME SENSOR FUSION + VALIDATION
# ============================================================
#
# Camera:
#   Mac webcam -> NIDAR V11 -> perception JSON -> Raspberry Pi
#
# LiDAR:
#   STL-19P simulator
#
# Adds:
#   - Camera/LiDAR association
#   - Persistent fusion history
#   - Object entry/exit tracking
#   - Bearing history
#   - Range history
#   - Fusion stability
#   - Packet/frame monitoring
#   - Processing latency
#   - FPS estimation
#   - Sensor health
#   - CSV logging
#   - JSON state
#   - Validation summary
#
# Display:
#   DISABLED
#
# Flight control:
#   DISABLED
#
# ============================================================


PROJECT_DIR = os.path.expanduser("~/drone_project")

CAMERA_PACKET = os.path.join(
    PROJECT_DIR,
    "nidar_latest_perception.json"
)

OUTPUT_JSON = os.path.join(
    PROJECT_DIR,
    "nidar_v16_validation_state.json"
)

OUTPUT_CSV = os.path.join(
    PROJECT_DIR,
    "nidar_v16_fusion_history.csv"
)

REPORT_JSON = os.path.join(
    PROJECT_DIR,
    "nidar_v16_validation_report.json"
)


# ============================================================
# CAMERA
# ============================================================

CAMERA_WIDTH = 1280.0
CAMERA_HEIGHT = 720.0

# Temporary software FOV.
# Must be physically calibrated later.
CAMERA_HFOV_DEG = 70.0


# ============================================================
# FUSION
# ============================================================

FUSION_GATE_DEG = 10.0

MAX_CAMERA_AGE_SEC = 2.0

LOOP_DELAY = 0.02


# ============================================================
# HISTORY
# ============================================================

HISTORY_LENGTH = 30

STALE_OBJECT_SEC = 2.0


# ============================================================
# GLOBAL VALIDATION COUNTERS
# ============================================================

frames_processed = 0

frames_with_camera = 0

frames_with_lidar = 0

frames_with_fusion = 0

total_fused_objects = 0

total_camera_objects = 0

total_lidar_tracks = 0

total_bearing_error = 0.0

max_bearing_error = 0.0

camera_frame_gaps = 0

last_camera_frame = None

first_runtime = None

last_runtime = None

processing_times = deque(
    maxlen=100
)


# ============================================================
# OBJECT HISTORY
# ============================================================

object_history = defaultdict(
    lambda: {
        "first_seen": None,
        "last_seen": None,
        "frames_seen": 0,
        "ranges": deque(
            maxlen=HISTORY_LENGTH
        ),
        "bearings": deque(
            maxlen=HISTORY_LENGTH
        ),
        "errors": deque(
            maxlen=HISTORY_LENGTH
        ),
        "fusion_confidence": deque(
            maxlen=HISTORY_LENGTH
        ),
    }
)


active_objects = set()


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
# ANGLES
# ============================================================

def normalize_angle_deg(
    angle
):

    angle = safe_float(
        angle
    )

    return (
        (angle + 180.0) % 360.0
    ) - 180.0


def angular_error_deg(
    a,
    b
):

    return abs(
        normalize_angle_deg(
            normalize_angle_deg(a)
            -
            normalize_angle_deg(b)
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
# NORMALIZE LIDAR TRACK
# ============================================================

def normalize_track(
    track
):

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

    track_id = raw.get(
        "track_id"
    )

    if track_id is None:

        track_id = raw.get(
            "id"
        )

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

    x = safe_float(
        raw.get(
            "x_m"
        )
    )

    y = safe_float(
        raw.get(
            "y_m"
        )
    )

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
            raw.get(
                "state",
                "UNKNOWN"
            ),

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

def fusion_confidence(
    camera_confidence,
    lidar_confidence,
    error
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

    bearing_score = max(
        0.0,
        1.0 -
        (
            error /
            FUSION_GATE_DEG
        )
    )

    value = (
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
            value
        )
    )


# ============================================================
# FUSE
# ============================================================

def fuse_objects(
    camera_objects,
    lidar_tracks
):

    fused = []

    used_lidar = set()

    camera_measurements = []

    for obj in camera_objects:

        bearing = camera_bearing_deg(
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

    for obj, cam_bearing in camera_measurements:

        best = None

        best_error = float(
            "inf"
        )

        for lidar in lidar_tracks:

            lidar_id = lidar.get(
                "track_id"
            )

            if lidar_id in used_lidar:

                continue

            error = angular_error_deg(
                cam_bearing,
                lidar.get(
                    "bearing_deg",
                    0.0
                )
            )

            if (
                error <= FUSION_GATE_DEG
                and error < best_error
            ):

                best = lidar

                best_error = error

        if best is None:

            continue

        lidar_id = best.get(
            "track_id"
        )

        used_lidar.add(
            lidar_id
        )

        camera_id = obj.get(
            "id"
        )

        camera_confidence = safe_float(
            obj.get(
                "confidence",
                0.0
            )
        )

        lidar_confidence = safe_float(
            best.get(
                "confidence",
                0.0
            )
        )

        confidence = fusion_confidence(
            camera_confidence,
            lidar_confidence,
            best_error
        )

        fused.append({

            "fused_id":
                f"C{camera_id}-L{lidar_id}",

            "camera_id":
                camera_id,

            "lidar_id":
                lidar_id,

            "class":
                obj.get(
                    "class",
                    "unknown"
                ),

            "camera_bearing_deg":
                round(
                    cam_bearing,
                    3
                ),

            "lidar_bearing_deg":
                round(
                    best.get(
                        "bearing_deg",
                        0.0
                    ),
                    3
                ),

            "bearing_error_deg":
                round(
                    best_error,
                    3
                ),

            "range_m":
                round(
                    safe_float(
                        best.get(
                            "distance_m"
                        )
                    ),
                    3
                ),

            "x_m":
                round(
                    safe_float(
                        best.get(
                            "x_m"
                        )
                    ),
                    3
                ),

            "y_m":
                round(
                    safe_float(
                        best.get(
                            "y_m"
                        )
                    ),
                    3
                ),

            "speed_mps":
                round(
                    safe_float(
                        best.get(
                            "speed_mps"
                        )
                    ),
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
                    confidence,
                    3
                ),

            "lidar_state":
                best.get(
                    "state",
                    "UNKNOWN"
                ),

            "fusion_status":
                "FUSED",

        })

    return fused


# ============================================================
# UPDATE HISTORY
# ============================================================

def update_history(
    fused,
    frame,
    timestamp
):

    global active_objects

    current_ids = set()

    entered = []

    exited = []

    for obj in fused:

        fused_id = obj[
            "fused_id"
        ]

        current_ids.add(
            fused_id
        )

        history = object_history[
            fused_id
        ]

        if history[
            "first_seen"
        ] is None:

            history[
                "first_seen"
            ] = timestamp

            entered.append(
                fused_id
            )

        history[
            "last_seen"
        ] = timestamp

        history[
            "frames_seen"
        ] += 1

        history[
            "ranges"
        ].append(
            obj[
                "range_m"
            ]
        )

        history[
            "bearings"
        ].append(
            obj[
                "camera_bearing_deg"
            ]
        )

        history[
            "errors"
        ].append(
            obj[
                "bearing_error_deg"
            ]
        )

        history[
            "fusion_confidence"
        ].append(
            obj[
                "fusion_confidence"
            ]
        )

    # --------------------------------------------------------
    # Detect exited objects
    # --------------------------------------------------------

    for object_id in list(
        active_objects
    ):

        history = object_history[
            object_id
        ]

        last_seen = history[
            "last_seen"
        ]

        if last_seen is None:

            continue

        if (
            object_id not in current_ids
            and
            timestamp - last_seen
            > STALE_OBJECT_SEC
        ):

            exited.append(
                object_id
            )

    active_objects = current_ids

    return (
        entered,
        exited
    )


# ============================================================
# OBJECT METRICS
# ============================================================

def object_metrics(
    fused_id
):

    history = object_history[
        fused_id
    ]

    errors = list(
        history[
            "errors"
        ]
    )

    ranges = list(
        history[
            "ranges"
        ]
    )

    bearings = list(
        history[
            "bearings"
        ]
    )

    confidences = list(
        history[
            "fusion_confidence"
        ]
    )

    if errors:

        mean_error = (
            sum(errors) /
            len(errors)
        )

        max_error = max(
            errors
        )

    else:

        mean_error = 0.0
        max_error = 0.0

    if ranges:

        mean_range = (
            sum(ranges) /
            len(ranges)
        )

    else:

        mean_range = 0.0

    if bearings:

        bearing_span = (
            max(bearings)
            -
            min(bearings)
        )

    else:

        bearing_span = 0.0

    if confidences:

        mean_confidence = (
            sum(confidences) /
            len(confidences)
        )

    else:

        mean_confidence = 0.0

    return {

        "frames_seen":
            history[
                "frames_seen"
            ],

        "mean_range_m":
            round(
                mean_range,
                3
            ),

        "mean_bearing_error_deg":
            round(
                mean_error,
                3
            ),

        "max_bearing_error_deg":
            round(
                max_error,
                3
            ),

        "bearing_span_deg":
            round(
                bearing_span,
                3
            ),

        "mean_fusion_confidence":
            round(
                mean_confidence,
                3
            ),

        "first_seen":
            history[
                "first_seen"
            ],

        "last_seen":
            history[
                "last_seen"
            ],
    }


# ============================================================
# SENSOR HEALTH
# ============================================================

def sensor_health(
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

    age = max(
        0.0,
        now -
        packet_timestamp
    )

    return {

        "camera":
            (
                "OK"
                if age <= MAX_CAMERA_AGE_SEC
                else "STALE"
            ),

        "lidar":
            (
                "OK"
                if scan is not None
                else "ERROR"
            ),

        "lidar_tracker":
            (
                "OK"
                if lidar_tracks is not None
                else "ERROR"
            ),

        "fusion":
            (
                "MATCH"
                if fused
                else "NO_MATCH"
            ),

        "display":
            "DISABLED",

        "flight_control":
            "DISABLED",

        "camera_packet_age_sec":
            round(
                age,
                3
            ),
    }


# ============================================================
# PERFORMANCE
# ============================================================

def performance_metrics():

    if processing_times:

        avg_ms = (
            sum(
                processing_times
            )
            /
            len(
                processing_times
            )
        ) * 1000.0

        max_ms = (
            max(
                processing_times
            )
        ) * 1000.0

    else:

        avg_ms = 0.0
        max_ms = 0.0

    if (
        first_runtime is not None
        and
        last_runtime is not None
        and
        last_runtime >
        first_runtime
    ):

        runtime = (
            last_runtime -
            first_runtime
        )

        fps = (
            frames_processed /
            runtime
        )

    else:

        fps = 0.0

    return {

        "frames_processed":
            frames_processed,

        "runtime_sec":
            round(
                (
                    (
                        last_runtime -
                        first_runtime
                    )
                    if
                    (
                        first_runtime is not None
                        and
                        last_runtime is not None
                    )
                    else 0.0
                ),
                3
            ),

        "estimated_fps":
            round(
                fps,
                2
            ),

        "average_processing_ms":
            round(
                avg_ms,
                3
            ),

        "max_processing_ms":
            round(
                max_ms,
                3
            ),

        "camera_frames_with_data":
            frames_with_camera,

        "lidar_frames_with_data":
            frames_with_lidar,

        "frames_with_fusion":
            frames_with_fusion,

        "camera_frame_gaps":
            camera_frame_gaps,

        "total_camera_objects":
            total_camera_objects,

        "total_lidar_tracks":
            total_lidar_tracks,

        "total_fused_objects":
            total_fused_objects,

        "mean_bearing_error_deg":
            round(
                (
                    total_bearing_error /
                    total_fused_objects
                )
                if total_fused_objects
                else 0.0,
                3
            ),

        "max_bearing_error_deg":
            round(
                max_bearing_error,
                3
            ),

    }


# ============================================================
# SAVE STATE
# ============================================================

def save_state(
    packet,
    lidar_tracks,
    fused,
    health,
    entered,
    exited
):

    state = {

        "protocol":
            "NIDAR_V16_HEADLESS_VALIDATION",

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

        "performance":
            performance_metrics(),

        "events": {

            "entered":
                entered,

            "exited":
                exited,

        },

        "camera_objects":
            packet.get(
                "objects",
                []
            ),

        "lidar_tracks":
            lidar_tracks,

        "fused_objects":
            fused,

        "object_history": {

            object_id:
                object_metrics(
                    object_id
                )

            for object_id
            in active_objects

        },

    }

    with open(
        OUTPUT_JSON,
        "w"
    ) as f:

        json.dump(
            state,
            f,
            indent=2
        )


# ============================================================
# CSV
# ============================================================

def append_csv(
    frame,
    fused
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

            ])


# ============================================================
# TERMINAL
# ============================================================

def print_frame(
    frame,
    camera_objects,
    lidar_tracks,
    fused,
    health,
    entered,
    exited
):

    print()

    print(
        "=" * 100
    )

    print(
        f"NIDAR V16 | FRAME {frame}"
    )

    print(
        f"Camera: {len(camera_objects)} | "
        f"LiDAR: {len(lidar_tracks)} | "
        f"Fused: {len(fused)}"
    )

    print(
        "-" * 100
    )

    print(
        "HEALTH  "
        f"Camera={health['camera']} | "
        f"LiDAR={health['lidar']} | "
        f"Tracker={health['lidar_tracker']} | "
        f"Fusion={health['fusion']}"
    )

    if entered:

        print(
            "ENTERED:",
            ", ".join(
                entered
            )
        )

    if exited:

        print(
            "EXITED:",
            ", ".join(
                exited
            )
        )

    if fused:

        print()

        print(
            "FUSED OBJECTS"
        )

        for obj in fused:

            print(

                f"  {obj['fused_id']:<10} | "
                f"{obj['class']:<12} | "
                f"CAM={obj['camera_bearing_deg']:+7.2f}° | "
                f"LIDAR={obj['lidar_bearing_deg']:+7.2f}° | "
                f"ERR={obj['bearing_error_deg']:5.2f}° | "
                f"R={obj['range_m']:5.2f}m | "
                f"XY=({obj['x_m']:+5.2f},"
                f"{obj['y_m']:+5.2f}) | "
                f"F={obj['fusion_confidence']:.2f}"

            )

    else:

        print(
            "FUSED OBJECTS: none"
        )

    perf = performance_metrics()

    print()

    print(

        f"FPS={perf['estimated_fps']:.2f} | "
        f"Avg latency={perf['average_processing_ms']:.2f}ms | "
        f"Mean error={perf['mean_bearing_error_deg']:.2f}° | "
        f"Max error={perf['max_bearing_error_deg']:.2f}°"

    )


# ============================================================
# VALIDATION REPORT
# ============================================================

def save_validation_report():

    perf = performance_metrics()

    if frames_processed:

        camera_data_rate = (
            frames_with_camera /
            frames_processed
        )

        lidar_data_rate = (
            frames_with_lidar /
            frames_processed
        )

        fusion_rate = (
            frames_with_fusion /
            frames_processed
        )

    else:

        camera_data_rate = 0.0
        lidar_data_rate = 0.0
        fusion_rate = 0.0

    report = {

        "protocol":
            "NIDAR_V16_VALIDATION_REPORT",

        "generated_at":
            datetime.now().isoformat(),

        "configuration": {

            "camera":
                "MAC_WEBCAM",

            "lidar":
                "STL-19P_SIMULATOR",

            "fusion_gate_deg":
                FUSION_GATE_DEG,

            "display":
                "DISABLED",

            "flight_control":
                "DISABLED",

        },

        "results": {

            "frames_processed":
                frames_processed,

            "camera_data_rate":
                round(
                    camera_data_rate *
                    100.0,
                    2
                ),

            "lidar_data_rate":
                round(
                    lidar_data_rate *
                    100.0,
                    2
                ),

            "fusion_frame_rate":
                round(
                    fusion_rate *
                    100.0,
                    2
                ),

            "total_camera_objects":
                total_camera_objects,

            "total_lidar_tracks":
                total_lidar_tracks,

            "total_fused_objects":
                total_fused_objects,

            "mean_bearing_error_deg":
                perf[
                    "mean_bearing_error_deg"
                ],

            "max_bearing_error_deg":
                perf[
                    "max_bearing_error_deg"
                ],

            "estimated_fps":
                perf[
                    "estimated_fps"
                ],

            "average_processing_ms":
                perf[
                    "average_processing_ms"
                ],

            "max_processing_ms":
                perf[
                    "max_processing_ms"
                ],

            "camera_frame_gaps":
                camera_frame_gaps,

        },

        "object_history": {

            object_id:
                object_metrics(
                    object_id
                )

            for object_id
            in object_history

        },

    }

    with open(
        REPORT_JSON,
        "w"
    ) as f:

        json.dump(
            report,
            f,
            indent=2
        )


# ============================================================
# MAIN
# ============================================================

def main():

    global frames_processed
    global frames_with_camera
    global frames_with_lidar
    global frames_with_fusion
    global total_fused_objects
    global total_camera_objects
    global total_lidar_tracks
    global total_bearing_error
    global max_bearing_error
    global camera_frame_gaps
    global last_camera_frame
    global first_runtime
    global last_runtime

    print(
        "=" * 100
    )

    print(
        "NIDAR V16 - HEADLESS REAL-TIME SENSOR FUSION + VALIDATION"
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
        "Display       : DISABLED"
    )

    print(
        "Flight control: DISABLED"
    )

    print(
        "=" * 100
    )

    try:

        (
            backend,
            detector,
            tracker,
        ) = create_lidar_pipeline()

    except Exception as e:

        print(
            "PIPELINE INITIALIZATION FAILED:"
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
        "Fusion engine      : READY"
    )

    print(
        "Validation engine  : READY"
    )

    print(
        "Headless mode      : ENABLED"
    )

    print(
        "=" * 100
    )

    try:

        while True:

            loop_start = time.perf_counter()

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

            if frame == last_camera_frame:

                time.sleep(
                    LOOP_DELAY
                )

                continue

            # ------------------------------------------------
            # Frame continuity
            # ------------------------------------------------

            if (
                last_camera_frame is not None
                and
                frame >
                last_camera_frame + 1
            ):

                camera_frame_gaps += (
                    frame -
                    last_camera_frame -
                    1
                )

            last_camera_frame = frame

            timestamp = safe_float(
                packet.get(
                    "timestamp",
                    time.time()
                ),
                time.time()
            )

            camera_objects = packet.get(
                "objects",
                []
            )

            frames_processed += 1

            total_camera_objects += (
                len(
                    camera_objects
                )
            )

            if camera_objects:

                frames_with_camera += 1

            # ------------------------------------------------
            # LiDAR
            # ------------------------------------------------

            try:

                scan = backend.next_scan()

                frames_with_lidar += 1

            except Exception as e:

                print()

                print(
                    "LiDAR ERROR:",
                    repr(e)
                )

                continue

            # ------------------------------------------------
            # Detection
            # ------------------------------------------------

            try:

                detections = detector.detect(
                    scan
                )

            except Exception as e:

                print()

                print(
                    "DETECTOR ERROR:",
                    repr(e)
                )

                continue

            # ------------------------------------------------
            # Tracking
            # ------------------------------------------------

            try:

                tracks_raw = tracker.update(
                    detections,
                    scan.timestamp
                )

            except Exception as e:

                print()

                print(
                    "TRACKER ERROR:",
                    repr(e)
                )

                continue

            lidar_tracks = [

                normalize_track(
                    track
                )

                for track
                in tracks_raw

            ]

            total_lidar_tracks += (
                len(
                    lidar_tracks
                )
            )

            # ------------------------------------------------
            # Fusion
            # ------------------------------------------------

            fused = fuse_objects(
                camera_objects,
                lidar_tracks
            )

            if fused:

                frames_with_fusion += 1

            total_fused_objects += (
                len(
                    fused
                )
            )

            # ------------------------------------------------
            # Bearing statistics
            # ------------------------------------------------

            for obj in fused:

                error = safe_float(
                    obj.get(
                        "bearing_error_deg"
                    )
                )

                total_bearing_error += (
                    error
                )

                max_bearing_error = max(
                    max_bearing_error,
                    error
                )

            # ------------------------------------------------
            # History
            # ------------------------------------------------

            (
                entered,
                exited
            ) = update_history(
                fused,
                frame,
                timestamp
            )

            # ------------------------------------------------
            # Health
            # ------------------------------------------------

            health = sensor_health(
                packet,
                scan,
                lidar_tracks,
                fused
            )

            # ------------------------------------------------
            # Performance
            # ------------------------------------------------

            processing_time = (
                time.perf_counter()
                -
                loop_start
            )

            processing_times.append(
                processing_time
            )

            now = time.time()

            if first_runtime is None:

                first_runtime = now

            last_runtime = now

            # ------------------------------------------------
            # Save
            # ------------------------------------------------

            try:

                save_state(
                    packet,
                    lidar_tracks,
                    fused,
                    health,
                    entered,
                    exited
                )

                append_csv(
                    frame,
                    fused
                )

                save_validation_report()

            except Exception as e:

                print()

                print(
                    "OUTPUT ERROR:",
                    repr(e)
                )

            # ------------------------------------------------
            # Terminal
            # ------------------------------------------------

            print_frame(
                frame,
                camera_objects,
                lidar_tracks,
                fused,
                health,
                entered,
                exited
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
            "NIDAR V16 STOPPED"
        )

        print(
            "Generating final validation report..."
        )

        save_validation_report()

        print(
            f"JSON state : {OUTPUT_JSON}"
        )

        print(
            f"CSV log    : {OUTPUT_CSV}"
        )

        print(
            f"Report     : {REPORT_JSON}"
        )

        print(
            "Flight control remained DISABLED."
        )

        print(
            "=" * 100
        )


# ============================================================
# ENTRY
# ============================================================

if __name__ == "__main__":

    main()
