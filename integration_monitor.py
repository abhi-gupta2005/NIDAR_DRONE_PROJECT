#!/usr/bin/env python3

import json
import os
import subprocess
import time
from pathlib import Path


# ============================================================
# NIDAR UNIFIED INTEGRATION MONITOR
#
# Monitors:
#   1. NIDAR Pi perception receiver
#   2. NIDAR V18 camera + LiDAR fusion
#   3. NIDAR SLAM V5.1
#
# Purpose:
#   Replace multiple manual monitoring terminals with one
#   unified system dashboard.
# ============================================================


BASE_DIR = Path(__file__).resolve().parent

# ------------------------------------------------------------
# DATA FILES
# ------------------------------------------------------------

PERCEPTION_FILE = (
    BASE_DIR
    / "src"
    / "communication"
    / "nidar_latest_perception.json"
)

V18_FILE = (
    BASE_DIR
    / "nidar_v18_headless_persistent_fusion.json"
)

SLAM_FILE = (
    BASE_DIR
    / "outputs"
    / "slam"
    / "state"
    / "nidar_slam_v5_1_state.json"
)

# 2026-11-02e: with NIDAR_SLAM=V6 (set by start_nidar_pi.sh) show SLAM V6's state;
# its JSON uses the same frame/pose/statistics keys.
if os.environ.get("NIDAR_SLAM", "V5").strip().upper() == "V6":
    SLAM_FILE = SLAM_FILE.with_name("nidar_slam_v6_state.json")

# ------------------------------------------------------------
# LOG FILES
# ------------------------------------------------------------

LOG_DIR = BASE_DIR / "logs"

RECEIVER_LOG = LOG_DIR / "receiver.log"
V18_LOG = LOG_DIR / "v18.log"
SLAM_LOG = LOG_DIR / "slam.log"


# ------------------------------------------------------------
# MONITOR SETTINGS
# ------------------------------------------------------------

MONITOR_INTERVAL = 1.0

PERCEPTION_LIVE_SEC = 3.0
PERCEPTION_STALE_SEC = 7.0

V18_LIVE_SEC = 3.0
V18_STALE_SEC = 7.0

SLAM_LIVE_SEC = 3.0
SLAM_STALE_SEC = 7.0


# ============================================================
# TERMINAL HELPERS
# ============================================================

def clear_screen():
    print("\033[2J\033[H", end="")


def line(char="=", length=100):
    print(char * length)


def safe_float(value, default=0.0):

    try:
        return float(value)

    except (TypeError, ValueError):
        return default


def safe_int(value, default=0):

    try:
        return int(value)

    except (TypeError, ValueError):
        return default


# ============================================================
# JSON READER
# ============================================================

def read_json(path):

    try:

        with path.open(
            "r",
            encoding="utf-8"
        ) as f:

            return json.load(f)

    except (
        FileNotFoundError,
        json.JSONDecodeError,
        OSError,
        ValueError
    ):

        return None


# ============================================================
# PROCESS CHECK
# ============================================================

def process_running(pattern):

    try:

        result = subprocess.run(
            [
                "pgrep",
                "-f",
                pattern
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        return result.returncode == 0

    except Exception:

        return False


def process_status():

    return {

        "receiver":
            process_running(
                "src/communication/nidar_pi_receiver.py"
            ),

        "v18":
            process_running(
                "src/perception/nidar_v18_headless_persistent_fusion.py"
            ),

        "slam":
            process_running(
                "src.core.nidar_slam_v5_1"
            ),

    }


# ============================================================
# FILE AGE
# ============================================================

def file_age(path):

    try:

        modified = path.stat().st_mtime

        return max(
            0.0,
            time.time() - modified
        )

    except (
        FileNotFoundError,
        OSError
    ):

        return None


# ============================================================
# PERCEPTION STATUS
# ============================================================

def perception_status(data):

    if data is None:

        return {

            "frame": None,
            "objects": 0,
            "age": file_age(PERCEPTION_FILE),
            "status": "NO DATA",

        }

    frame = data.get(
        "frame",
        -1
    )

    objects = data.get(
        "objects",
        []
    )

    if not isinstance(objects, list):

        objects = []

    timestamp = data.get(
        "timestamp"
    )

    age = None

    if timestamp is not None:

        try:

            age = max(
                0.0,
                time.time() - float(timestamp)
            )

        except (
            TypeError,
            ValueError
        ):

            age = None

    if age is None:

        age = file_age(
            PERCEPTION_FILE
        )

    if age is None:

        status = "NO DATA"

    elif age < PERCEPTION_LIVE_SEC:

        status = "LIVE"

    elif age < PERCEPTION_STALE_SEC:

        status = "STALE"

    else:

        status = "OFFLINE"

    return {

        "frame": frame,
        "objects": len(objects),
        "age": age,
        "status": status,
        "objects_data": objects,

    }


# ============================================================
# V18 STATUS
# ============================================================

def v18_status(data):

    if data is None:

        return {

            "frame": None,
            "objects": 0,
            "active": 0,
            "temp_lost": 0,
            "predicted": 0,
            "age": file_age(V18_FILE),
            "status": "NO DATA",
            "objects_data": [],

        }

    frame = data.get(
        "frame",
        -1
    )

    # --------------------------------------------------------
    # V18 normally stores persistent objects.
    # Support several possible field names so the monitor
    # remains compatible with previous V18 revisions.
    # --------------------------------------------------------

    objects = data.get(
        "persistent_objects"
    )

    if objects is None:

        objects = data.get(
            "objects",
            []
        )

    if not isinstance(objects, list):

        objects = []

    active = 0
    temp_lost = 0
    predicted = 0

    for obj in objects:

        state = str(
            obj.get(
                "state",
                ""
            )
        ).upper()

        if state == "ACTIVE":

            active += 1

        elif state == "TEMP_LOST":

            temp_lost += 1

        elif state == "PREDICTED":

            predicted += 1

    timestamp = data.get(
        "timestamp"
    )

    age = None

    if timestamp is not None:

        try:

            age = max(
                0.0,
                time.time() - float(timestamp)
            )

        except (
            TypeError,
            ValueError
        ):

            age = None

    if age is None:

        age = file_age(
            V18_FILE
        )

    if age is None:

        status = "NO DATA"

    elif age < V18_LIVE_SEC:

        status = "LIVE"

    elif age < V18_STALE_SEC:

        status = "STALE"

    else:

        status = "OFFLINE"

    return {

        "frame": frame,
        "objects": len(objects),
        "active": active,
        "temp_lost": temp_lost,
        "predicted": predicted,
        "age": age,
        "status": status,
        "objects_data": objects,

    }


def fmt_num(value, spec, missing="---"):
    """Format a number, or show `missing` when it is None (2026-11-02)."""
    if value is None:
        return missing
    try:
        return format(value, spec)
    except (TypeError, ValueError):
        return missing


# ============================================================
# SLAM STATUS
# ============================================================

def slam_status(data):

    if data is None:

        return {

            "frame": None,
            "x": None,
            "y": None,
            "yaw": None,
            "fps": None,
            "age": file_age(SLAM_FILE),
            "status": "NO DATA",

        }

    frame = data.get(
        "frame",
        -1
    )

    pose = data.get(
        "pose",
        {}
    )

    statistics = data.get(
        "statistics",
        {}
    )

    if not isinstance(
        pose,
        dict
    ):

        pose = {}

    if not isinstance(
        statistics,
        dict
    ):

        statistics = {}

    x = pose.get(
        "x_m",
        0.0
    )

    y = pose.get(
        "y_m",
        0.0
    )

    yaw = pose.get(
        "yaw_deg",
        0.0
    )

    fps = statistics.get(
        "processing_fps",
        0.0
    )

    timestamp = data.get(
        "timestamp"
    )

    age = None

    if timestamp is not None:

        try:

            age = max(
                0.0,
                time.time() - float(timestamp)
            )

        except (
            TypeError,
            ValueError
        ):

            age = None

    if age is None:

        age = file_age(
            SLAM_FILE
        )

    if age is None:

        status = "NO DATA"

    elif age < SLAM_LIVE_SEC:

        status = "LIVE"

    elif age < SLAM_STALE_SEC:

        status = "STALE"

    else:

        status = "OFFLINE"

    return {

        "frame": frame,
        "x": safe_float(x),
        "y": safe_float(y),
        "yaw": safe_float(yaw),
        "fps": safe_float(fps),
        "age": age,
        "status": status,

    }


# ============================================================
# OBJECT DISPLAY
# ============================================================

def print_v18_objects(objects):

    if not objects:

        print(
            "  Objects          : none"
        )

        return

    print(
        f"  Objects          : {len(objects)}"
    )

    for obj in objects[:10]:

        stable_id = obj.get(
            "stable_id",
            obj.get(
                "id",
                "?"
            )
        )

        obj_class = obj.get(
            "class",
            "unknown"
        )

        state = obj.get(
            "state",
            "?"
        )

        range_m = safe_float(
            obj.get(
                "range_m",
                0.0
            )
        )

        bearing = safe_float(
            obj.get(
                "bearing_deg",
                0.0
            )
        )

        x = safe_float(
            obj.get(
                "x_m",
                0.0
            )
        )

        y = safe_float(
            obj.get(
                "y_m",
                0.0
            )
        )

        speed = safe_float(
            obj.get(
                "speed_mps",
                0.0
            )
        )

        confidence = safe_float(
            obj.get(
                "fusion_confidence",
                obj.get(
                    "confidence",
                    0.0
                )
            )
        )

        outliers = safe_int(
            obj.get(
                "outlier_count",
                0
            )
        )

        print(
            f"    S{stable_id:<4} "
            f"{str(obj_class):<12} "
            f"{str(state):<10} "
            f"R={range_m:>5.2f}m "
            f"B={bearing:+6.2f}° "
            f"X={x:+5.2f} "
            f"Y={y:+5.2f} "
            f"V={speed:>4.2f}m/s "
            f"C={confidence:.2f} "
            f"O={outliers}"
        )

    if len(objects) > 10:

        print(
            f"    ... {len(objects) - 10} more objects"
        )


# ============================================================
# INTEGRATION DECISION
# ============================================================

def integration_status(
    perception,
    v18,
    slam,
    processes
):

    if not processes["receiver"]:

        return "RECEIVER PROCESS DOWN"

    if not processes["v18"]:

        return "V18 PROCESS DOWN"

    if not processes["slam"]:

        return "SLAM PROCESS DOWN"

    if (
        perception["status"] == "LIVE"
        and
        v18["status"] == "LIVE"
        and
        slam["status"] == "LIVE"
    ):

        return "FULL NIDAR INTEGRATION LIVE"

    if (
        perception["status"] == "LIVE"
        and
        v18["status"] == "LIVE"
        and
        slam["status"] != "LIVE"
    ):

        return "PERCEPTION + V18 LIVE / SLAM NOT LIVE"

    if (
        perception["status"] == "LIVE"
        and
        slam["status"] == "LIVE"
        and
        v18["status"] != "LIVE"
    ):

        return "PERCEPTION + SLAM LIVE / V18 NOT LIVE"

    if (
        v18["status"] == "LIVE"
        and
        slam["status"] == "LIVE"
    ):

        return "V18 + SLAM LIVE / PERCEPTION NOT LIVE"

    return "SYSTEM NOT FULLY LIVE"


# ============================================================
# MAIN DASHBOARD
# ============================================================

def main():

    previous_perception_frame = None
    previous_v18_frame = None
    previous_slam_frame = None

    perception_updates = 0
    v18_updates = 0
    slam_updates = 0

    monitor_start = time.time()

    try:

        while True:

            # ------------------------------------------------
            # READ DATA
            # ------------------------------------------------

            perception_data = read_json(
                PERCEPTION_FILE
            )

            v18_data = read_json(
                V18_FILE
            )

            slam_data = read_json(
                SLAM_FILE
            )

            # ------------------------------------------------
            # PARSE STATUS
            # ------------------------------------------------

            perception = perception_status(
                perception_data
            )

            v18 = v18_status(
                v18_data
            )

            slam = slam_status(
                slam_data
            )

            processes = process_status()

            # ------------------------------------------------
            # FRAME UPDATE DETECTION
            # ------------------------------------------------

            perception_changed = False
            v18_changed = False
            slam_changed = False

            # PERCEPTION

            if perception["frame"] is not None:

                if (
                    previous_perception_frame is not None
                    and
                    perception["frame"]
                    !=
                    previous_perception_frame
                ):

                    perception_changed = True
                    perception_updates += 1

                previous_perception_frame = (
                    perception["frame"]
                )

            # V18

            if v18["frame"] is not None:

                if (
                    previous_v18_frame is not None
                    and
                    v18["frame"]
                    !=
                    previous_v18_frame
                ):

                    v18_changed = True
                    v18_updates += 1

                previous_v18_frame = (
                    v18["frame"]
                )

            # SLAM

            if slam["frame"] is not None:

                if (
                    previous_slam_frame is not None
                    and
                    slam["frame"]
                    !=
                    previous_slam_frame
                ):

                    slam_changed = True
                    slam_updates += 1

                previous_slam_frame = (
                    slam["frame"]
                )

            # ------------------------------------------------
            # CLEAR SCREEN
            # ------------------------------------------------

            clear_screen()

            # ------------------------------------------------
            # HEADER
            # ------------------------------------------------

            line()

            print(
                "        NIDAR V18 + SLAM V5.1 UNIFIED SYSTEM MONITOR"
            )

            line()

            print()
            print(
                f"Project : {BASE_DIR}"
            )

            print(
                f"Runtime : {time.time() - monitor_start:8.1f} s"
            )

            print()

            # ------------------------------------------------
            # PROCESS STATUS
            # ------------------------------------------------

            print(
                "PROCESS STATUS"
            )

            line("-")

            print(
                f"Receiver : "
                f"{'RUNNING' if processes['receiver'] else 'STOPPED'}"
            )

            print(
                f"V18      : "
                f"{'RUNNING' if processes['v18'] else 'STOPPED'}"
            )

            print(
                f"SLAM     : "
                f"{'RUNNING' if processes['slam'] else 'STOPPED'}"
            )

            print()

            # ------------------------------------------------
            # PERCEPTION
            # ------------------------------------------------

            print(
                "PERCEPTION / MAC → PI"
            )

            line("-")

            print(
                f"Status   : {perception['status']}"
            )

            print(
                f"Frame    : {str(perception['frame'])}"
            )

            print(
                f"Objects  : {perception['objects']}"
            )

            if perception["age"] is not None:

                print(
                    f"Age      : {perception['age']:.2f} s"
                )

            print(
                f"Update   : "
                f"{'YES' if perception_changed else 'NO'}"
            )

            print()

            # ------------------------------------------------
            # V18
            # ------------------------------------------------

            print(
                "NIDAR V18 CAMERA + LiDAR FUSION"
            )

            line("-")

            print(
                f"Status   : {v18['status']}"
            )

            print(
                f"Frame    : {str(v18['frame'])}"
            )

            print(
                f"Objects  : {v18['objects']}"
            )

            print(
                f"Active   : {v18['active']}"
            )

            print(
                f"TempLost : {v18['temp_lost']}"
            )

            print(
                f"Predicted: {v18['predicted']}"
            )

            if v18["age"] is not None:

                print(
                    f"Age      : {v18['age']:.2f} s"
                )

            print(
                f"Update   : "
                f"{'YES' if v18_changed else 'NO'}"
            )

            print()

            # ------------------------------------------------
            # V18 OBJECTS
            # ------------------------------------------------

            print(
                "V18 TRACKED OBJECTS"
            )

            line("-")

            print_v18_objects(
                v18["objects_data"]
            )

            print()

            # ------------------------------------------------
            # SLAM
            # ------------------------------------------------

            print(
                "NIDAR SLAM V5.1"
            )

            line("-")

            print(
                f"Status   : {slam['status']}"
            )

            print(
                f"Frame    : {str(slam['frame'])}"
            )

            # 2026-11-02: before SLAM has written its first state (fresh
            # outputs/ folder, or Gazebo LiDAR not yet sending) x/y/yaw are
            # None; formatting None with :+.3f crashed the monitor, and
            # start_nidar_pi.sh then stopped the WHOLE Pi system.
            print(
                f"Pose     : "
                f"X={fmt_num(slam['x'], '+.3f')} m  "
                f"Y={fmt_num(slam['y'], '+.3f')} m"
            )

            print(
                f"Yaw      : "
                f"{fmt_num(slam['yaw'], '+.2f')}°"
            )

            if slam["fps"] is not None:

                print(
                    f"SLAM FPS : "
                    f"{slam['fps']:.2f}"
                )

            if slam["age"] is not None:

                print(
                    f"Age      : "
                    f"{slam['age']:.2f} s"
                )

            print(
                f"Update   : "
                f"{'YES' if slam_changed else 'NO'}"
            )

            print()

            # ------------------------------------------------
            # INTEGRATION
            # ------------------------------------------------

            print(
                "SYSTEM INTEGRATION"
            )

            line("-")

            system_status = integration_status(
                perception,
                v18,
                slam,
                processes
            )

            print()
            print(
                f"STATUS: {system_status}"
            )
            print()

            # ------------------------------------------------
            # FRAME COUNTERS
            # ------------------------------------------------

            print(
                "UPDATE COUNTERS"
            )

            line("-")

            print(
                f"Perception updates : "
                f"{perception_updates}"
            )

            print(
                f"V18 updates        : "
                f"{v18_updates}"
            )

            print(
                f"SLAM updates       : "
                f"{slam_updates}"
            )

            print()

            # ------------------------------------------------
            # FILE LOCATIONS
            # ------------------------------------------------

            print(
                "DATA SOURCES"
            )

            line("-")

            print(
                f"Perception : "
                f"{PERCEPTION_FILE}"
            )

            print(
                f"V18        : "
                f"{V18_FILE}"
            )

            print(
                f"SLAM       : "
                f"{SLAM_FILE}"
            )

            print()

            # ------------------------------------------------
            # LOG LOCATIONS
            # ------------------------------------------------

            print(
                "LOG FILES"
            )

            line("-")

            print(
                f"Receiver : {RECEIVER_LOG}"
            )

            print(
                f"V18      : {V18_LOG}"
            )

            print(
                f"SLAM     : {SLAM_LOG}"
            )

            print()

            line()

            print(
                "Press Ctrl+C to stop monitor"
            )

            line()

            time.sleep(
                MONITOR_INTERVAL
            )

    except KeyboardInterrupt:

        clear_screen()

        line()

        print(
            "             NIDAR INTEGRATION MONITOR STOPPED"
        )

        line()

        runtime = (
            time.time()
            -
            monitor_start
        )

        print()

        print(
            f"Monitor runtime      : "
            f"{runtime:.1f} s"
        )

        print(
            f"Perception updates   : "
            f"{perception_updates}"
        )

        print(
            f"V18 updates          : "
            f"{v18_updates}"
        )

        print(
            f"SLAM updates         : "
            f"{slam_updates}"
        )

        print()

        line()

        return 0


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    raise SystemExit(
        main()
    )

