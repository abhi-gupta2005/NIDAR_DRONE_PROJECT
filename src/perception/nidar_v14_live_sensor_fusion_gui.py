#!/usr/bin/env python3
"""
NIDAR V14 - LIVE CAMERA + LiDAR SENSOR FUSION GUI

Pipeline:

Mac Camera
    ↓
NIDAR JSON
    ↓
Pi Receiver
    ↓
Camera Detection
    +
Simulated STL-19P LiDAR
    ↓
LiDAR Obstacle Detector
    ↓
LiDAR Tracker V3
    ↓
CameraLidarFusion
    ↓
PyQt5 / pyqtgraph GUI

The LiDAR source is currently simulated.
The detector, tracker and fusion interfaces are kept compatible
with the future physical LiDAR backend.
"""

import json
import math
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg


PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))


# ------------------------------------------------------------
# Existing LiDAR components
# ------------------------------------------------------------

from lidar_interface import (
    LidarInterfaceConfig,
    SimulatedSTL19PBackend,
)

from lidar_obstacle_detector import (
    ObstacleDetectorConfig,
    LidarObstacleDetector,
)

from lidar_obstacle_tracker_v3 import (
    TrackerV3Config,
    LidarObstacleTrackerV3,
)


# ------------------------------------------------------------
# New validated Camera + LiDAR fusion
# ------------------------------------------------------------

from nidar_camera_lidar_fusion_v1 import (
    CameraLidarFusion,
    LidarTrackInput,
    camera_detection_from_nidar_record,
)


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

PACKET_PATH = PROJECT_DIR / "nidar_latest_perception.json"

MAX_DISPLAY_RANGE_M = 12.0
UPDATE_HZ = 10.0


# ------------------------------------------------------------
# Utility helpers
# ------------------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def attr(obj, name, default=None):
    """
    Read an attribute from either an object or dictionary.

    Supports both:
        obj.field
    and:
        obj["field"]
    """

    if isinstance(obj, dict):
        return obj.get(name, default)

    return getattr(obj, name, default)


def angle_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------
# Camera bearing
# ------------------------------------------------------------

def camera_bearing(obj):
    """
    Calculate camera bearing from NIDAR camera data.

    Supported schemas:

        center: [x_px, y_px]

        normalized_center: [x_norm, y_norm]

        bbox: [x1, y1, x2, y2]
    """

    try:

        width = float(
            obj.get(
                "_camera_width",
                obj.get("camera_width", 1280),
            )
        )

        # Preferred normalized center
        normalized = obj.get("normalized_center")

        if (
            isinstance(normalized, (list, tuple))
            and len(normalized) >= 1
        ):

            x_norm = float(normalized[0])

            # Normalized x expected around 0..1
            x_norm = clamp(x_norm, 0.0, 1.0)

            return (x_norm - 0.5) * 70.0

        # Pixel center
        center = obj.get("center")

        if (
            isinstance(center, (list, tuple))
            and len(center) >= 1
        ):

            x_px = float(center[0])

            x_norm = x_px / width

            return (
                clamp(x_norm, 0.0, 1.0) - 0.5
            ) * 70.0

        # Bounding box fallback
        bbox = obj.get("bbox")

        if (
            isinstance(bbox, (list, tuple))
            and len(bbox) >= 4
        ):

            x1 = float(bbox[0])
            x2 = float(bbox[2])

            x_px = (x1 + x2) / 2.0

            x_norm = x_px / width

            return (
                clamp(x_norm, 0.0, 1.0) - 0.5
            ) * 70.0

    except (
        TypeError,
        ValueError,
        IndexError,
        ZeroDivisionError,
    ):
        pass

    return None


# ------------------------------------------------------------
# LiDAR track bearing
# ------------------------------------------------------------

def track_bearing(track):

    value = attr(
        track,
        "bearing_deg",
        attr(
            track,
            "bearing",
            0.0,
        ),
    )

    try:
        return float(value)
    except (
        TypeError,
        ValueError,
    ):
        return 0.0


# ------------------------------------------------------------
# Read latest camera packet
# ------------------------------------------------------------

def read_packet():

    try:

        with PACKET_PATH.open(
            "r",
            encoding="utf-8",
        ) as f:

            return json.load(f)

    except (
        FileNotFoundError,
        json.JSONDecodeError,
        OSError,
    ):

        return None


# ------------------------------------------------------------
# Camera objects for GUI
# ------------------------------------------------------------

def camera_objects(packet):

    if packet is None:
        return []

    raw = packet.get(
        "objects",
        [],
    )

    if not isinstance(raw, list):
        return []

    camera = packet.get(
        "camera",
        {},
    )

    width = camera.get(
        "width",
        packet.get(
            "camera_width",
            1280,
        ),
    )

    height = camera.get(
        "height",
        packet.get(
            "camera_height",
            720,
        ),
    )

    result = []

    for item in raw:

        if not isinstance(item, dict):
            continue

        obj = dict(item)

        obj["_camera_width"] = width
        obj["_camera_height"] = height

        result.append(obj)

    return result


# ------------------------------------------------------------
# Convert camera objects to validated fusion objects
# ------------------------------------------------------------

def build_camera_detections(packet):

    if packet is None:
        return []

    camera = packet.get(
        "camera",
        {},
    )

    width = int(
        camera.get(
            "width",
            packet.get(
                "camera_width",
                1280,
            ),
        )
    )

    height = int(
        camera.get(
            "height",
            packet.get(
                "camera_height",
                720,
            ),
        )
    )

    frame = int(
        packet.get(
            "frame",
            0,
        )
    )

    timestamp = float(
        packet.get(
            "timestamp",
            time.time(),
        )
    )

    detections = []

    for record in packet.get(
        "objects",
        [],
    ):

        if not isinstance(record, dict):
            continue

        try:

            detection = camera_detection_from_nidar_record(
                record=record,
                image_width=width,
                image_height=height,
                frame=frame,
                timestamp=timestamp,
            )

        except Exception:
            detection = None

        if detection is not None:
            detections.append(
                detection
            )

    return detections


# ------------------------------------------------------------
# Convert LiDAR tracker output to fusion input
# ------------------------------------------------------------

def build_lidar_inputs(tracks):

    inputs = []

    for track in tracks:

        try:

            inputs.append(
                LidarTrackInput(
                    track_id=int(
                        track.track_id
                    ),

                    x_m=float(
                        track.x_m
                    ),

                    y_m=float(
                        track.y_m
                    ),

                    distance_m=float(
                        track.distance_m
                    ),

                    bearing_deg=float(
                        track.bearing_deg
                    ),

                    speed_mps=float(
                        track.speed_mps
                    ),

                    state=str(
                        track.state
                    ),

                    observation_state=str(
                        track.observation_state
                    ),

                    motion_state=str(
                        track.motion_state
                    ),

                    confidence=float(
                        track.confidence
                    ),

                    track_quality=float(
                        track.track_quality
                    ),

                    missed_scans=int(
                        track.missed_scans
                    ),
                )
            )

        except Exception:
            continue

    return inputs


# ------------------------------------------------------------
# Convert LiDAR scan to XY points
# ------------------------------------------------------------

def scan_xy(scan):

    if scan is None:
        return []

    try:

        angles = scan.angle_deg
        distances = scan.distance_m
        valid = scan.valid

    except AttributeError:

        return []

    points = []

    n = min(
        len(angles),
        len(distances),
    )

    for i in range(n):

        try:

            if valid is not None:

                if not bool(
                    valid[i]
                ):
                    continue

            angle = math.radians(
                float(
                    angles[i]
                )
            )

            distance = float(
                distances[i]
            )

            if not (
                0.05
                <= distance
                <= MAX_DISPLAY_RANGE_M
            ):
                continue

            x = distance * math.cos(
                angle
            )

            y = distance * math.sin(
                angle
            )

            points.append(
                (x, y)
            )

        except (
            TypeError,
            ValueError,
            IndexError,
        ):
            continue

    return points


# ============================================================
# Fusion Engine
# ============================================================

class FusionEngine:

    def __init__(self):

        cfg = LidarInterfaceConfig(
            scan_rate_hz=10.0,
            measurements_per_second=5000,
            max_range_m=12.0,
            min_range_m=0.05,
        )

        self.backend = SimulatedSTL19PBackend(
            cfg
        )

        self.detector = LidarObstacleDetector(
            ObstacleDetectorConfig()
        )

        self.tracker = LidarObstacleTrackerV3(
            TrackerV3Config()
        )

        self.fusion = CameraLidarFusion()

        self.last_packet_frame = None
        self.last_packet_time = 0.0

        self.camera_ok = False
        self.lidar_ok = False
        self.detection_ok = False
        self.tracking_ok = False
        self.fusion_ok = False

    def update(self):

        # ----------------------------------------------------
        # Camera packet
        # ----------------------------------------------------

        packet = read_packet()

        self.camera_ok = (
            packet is not None
        )

        if packet is None:

            return {
                "packet": None,
                "cameras": [],
                "camera_detections": [],
                "points": [],
                "detections": [],
                "tracks": [],
                "fused": [],
            }

        cameras = camera_objects(
            packet
        )

        camera_detections = build_camera_detections(
            packet
        )

        self.last_packet_frame = packet.get(
            "frame",
            -1,
        )

        self.last_packet_time = time.time()

        # ----------------------------------------------------
        # LiDAR
        # ----------------------------------------------------

        scan = self.backend.next_scan()

        self.lidar_ok = (
            scan is not None
        )

        if scan is None:

            return {
                "packet": packet,
                "cameras": cameras,
                "camera_detections": camera_detections,
                "points": [],
                "detections": [],
                "tracks": [],
                "fused": [],
            }

        # ----------------------------------------------------
        # Obstacle detection
        # ----------------------------------------------------

        detections = self.detector.detect(
            scan
        )

        self.detection_ok = True

        # ----------------------------------------------------
        # LiDAR tracking
        # ----------------------------------------------------

        try:

            tracks = self.tracker.update(
                detections,
                time.time(),
            )

        except TypeError:

            tracks = self.tracker.update(
                detections
            )

        self.tracking_ok = True

        # ----------------------------------------------------
        # Convert tracks for fusion
        # ----------------------------------------------------

        lidar_inputs = build_lidar_inputs(
            tracks
        )

        # ----------------------------------------------------
        # Camera + LiDAR fusion
        # ----------------------------------------------------

        try:

            fused = self.fusion.associate(
                camera_detections,
                lidar_inputs,
            )

            self.fusion_ok = True

        except Exception:

            fused = []

            self.fusion_ok = False

        # ----------------------------------------------------
        # Radar points
        # ----------------------------------------------------

        points = scan_xy(
            scan
        )

        return {
            "packet": packet,
            "cameras": cameras,
            "camera_detections": camera_detections,
            "points": points,
            "detections": detections,
            "tracks": tracks,
            "fused": fused,
        }

    def close(self):

        try:
            self.backend.close()
        except Exception:
            pass


# ============================================================
# Radar Widget
# ============================================================

class RadarWidget(pg.PlotWidget):

    def __init__(self):

        super().__init__()

        self.setBackground(
            "#05080c"
        )

        self.showGrid(
            x=True,
            y=True,
            alpha=0.15,
        )

        self.setAspectLocked(
            True
        )

        self.setXRange(
            -MAX_DISPLAY_RANGE_M,
            MAX_DISPLAY_RANGE_M,
        )

        self.setYRange(
            -MAX_DISPLAY_RANGE_M,
            MAX_DISPLAY_RANGE_M,
        )

        self.point_item = pg.ScatterPlotItem(
            size=4
        )

        self.track_item = pg.ScatterPlotItem(
            size=12
        )

        self.fused_item = pg.ScatterPlotItem(
            size=18,
            pen=pg.mkPen(
                "w",
                width=2,
            ),
        )

        self.addItem(
            self.point_item
        )

        self.addItem(
            self.track_item
        )

        self.addItem(
            self.fused_item
        )

        # Vehicle center
        self.addItem(
            pg.ScatterPlotItem(
                [0],
                [0],
                size=18,
                pen=pg.mkPen(
                    255,
                    255,
                    255,
                    240,
                    width=2,
                ),
                brush=pg.mkBrush(
                    30,
                    30,
                    30,
                    255,
                ),
            )
        )

    def update_data(
        self,
        points,
        tracks,
        fused=None,
    ):

        # ----------------------------------------------------
        # Raw LiDAR points
        # ----------------------------------------------------

        pts = [
            (x, y)
            for x, y in points
            if math.hypot(
                x,
                y,
            ) <= MAX_DISPLAY_RANGE_M
        ]

        if pts:

            self.point_item.setData(
                [p[0] for p in pts],
                [p[1] for p in pts],
            )

        else:

            self.point_item.clear()

        # ----------------------------------------------------
        # LiDAR tracks
        # ----------------------------------------------------

        track_xy = []

        for tr in tracks:

            state = str(
                attr(
                    tr,
                    "state",
                    "",
                )
            )

            if state not in (
                "CONFIRMED",
                "RECOVERED",
            ):
                continue

            x = attr(
                tr,
                "x_m",
                attr(
                    tr,
                    "x",
                    None,
                ),
            )

            y = attr(
                tr,
                "y_m",
                attr(
                    tr,
                    "y",
                    None,
                ),
            )

            if (
                x is None
                or y is None
            ):

                distance = float(
                    attr(
                        tr,
                        "distance_m",
                        attr(
                            tr,
                            "distance",
                            0.0,
                        ),
                    )
                )

                bearing = track_bearing(
                    tr
                )

                angle = math.radians(
                    bearing
                )

                x = (
                    distance
                    * math.cos(angle)
                )

                y = (
                    distance
                    * math.sin(angle)
                )

            if math.hypot(
                float(x),
                float(y),
            ) <= MAX_DISPLAY_RANGE_M:

                track_xy.append(
                    (
                        float(x),
                        float(y),
                    )
                )

        if track_xy:

            self.track_item.setData(
                [p[0] for p in track_xy],
                [p[1] for p in track_xy],
            )

        else:

            self.track_item.clear()

        # ----------------------------------------------------
        # Fused objects
        # ----------------------------------------------------

        fused_xy = []

        if fused:

            for obj in fused:

                try:

                    x = float(
                        obj.x_m
                    )

                    y = float(
                        obj.y_m
                    )

                    if math.hypot(
                        x,
                        y,
                    ) <= MAX_DISPLAY_RANGE_M:

                        fused_xy.append(
                            (x, y)
                        )

                except (
                    TypeError,
                    ValueError,
                    AttributeError,
                ):
                    continue

        if fused_xy:

            self.fused_item.setData(
                [p[0] for p in fused_xy],
                [p[1] for p in fused_xy],
            )

        else:

            self.fused_item.clear()


# ============================================================
# Main Window
# ============================================================

class MainWindow(
    QtWidgets.QMainWindow
):

    def __init__(self):

        super().__init__()

        self.setWindowTitle(
            "NIDAR V14 — Live Camera + STL-19P Sensor Fusion"
        )

        self.resize(
            1280,
            760,
        )

        self.engine = FusionEngine()

        self.last_update = time.monotonic()

        self.gui_fps = 0.0

        # ----------------------------------------------------
        # Layout
        # ----------------------------------------------------

        root = QtWidgets.QWidget()

        self.setCentralWidget(
            root
        )

        layout = QtWidgets.QGridLayout(
            root
        )

        layout.setContentsMargins(
            10,
            10,
            10,
            10,
        )

        layout.setSpacing(
            10
        )

        title = QtWidgets.QLabel(
            "NIDAR V14  •  LIVE SENSOR FUSION"
        )

        title.setStyleSheet(
            "font-size: 22px; font-weight: 700;"
        )

        layout.addWidget(
            title,
            0,
            0,
            1,
            2,
        )

        # ----------------------------------------------------
        # Radar
        # ----------------------------------------------------

        self.radar = RadarWidget()

        layout.addWidget(
            self.radar,
            1,
            0,
            3,
            1,
        )

        # ----------------------------------------------------
        # Right side
        # ----------------------------------------------------

        right = QtWidgets.QVBoxLayout()

        layout.addLayout(
            right,
            1,
            1,
        )

        self.status = QtWidgets.QLabel()

        self.status.setStyleSheet(
            "font-size: 14px; "
            "padding: 8px; "
            "background:#101820;"
        )

        right.addWidget(
            self.status
        )

        self.stats = QtWidgets.QLabel()

        self.stats.setStyleSheet(
            "font-size: 13px;"
        )

        right.addWidget(
            self.stats
        )

        right.addWidget(
            QtWidgets.QLabel(
                "FUSED OBJECTS"
            )
        )

        # ----------------------------------------------------
        # Fusion table
        # ----------------------------------------------------

        self.fusion_table = QtWidgets.QTableWidget(
            0,
            9,
        )

        self.fusion_table.setHorizontalHeaderLabels(
            [
                "Cam",
                "LiDAR",
                "Class",
                "Bearing",
                "Range",
                "Speed",
                "Error",
                "Confidence",
                "State",
            ]
        )

        self.fusion_table.horizontalHeader().setStretchLastSection(
            True
        )

        self.fusion_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )

        right.addWidget(
            self.fusion_table,
            1,
        )

        # ----------------------------------------------------
        # LiDAR table
        # ----------------------------------------------------

        right.addWidget(
            QtWidgets.QLabel(
                "LiDAR TRACKS"
            )
        )

        self.track_table = QtWidgets.QTableWidget(
            0,
            6,
        )

        self.track_table.setHorizontalHeaderLabels(
            [
                "ID",
                "State",
                "Bearing",
                "Range",
                "Speed",
                "Quality",
            ]
        )

        self.track_table.horizontalHeader().setStretchLastSection(
            True
        )

        self.track_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )

        right.addWidget(
            self.track_table,
            1,
        )

        # ----------------------------------------------------
        # Timer
        # ----------------------------------------------------

        self.timer = QtCore.QTimer(
            self
        )

        self.timer.timeout.connect(
            self.tick
        )

        self.timer.start(
            int(
                1000 / UPDATE_HZ
            )
        )

    # --------------------------------------------------------
    # Table helper
    # --------------------------------------------------------

    def set_table(
        self,
        table,
        rows,
    ):

        table.setRowCount(
            len(rows)
        )

        for r, row in enumerate(
            rows
        ):

            for c, value in enumerate(
                row
            ):

                table.setItem(
                    r,
                    c,
                    QtWidgets.QTableWidgetItem(
                        str(value)
                    ),
                )

    # --------------------------------------------------------
    # GUI update
    # --------------------------------------------------------

    def tick(self):

        try:

            data = self.engine.update()

            packet = data["packet"]

            cameras = data["cameras"]

            points = data["points"]

            tracks = data["tracks"]

            fused = data["fused"]

            # ------------------------------------------------
            # Radar
            # ------------------------------------------------

            self.radar.update_data(
                points,
                tracks,
                fused,
            )

            # ------------------------------------------------
            # Status
            # ------------------------------------------------

            frame = (
                packet.get(
                    "frame",
                    "-",
                )
                if packet
                else "-"
            )

            camera_status = (
                "ONLINE"
                if self.engine.camera_ok
                else "WAITING"
            )

            lidar_status = (
                "ONLINE"
                if self.engine.lidar_ok
                else "OFFLINE"
            )

            fusion_status = (
                "ACTIVE"
                if self.engine.fusion_ok
                and fused
                else (
                    "READY"
                    if self.engine.fusion_ok
                    else "ERROR"
                )
            )

            self.status.setText(
                f"<b>CAMERA:</b> {camera_status}"
                f" &nbsp;&nbsp; "
                f"<b>LiDAR:</b> {lidar_status}"
                f" &nbsp;&nbsp; "
                f"<b>FUSION:</b> {fusion_status}"
            )

            # ------------------------------------------------
            # FPS
            # ------------------------------------------------

            now = time.monotonic()

            dt = (
                now
                - self.last_update
            )

            if dt > 0:

                self.gui_fps = (
                    0.9
                    * self.gui_fps
                    + 0.1 / dt
                )

            self.last_update = now

            # ------------------------------------------------
            # Confirmed tracks
            # ------------------------------------------------

            confirmed = sum(
                1
                for track in tracks
                if str(
                    attr(
                        track,
                        "state",
                        "",
                    )
                )
                in (
                    "CONFIRMED",
                    "RECOVERED",
                )
            )

            self.stats.setText(
                f"Frame: <b>{frame}</b> &nbsp; "
                f"Camera objects: <b>{len(cameras)}</b> &nbsp; "
                f"LiDAR tracks: <b>{len(tracks)}</b> &nbsp; "
                f"Confirmed: <b>{confirmed}</b> &nbsp; "
                f"Fusion objects: <b>{len(fused)}</b> &nbsp; "
                f"GUI: <b>{self.gui_fps:.1f} Hz</b>"
            )

            # ------------------------------------------------
            # Fusion table
            # ------------------------------------------------

            fusion_rows = []

            for obj in fused:

                fusion_rows.append(
                    [
                        obj.camera_id,
                        obj.lidar_id,
                        obj.class_name,

                        (
                            f"{obj.bearing_lidar_deg:+.1f}°"
                        ),

                        (
                            f"{obj.distance_m:.2f} m"
                        ),

                        (
                            f"{obj.speed_mps:.2f} m/s"
                        ),

                        (
                            f"{obj.bearing_error_deg:.1f}°"
                        ),

                        (
                            f"{obj.fusion_confidence:.2f}"
                        ),

                        (
                            f"{obj.motion_state}"
                        ),
                    ]
                )

            self.set_table(
                self.fusion_table,
                fusion_rows,
            )

            # ------------------------------------------------
            # LiDAR tracks table
            # ------------------------------------------------

            track_rows = []

            for track in tracks:

                state = str(
                    attr(
                        track,
                        "state",
                        "",
                    )
                )

                if state not in (
                    "CONFIRMED",
                    "RECOVERED",
                    "TEMP_LOST",
                ):
                    continue

                quality = attr(
                    track,
                    "track_quality",
                    attr(
                        track,
                        "quality",
                        0.0,
                    ),
                )

                distance = attr(
                    track,
                    "distance_m",
                    attr(
                        track,
                        "distance",
                        0.0,
                    ),
                )

                speed = attr(
                    track,
                    "speed_mps",
                    attr(
                        track,
                        "speed",
                        0.0,
                    ),
                )

                track_rows.append(
                    [
                        attr(
                            track,
                            "track_id",
                            "?",
                        ),

                        state,

                        (
                            f"{track_bearing(track):+.1f}°"
                        ),

                        (
                            f"{float(distance):.2f} m"
                        ),

                        (
                            f"{float(speed):.2f} m/s"
                        ),

                        (
                            f"{float(quality):.2f}"
                        ),
                    ]
                )

            self.set_table(
                self.track_table,
                track_rows,
            )

        except Exception as exc:

            self.status.setText(
                "<b>PIPELINE ERROR:</b> "
                f"{type(exc).__name__}: {exc}"
            )

    # --------------------------------------------------------
    # Close
    # --------------------------------------------------------

    def closeEvent(
        self,
        event,
    ):

        self.engine.close()

        event.accept()


# ============================================================
# Main
# ============================================================

def main():

    app = QtWidgets.QApplication(
        sys.argv
    )

    app.setStyle(
        "Fusion"
    )

    win = MainWindow()

    win.show()

    sys.exit(
        app.exec_()
    )


if __name__ == "__main__":
    main()
