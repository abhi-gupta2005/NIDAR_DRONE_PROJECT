
#!/usr/bin/env python3
"""
NIDAR V12.1 - GUI LiDAR Simulator
PySide6 + PyQtGraph

Simulates the logical output of the LDROBOT STL-19P:
    360 degree scan
    10 Hz scan rate
    ~500 points/scan
    noisy distance measurements

The GUI is intentionally built as a development tool for NIDAR.
It does NOT emulate proprietary raw UART bytes.

Controls:
    Start / Pause
    Reset
    Noise slider
    Simulation speed
    Show room / obstacles
    Show rays
    Max range

Install:
    python3 -m pip install pyside6 pyqtgraph numpy

Run:
    python3 nidar_v12_gui.py
"""

import math
import random
import sys
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets


SCAN_HZ = 10.0
MEASUREMENTS_PER_SECOND = 5000
SAMPLES_PER_SCAN = int(MEASUREMENTS_PER_SECOND / SCAN_HZ)
MAX_RANGE_M = 12.0

ROOM_W = 8.0
ROOM_H = 6.0

SENSOR_X = ROOM_W / 2
SENSOR_Y = ROOM_H / 2

PERSON_RADIUS = 0.28
PERSON_START = np.array([4.8, 2.0])
PERSON_END = np.array([3.2, 4.2])
PERSON_PERIOD = 12.0


@dataclass
class CircleObstacle:
    name: str
    x: float
    y: float
    radius: float


STATIC_OBSTACLES = [
    CircleObstacle("Chair", 2.0, 1.7, 0.35),
    CircleObstacle("Box", 6.1, 1.4, 0.45),
    CircleObstacle("Pillar", 5.8, 4.7, 0.30),
]


def moving_person(t):
    phase = (t % PERSON_PERIOD) / PERSON_PERIOD
    s = 0.5 - 0.5 * math.cos(2 * math.pi * phase)

    x = PERSON_START[0] + (PERSON_END[0] - PERSON_START[0]) * s
    y = PERSON_START[1] + (PERSON_END[1] - PERSON_START[1]) * s

    return CircleObstacle("Person", float(x), float(y), PERSON_RADIUS)


def ray_to_wall(angle):
    dx = math.cos(angle)
    dy = math.sin(angle)
    distances = []

    if dx > 1e-9:
        distances.append((ROOM_W - SENSOR_X) / dx)
    elif dx < -1e-9:
        distances.append((0 - SENSOR_X) / dx)

    if dy > 1e-9:
        distances.append((ROOM_H - SENSOR_Y) / dy)
    elif dy < -1e-9:
        distances.append((0 - SENSOR_Y) / dy)

    positive = [d for d in distances if d > 0]
    return min(positive) if positive else MAX_RANGE_M


def ray_circle(angle, obstacle):
    ox = SENSOR_X - obstacle.x
    oy = SENSOR_Y - obstacle.y
    dx = math.cos(angle)
    dy = math.sin(angle)

    b = 2 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - obstacle.radius * obstacle.radius
    disc = b * b - 4 * c

    if disc < 0:
        return None

    root = math.sqrt(disc)
    t1 = (-b - root) / 2
    t2 = (-b + root) / 2

    valid = [t for t in (t1, t2) if t > 0]
    return min(valid) if valid else None


def make_scan(t, noise_scale=1.0):
    angles = np.linspace(0, 360, SAMPLES_PER_SCAN, endpoint=False)
    person = moving_person(t)
    obstacles = STATIC_OBSTACLES + [person]

    distances = np.empty(len(angles), dtype=float)
    hits = []

    for i, deg in enumerate(angles):
        angle = math.radians(float(deg))

        d = ray_to_wall(angle)
        hit = "Wall"

        for obstacle in obstacles:
            h = ray_circle(angle, obstacle)
            if h is not None and h < d:
                d = h
                hit = obstacle.name

        sigma = (0.006 + 0.0015 * d) * noise_scale
        d += random.gauss(0, sigma)
        d = max(0.03, min(MAX_RANGE_M, d))

        # occasional invalid measurement
        if random.random() < 0.008:
            d = np.nan
            hit = "Invalid"

        distances[i] = d
        hits.append(hit)

    return angles, distances, hits, person


class RadarPlot(pg.PlotWidget):
    def __init__(self):
        super().__init__()

        self.setBackground("#071018")
        self.setAspectLocked(True)
        self.hideAxis("left")
        self.hideAxis("bottom")
        self.setMouseEnabled(x=False, y=False)

        self.setXRange(-MAX_RANGE_M, MAX_RANGE_M, padding=0)
        self.setYRange(-MAX_RANGE_M, MAX_RANGE_M, padding=0)

        self.grid_pen = pg.mkPen((45, 65, 78), width=1)
        self.wall_pen = pg.mkPen((100, 130, 145), width=2)
        self.ray_pen = pg.mkPen((80, 180, 210), width=1)

        self.scan_curve = self.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=3,
            symbolBrush=(80, 220, 255),
        )

        self.predicted_person = pg.ScatterPlotItem(
            size=14,
            brush=pg.mkBrush(255, 170, 60),
            pen=pg.mkPen(255, 220, 120, width=1),
        )
        self.addItem(self.predicted_person)

        self.sensor_point = pg.ScatterPlotItem(
            size=12,
            brush=pg.mkBrush(100, 255, 150),
            pen=pg.mkPen(220, 255, 220, width=2),
        )
        self.sensor_point.setData([0], [0])
        self.addItem(self.sensor_point)

        self._build_grid()

    def _build_grid(self):
        for r in (2, 4, 6, 8, 10, 12):
            theta = np.linspace(0, 2 * np.pi, 300)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            self.plot(x, y, pen=self.grid_pen)

        for deg in range(0, 360, 45):
            a = math.radians(deg)
            self.plot(
                [0, MAX_RANGE_M * math.cos(a)],
                [0, MAX_RANGE_M * math.sin(a)],
                pen=self.grid_pen,
            )

    def update_scan(self, angles, distances, show_rays):
        valid = np.isfinite(distances)
        a = np.radians(angles[valid])
        r = distances[valid]

        x = r * np.cos(a)
        y = r * np.sin(a)

        self.scan_curve.setData(x, y)

        if show_rays and len(x):
            # Keep the visualization light: display every 10th ray.
            idx = np.arange(0, len(x), 10)
            for item in getattr(self, "_ray_items", []):
                self.removeItem(item)

            self._ray_items = []
            for i in idx:
                item = pg.PlotDataItem(
                    [0, x[i]],
                    [0, y[i]],
                    pen=self.ray_pen,
                )
                self.addItem(item)
                self._ray_items.append(item)

    def update_person(self, person):
        x = person.x - SENSOR_X
        y = person.y - SENSOR_Y
        self.predicted_person.setData([x], [y])


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("NIDAR V12.1 — STL-19P LiDAR Simulation")
        self.resize(1400, 850)

        self.running = True
        self.sim_time = 0.0
        self.scan_id = 0
        self.last_real = time.monotonic()
        self.total_valid = 0
        self.total_points = 0

        root = QtWidgets.QWidget()
        self.setCentralWidget(root)

        layout = QtWidgets.QHBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 10)

        self.radar = RadarPlot()
        layout.addWidget(self.radar, 1)

        side = QtWidgets.QFrame()
        side.setMaximumWidth(330)
        side.setStyleSheet("""
            QFrame {
                background: #101820;
                border-radius: 10px;
            }
            QLabel {
                color: #dce8ee;
            }
            QPushButton {
                padding: 9px;
                background: #1d2c36;
                color: white;
                border: 1px solid #3a4d58;
                border-radius: 6px;
            }
            QPushButton:hover {
                background: #29404d;
            }
            QSlider::groove:horizontal {
                height: 5px;
                background: #344650;
            }
            QSlider::handle:horizontal {
                width: 14px;
                margin: -5px 0;
                background: #5ad9ff;
                border-radius: 7px;
            }
        """)

        side_layout = QtWidgets.QVBoxLayout(side)

        title = QtWidgets.QLabel("NIDAR LiDAR Simulator")
        title.setStyleSheet("font-size: 22px; font-weight: bold; color: #7fe4ff;")
        side_layout.addWidget(title)

        subtitle = QtWidgets.QLabel("STL-19P logical sensor simulation")
        subtitle.setStyleSheet("color: #91a7b2;")
        side_layout.addWidget(subtitle)

        side_layout.addSpacing(20)

        self.status = QtWidgets.QLabel("● RUNNING")
        self.status.setStyleSheet("font-size: 16px; font-weight: bold; color: #70f0a0;")
        side_layout.addWidget(self.status)

        self.start_btn = QtWidgets.QPushButton("Pause Simulation")
        self.start_btn.clicked.connect(self.toggle_running)
        side_layout.addWidget(self.start_btn)

        reset_btn = QtWidgets.QPushButton("Reset")
        reset_btn.clicked.connect(self.reset)
        side_layout.addWidget(reset_btn)

        side_layout.addSpacing(18)

        self.stats = QtWidgets.QLabel()
        self.stats.setStyleSheet("font-family: monospace; font-size: 13px;")
        side_layout.addWidget(self.stats)

        side_layout.addSpacing(15)

        side_layout.addWidget(QtWidgets.QLabel("Noise"))
        self.noise_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.noise_slider.setRange(0, 300)
        self.noise_slider.setValue(100)
        side_layout.addWidget(self.noise_slider)

        self.noise_value = QtWidgets.QLabel("100%")
        side_layout.addWidget(self.noise_value)

        self.noise_slider.valueChanged.connect(
            lambda v: self.noise_value.setText(f"{v}%")
        )

        side_layout.addSpacing(10)

        side_layout.addWidget(QtWidgets.QLabel("Simulation speed"))
        self.speed = QtWidgets.QComboBox()
        self.speed.addItems(["0.25×", "0.5×", "1×", "2×", "5×"])
        self.speed.setCurrentIndex(2)
        side_layout.addWidget(self.speed)

        side_layout.addSpacing(10)

        self.rays = QtWidgets.QCheckBox("Show LiDAR rays")
        self.rays.setChecked(False)
        self.rays.setStyleSheet("color: #dce8ee;")
        side_layout.addWidget(self.rays)

        self.room = QtWidgets.QCheckBox("Show environment")
        self.room.setChecked(True)
        self.room.setStyleSheet("color: #dce8ee;")
        side_layout.addWidget(self.room)

        side_layout.addStretch()

        info = QtWidgets.QLabel(
            "Sensor: STL-19P\n"
            "Scan: 360°\n"
            "Nominal rate: 10 Hz\n"
            "Nominal measurements: 5000/s\n\n"
            "This GUI simulates logical angle + distance data.\n"
            "The real UART parser will later feed the same pipeline."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color: #91a7b2; line-height: 140%;")
        side_layout.addWidget(info)

        layout.addWidget(side)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(20)

    def speed_multiplier(self):
        return [0.25, 0.5, 1.0, 2.0, 5.0][self.speed.currentIndex()]

    def toggle_running(self):
        self.running = not self.running

        if self.running:
            self.status.setText("● RUNNING")
            self.status.setStyleSheet(
                "font-size: 16px; font-weight: bold; color: #70f0a0;"
            )
            self.start_btn.setText("Pause Simulation")
            self.last_real = time.monotonic()
        else:
            self.status.setText("● PAUSED")
            self.status.setStyleSheet(
                "font-size: 16px; font-weight: bold; color: #ffd166;"
            )
            self.start_btn.setText("Start Simulation")

    def reset(self):
        self.sim_time = 0
        self.scan_id = 0
        self.total_valid = 0
        self.total_points = 0
        self.last_real = time.monotonic()

    def update_simulation(self):
        now = time.monotonic()
        dt = min(now - self.last_real, 0.1)
        self.last_real = now

        if not self.running:
            return

        self.sim_time += dt * self.speed_multiplier()

        # Produce approximately 10 logical scans/sec.
        if self.scan_id == 0 or self.sim_time * SCAN_HZ >= self.scan_id:
            noise = self.noise_slider.value() / 100.0

            angles, distances, hits, person = make_scan(
                self.sim_time,
                noise,
            )

            self.radar.update_scan(
                angles,
                distances,
                self.rays.isChecked(),
            )
            self.radar.update_person(person)

            valid = int(np.isfinite(distances).sum())
            self.total_valid += valid
            self.total_points += len(distances)

            nearest = np.nanmin(distances)

            self.stats.setText(
                f"SCAN ID       {self.scan_id:06d}\n"
                f"SIM TIME      {self.sim_time:7.2f} s\n"
                f"POINTS/SCAN   {len(distances):7d}\n"
                f"VALID POINTS   {valid:7d}\n"
                f"NEAREST        {nearest:7.2f} m\n"
                f"SCAN RATE      {SCAN_HZ:7.1f} Hz\n"
                f"MEASUREMENTS   {MEASUREMENTS_PER_SECOND:7d}/s"
            )

            self.scan_id += 1


def main():
    pg.setConfigOptions(antialias=True)

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
