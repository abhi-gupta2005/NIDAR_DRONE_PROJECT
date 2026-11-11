#!/usr/bin/env python3
"""
NIDAR AirMouse Ground Station -- desktop window (2026-11-02g, "ABHISHEK DRONE" edition).

2026-11-02g: professional dark theme, header bar with the drone's name, status pills,
survivor counter and mission clock; camera HUD; quadcopter drone marker with its name;
map with glow path, survivor halo markers, launch pad, compass and scale bar; larger text.
The drone name comes from --drone-name or NIDAR_DRONE_NAME (default "ABHISHEK DRONE").

Shows, in one window:
  * CAMERA   - the drone camera (/nidar/camera/image via ROS 2) with the detection
               boxes the Pi used (from the mission server);
  * MAP      - the SLAM V6 occupancy map in the arena frame, competition grid
               (A..N x 1..14), drone pose + heading + camera field of view, flown path,
               survivors as RED dots with their grid cell (e.g. "S2 E3");
               optional ground truth (walls, true survivors, true pose) in simulation;
  * SURVIVORS table, TELEMETRY (PX4 via ROS 2 when PX4 runs, AirMouse command),
    PIPELINE HEALTH (perception, V18, LiDAR, SLAM ages and rates) and an event log.

Data sources:
  Pi mission server  NIDAR_MISSION_V1, TCP <pi>:5020 (src/mission/nidar_mission_server.py)
  ROS 2 (optional)   /nidar/camera/image, /nidar/airmouse_command, /nidar/rig/odom (sim truth),
                     /fmu/out/* (PX4, auto-discovered, best-effort QoS)
Read-only: this program never sends a command to the drone.

Run in WSL (ROS 2 sourced; PyQt5: sudo apt install python3-pyqt5):
    python3 gcs/nidar_gcs.py --pi-host 10.218.70.147 --layout gazebo/layouts/nidar_airmouse_arena_s1.json
"""

from __future__ import annotations

import argparse
import base64
import csv
import json
import math
import os
import socket
import sys
import threading
import time
import zlib
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:                                   # ROS installs ship python_qt_binding
    from python_qt_binding import QtCore, QtGui, QtWidgets   # type: ignore

REPO = Path(__file__).resolve().parents[1]
Qt = QtCore.Qt

GREEN, ORANGE, RED, GREY = "#22c55e", "#f59e0b", "#ef4444", "#7c8a9e"
ACCENT, YELLOW, TEXT, MUTED, BORDER = "#22d3ee", "#facc15", "#e6edf6", "#8a9ab0", "#24324a"
PLAN = "#e879f9"                                      # the autonomous brain's planned path (2026-11-09)
BRAIN_COLORS = {"EXPLORE": GREEN, "RETURN": ACCENT, "DONE": GREEN, "COMPLETE": GREEN, "HOLD": ORANGE}


def grid_bounded(g: Optional[dict]) -> bool:
    return bool((g or {}).get("bounded", True))


def where(s: dict) -> str:
    """Survivor location for labels: the grid cell, or coordinates when the arena is unknown."""
    cell = s.get("grid")
    return cell if cell and cell not in ("--", "OUT") else f"({s['x_m']:.1f}, {s['y_m']:.1f})"
DRONE_NAME = os.environ.get("NIDAR_DRONE_NAME", "ABHISHEK DRONE")
UI_FONT = "Segoe UI, Ubuntu, Noto Sans, DejaVu Sans, sans-serif"
STYLE = f"""
QMainWindow, QWidget {{ background:#0a0f1a; color:{TEXT}; font-family:"Segoe UI","Ubuntu","Noto Sans","DejaVu Sans";
                       font-size:13px; }}
QFrame#header {{ background:qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #0f1d36, stop:1 #10243f);
                border:1px solid {BORDER}; border-radius:10px; }}
QLabel#droneName {{ font-size:24px; font-weight:800; color:{YELLOW}; letter-spacing:2px; background:transparent; }}
QLabel#subTitle {{ font-size:11px; color:{MUTED}; letter-spacing:1px; background:transparent; }}
QLabel#bigCount {{ font-size:26px; font-weight:800; color:{RED}; background:transparent; }}
QLabel#clock {{ font-size:22px; font-weight:700; color:{TEXT}; background:transparent; }}
QLabel#pill {{ font-size:12px; font-weight:700; padding:5px 10px; border-radius:12px; color:{GREY};
              border:1px solid {GREY}; background:transparent; }}
QFrame#panel {{ background:#0f1726; border:1px solid {BORDER}; border-radius:10px; }}
QFrame#panelHead {{ background:#121d31; border:none; border-bottom:1px solid {BORDER};
                   border-top-left-radius:10px; border-top-right-radius:10px; }}
QLabel#panelTitle {{ font-size:13px; font-weight:700; color:#cfe3f7; letter-spacing:1.5px; background:transparent; }}
QLabel#telemetry {{ background:#0f1726; font-size:13px; padding:4px; }}
QScrollArea {{ background:#0f1726; border:none; }}
QTableWidget {{ background:#0f1726; alternate-background-color:#131f33; color:{TEXT}; border:none;
               font-size:13px; selection-background-color:#1e3a5f; }}
QHeaderView::section {{ background:#16233a; color:#9fb3c8; padding:6px 8px; border:none;
                       border-bottom:1px solid {BORDER}; font-weight:700; font-size:12px; }}
QPlainTextEdit {{ background:#0b1220; color:{TEXT}; border:none; font-family:"DejaVu Sans Mono","Consolas",monospace;
                 font-size:12px; }}
QCheckBox {{ color:#cbd5e1; spacing:6px; background:transparent; }}
QCheckBox::indicator {{ width:15px; height:15px; border-radius:4px; border:1px solid #475569; background:#0b1220; }}
QCheckBox::indicator:checked {{ background:{ACCENT}; border:1px solid {ACCENT}; }}
QPushButton#primary {{ background:#0e7490; color:white; font-weight:700; border:none; border-radius:6px;
                       padding:7px 16px; }}
QPushButton#primary:hover {{ background:#0891b2; }}
QStatusBar {{ background:#0a0f1a; color:{MUTED}; font-size:12px; }}
QSplitter::handle {{ background:#0a0f1a; }}
QScrollBar:vertical {{ background:#0b1220; width:10px; }}
QScrollBar::handle:vertical {{ background:#24324a; border-radius:5px; min-height:20px; }}
QScrollBar:horizontal {{ background:#0b1220; height:10px; }}
QScrollBar::handle:horizontal {{ background:#24324a; border-radius:5px; min-width:20px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width:0px; height:0px; }}
QTableCornerButton::section {{ background:#16233a; border:none; }}
"""
NAV_STATES = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
              10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD", 15: "STAB",
              17: "AUTO_TAKEOFF", 18: "AUTO_LAND", 19: "AUTO_FOLLOW", 20: "AUTO_PRECLAND"}


def age_color(age: Optional[float], ok: float = 1.0, warn: float = 3.0) -> str:
    if age is None:
        return RED
    return GREEN if age < ok else (ORANGE if age < warn else RED)


def fmt(v, nd=2, unit=""):
    if v is None:
        return "—"
    try:
        return f"{float(v):.{nd}f}{unit}"
    except (TypeError, ValueError):
        return str(v)


# ====================================================================== data ==

class MissionClient(threading.Thread):
    """TCP client for the Pi mission server; keeps the newest state and map."""

    def __init__(self, host: str, port: int):
        super().__init__(daemon=True, name="mission-client")
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.state: Optional[dict] = None
        self.map: Optional[dict] = None              # decoded: grid ndarray + meta
        self.map_version = 0
        self.last_rx = None
        self.connected = False
        self.error = ""
        self._halt = threading.Event()

    def stop(self):
        self._halt.set()

    def run(self):
        while not self._halt.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                sock.settimeout(5.0)
                self.connected, self.error = True, ""
                buf = b""
                while not self._halt.is_set():
                    chunk = sock.recv(1 << 20)
                    if not chunk:
                        raise ConnectionError("server closed the connection")
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if line.strip():
                            self._on_message(json.loads(line))
            except (OSError, ValueError, ConnectionError) as exc:
                self.connected, self.error = False, str(exc)
                self._halt.wait(1.0)

    def _on_message(self, msg: dict):
        self.last_rx = time.time()
        if msg.get("type") == "map":
            try:
                raw = zlib.decompress(base64.b64decode(msg["data"]))
                grid = np.frombuffer(raw, dtype=np.int8).reshape(msg["shape"]).copy()
            except (KeyError, ValueError, zlib.error):
                return
            with self.lock:
                self.map = dict(msg, grid=grid)
                self.map_version += 1
        elif msg.get("type") == "state":
            with self.lock:
                self.state = msg

    def snapshot(self):
        with self.lock:
            return self.state, self.map, self.map_version


class RosLink:
    """Optional ROS 2 side: camera image, AirMouse command, sim truth odometry, PX4 telemetry."""

    PX4_PATTERNS = ("vehicle_status", "vehicle_local_position", "vehicle_attitude", "battery_status")

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        self.rclpy = rclpy
        self.qos = qos_profile_sensor_data
        rclpy.init()
        self.node = Node("nidar_gcs")
        self.lock = threading.Lock()
        self.image = None            # (w, h, encoding, bytes, t)
        self.command = None          # ([f, l, v, valid], t)
        self.truth = None            # (x, y, yaw, t)
        self.px4 = {}                # name -> (msg, t)
        self.subscribed = set()
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32MultiArray
        from nav_msgs.msg import Odometry
        self.node.create_subscription(Image, "/nidar/camera/image", self._on_image, self.qos)
        self.node.create_subscription(Float32MultiArray, "/nidar/airmouse_command", self._on_cmd, 10)
        self.node.create_subscription(Odometry, "/nidar/rig/odom", self._on_odom, 10)
        self.node.create_timer(2.0, self._discover_px4)
        self.thread = threading.Thread(target=self._spin, daemon=True, name="ros-spin")
        self.thread.start()

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:                               # shutdown
            pass

    def _on_image(self, msg):
        with self.lock:
            self.image = (msg.width, msg.height, msg.encoding, bytes(msg.data), msg.step, time.time())

    def _on_cmd(self, msg):
        with self.lock:
            self.command = (list(msg.data), time.time())

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.truth = (p.x, p.y, yaw, time.time())

    def _discover_px4(self):
        try:
            from rosidl_runtime_py.utilities import get_message
        except ImportError:
            return
        for name, types in self.node.get_topic_names_and_types():
            if not name.startswith("/fmu/out/") or name in self.subscribed:
                continue
            short = name.rsplit("/", 1)[-1]
            if not any(short.startswith(p) for p in self.PX4_PATTERNS):
                continue
            try:
                cls = get_message(types[0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                continue
            key = next(p for p in self.PX4_PATTERNS if short.startswith(p))
            if key == "vehicle_local_position" and "groundtruth" in short:
                continue
            self.node.create_subscription(cls, name, lambda m, k=key, n=name: self._on_px4(k, n, m), self.qos)
            self.subscribed.add(name)

    def _on_px4(self, key, name, msg):
        with self.lock:
            self.px4[key] = (msg, time.time(), name)

    def snapshot(self):
        with self.lock:
            return self.image, self.command, self.truth, dict(self.px4)

    def shutdown(self):
        try:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


def load_truth(layout_path: Optional[str]):
    if not layout_path:
        return None
    p = Path(layout_path)
    if not p.is_absolute():
        p = REPO / p
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


# =================================================================== widgets ==

def _font(px: int, bold: bool = False, family: Optional[str] = None) -> QtGui.QFont:
    f = QtGui.QFont(family or UI_FONT)
    f.setPixelSize(px)
    f.setBold(bold)
    return f


def _round_label(p: QtGui.QPainter, x: float, y: float, text: str, bg, fg="#ffffff", px: int = 12,
                 bold: bool = True, pad: int = 6, anchor: str = "left") -> QtCore.QRectF:
    """Rounded tag in screen coordinates; (x, y) = left/centre of its baseline row."""
    f = _font(px, bold)
    fm = QtGui.QFontMetrics(f)
    w, h = fm.horizontalAdvance(text) + 2 * pad, fm.height() + 4
    left = x if anchor == "left" else (x - w / 2 if anchor == "center" else x - w)
    rect = QtCore.QRectF(left, y - h / 2, w, h)
    p.setPen(Qt.NoPen)
    p.setBrush(bg if isinstance(bg, QtGui.QColor) else QtGui.QColor(bg))
    p.drawRoundedRect(rect, 4, 4)
    p.setPen(QtGui.QColor(fg))
    p.setFont(f)
    p.drawText(rect, Qt.AlignCenter, text)
    return rect


def draw_quad_icon(p: QtGui.QPainter, cx: float, cy: float, heading_screen: float, size: float,
                   body="#facc15", rotor="#fde68a", outline="#1f2937", nose="#ef4444") -> None:
    """Top view of a quadcopter in SCREEN coordinates (heading_screen: radians, 0 = right, CCW on screen)."""
    p.save()
    p.translate(cx, cy)
    p.rotate(-math.degrees(heading_screen))          # Qt rotates clockwise on screen
    arm = size * 0.42
    r_rot = size * 0.2
    pen = QtGui.QPen(QtGui.QColor(outline), max(2.0, size * 0.09))
    pen.setCapStyle(Qt.RoundCap)
    p.setPen(pen)
    for ang in (45, 135, 225, 315):
        a = math.radians(ang)
        p.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(arm * math.cos(a), arm * math.sin(a)))
    pen = QtGui.QPen(QtGui.QColor(body), max(1.5, size * 0.06))
    pen.setCapStyle(Qt.RoundCap)
    p.setPen(pen)
    for ang in (45, 135, 225, 315):
        a = math.radians(ang)
        p.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(arm * math.cos(a), arm * math.sin(a)))
    for ang in (45, 135, 225, 315):
        a = math.radians(ang)
        c = QtCore.QPointF(arm * math.cos(a), arm * math.sin(a))
        p.setPen(QtGui.QPen(QtGui.QColor(outline), 1.5))
        p.setBrush(QtGui.QColor(255, 255, 255, 40))
        p.drawEllipse(c, r_rot, r_rot)
        p.setBrush(QtGui.QColor(nose if ang in (45, 315) else rotor))   # front rotors coloured
        p.drawEllipse(c, r_rot * 0.35, r_rot * 0.35)
    p.setPen(QtGui.QPen(QtGui.QColor(outline), 1.5))
    p.setBrush(QtGui.QColor(body))
    p.drawRoundedRect(QtCore.QRectF(-size * 0.16, -size * 0.12, size * 0.32, size * 0.24), 3, 3)
    p.setBrush(QtGui.QColor(nose))                   # heading arrow
    tip = size * 0.62
    p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(tip, 0), QtCore.QPointF(tip - size * 0.2, -size * 0.1),
                                   QtCore.QPointF(tip - size * 0.2, size * 0.1)]))
    p.restore()


def drone_pixmap(size: int = 64) -> QtGui.QPixmap:
    pm = QtGui.QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    draw_quad_icon(p, size / 2 - size * 0.06, size / 2, math.pi / 2, size * 0.78)
    p.end()
    return pm


class CameraView(QtWidgets.QWidget):
    """Drone camera with a HUD: live/stale badge, frame age, detections, corner brackets, boxes."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 240)
        self.qimage = None
        self.img_age = None
        self.objects = []
        self.cam_size = (640, 480)
        self.message = "Waiting for /nidar/camera/image (ROS 2)"
        self._t_prev = None
        self.fps = None

    def set_image(self, img_tuple):
        if img_tuple is None:
            self.qimage = None
            return
        w, h, enc, data, step, t = img_tuple
        if self._t_prev is not None and t > self._t_prev:
            inst = 1.0 / (t - self._t_prev)
            self.fps = inst if self.fps is None else 0.8 * self.fps + 0.2 * inst
        if t != self._t_prev:
            self._t_prev = t
        self.img_age = time.time() - t
        if enc in ("rgb8", "bgr8"):
            img = QtGui.QImage(data, w, h, step, QtGui.QImage.Format_RGB888).copy()
            self.qimage = img.rgbSwapped() if enc == "bgr8" else img
        elif enc in ("mono8",):
            self.qimage = QtGui.QImage(data, w, h, step, QtGui.QImage.Format_Grayscale8).copy()
        else:
            self.message = f"Unsupported image encoding {enc}"
            self.qimage = None

    def set_qimage(self, qimage: QtGui.QImage):
        self.qimage, self.img_age = qimage, 0.0

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        g = QtGui.QLinearGradient(0, 0, 0, self.height())
        g.setColorAt(0, QtGui.QColor("#0b1220"))
        g.setColorAt(1, QtGui.QColor("#05080f"))
        p.fillRect(self.rect(), g)
        if self.qimage is None:
            p.setPen(QtGui.QColor(MUTED))
            p.setFont(_font(20, True))
            p.drawText(self.rect().adjusted(0, -20, 0, -20), Qt.AlignCenter, "NO VIDEO SIGNAL")
            p.setFont(_font(13))
            p.drawText(self.rect().adjusted(0, 30, 0, 30), Qt.AlignCenter, self.message)
            return
        iw, ih = self.qimage.width(), self.qimage.height()
        s = min(self.width() / iw, self.height() / ih)
        ox, oy = (self.width() - iw * s) / 2, (self.height() - ih * s) / 2
        target = QtCore.QRectF(ox, oy, iw * s, ih * s)
        p.drawImage(target, self.qimage)
        # corner brackets + centre reticle
        pen = QtGui.QPen(QtGui.QColor(ACCENT), 2)
        p.setPen(pen)
        L = min(target.width(), target.height()) * 0.08
        for cx, cy, dx, dy in ((target.left(), target.top(), 1, 1), (target.right(), target.top(), -1, 1),
                               (target.left(), target.bottom(), 1, -1), (target.right(), target.bottom(), -1, -1)):
            p.drawLine(QtCore.QPointF(cx + 6 * dx, cy + 6 * dy), QtCore.QPointF(cx + (6 + L) * dx, cy + 6 * dy))
            p.drawLine(QtCore.QPointF(cx + 6 * dx, cy + 6 * dy), QtCore.QPointF(cx + 6 * dx, cy + (6 + L) * dy))
        c = target.center()
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 150), 1.5))
        for a, b in (((-14, 0), (-5, 0)), ((5, 0), (14, 0)), ((0, -14), (0, -5)), ((0, 5), (0, 14))):
            p.drawLine(QtCore.QPointF(c.x() + a[0], c.y() + a[1]), QtCore.QPointF(c.x() + b[0], c.y() + b[1]))
        # detection boxes
        cw, ch = self.cam_size
        kx, ky = iw * s / max(cw, 1), ih * s / max(ch, 1)
        for o in self.objects:
            try:
                x1, y1, x2, y2 = [float(v) for v in o["bbox"]]
            except (KeyError, TypeError, ValueError):
                continue
            r = QtCore.QRectF(ox + x1 * kx, oy + y1 * ky, (x2 - x1) * kx, (y2 - y1) * ky)
            p.setPen(QtGui.QPen(QtGui.QColor(RED), 2.5))
            p.setBrush(QtGui.QColor(239, 68, 68, 28))
            p.drawRect(r)
            label = f"{str(o.get('class', '?')).upper()}  {fmt(o.get('confidence'), 2)}"
            _round_label(p, r.left(), max(r.top() - 11, target.top() + 11), label, RED, px=12)
        # HUD badges
        age = self.img_age
        live = age is not None and age < 1.0
        badge = "● LIVE" if live else ("● STALE" if age is not None else "● NO DATA")
        _round_label(p, target.left() + 10, target.top() + 18, badge, "#15803d" if live else "#b45309", px=12)
        _round_label(p, target.right() - 10, target.top() + 18,
                     f"{iw}×{ih}   {fmt(self.fps, 1)} FPS   AGE {fmt(age, 2)} s", QtGui.QColor(15, 23, 42, 210), px=12,
                     bold=False, anchor="right")
        n = sum(1 for o in self.objects if o.get("class") == "person")   # other classes are boxed, not counted
        _round_label(p, target.left() + 10, target.bottom() - 16,
                     f"PERSONS IN VIEW: {n}", RED if n else QtGui.QColor(15, 23, 42, 210), px=12)


class MapView(QtWidgets.QWidget):
    """SLAM occupancy map in the arena frame with the competition grid, drone, path and survivors."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(520, 520)
        self.state = None
        self.map = None
        self.map_img = None
        self.map_version = -1
        self.truth_layout = None
        self.truth_pose = None
        self.drone_name = DRONE_NAME
        self.show_truth = False          # sim ground truth overlay (walls, true survivors)
        self.show_path = True
        self.show_meas = True
        self.show_candidates = False
        self.clip_map = True             # draw map/path only over the arena (+1.5 m)
        self.path = []
        lut = [QtGui.qRgba(0, 0, 0, 0)]                     # -1 unknown: transparent (arena colour shows)
        for v in range(101):                                # 0..100 % occupied
            if v < 35:
                lut.append(QtGui.qRgba(214, 226, 240, 255))  # free floor
            elif v < 65:
                lut.append(QtGui.qRgba(120, 138, 160, 255))  # uncertain
            else:
                lut.append(QtGui.qRgba(8, 14, 28, 255))      # wall
        self.lut = lut

    # ------------------------------------------------------------- data --
    def set_data(self, state, mp, version, truth_pose=None):
        self.state = state
        self.truth_pose = truth_pose
        if mp is not None and version != self.map_version:
            self.map_version = version
            self.map = mp
            g = mp["grid"].astype(np.int16) + 1                # -1..100 -> 0..101
            g = np.ascontiguousarray(g.astype(np.uint8))
            h, w = g.shape
            img = QtGui.QImage(g.data, w, h, w, QtGui.QImage.Format_Indexed8)
            img.setColorTable(self.lut)
            self.map_img = img.convertToFormat(QtGui.QImage.Format_ARGB32)
            if mp.get("path"):
                self.path = list(mp["path"])
        if state and state.get("path_tail"):
            tail = state["path_tail"]
            if not self.path or len(self.path) < 2:
                self.path = list(tail)
            else:
                last = self.path[-1]
                k = next((i for i in range(len(tail) - 1, -1, -1) if tail[i] == last), None)
                self.path.extend(tail[k + 1:] if k is not None else tail[-1:])
                self.path = self.path[-20000:]

    # -------------------------------------------------------- transforms --
    def bounds(self):
        g = (self.state or {}).get("grid") or {"pitch_m": 1.05, "wall_m": 0.05, "cells": 14}
        if not grid_bounded(g):
            return self.fit_bounds()
        size = g["cells"] * g["pitch_m"] + g["wall_m"]
        return -1.3, size + 1.3, -2.3, size + 1.3

    def fit_bounds(self):
        """Arena unknown (no layout): fit the view to the mapped area, the path and the pad."""
        pts = []
        mp = self.map
        if mp is not None:
            T = self.image_to_arena(mp)
            h, w = mp["shape"]
            for u, v in ((0, 0), (w, 0), (0, h), (w, h)):
                q = T.map(QtCore.QPointF(u, v))
                pts.append((q.x(), q.y()))
        pts += [tuple(xy) for xy in self.path[-2000:]]
        sp = (self.state or {}).get("start_pose")
        if sp:
            pts.append((sp[0], sp[1]))
        if not pts:
            return -5.0, 5.0, -5.0, 5.0
        xs, ys = [a for a, _ in pts], [b for _, b in pts]
        x0, x1, y0, y1 = min(xs) - 1.0, max(xs) + 1.0, min(ys) - 1.0, max(ys) + 1.0
        span = max(x1 - x0, y1 - y0, 6.0)                 # keep the aspect, never zoom in closer than 6 m
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        return cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2

    def world_transform(self) -> QtGui.QTransform:
        xmin, xmax, ymin, ymax = self.bounds()
        top, bottom = 58, 58                                 # room for the info strip / legend
        s = min((self.width() - 24) / (xmax - xmin), (self.height() - top - bottom) / (ymax - ymin))
        ox = (self.width() - (xmax - xmin) * s) / 2
        oy = top + (self.height() - top - bottom - (ymax - ymin) * s) / 2
        return QtGui.QTransform(s, 0, 0, -s, ox - xmin * s, oy + ymax * s)

    @staticmethod
    def image_to_arena(mp) -> QtGui.QTransform:
        """map image pixel (u, v) -> arena (x, y): start_pose o (origin + res*(u, v))."""
        sx, sy, syaw = mp["start_pose"]
        c, s = math.cos(math.radians(syaw)), math.sin(math.radians(syaw))
        r = float(mp["resolution_m"])
        ox, oy = mp["origin_m"]
        return QtGui.QTransform(c * r, s * r, -s * r, c * r, c * ox - s * oy + sx, s * ox + c * oy + sy)

    # -------------------------------------------------------------- paint --
    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        bg = QtGui.QLinearGradient(0, 0, 0, self.height())
        bg.setColorAt(0, QtGui.QColor("#0d1628"))
        bg.setColorAt(1, QtGui.QColor("#070b14"))
        p.fillRect(self.rect(), bg)
        T = self.world_transform()
        g = (self.state or {}).get("grid") or {"pitch_m": 1.05, "wall_m": 0.05, "cells": 14}
        size = g["cells"] * g["pitch_m"] + g["wall_m"]
        st = self.state or {}
        bounded = grid_bounded(g)
        p.setTransform(T)
        if bounded:
            p.fillRect(QtCore.QRectF(0, 0, size, size), QtGui.QColor("#16233a"))
            if self.clip_map:
                p.setClipRect(QtCore.QRectF(-1.5, -1.5, size + 3.0, size + 3.0))
        if self.map_img is not None and self.map is not None:
            p.setTransform(self.image_to_arena(self.map) * T)
            if not bounded:
                p.fillRect(QtCore.QRectF(0, 0, self.map_img.width(), self.map_img.height()), QtGui.QColor("#16233a"))
            p.drawImage(QtCore.QPointF(0, 0), self.map_img)
            p.setTransform(T)
        if bounded:
            self._draw_cells(p, g, st)
            self._draw_grid(p, T, g, size)
        self._draw_pad(p, T, st)
        if self.show_truth and self.truth_layout:
            self._draw_truth(p, T)
        if self.show_path and len(self.path) > 1:
            poly = QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in self.path])
            for width, col in ((6, QtGui.QColor(56, 189, 248, 50)), (2.4, QtGui.QColor("#38bdf8"))):
                pen = QtGui.QPen(col, width)
                pen.setCosmetic(True)
                pen.setJoinStyle(Qt.RoundJoin)
                pen.setCapStyle(Qt.RoundCap)
                p.setPen(pen)
                p.drawPolyline(poly)
        self._draw_plan(p, T, st)
        if self.show_meas:
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QColor(251, 191, 36, 170))
            for m in st.get("measurements") or []:
                p.drawEllipse(QtCore.QPointF(m["x_m"], m["y_m"]), 0.06, 0.06)
        pose = st.get("pose")
        if pose:
            self._draw_fov(p, pose["x_m"], pose["y_m"], math.radians(pose["yaw_deg"]))
        p.setClipping(False)
        if self.truth_pose is not None:
            self._draw_drone(p, T, *self.truth_pose, truth=True)
        if pose:
            self._draw_drone(p, T, pose["x_m"], pose["y_m"], math.radians(pose["yaw_deg"]))
        self._draw_survivors(p, T, st.get("survivors") or [])
        p.resetTransform()
        self._draw_info(p, st)
        self._draw_legend(p)
        self._draw_compass(p, T)
        self._draw_banner(p, st, size)

    def _draw_plan(self, p, T, st):
        """The autonomous brain's current plan: dashed line from the drone through the waypoints, goal ring."""
        ex = st.get("explorer") or {}
        plan = ex.get("plan")
        if not plan or not plan.get("waypoints"):
            return
        pts = [QtCore.QPointF(x, y) for x, y in plan["waypoints"]]
        pose = st.get("pose")
        if pose:
            pts.insert(0, QtCore.QPointF(pose["x_m"], pose["y_m"]))
        for width, col, style in ((7, QtGui.QColor(232, 121, 249, 45), Qt.SolidLine),
                                  (2.4, QtGui.QColor(PLAN), Qt.DashLine)):
            pen = QtGui.QPen(col, width)
            pen.setCosmetic(True)
            pen.setStyle(style)
            pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(QtGui.QPolygonF(pts))
        p.save()
        p.resetTransform()
        p.setPen(QtGui.QPen(QtGui.QColor(PLAN), 1.5))
        p.setBrush(QtGui.QColor(PLAN))
        for q in pts[1:-1]:
            p.drawEllipse(T.map(q), 3.0, 3.0)
        gx, gy = plan.get("goal") or plan["waypoints"][-1]
        c = T.map(QtCore.QPointF(gx, gy))
        p.setBrush(QtGui.QColor(232, 121, 249, 60))
        p.setPen(QtGui.QPen(QtGui.QColor(PLAN), 2.5))
        p.drawEllipse(c, 9, 9)
        p.drawLine(c + QtCore.QPointF(-13, 0), c + QtCore.QPointF(-5, 0))
        p.drawLine(c + QtCore.QPointF(5, 0), c + QtCore.QPointF(13, 0))
        p.drawLine(c + QtCore.QPointF(0, -13), c + QtCore.QPointF(0, -5))
        p.drawLine(c + QtCore.QPointF(0, 5), c + QtCore.QPointF(0, 13))
        end = {"look": "LOOK", "land": "LAND", "none": "GOAL"}.get(plan.get("end"), "GOAL")
        _round_label(p, c.x() + 14, c.y() + 12, end, "#a21caf", px=11)
        p.restore()

    @staticmethod
    def cell_rect(g, label: str) -> Optional[QtCore.QRectF]:
        if not label or label == "OUT":
            return None
        try:
            c, r = ord(label[0]) - ord("A"), int(label[1:]) - 1
        except ValueError:
            return None
        pitch, wall = g["pitch_m"], g["wall_m"]
        return QtCore.QRectF(wall / 2 + c * pitch, wall / 2 + r * pitch, pitch, pitch)

    def _draw_cells(self, p, g, st):
        """Competition-grid cells: survivor cells filled red, drone's cell outlined."""
        p.setPen(Qt.NoPen)
        for sv in st.get("survivors") or []:
            if sv.get("status") != "CONFIRMED":
                continue
            r = self.cell_rect(g, sv.get("grid"))
            if r is not None:
                p.fillRect(r, QtGui.QColor(239, 68, 68, 85))
                pen = QtGui.QPen(QtGui.QColor(239, 68, 68, 220), 2)
                pen.setCosmetic(True)
                p.setPen(pen)
                p.setBrush(Qt.NoBrush)
                p.drawRect(r)
                p.setPen(Qt.NoPen)
        pose = st.get("pose") or {}
        r = self.cell_rect(g, pose.get("grid"))
        if r is not None:
            pen = QtGui.QPen(QtGui.QColor(YELLOW), 2.5)
            pen.setCosmetic(True)
            pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.setBrush(QtGui.QColor(250, 204, 21, 30))
            p.drawRect(r)

    def _draw_grid(self, p, T, g, size):
        pitch, wall, n = g["pitch_m"], g["wall_m"], g["cells"]
        pen = QtGui.QPen(QtGui.QColor(125, 211, 252, 55), 1)
        pen.setCosmetic(True)
        p.setPen(pen)
        for k in range(1, n):
            v = wall / 2 + k * pitch
            p.drawLine(QtCore.QPointF(v, 0), QtCore.QPointF(v, size))
            p.drawLine(QtCore.QPointF(0, v), QtCore.QPointF(size, v))
        for width, col in ((6, QtGui.QColor(34, 211, 238, 45)), (2, QtGui.QColor(ACCENT))):
            pen = QtGui.QPen(col, width)
            pen.setCosmetic(True)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawRect(QtCore.QRectF(0, 0, size, size))
        p.save()
        p.resetTransform()
        p.setFont(_font(12, True))
        p.setPen(QtGui.QColor("#9fb3c8"))
        for k in range(n):
            c = wall / 2 + (k + 0.5) * pitch
            for yy in (size + 0.42,):
                pt = T.map(QtCore.QPointF(c, yy))
                p.drawText(QtCore.QRectF(pt.x() - 12, pt.y() - 9, 24, 18), Qt.AlignCenter, chr(ord("A") + k))
            for xx in (-0.5, size + 0.5):
                pt = T.map(QtCore.QPointF(xx, c))
                p.drawText(QtCore.QRectF(pt.x() - 14, pt.y() - 9, 28, 18), Qt.AlignCenter, str(k + 1))
        p.restore()

    def _draw_pad(self, p, T, st):
        sp = st.get("start_pose")
        if not sp:
            return
        c = T.map(QtCore.QPointF(sp[0], sp[1]))
        r = max(8.0, 0.3 * T.m11())
        p.save()
        p.resetTransform()
        p.setPen(QtGui.QPen(QtGui.QColor(YELLOW), 2))
        p.setBrush(QtGui.QColor(30, 41, 59, 230))
        p.drawEllipse(c, r, r)
        p.setPen(QtGui.QColor(YELLOW))
        p.setFont(_font(max(9, int(r)), True))
        p.drawText(QtCore.QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r), Qt.AlignCenter, "H")
        p.restore()

    def _draw_truth(self, p, T):
        lay = self.truth_layout
        pen = QtGui.QPen(QtGui.QColor(74, 222, 128, 150), 1.5)
        pen.setCosmetic(True)
        p.setPen(pen)
        for x1, y1, x2, y2 in lay.get("walls", []):
            p.drawLine(QtCore.QPointF(x1, y1), QtCore.QPointF(x2, y2))
        pen = QtGui.QPen(QtGui.QColor("#4ade80"), 2)
        pen.setCosmetic(True)
        pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        for s in lay.get("survivors", []):
            p.drawEllipse(QtCore.QPointF(s["x"], s["y"]), 0.34, 0.34)

    def _draw_fov(self, p, x, y, yaw, rng=3.0, half=math.radians(35)):
        poly = QtGui.QPolygonF([QtCore.QPointF(x, y)] + [
            QtCore.QPointF(x + rng * math.cos(yaw + a), y + rng * math.sin(yaw + a))
            for a in np.linspace(half, -half, 12)])
        grad = QtGui.QRadialGradient(QtCore.QPointF(x, y), rng)
        grad.setColorAt(0.0, QtGui.QColor(250, 204, 21, 110))
        grad.setColorAt(1.0, QtGui.QColor(250, 204, 21, 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QBrush(grad))
        p.drawPolygon(poly)

    def _draw_drone(self, p, T, x, y, yaw, truth=False):
        c = T.map(QtCore.QPointF(x, y))
        p.save()
        p.resetTransform()
        if truth:
            pen = QtGui.QPen(QtGui.QColor("#4ade80"), 2)
            pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(c, 16, 16)
        else:
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QColor(250, 204, 21, 45))
            p.drawEllipse(c, 24, 24)
            draw_quad_icon(p, c.x(), c.y(), yaw, 34)
            _round_label(p, c.x() + 24, c.y() + 26, self.drone_name, YELLOW, fg="#111827", px=11)
        p.restore()

    def _draw_survivors(self, p, T, survivors):
        p.save()
        p.resetTransform()
        for s in survivors:
            conf = s.get("status") == "CONFIRMED"
            if not conf and not self.show_candidates:
                continue
            c = T.map(QtCore.QPointF(s["x_m"], s["y_m"]))
            if conf:
                p.setPen(Qt.NoPen)
                p.setBrush(QtGui.QColor(239, 68, 68, 55))
                p.drawEllipse(c, 17, 17)
                p.setBrush(QtGui.QColor(239, 68, 68, 90))
                p.drawEllipse(c, 12, 12)
                p.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
                p.setBrush(QtGui.QColor(RED))
                p.drawEllipse(c, 7.5, 7.5)
                p.setPen(QtGui.QPen(QtGui.QColor(239, 68, 68, 200), 1.5))
                p.drawLine(c + QtCore.QPointF(7, -7), c + QtCore.QPointF(16, -16))
                _round_label(p, c.x() + 16, c.y() - 18, f"{s['id']} · {where(s)}", RED, px=13)
            else:
                p.setPen(QtGui.QPen(QtGui.QColor(ORANGE), 2))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(c, 6, 6)
        p.restore()

    def _draw_info(self, p, st):
        pose = st.get("pose") or {}
        rect = QtCore.QRectF(10, 8, min(self.width() - 20, 560), 42)
        p.setPen(QtGui.QPen(QtGui.QColor(BORDER), 1))
        p.setBrush(QtGui.QColor(15, 23, 42, 225))
        p.drawRoundedRect(rect, 8, 8)
        p.setFont(_font(11, True))
        p.setPen(QtGui.QColor(ACCENT))
        p.drawText(QtCore.QPointF(rect.left() + 12, rect.top() + 16), f"{self.drone_name} · SLAM POSITION (ARENA FRAME)")
        txt = (f"Cell {pose.get('grid', '—')}    x {fmt(pose.get('x_m'))} m    y {fmt(pose.get('y_m'))} m    "
               f"heading {fmt(pose.get('yaw_deg'), 1)}°")
        if self.truth_pose is not None and pose:
            err = math.hypot(self.truth_pose[0] - pose["x_m"], self.truth_pose[1] - pose["y_m"])
            txt += f"    truth error {err:.2f} m"
        p.setFont(_font(14, True))
        p.setPen(QtGui.QColor(TEXT))
        p.drawText(QtCore.QPointF(rect.left() + 12, rect.top() + 35), txt)

    def _draw_legend(self, p):
        items = [("drone", self.drone_name.title()), ("path", "Flown path"), ("survivor", "Survivor + grid cell")]
        if (self.state or {}).get("explorer"):
            items.insert(2, ("plan", "Brain plan"))
        if self.show_candidates:
            items.append(("candidate", "Candidate"))
        if self.truth_layout and self.show_truth:
            items.append(("truth", "Ground truth (sim)"))
        f = _font(12)
        fm = QtGui.QFontMetrics(f)
        w = sum(34 + fm.horizontalAdvance(t) for _, t in items) + 16
        rect = QtCore.QRectF(10, self.height() - 40, w, 30)
        p.setPen(QtGui.QPen(QtGui.QColor(BORDER), 1))
        p.setBrush(QtGui.QColor(15, 23, 42, 225))
        p.drawRoundedRect(rect, 8, 8)
        x, y = rect.left() + 12, rect.center().y()
        p.setFont(f)
        for kind, text in items:
            if kind == "drone":
                draw_quad_icon(p, x + 8, y, math.pi / 2, 18)
            elif kind == "path":
                p.setPen(QtGui.QPen(QtGui.QColor("#38bdf8"), 3))
                p.drawLine(QtCore.QPointF(x, y), QtCore.QPointF(x + 18, y))
            elif kind == "plan":
                pen = QtGui.QPen(QtGui.QColor(PLAN), 2.5)
                pen.setStyle(Qt.DashLine)
                p.setPen(pen)
                p.drawLine(QtCore.QPointF(x, y), QtCore.QPointF(x + 18, y))
            elif kind == "survivor":
                p.setPen(QtGui.QPen(QtGui.QColor("white"), 1.5))
                p.setBrush(QtGui.QColor(RED))
                p.drawEllipse(QtCore.QPointF(x + 9, y), 6, 6)
            elif kind == "candidate":
                p.setPen(QtGui.QPen(QtGui.QColor(ORANGE), 2))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QtCore.QPointF(x + 9, y), 5, 5)
            else:
                pen = QtGui.QPen(QtGui.QColor("#4ade80"), 2)
                pen.setStyle(Qt.DashLine)
                p.setPen(pen)
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QtCore.QPointF(x + 9, y), 6, 6)
            p.setPen(QtGui.QColor(TEXT))
            p.drawText(QtCore.QPointF(x + 24, y + 4), text)
            x += 34 + fm.horizontalAdvance(text)

    def _draw_compass(self, p, T):
        # north arrow (+y) and a 2 m scale bar, bottom right
        x0, y0 = self.width() - 36, self.height() - 92
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QColor(15, 23, 42, 225))
        p.drawEllipse(QtCore.QPointF(x0, y0), 20, 20)
        p.setBrush(QtGui.QColor(RED))
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x0, y0 - 15), QtCore.QPointF(x0 - 6, y0 + 2),
                                       QtCore.QPointF(x0 + 6, y0 + 2)]))
        p.setBrush(QtGui.QColor("#cbd5e1"))
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x0, y0 + 15), QtCore.QPointF(x0 - 6, y0 + 2),
                                       QtCore.QPointF(x0 + 6, y0 + 2)]))
        p.setPen(QtGui.QColor(TEXT))
        p.setFont(_font(11, True))
        p.drawText(QtCore.QRectF(x0 - 10, y0 - 38, 20, 16), Qt.AlignCenter, "N")
        px_per_m = T.m11()
        bar = 2.0 * px_per_m
        xr, yb = self.width() - 16, self.height() - 26
        p.setPen(QtGui.QPen(QtGui.QColor(TEXT), 2))
        p.drawLine(QtCore.QPointF(xr - bar, yb), QtCore.QPointF(xr, yb))
        for xx in (xr - bar, xr - bar / 2, xr):
            p.drawLine(QtCore.QPointF(xx, yb - 5), QtCore.QPointF(xx, yb + 1))
        p.setFont(_font(11))
        p.drawText(QtCore.QRectF(xr - bar, yb - 20, bar, 14), Qt.AlignCenter, "2 m")

    def _draw_banner(self, p, st, size):
        pose = st.get("pose") or {}
        slam = (st.get("health") or {}).get("slam") or {}
        ex = st.get("explorer") or {}
        msg, col = None, None
        if slam.get("status") == "LOST" or pose.get("trusted") is False:
            msg = "SLAM LOST / poor match — position not trusted, survivors are not being placed"
            col = QtGui.QColor(220, 38, 38, 235)
        elif grid_bounded(st.get("grid")) and pose and not (-1.5 <= pose.get("x_m", 0) <= size + 1.5
                                                            and -1.5 <= pose.get("y_m", 0) <= size + 1.5):
            msg = "Drone position is outside the arena — check SLAM"
            col = QtGui.QColor(217, 119, 6, 235)
        elif ex.get("state") == "HOLD":
            msg = f"AUTONOMY HOLD — {ex.get('hold_reason') or 'waiting'}"
            col = QtGui.QColor(217, 119, 6, 235)
        elif ex.get("state") == "COMPLETE":
            why = ex.get("return_reason") or ""
            if "time" in why:                         # 2026-11-09 run: time ran out, two rooms never visited
                msg = f"MISSION COMPLETE — TIME LIMIT ({why}): back on the pad, NOT everything explored"
                col = QtGui.QColor(217, 119, 6, 235)
            else:
                msg = "MISSION COMPLETE — everything reachable explored, back at the take-off point"
                col = QtGui.QColor(22, 163, 74, 235)
        if msg:
            _round_label(p, self.width() / 2, 70, msg, col, px=13, pad=12, anchor="center")


class Panel(QtWidgets.QFrame):
    """Card with a header strip (accent bar + title) around a widget."""

    def __init__(self, title: str, widget: QtWidgets.QWidget, right: Optional[QtWidgets.QWidget] = None):
        super().__init__()
        self.setObjectName("panel")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        head = QtWidgets.QFrame()
        head.setObjectName("panelHead")
        hl = QtWidgets.QHBoxLayout(head)
        hl.setContentsMargins(12, 7, 10, 7)
        bar = QtWidgets.QFrame()
        bar.setFixedSize(4, 16)
        bar.setStyleSheet(f"background:{ACCENT}; border-radius:2px;")
        hl.addWidget(bar)
        lab = QtWidgets.QLabel(title)
        lab.setObjectName("panelTitle")
        hl.addWidget(lab)
        hl.addStretch(1)
        if right is not None:
            hl.addWidget(right)
        lay.addWidget(head)
        body = QtWidgets.QWidget()
        bl = QtWidgets.QVBoxLayout(body)
        bl.setContentsMargins(8, 8, 8, 8)
        bl.addWidget(widget)
        lay.addWidget(body, 1)


class HeaderBar(QtWidgets.QFrame):
    """Drone badge + name, status pills, survivors counter and mission clock."""

    PILLS = ("LINK", "SLAM", "LIDAR", "CAMERA", "FUSION", "BRAIN", "PX4")

    def __init__(self, drone_name: str):
        super().__init__()
        self.setObjectName("header")
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(14, 8, 14, 8)
        lay.setSpacing(10)
        icon = QtWidgets.QLabel()
        icon.setPixmap(drone_pixmap(46))
        lay.addWidget(icon)
        names = QtWidgets.QVBoxLayout()
        names.setSpacing(0)
        t = QtWidgets.QLabel(drone_name)
        t.setObjectName("droneName")
        s = QtWidgets.QLabel("NIDAR AirMouse  ·  Ground Control Station  ·  read-only, sends no commands")
        s.setObjectName("subTitle")
        names.addWidget(t)
        names.addWidget(s)
        lay.addLayout(names)
        lay.addStretch(1)
        self.pills = {}
        for name in self.PILLS:
            lab = QtWidgets.QLabel(name)
            lab.setObjectName("pill")
            lab.setAlignment(Qt.AlignCenter)
            lab.setMinimumWidth(78)
            self.pills[name] = lab
            lay.addWidget(lab)
        lay.addSpacing(12)
        box = QtWidgets.QVBoxLayout()
        box.setSpacing(0)
        self.count = QtWidgets.QLabel("0")
        self.count.setObjectName("bigCount")
        self.count.setAlignment(Qt.AlignCenter)
        cap = QtWidgets.QLabel("SURVIVORS")
        cap.setObjectName("subTitle")
        cap.setAlignment(Qt.AlignCenter)
        box.addWidget(self.count)
        box.addWidget(cap)
        lay.addLayout(box)
        lay.addSpacing(12)
        clk = QtWidgets.QVBoxLayout()
        clk.setSpacing(0)
        self.clock = QtWidgets.QLabel("00:00")
        self.clock.setObjectName("clock")
        self.clock.setAlignment(Qt.AlignCenter)
        cap2 = QtWidgets.QLabel("MISSION TIME")
        cap2.setObjectName("subTitle")
        cap2.setAlignment(Qt.AlignCenter)
        clk.addWidget(self.clock)
        clk.addWidget(cap2)
        lay.addLayout(clk)

    def set_pill(self, name: str, color: str, text: Optional[str] = None):
        lab = self.pills[name]
        lab.setText(f"● {name}" + (f"  {text}" if text else ""))
        lab.setStyleSheet(f"QLabel#pill {{ color:{color}; border:1px solid {color}; background:{color}22; }}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, client: Optional[MissionClient], ros: Optional[RosLink], truth_layout: Optional[dict],
                 report_dir: Path, drone_name: str = None):
        super().__init__()
        self.client, self.ros, self.report_dir = client, ros, report_dir
        self.drone_name = (drone_name or DRONE_NAME).upper()
        self.setWindowTitle(f"{self.drone_name} — NIDAR Ground Control Station")
        self.setWindowIcon(QtGui.QIcon(drone_pixmap(64)))
        self.resize(1600, 960)
        self.t0 = time.time()
        self.known_survivors = set()
        self.known_merges = set()
        self.known_brain_events = set()
        self.last_conn = None

        self.header = HeaderBar(self.drone_name)
        self.camera = CameraView()
        self.map = MapView()
        self.map.drone_name = self.drone_name
        self.map.truth_layout = truth_layout
        self.telemetry = QtWidgets.QLabel()
        self.telemetry.setObjectName("telemetry")
        self.telemetry.setTextFormat(Qt.RichText)
        self.telemetry.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.telemetry.setWordWrap(True)
        self.table = QtWidgets.QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(["ID", "Status", "Grid", "X (m)", "Y (m)", "± (m)", "Inliers/Obs",
                                              "Method", "Last seen", "Also seen as"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(30)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)

        tel_scroll = QtWidgets.QScrollArea()
        tel_scroll.setWidget(self.telemetry)
        tel_scroll.setWidgetResizable(True)
        tel_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        tel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left = QtWidgets.QSplitter(Qt.Vertical)
        left.addWidget(Panel("DRONE CAMERA", self.camera))
        left.addWidget(Panel("TELEMETRY & SYSTEM HEALTH", tel_scroll))
        left.setSizes([430, 470])

        controls = QtWidgets.QHBoxLayout()
        controls.setSpacing(14)
        for name, attr in (("Clip to arena", "clip_map"), ("Path", "show_path"), ("Measurements", "show_meas"),
                           ("Candidates", "show_candidates"), ("Ground truth (sim)", "show_truth")):
            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(getattr(self.map, attr))
            cb.toggled.connect(lambda v, a=attr: (setattr(self.map, a, v), self.map.update()))
            controls.addWidget(cb)
        controls.addStretch(1)
        export = QtWidgets.QPushButton("Export report")
        export.setObjectName("primary")
        export.clicked.connect(self.export_report)
        controls.addWidget(export)
        map_panel = QtWidgets.QWidget()
        mv = QtWidgets.QVBoxLayout(map_panel)
        mv.setContentsMargins(0, 0, 0, 0)
        mv.addWidget(self.map, 1)
        mv.addLayout(controls)

        right = QtWidgets.QSplitter(Qt.Vertical)
        right.addWidget(Panel("SLAM MAP  ·  DRONE  ·  SURVIVORS", map_panel))
        bottom = QtWidgets.QSplitter(Qt.Horizontal)
        bottom.addWidget(Panel("SURVIVORS FOUND", self.table))
        bottom.addWidget(Panel("EVENTS", self.log))
        bottom.setSizes([700, 380])
        right.addWidget(bottom)
        right.setSizes([760, 200])

        main = QtWidgets.QSplitter(Qt.Horizontal)
        main.addWidget(left)
        main.addWidget(right)
        main.setSizes([540, 1060])
        central = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(central)
        cl.setContentsMargins(8, 8, 8, 4)
        cl.setSpacing(8)
        cl.addWidget(self.header)
        cl.addWidget(main, 1)
        self.setCentralWidget(central)
        self.status = self.statusBar()
        self.setStyleSheet(STYLE)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.log_event("Ground station started (read-only: sends no commands).")

    def log_event(self, text):
        color = RED if text.startswith("SURVIVOR") else (ORANGE if "NOT CONNECTED" in text or "merged" in text
                                                         else (PLAN if text.startswith("BRAIN") else TEXT))
        weight = "700" if text.startswith("SURVIVOR") else "400"
        safe = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.log.appendHtml(f"<span style='color:{MUTED}'>[{time.strftime('%H:%M:%S')}]</span> "
                            f"<span style='color:{color}; font-weight:{weight}'>{safe}</span>")

    # ------------------------------------------------------------ refresh --
    def refresh(self):
        state, mp, ver = self.client.snapshot() if self.client else (None, None, 0)
        img = cmd = truth = None
        px4 = {}
        if self.ros:
            img, cmd, truth, px4 = self.ros.snapshot()
            self.camera.set_image(img)
        truth_pose = truth[:3] if truth and time.time() - truth[3] < 2.0 else None
        self.map.set_data(state, mp, ver, truth_pose)
        if state:
            cam = state.get("camera") or {}
            self.camera.objects = cam.get("objects") or []
            self.camera.cam_size = (cam.get("width", 640), cam.get("height", 480))
            self._update_table(state.get("survivors") or [])
            for e in (state.get("explorer") or {}).get("events") or []:
                if e not in self.known_brain_events:
                    self.known_brain_events.add(e)
                    self.log_event(f"BRAIN {e}")
            for a, b in ((state.get("health") or {}).get("mapper") or {}).get("merges") or []:
                if (a, b) not in self.known_merges:
                    self.known_merges.add((a, b))
                    self.log_event(f"{a} merged into {b} (same person seen again) — keeping ID {b}")
        conn = bool(self.client and self.client.connected)
        if conn != self.last_conn:
            self.last_conn = conn
            if self.client:
                self.log_event(f"Pi mission server {self.client.host}:{self.client.port} "
                               + ("CONNECTED" if conn else f"NOT CONNECTED ({self.client.error or 'waiting'})"))
        self.telemetry.setText(self._telemetry_html(state, cmd, px4, truth_pose))
        self._update_header(state, conn, px4)
        self.camera.update()
        self.map.update()
        n_conf = sum(1 for s in (state or {}).get("survivors", []) if s["status"] == "CONFIRMED")
        self.status.showMessage(f"{self.drone_name}  |  mission time {time.time() - self.t0:6.0f} s  |  "
                                f"survivors confirmed: {n_conf}  |  Pi link: {'UP' if conn else 'DOWN'}  |  "
                                f"ROS 2: {'on' if self.ros else 'off'}  |  flight control: none")

    def _update_header(self, state, conn, px4):
        h = (state or {}).get("health") or {}
        link_age = time.time() - self.client.last_rx if self.client and self.client.last_rx else None
        self.header.set_pill("LINK", age_color(link_age) if conn else RED)
        sl = h.get("slam") or {}
        if sl.get("status") == "LOST":
            self.header.set_pill("SLAM", RED, "LOST")
        else:
            self.header.set_pill("SLAM", age_color(sl.get("age_s")), fmt(sl.get("fps"), 1, " Hz") if sl else None)
        self.header.set_pill("LIDAR", age_color((h.get("lidar") or {}).get("age_s")))
        cam_age = (h.get("perception") or {}).get("age_s")
        self.header.set_pill("CAMERA", age_color(cam_age))
        self.header.set_pill("FUSION", age_color((h.get("v18") or {}).get("age_s")))
        ex = (state or {}).get("explorer")
        if ex:
            st_ = ex.get("state") or "?"
            self.header.set_pill("BRAIN", BRAIN_COLORS.get(st_, GREY), st_)
        else:
            # 2026-11-09: PX4 runs fly the preset route (the brain cannot fly PX4 yet): say so instead of "off"
            route = os.environ.get("NIDAR_BRAIN_MODE", "").strip().lower() == "route"
            self.header.set_pill("BRAIN", ACCENT if route else GREY, "PX4 route" if route else "off")
        self.header.set_pill("PX4", GREEN if px4 else GREY, None if px4 else "off")
        n_conf = sum(1 for s in (state or {}).get("survivors", []) if s["status"] == "CONFIRMED")
        self.header.count.setText(str(n_conf))
        el = int(time.time() - self.t0)
        self.header.clock.setText(f"{el // 60:02d}:{el % 60:02d}")

    def _update_table(self, survivors):
        now = time.time()
        if not self.map.show_candidates:
            survivors = [s for s in survivors if s.get("status") == "CONFIRMED"]
        self.table.setRowCount(len(survivors))
        bold = _font(13, True)
        for r, s in enumerate(survivors):
            meth = ", ".join(f"{k}:{v}" for k, v in (s.get("methods") or {}).items())
            last = float(s.get("last_seen") or 0.0)
            seen = f"{now - last:.0f} s ago" if 1e9 < last <= now + 5 else "—"
            inl = s.get("inliers")
            obs = f"{inl}/{s['observations']}" if inl is not None else str(s["observations"])
            vals = [s["id"], s["status"], s["grid"], fmt(s["x_m"]), fmt(s["y_m"]), fmt(s.get("std_m")),
                    obs, meth, seen, ", ".join(s.get("aliases") or [])]
            conf = s["status"] == "CONFIRMED"
            for c, v in enumerate(vals):
                item = QtWidgets.QTableWidgetItem(v)
                if c < 3:
                    item.setFont(bold)
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#fca5a5" if conf else ORANGE)))
                if c in (0, 1, 2, 5, 6):
                    item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, item)
            if conf and s["id"] not in self.known_survivors:
                self.known_survivors.add(s["id"])
                at = f"grid {s['grid']}" if s.get("grid") not in (None, "--", "OUT") else where(s)
                self.log_event(f"SURVIVOR {s['id']} found at {at} "
                               f"(x={s['x_m']:.2f} m, y={s['y_m']:.2f} m, {s['observations']} observations)")

    def _telemetry_html(self, state, cmd, px4, truth_pose) -> str:
        def row(name, value, color=TEXT, dot=True):
            d = f"<span style='color:{color}'>●</span>&nbsp;" if dot and color != TEXT else ""
            return (f"<tr><td style='color:{MUTED}; padding:3px 14px 3px 4px'>{name}</td>"
                    f"<td style='color:{color}; padding:3px 4px'>{d}{value}</td></tr>")

        def section(title):
            return (f"<tr><td colspan=2 bgcolor='#14213a' style='color:{ACCENT}; font-weight:700; "
                    f"padding:5px 6px; letter-spacing:1px'>{title}</td></tr>")

        h = (state or {}).get("health") or {}
        rows = []
        link_age = time.time() - self.client.last_rx if self.client and self.client.last_rx else None
        rows.append(section("LINK"))
        rows.append(row("Pi mission server",
                        f"{self.client.host}:{self.client.port} " + ("UP" if self.client and self.client.connected else "DOWN")
                        if self.client else "not configured", age_color(link_age)))
        rows.append(row("last message", fmt(link_age, 2, " s"), age_color(link_age)))
        pose = (state or {}).get("pose") or {}
        rows.append(section(f"{self.drone_name} · SLAM (ARENA FRAME)"))
        rows.append(row("position", f"<b>x {fmt(pose.get('x_m'))} m &nbsp; y {fmt(pose.get('y_m'))} m</b>"
                                    f" &nbsp; cell <b>{pose.get('grid', '—')}</b>"))
        rows.append(row("heading", fmt(pose.get("yaw_deg"), 1, "°")))
        if truth_pose is not None:
            err = math.hypot(truth_pose[0] - pose.get("x_m", 0), truth_pose[1] - pose.get("y_m", 0)) if pose else None
            rows.append(row("sim truth", f"x {truth_pose[0]:.2f} &nbsp; y {truth_pose[1]:.2f} &nbsp; SLAM error {fmt(err)} m",
                            GREEN if err is not None and err < 0.3 else ORANGE))
        rows.append(section("PIPELINE HEALTH (PI)"))
        pc, v, li, sl, mp = (h.get(k, {}) for k in ("perception", "v18", "lidar", "slam", "mapper"))
        rows.append(row("camera → Pi", f"frame {pc.get('frame', '—')} &nbsp; age {fmt(pc.get('age_s'), 2, ' s')} &nbsp; "
                        f"objects {pc.get('objects', 0)} &nbsp; {pc.get('source') or ''}", age_color(pc.get("age_s"))))
        rows.append(row("V18 fusion", f"age {fmt(v.get('age_s'), 2, ' s')} &nbsp; fused {v.get('fused', 0)} &nbsp; "
                        f"active {v.get('active', 0)}", age_color(v.get("age_s"))))
        rows.append(row("LiDAR", f"age {fmt(li.get('age_s'), 2, ' s')} &nbsp; seq {li.get('seq', '—')} &nbsp; "
                        f"points {li.get('points', 0)}", age_color(li.get("age_s"))))
        lost = sl.get("status") == "LOST"
        rows.append(row("SLAM status", f"<b>{sl.get('status', '—')}</b> &nbsp; reject streak {sl.get('reject_streak', '—')}",
                        RED if lost else (GREEN if sl.get("status") == "TRACKING" else GREY)))
        rows.append(row("SLAM V6", f"age {fmt(sl.get('age_s'), 2, ' s')} &nbsp; {fmt(sl.get('fps'), 1, ' Hz')} &nbsp; "
                        f"score {fmt(sl.get('score'))} &nbsp; acc {sl.get('accepted', '—')} / rej {sl.get('rejected', '—')}",
                        age_color(sl.get("age_s"))))
        rows.append(row("survivor mapper", f"measurements {mp.get('measurements', 0)} &nbsp; "
                        f"LiDAR rejected {mp.get('lidar_rejected_f22', 0)} &nbsp; ghosts {mp.get('ghosts_absorbed', 0)}"))
        rows.append(row("mapper rejects", f"outside arena {mp.get('outside_arena_rejected', 0)} &nbsp; "
                        f"edge-cut boxes {mp.get('edge_clipped', 0)} &nbsp; bad pose {mp.get('skipped_bad_pose', 0)} "
                        f"&nbsp; no pose {mp.get('skipped_no_pose', 0)}"))
        rows.append(section("AIRMOUSE COMMAND (ROS 2)"))
        if cmd:
            d, t = cmd
            age = time.time() - t
            vals = (d + [0, 0, 0, 0])[:4]
            rows.append(row("fwd / lat / vert", f"{vals[0]:+.2f} / {vals[1]:+.2f} / {vals[2]:+.2f} m/s &nbsp; "
                            f"valid={int(vals[3])}", age_color(age, 1.5, 5)))
        else:
            rows.append(row("/nidar/airmouse_command", "no messages (bridge not running)" if self.ros else "ROS 2 off", GREY))
        rows.append(section("PX4 FLIGHT CONTROLLER (ROS 2 /fmu/out)"))
        if not px4:
            rows.append(row("status", "not connected — PX4 SITL + uXRCE-DDS agent not running "
                                      "(normal in the rig test)" if self.ros else "ROS 2 off", GREY))
        else:
            if "vehicle_status" in px4:
                m, t, _ = px4["vehicle_status"]
                arm = getattr(m, "arming_state", None)
                nav = getattr(m, "nav_state", None)
                rows.append(row("armed", "ARMED" if arm == 2 else f"disarmed ({arm})", RED if arm == 2 else GREEN))
                rows.append(row("mode", f"{NAV_STATES.get(nav, nav)}", age_color(time.time() - t)))
            if "vehicle_local_position" in px4:
                m, t, _ = px4["vehicle_local_position"]
                rows.append(row("local pos NED", f"x {fmt(getattr(m, 'x', None))} &nbsp; y {fmt(getattr(m, 'y', None))} &nbsp; "
                                f"z {fmt(getattr(m, 'z', None))} m", age_color(time.time() - t)))
                rows.append(row("velocity NED", f"{fmt(getattr(m, 'vx', None))} &nbsp; {fmt(getattr(m, 'vy', None))} &nbsp; "
                                f"{fmt(getattr(m, 'vz', None))} m/s"))
                rows.append(row("heading", fmt(math.degrees(getattr(m, "heading", float("nan"))), 1, "°")))
            if "vehicle_attitude" in px4:
                m, t, _ = px4["vehicle_attitude"]
                q = list(getattr(m, "q", [1, 0, 0, 0]))
                w, x, y, z = q
                roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
                pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
                yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
                rows.append(row("attitude", f"roll {roll:+.1f}° &nbsp; pitch {pitch:+.1f}° &nbsp; yaw {yaw:+.1f}°",
                                age_color(time.time() - t)))
            if "battery_status" in px4:
                m, t, _ = px4["battery_status"]
                rem = getattr(m, "remaining", None)
                rows.append(row("battery", f"{fmt(getattr(m, 'voltage_v', None), 2, ' V')} &nbsp; "
                                f"{fmt(rem * 100 if rem is not None else None, 0, ' %')}",
                                GREEN if rem is None or rem > 0.3 else RED))
        return "<table width='100%' cellspacing=0 cellpadding=0>" + "".join(rows) + "</table>"

    # ------------------------------------------------------------- export --
    def export_report(self):
        out = self.report_dir / time.strftime("%Y%m%d_%H%M%S")
        out.mkdir(parents=True, exist_ok=True)
        state, _, _ = self.client.snapshot() if self.client else (None, None, 0)
        survivors = (state or {}).get("survivors") or []
        (out / "survivors.json").write_text(json.dumps({"exported": time.time(), "drone": self.drone_name,
                                                        "survivors": survivors,
                                                        "start_pose": (state or {}).get("start_pose")}, indent=1))
        with open(out / "survivors.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "status", "grid", "x_m", "y_m", "std_m", "observations", "inliers"])
            for s in survivors:
                w.writerow([s["id"], s["status"], s["grid"], s["x_m"], s["y_m"], s.get("std_m"), s["observations"],
                            s.get("inliers")])
        self.map.grab().save(str(out / "map.png"))
        self.camera.grab().save(str(out / "camera.png"))
        self.grab().save(str(out / "ground_station.png"))
        self.log_event(f"Report exported to {out}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi-host", default=os.environ.get("NIDAR_PI_HOST", ""), help="Pi address (mission server)")
    ap.add_argument("--port", type=int, default=int(os.environ.get("NIDAR_MISSION_PORT", "5020")))
    ap.add_argument("--layout", default=None, help="arena layout JSON for the ground-truth overlay (simulation)")
    ap.add_argument("--no-ros", action="store_true", help="do not use ROS 2 (no camera/PX4 panels)")
    ap.add_argument("--reports", default=str(REPO / "gcs_reports"))
    ap.add_argument("--drone-name", default=DRONE_NAME, help="name shown in the window (default ABHISHEK DRONE)")
    ap.add_argument("--screenshot", default=None, help="save a window screenshot after --duration s and exit")
    ap.add_argument("--duration", type=float, default=5.0)
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv[:1])
    client = None
    if args.pi_host:
        client = MissionClient(args.pi_host, args.port)
        client.start()
    ros = None
    if not args.no_ros:
        try:
            ros = RosLink()
        except Exception as exc:                          # ROS not sourced / not installed
            print(f"[GCS] ROS 2 not available ({exc}); camera and PX4 panels disabled", flush=True)
    win = MainWindow(client, ros, load_truth(args.layout), Path(args.reports), args.drone_name)
    if not args.pi_host:
        win.log_event("No --pi-host given: map/survivors need the Pi mission server (NIDAR_SLAM=V6).")
    win.show()
    if args.screenshot:
        def snap():
            win.refresh()
            win.grab().save(args.screenshot)
            app.quit()
        QtCore.QTimer.singleShot(int(args.duration * 1000), snap)
    rc = app.exec_()
    if client:
        client.stop()
    if ros:
        ros.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
