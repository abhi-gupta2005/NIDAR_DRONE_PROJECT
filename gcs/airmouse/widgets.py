"""Shared widgets: painted icons, top bar, navigation rail, cards, stat tiles and the map view."""

from __future__ import annotations

import math
from typing import List, Optional

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    from python_qt_binding import QtCore, QtGui, QtWidgets  # type: ignore

from gcs import nidar_gcs as classic
from gcs.airmouse import theme as T

Qt = QtCore.Qt


# ---------------------------------------------------------------- icons --
def icon(name: str, size: int = 26, color: str = T.TEXT) -> QtGui.QIcon:
    pm = QtGui.QPixmap(size * 2, size * 2)
    pm.setDevicePixelRatio(2.0)
    pm.fill(Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing)
    pen = QtGui.QPen(QtGui.QColor(color), 1.8)
    pen.setCapStyle(Qt.RoundCap)
    pen.setJoinStyle(Qt.RoundJoin)
    p.setPen(pen)
    p.setBrush(Qt.NoBrush)
    s = float(size)
    if name == "fly":
        c = s / 2
        for ang in (45, 135, 225, 315):
            a = math.radians(ang)
            p.drawLine(QtCore.QPointF(c, c), QtCore.QPointF(c + 0.28 * s * math.cos(a), c + 0.28 * s * math.sin(a)))
            p.drawEllipse(QtCore.QPointF(c + 0.3 * s * math.cos(a), c + 0.3 * s * math.sin(a)), 0.13 * s, 0.13 * s)
        p.setBrush(QtGui.QColor(color))
        p.drawRoundedRect(QtCore.QRectF(c - 0.09 * s, c - 0.07 * s, 0.18 * s, 0.14 * s), 2, 2)
    elif name == "check":
        p.drawRoundedRect(QtCore.QRectF(0.2 * s, 0.16 * s, 0.6 * s, 0.72 * s), 3, 3)
        p.drawLine(QtCore.QPointF(0.38 * s, 0.16 * s), QtCore.QPointF(0.62 * s, 0.16 * s))
        p.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(0.33 * s, 0.52 * s), QtCore.QPointF(0.45 * s, 0.64 * s),
                                        QtCore.QPointF(0.67 * s, 0.4 * s)]))
    elif name == "survivor":
        path = QtGui.QPainterPath()
        path.moveTo(0.5 * s, 0.9 * s)
        path.cubicTo(0.18 * s, 0.56 * s, 0.18 * s, 0.38 * s, 0.22 * s, 0.3 * s)
        path.arcTo(QtCore.QRectF(0.2 * s, 0.08 * s, 0.6 * s, 0.6 * s), 200, -220)
        path.cubicTo(0.82 * s, 0.38 * s, 0.82 * s, 0.56 * s, 0.5 * s, 0.9 * s)
        p.drawPath(path)
        p.drawEllipse(QtCore.QPointF(0.5 * s, 0.32 * s), 0.08 * s, 0.08 * s)
        p.drawArc(QtCore.QRectF(0.36 * s, 0.42 * s, 0.28 * s, 0.2 * s), 0, 180 * 16)
    elif name == "logs":
        p.drawEllipse(QtCore.QPointF(0.5 * s, 0.5 * s), 0.34 * s, 0.34 * s)
        p.drawLine(QtCore.QPointF(0.5 * s, 0.5 * s), QtCore.QPointF(0.5 * s, 0.3 * s))
        p.drawLine(QtCore.QPointF(0.5 * s, 0.5 * s), QtCore.QPointF(0.64 * s, 0.58 * s))
        p.drawArc(QtCore.QRectF(0.08 * s, 0.08 * s, 0.84 * s, 0.84 * s), 100 * 16, 60 * 16)
    elif name == "play":
        p.setBrush(QtGui.QColor(color))
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(0.32 * s, 0.22 * s), QtCore.QPointF(0.78 * s, 0.5 * s),
                                       QtCore.QPointF(0.32 * s, 0.78 * s)]))
    elif name == "pause":
        p.setBrush(QtGui.QColor(color))
        p.drawRoundedRect(QtCore.QRectF(0.28 * s, 0.22 * s, 0.14 * s, 0.56 * s), 2, 2)
        p.drawRoundedRect(QtCore.QRectF(0.58 * s, 0.22 * s, 0.14 * s, 0.56 * s), 2, 2)
    elif name == "export":
        p.drawLine(QtCore.QPointF(0.5 * s, 0.18 * s), QtCore.QPointF(0.5 * s, 0.62 * s))
        p.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(0.33 * s, 0.35 * s), QtCore.QPointF(0.5 * s, 0.18 * s),
                                        QtCore.QPointF(0.67 * s, 0.35 * s)]))
        p.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(0.2 * s, 0.6 * s), QtCore.QPointF(0.2 * s, 0.82 * s),
                                        QtCore.QPointF(0.8 * s, 0.82 * s), QtCore.QPointF(0.8 * s, 0.6 * s)]))
    p.end()
    return QtGui.QIcon(pm)


# ---------------------------------------------------------------- small parts --
def rounded_label(p: QtGui.QPainter, x: float, y: float, text: str, bg, fg=T.TEXT, px: int = 12, weight: int = 700,
                  pad: int = 8, anchor: str = "left", radius: float = 6) -> QtCore.QRectF:
    f = T.font(px, weight)
    fm = QtGui.QFontMetricsF(f)
    w, h = fm.horizontalAdvance(text) + 2 * pad, fm.height() + 6
    left = x if anchor == "left" else (x - w / 2 if anchor == "center" else x - w)
    rect = QtCore.QRectF(left, y - h / 2, w, h)
    p.setPen(Qt.NoPen)
    p.setBrush(bg if isinstance(bg, QtGui.QColor) else QtGui.QColor(bg))
    p.drawRoundedRect(rect, radius, radius)
    p.setPen(QtGui.QColor(fg))
    p.setFont(f)
    p.drawText(rect, Qt.AlignCenter, text)
    return rect


class Card(QtWidgets.QFrame):
    def __init__(self, title: str = "", widget: Optional[QtWidgets.QWidget] = None, margins=(16, 14, 16, 16),
                 right: Optional[QtWidgets.QWidget] = None):
        super().__init__()
        self.setObjectName("card")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(*margins)
        lay.setSpacing(10)
        if title or right is not None:
            head = QtWidgets.QHBoxLayout()
            lab = QtWidgets.QLabel(title.upper())
            lab.setObjectName("cardTitle")
            head.addWidget(lab)
            head.addStretch(1)
            if right is not None:
                head.addWidget(right)
            lay.addLayout(head)
        if widget is not None:
            lay.addWidget(widget, 1)
        self.body = lay


class StatTile(QtWidgets.QFrame):
    """Label · big value · small line, with a colour bar on the left."""

    def __init__(self, label: str, color: str = T.ACCENT):
        super().__init__()
        self.label, self.color = label, color
        self.value, self.sub = "—", ""
        self.setMinimumHeight(96)
        self.setMinimumWidth(150)

    def set(self, value: str, sub: str = "", color: Optional[str] = None):
        self.value, self.sub = value, sub
        if color:
            self.color = color
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = QtCore.QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        p.setPen(QtGui.QPen(QtGui.QColor(T.BORDER), 1))
        p.setBrush(QtGui.QColor(T.SURFACE))
        p.drawRoundedRect(r, 14, 14)
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QColor(self.color))
        p.drawRoundedRect(QtCore.QRectF(r.left() + 1, r.top() + 18, 4, r.height() - 36), 2, 2)
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(11, 700))
        p.drawText(QtCore.QPointF(r.left() + 20, r.top() + 28), self.label.upper())
        p.setPen(QtGui.QColor(self.color))
        p.setFont(T.font(30, 700))
        p.drawText(QtCore.QPointF(r.left() + 20, r.top() + 66), self.value)
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(12))
        fm = QtGui.QFontMetrics(p.font())
        p.drawText(QtCore.QPointF(r.left() + 20, r.top() + 86), fm.elidedText(self.sub, Qt.ElideRight, int(r.width() - 30)))


class SignalBars(QtWidgets.QWidget):
    """Four bars + a label, like a phone's signal meter: level 0..4, colour by state."""

    def __init__(self, label: str):
        super().__init__()
        self.label, self.level, self.color, self.tip = label, 0, T.GREY, ""
        self.setFixedSize(58, 40)

    def set(self, level: int, color: str, tip: str = ""):
        if (level, color, tip) != (self.level, self.color, self.tip):
            self.level, self.color, self.tip = level, color, tip
            self.setToolTip(f"{self.label}: {tip}")
            self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        base_x, base_y = 14, 22
        for i in range(4):
            h = 5 + i * 4
            r = QtCore.QRectF(base_x + i * 8, base_y - h, 5, h)
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QColor(self.color) if i < self.level else QtGui.QColor("#24324a"))
            p.drawRoundedRect(r, 1.5, 1.5)
        p.setPen(QtGui.QColor(T.MUTED if self.level else T.DIM))
        p.setFont(T.font(10, 700))
        p.drawText(QtCore.QRectF(0, 25, self.width(), 14), Qt.AlignCenter, self.label)


def age_level(age: Optional[float], ok: float = 0.5, warn: float = 3.0) -> tuple:
    """(bars, colour) from a data age."""
    if age is None:
        return 0, T.RED
    if age <= ok:
        return 4, T.GREEN
    if age <= 1.0:
        return 3, T.GREEN
    if age <= warn:
        return 2, T.AMBER
    return 1, T.RED


class TopBar(QtWidgets.QFrame):
    """Drone · flight status · link meters · battery · survivors · clock · REC/DEMO · WATCH ONLY."""

    def __init__(self, drone_name: str):
        super().__init__()
        self.setObjectName("topbar")
        self.setFixedHeight(64)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(16, 6, 16, 6)
        lay.setSpacing(14)
        logo = QtWidgets.QLabel()
        logo.setPixmap(classic.drone_pixmap(44))
        lay.addWidget(logo)
        names = QtWidgets.QVBoxLayout()
        names.setSpacing(0)
        self.name = QtWidgets.QLabel(drone_name)
        self.name.setFont(T.font(17, 800))
        self.name.setStyleSheet(f"color:{T.YELLOW}; letter-spacing:1.5px;")
        sub = QtWidgets.QLabel("NIDAR AirMouse")
        sub.setFont(T.font(11, 600))
        sub.setStyleSheet(f"color:{T.MUTED}; letter-spacing:1px;")
        names.addWidget(self.name)
        names.addWidget(sub)
        lay.addLayout(names)
        lay.addSpacing(18)
        self.status = StatusPill()
        lay.addWidget(self.status, 1)
        self.meters = {}
        for k in ("LINK", "LIDAR", "CAMERA", "SLAM", "PX4"):
            m = SignalBars(k)
            self.meters[k] = m
            lay.addWidget(m)
        lay.addSpacing(6)
        self.battery = BigReadout("BATTERY", "—")
        self.survivors = BigReadout("SURVIVORS", "0", T.GREEN)
        self.clock = BigReadout("MISSION", "00:00")
        for w in (self.battery, self.survivors, self.clock):
            lay.addWidget(w)
        self.rec = RecBadge()
        lay.addWidget(self.rec)
        watch = QtWidgets.QLabel("WATCH ONLY")
        watch.setToolTip("This ground station never sends a command to the drone.")
        watch.setFont(T.font(11, 800))
        watch.setStyleSheet(f"color:{T.ACCENT}; border:1px solid {T.ACCENT}; border-radius:9px; padding:4px 9px;"
                            "background:#0b2230; letter-spacing:1px;")
        lay.addWidget(watch)


class StatusPill(QtWidgets.QWidget):
    """Big flight-status line: coloured dot, headline, detail."""

    def __init__(self):
        super().__init__()
        self.head, self.detail, self.color = "CONNECTING", "", T.GREY
        self.setMinimumWidth(260)
        self.setFixedHeight(46)

    def set(self, head: str, detail: str, color: str):
        if (head, detail, color) != (self.head, self.detail, self.color):
            self.head, self.detail, self.color = head, detail, color
            self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = QtCore.QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        c = QtGui.QColor(self.color)
        p.setPen(QtGui.QPen(T.colour(self.color, 140), 1.2))
        p.setBrush(T.colour(self.color, 26))
        p.drawRoundedRect(r, 23, 23)
        p.setPen(Qt.NoPen)
        p.setBrush(T.colour(self.color, 60))
        p.drawEllipse(QtCore.QPointF(r.left() + 24, r.center().y()), 9, 9)
        p.setBrush(c)
        p.drawEllipse(QtCore.QPointF(r.left() + 24, r.center().y()), 5, 5)
        p.setPen(c)
        p.setFont(T.font(15, 800))
        fm = QtGui.QFontMetrics(p.font())
        p.drawText(QtCore.QPointF(r.left() + 42, r.center().y() + 5), self.head)
        x = r.left() + 42 + fm.horizontalAdvance(self.head) + 12
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(13))
        fm2 = QtGui.QFontMetrics(p.font())
        p.drawText(QtCore.QPointF(x, r.center().y() + 5),
                   fm2.elidedText(self.detail, Qt.ElideRight, int(r.right() - x - 16)))


class BigReadout(QtWidgets.QWidget):
    def __init__(self, label: str, value: str, color: str = T.TEXT):
        super().__init__()
        self.label, self.value, self.color = label, value, color
        self.setFixedSize(96, 46)

    def set(self, value: str, color: Optional[str] = None):
        if value != self.value or (color and color != self.color):
            self.value = value
            self.color = color or self.color
            self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setPen(QtGui.QColor(self.color))
        p.setFont(T.font(20, 800))
        p.drawText(QtCore.QRectF(0, 0, self.width(), 28), Qt.AlignCenter, self.value)
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(10, 700))
        p.drawText(QtCore.QRectF(0, 28, self.width(), 16), Qt.AlignCenter, self.label)


class RecBadge(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.mode = ""                 # "REC" | "DEMO" | ""
        self.blink = False
        self.setFixedSize(70, 30)

    def set(self, mode: str):
        self.blink = not self.blink
        self.mode = mode
        self.update()

    def paintEvent(self, _):
        if not self.mode:
            return
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        col = T.RED if self.mode == "REC" else T.VIOLET
        r = QtCore.QRectF(self.rect()).adjusted(1, 1, -1, -1)
        p.setPen(QtGui.QPen(T.colour(col, 160), 1.2))
        p.setBrush(T.colour(col, 30))
        p.drawRoundedRect(r, 14, 14)
        p.setPen(Qt.NoPen)
        p.setBrush(T.colour(col, 255 if (self.blink or self.mode != "REC") else 90))
        p.drawEllipse(QtCore.QPointF(r.left() + 15, r.center().y()), 5, 5)
        p.setPen(QtGui.QColor(col))
        p.setFont(T.font(12, 800))
        p.drawText(QtCore.QRectF(r.left() + 24, r.top(), r.width() - 28, r.height()), Qt.AlignVCenter, self.mode)


class NavRail(QtWidgets.QFrame):
    changed = QtCore.pyqtSignal(int) if hasattr(QtCore, "pyqtSignal") else QtCore.Signal(int)

    def __init__(self, items: List[tuple]):
        super().__init__()
        self.setObjectName("rail")
        self.setFixedWidth(92)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(8, 14, 8, 14)
        lay.setSpacing(8)
        self.group = QtWidgets.QButtonGroup(self)
        self.group.setExclusive(True)
        self.buttons = []
        for i, (name, ic, tip) in enumerate(items):
            b = QtWidgets.QToolButton()
            b.setObjectName("navButton")
            b.setText(name)
            b.setIcon(icon(ic, 26, T.TEXT))
            b.setIconSize(QtCore.QSize(26, 26))
            b.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            b.setCheckable(True)
            b.setToolTip(tip)
            b.setFixedSize(76, 70)
            b.setCursor(Qt.PointingHandCursor)
            self.group.addButton(b, i)
            self.buttons.append(b)
            lay.addWidget(b)
        lay.addStretch(1)
        self.buttons[0].setChecked(True)
        self.group.buttonClicked[int].connect(self.changed.emit)

    def select(self, i: int):
        self.buttons[i].setChecked(True)


# ---------------------------------------------------------------- map view --
class ProMapView(classic.MapView):
    """The classic SLAM map view (same transforms, same drawing), with a compact picture-in-picture mode,
    a highlighted survivor and an externally set path (replay)."""

    ARENA_FILL = "#0c1526"                              # unknown floor inside the arena

    def __init__(self, compact: bool = False):
        super().__init__()
        lut = [QtGui.qRgba(0, 0, 0, 0)]                  # -1 unknown: transparent (arena fill shows)
        for v in range(101):                             # dark theme: deep-blue floor, bright walls
            lut.append(QtGui.qRgba(28, 47, 77, 255) if v < 35 else
                       (QtGui.qRgba(79, 101, 133, 255) if v < 65 else QtGui.qRgba(230, 238, 248, 255)))
        self.lut = lut
        self.compact = compact
        self.overlays = True             # info strip, legend, compass, banner (off inside the Fly screen's HUD)
        self.highlight: Optional[str] = None
        self.setMinimumSize(200, 150)

    def paintEvent(self, _):
        """classic.MapView.paintEvent with the dark arena fill (same drawing order and transforms)."""
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        bg = QtGui.QLinearGradient(0, 0, 0, self.height())
        bg.setColorAt(0, QtGui.QColor("#0b1322"))
        bg.setColorAt(1, QtGui.QColor("#060a12"))
        p.fillRect(self.rect(), bg)
        T_ = self.world_transform()
        g = (self.state or {}).get("grid") or {"pitch_m": 1.05, "wall_m": 0.05, "cells": 14}
        size = g["cells"] * g["pitch_m"] + g["wall_m"]
        st = self.state or {}
        bounded = classic.grid_bounded(g)
        p.setTransform(T_)
        if bounded:
            p.fillRect(QtCore.QRectF(0, 0, size, size), QtGui.QColor(self.ARENA_FILL))
            if self.clip_map:
                p.setClipRect(QtCore.QRectF(-1.5, -1.5, size + 3.0, size + 3.0))
        if self.map_img is not None and self.map is not None:
            p.setTransform(self.image_to_arena(self.map) * T_)
            if not bounded:
                p.fillRect(QtCore.QRectF(0, 0, self.map_img.width(), self.map_img.height()),
                           QtGui.QColor(self.ARENA_FILL))
            p.drawImage(QtCore.QPointF(0, 0), self.map_img)
            p.setTransform(T_)
        if bounded:
            self._draw_cells(p, g, st)
            self._draw_grid(p, T_, g, size)
        self._draw_pad(p, T_, st)
        if self.show_truth and self.truth_layout:
            self._draw_truth(p, T_)
        if self.show_path and len(self.path) > 1:
            poly = QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in self.path])
            for width, col in ((6, QtGui.QColor(56, 189, 248, 50)), (2.4, QtGui.QColor("#38bdf8"))):
                pen = QtGui.QPen(col, width)
                pen.setCosmetic(True)
                pen.setJoinStyle(Qt.RoundJoin)
                pen.setCapStyle(Qt.RoundCap)
                p.setPen(pen)
                p.drawPolyline(poly)
        self._draw_plan(p, T_, st)
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
            self._draw_drone(p, T_, *self.truth_pose, truth=True)
        if pose:
            self._draw_drone(p, T_, pose["x_m"], pose["y_m"], math.radians(pose["yaw_deg"]))
        self._draw_survivors(p, T_, st.get("survivors") or [])
        p.resetTransform()
        self._draw_info(p, st)
        self._draw_legend(p)
        self._draw_compass(p, T_)
        self._draw_banner(p, st, size)

    def world_transform(self) -> QtGui.QTransform:
        if not self.compact:
            return super().world_transform()
        xmin, xmax, ymin, ymax = self.bounds()
        m = 8
        s = min((self.width() - 2 * m) / (xmax - xmin), (self.height() - 2 * m) / (ymax - ymin))
        ox = (self.width() - (xmax - xmin) * s) / 2
        oy = (self.height() - (ymax - ymin) * s) / 2
        return QtGui.QTransform(s, 0, 0, -s, ox - xmin * s, oy + ymax * s)

    def _draw_info(self, p, st):
        if self.overlays and not self.compact:
            super()._draw_info(p, st)

    def _draw_legend(self, p):
        if self.overlays and not self.compact:
            super()._draw_legend(p)

    def _draw_compass(self, p, T_):
        if self.overlays and not self.compact:
            super()._draw_compass(p, T_)

    def _draw_banner(self, p, st, size):
        if self.overlays and not self.compact:
            super()._draw_banner(p, st, size)

    def _draw_grid(self, p, T_, g, size):
        if not self.compact:
            return super()._draw_grid(p, T_, g, size)
        pen = QtGui.QPen(QtGui.QColor(34, 211, 238, 140), 1.5)
        pen.setCosmetic(True)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawRect(QtCore.QRectF(0, 0, size, size))

    def _draw_drone(self, p, T_, x, y, yaw, truth=False):
        if not self.compact or truth:
            return super()._draw_drone(p, T_, x, y, yaw, truth)
        c = T_.map(QtCore.QPointF(x, y))
        p.save()
        p.resetTransform()
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QColor(250, 204, 21, 60))
        p.drawEllipse(c, 13, 13)
        classic.draw_quad_icon(p, c.x(), c.y(), yaw, 20)
        p.restore()

    def _draw_survivors(self, p, T_, survivors):
        if self.compact:
            p.save()
            p.resetTransform()
            for s in survivors:
                if s.get("status") != "CONFIRMED":
                    continue
                c = T_.map(QtCore.QPointF(s["x_m"], s["y_m"]))
                p.setPen(QtGui.QPen(QtGui.QColor("white"), 1.5))
                p.setBrush(QtGui.QColor(T.RED))
                p.drawEllipse(c, 5, 5)
            p.restore()
            return
        super()._draw_survivors(p, T_, survivors)
        if self.highlight:
            for s in survivors:
                if s.get("id") == self.highlight:
                    c = T_.map(QtCore.QPointF(s["x_m"], s["y_m"]))
                    p.save()
                    p.resetTransform()
                    p.setPen(QtGui.QPen(QtGui.QColor(T.ACCENT), 3))
                    p.setBrush(Qt.NoBrush)
                    p.drawEllipse(c, 26, 26)
                    p.restore()
