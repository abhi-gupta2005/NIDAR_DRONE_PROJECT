"""Checklist screen: the pre-flight checks from checks.py, live, with one READY / NOT READY verdict."""

from __future__ import annotations

from typing import List

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    from python_qt_binding import QtCore, QtGui, QtWidgets  # type: ignore

from gcs.airmouse import checks as C
from gcs.airmouse import theme as T

Qt = QtCore.Qt
LEFT = ("Links", "Sensors", "Autonomy")
RIGHT = ("Navigation", "Flight controller", "Safety")
ROW_H = 54
HEAD_H = 46
GAP = 16
BANNER_H = 118


class ChecklistView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.checks: List[C.Check] = []
        self.mode = "rig"
        self.endpoint = ""

    def set_checks(self, checks: List[C.Check], mode: str):
        self.checks, self.mode = checks, mode
        self.setMinimumHeight(int(self._height()))
        self.update()

    def _groups(self, names):
        return [(g, [c for c in self.checks if c.group == g]) for g in names if any(c.group == g for c in self.checks)]

    def _col_height(self, names):
        return sum(HEAD_H + ROW_H * len(rows) + 14 + GAP for _, rows in self._groups(names))

    def _height(self):
        return 24 + BANNER_H + GAP + max(self._col_height(LEFT), self._col_height(RIGHT)) + 24

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        m = 24.0
        W = self.width() - 2 * m
        level, text = C.verdict(self.checks)
        col = {"ready": T.GREEN, "warn": T.AMBER, "blocked": T.RED}[level]
        # banner
        b = QtCore.QRectF(m, m, W, BANNER_H)
        g = QtGui.QLinearGradient(b.topLeft(), b.topRight())
        g.setColorAt(0, T.colour(col, 60))
        g.setColorAt(1, T.colour(col, 12))
        p.setPen(QtGui.QPen(T.colour(col, 180), 1.5))
        p.setBrush(QtGui.QBrush(g))
        p.drawRoundedRect(b, 18, 18)
        cx, cy = b.left() + 62, b.center().y()
        p.setPen(Qt.NoPen)
        p.setBrush(T.colour(col, 50))
        p.drawEllipse(QtCore.QPointF(cx, cy), 34, 34)
        p.setBrush(QtGui.QColor(col))
        p.drawEllipse(QtCore.QPointF(cx, cy), 24, 24)
        pen = QtGui.QPen(QtGui.QColor("#0b1220"), 4.5)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        p.setPen(pen)
        if level == "ready":
            p.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(cx - 10, cy + 1), QtCore.QPointF(cx - 3, cy + 8),
                                            QtCore.QPointF(cx + 11, cy - 7)]))
        elif level == "warn":
            p.drawLine(QtCore.QPointF(cx, cy - 11), QtCore.QPointF(cx, cy + 3))
            p.drawPoint(QtCore.QPointF(cx, cy + 10))
        else:
            p.drawLine(QtCore.QPointF(cx - 8, cy - 8), QtCore.QPointF(cx + 8, cy + 8))
            p.drawLine(QtCore.QPointF(cx + 8, cy - 8), QtCore.QPointF(cx - 8, cy + 8))
        p.setPen(QtGui.QColor(col))
        p.setFont(T.font(28, 800))
        p.drawText(QtCore.QPointF(b.left() + 118, b.top() + 54), text)
        n = {s: sum(1 for c in self.checks if c.status == s) for s in ("pass", "warn", "fail", "na")}
        p.setPen(QtGui.QColor("#cbd5e1"))
        p.setFont(T.font(14))
        mode = ("rig mode · the explorer brain on the Pi flies the drone" if self.mode == "rig"
                else "PX4 mode · PX4 flies the route")
        p.drawText(QtCore.QPointF(b.left() + 120, b.top() + 84),
                   f"{n['pass']} passed   ·   {n['warn']} warnings   ·   {n['fail']} failed   ·   "
                   f"{n['na']} not used        {mode}")
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(12))
        p.drawText(QtCore.QRectF(b.left(), b.top() + 14, b.width() - 22, 20), Qt.AlignRight,
                   "REQUIRED checks must pass · watch-only: starting the mission is done in the launcher")
        # two columns of group cards
        colw = (W - GAP) / 2
        for names, x0 in ((LEFT, m), (RIGHT, m + colw + GAP)):
            y = m + BANNER_H + GAP
            for gname, rows in self._groups(names):
                h = HEAD_H + ROW_H * len(rows) + 14
                card = QtCore.QRectF(x0, y, colw, h)
                p.setPen(QtGui.QPen(QtGui.QColor(T.BORDER), 1))
                p.setBrush(QtGui.QColor(T.SURFACE))
                p.drawRoundedRect(card, 16, 16)
                ok = sum(1 for c in rows if c.status in ("pass", "na"))
                p.setPen(QtGui.QColor(T.MUTED))
                p.setFont(T.font(11, 700))
                p.drawText(QtCore.QPointF(x0 + 20, y + 28), gname.upper())
                p.drawText(QtCore.QRectF(x0, y + 14, colw - 20, 18), Qt.AlignRight, f"{ok}/{len(rows)}")
                ry = y + HEAD_H
                for i, c in enumerate(rows):
                    self._row(p, QtCore.QRectF(x0 + 8, ry, colw - 16, ROW_H), c, i > 0)
                    ry += ROW_H
                y += h + GAP

    def _row(self, p, r: QtCore.QRectF, c: C.Check, sep: bool):
        col = T.STATUS_COLOURS[c.status]
        if sep:
            p.setPen(QtGui.QPen(T.colour(T.BORDER, 200), 1))
            p.drawLine(QtCore.QPointF(r.left() + 12, r.top()), QtCore.QPointF(r.right() - 12, r.top()))
        cx, cy = r.left() + 26, r.center().y()
        p.setPen(Qt.NoPen)
        p.setBrush(T.colour(col, 40))
        p.drawEllipse(QtCore.QPointF(cx, cy), 14, 14)
        pen = QtGui.QPen(QtGui.QColor(col), 2.6)
        pen.setCapStyle(Qt.RoundCap)
        p.setPen(pen)
        if c.status == "pass":
            p.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(cx - 6, cy), QtCore.QPointF(cx - 2, cy + 4),
                                            QtCore.QPointF(cx + 6, cy - 5)]))
        elif c.status == "warn":
            p.drawLine(QtCore.QPointF(cx, cy - 6), QtCore.QPointF(cx, cy + 1))
            p.drawPoint(QtCore.QPointF(cx, cy + 6))
        elif c.status == "fail":
            p.drawLine(QtCore.QPointF(cx - 5, cy - 5), QtCore.QPointF(cx + 5, cy + 5))
            p.drawLine(QtCore.QPointF(cx + 5, cy - 5), QtCore.QPointF(cx - 5, cy + 5))
        else:
            p.drawLine(QtCore.QPointF(cx - 5, cy), QtCore.QPointF(cx + 5, cy))
        tag = "REQUIRED" if c.required else "OPTIONAL"
        p.setFont(T.font(10, 700))
        fm = QtGui.QFontMetrics(p.font())
        tw = fm.horizontalAdvance(tag) + 14
        tr = QtCore.QRectF(r.right() - tw - 12, cy - 10, tw, 20)
        p.setPen(QtGui.QPen(T.colour(T.ACCENT if c.required else T.DIM, 160), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(tr, 10, 10)
        p.setPen(QtGui.QColor(T.ACCENT if c.required else T.DIM))
        p.drawText(tr, Qt.AlignCenter, tag)
        p.setPen(QtGui.QColor(T.TEXT if c.status != "na" else T.MUTED))
        p.setFont(T.font(14, 600))
        p.drawText(QtCore.QPointF(r.left() + 52, r.top() + 23), c.name)
        detail = c.detail
        if c.status in ("fail", "warn") and c.hint:
            detail = f"{c.detail}  —  {c.hint}"
        p.setPen(QtGui.QColor(col if c.status in ("fail", "warn") else T.MUTED))
        p.setFont(T.font(12))
        fm = QtGui.QFontMetrics(p.font())
        p.drawText(QtCore.QPointF(r.left() + 52, r.top() + 42),
                   fm.elidedText(detail, Qt.ElideRight, int(tr.left() - r.left() - 64)))


class ChecklistPage(QtWidgets.QScrollArea):
    def __init__(self):
        super().__init__()
        self.view = ChecklistView()
        self.setWidget(self.view)
        self.setWidgetResizable(True)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

    def set_checks(self, checks, mode):
        self.view.set_checks(checks, mode)
