"""Fly screen: full-size camera (or map) with a picture-in-picture inset and a flight HUD on top.

Click the inset (or press Space) to swap camera and map. While no video arrives the map is the main view.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    from python_qt_binding import QtCore, QtGui, QtWidgets  # type: ignore

from gcs import nidar_gcs as classic
from gcs.airmouse import model as M
from gcs.airmouse import theme as T
from gcs.airmouse.widgets import ProMapView, rounded_label

Qt = QtCore.Qt
BRAIN_TEXT = {"EXPLORE": ("EXPLORING", T.GREEN), "RETURN": ("RETURNING HOME", T.ACCENT), "HOLD": ("HOLD", T.AMBER),
              "COMPLETE": ("MISSION COMPLETE", T.GREEN), "DONE": ("DONE", T.GREEN)}


def compass_heading(yaw_deg: float) -> float:
    """Arena yaw (deg, CCW from +x = east) -> compass heading (deg, clockwise from north = +y)."""
    return (90.0 - yaw_deg) % 360.0


class Hud(QtWidgets.QWidget):
    """Everything drawn over the video: autonomy panel, alerts, mission panel, telemetry bar, compass, toasts."""

    def __init__(self, live: "LiveView"):
        super().__init__(live)
        self.live = live
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WA_NoSystemBackground)
        self.state: Optional[dict] = None
        self.model: Optional[M.MissionModel] = None
        self.link_up = False
        self.px4 = {}
        self.toasts: List[Tuple[float, str, str]] = []      # (expires wall time, headline, detail)

    def toast(self, head: str, detail: str, secs: float = 4.5):
        self.toasts.append((time.time() + secs, head, detail))

    # ------------------------------------------------------------------ paint --
    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        st = self.state or {}
        self._autonomy(p, st)
        self._mission(p, st)
        self._telemetry(p, st)
        self._compass(p, st)
        self._pip_caption(p)
        self._toasts(p)

    def _panel(self, p, rect: QtCore.QRectF, alpha: int = 215):
        p.setPen(QtGui.QPen(T.colour("#2a3a58", 200), 1))
        p.setBrush(T.colour("#0a1220", alpha))
        p.drawRoundedRect(rect, 14, 14)

    def _autonomy(self, p, st):
        ex = st.get("explorer") or {}
        x, y, w = 18.0, 18.0, 400.0
        if not self.link_up:
            head, col, line = "NO LINK", T.RED, "waiting for the Pi mission server"
        elif ex:
            head, col = BRAIN_TEXT.get(ex.get("state"), (str(ex.get("state")), T.GREY))
            line = M.plan_text(ex) or (ex.get("hold_reason") or "")
            if ex.get("state") == "COMPLETE":
                line = ex.get("return_reason") or line
        elif self.px4:
            vs = self.px4.get("vehicle_status")
            nav = classic.NAV_STATES.get(getattr(vs, "nav_state", None), "—") if vs is not None else "—"
            armed = vs is not None and getattr(vs, "arming_state", None) == 2
            head, col, line = f"PX4 {nav}", T.ACCENT, ("ARMED · " if armed else "disarmed · ") + "fixed route flyer"
        else:
            head, col, line = "BRAIN OFF", T.GREY, "no autonomy state from the Pi"
        rect = QtCore.QRectF(x, y, w, 86)
        self._panel(p, rect)
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(11, 700))
        p.drawText(QtCore.QPointF(x + 18, y + 24), "AUTONOMY")
        p.setPen(QtGui.QColor(col))
        p.setFont(T.font(24, 800))
        p.drawText(QtCore.QPointF(x + 18, y + 54), head)
        p.setPen(QtGui.QColor("#cbd5e1"))
        p.setFont(T.font(12.5))
        fm = QtGui.QFontMetrics(p.font())
        p.drawText(QtCore.QPointF(x + 18, y + 74), fm.elidedText(line, Qt.ElideRight, int(w - 36)))
        # current alerts under the panel
        yy = y + 86 + 20
        for kind, text in M.alerts(self.state, self.link_up)[:4]:
            colr = {"alert": T.RED, "warn": T.AMBER, "ok": T.GREEN}.get(kind, T.GREY)
            r = rounded_label(p, x, yy, ("⚠  " if kind != "ok" else "✓  ") + text, T.colour(colr, 225),
                              fg="#0b1220" if kind != "alert" else "white", px=13, pad=12, radius=9)
            yy += r.height() + 8

    def _mission(self, p, st):
        if self.width() < 900:
            return
        w = 330.0
        x, y = self.width() - w - 18, 18.0
        surv = sorted(M.confirmed(st), key=lambda s: (self.model.first_seen.get(s.get("id"), 1e18)
                                                      if self.model else 0))
        evs = [e for e in (self.model.events if self.model else []) if e.kind != "info"][-5:]
        h = 58 + 26 * max(1, min(6, len(surv))) + 40 + 22 * max(1, len(evs)) + 10
        rect = QtCore.QRectF(x, y, w, h)
        self._panel(p, rect)
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(11, 700))
        p.drawText(QtCore.QPointF(x + 18, y + 24), "SURVIVORS FOUND")
        p.setPen(QtGui.QColor(T.GREEN))
        p.setFont(T.font(22, 800))
        p.drawText(QtCore.QRectF(x, y + 6, w - 18, 30), Qt.AlignRight | Qt.AlignVCenter, str(len(surv)))
        yy = y + 52
        if not surv:
            p.setPen(QtGui.QColor(T.DIM))
            p.setFont(T.font(13))
            p.drawText(QtCore.QPointF(x + 18, yy + 14), "none yet")
            yy += 26
        for s in surv[:6]:
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QColor(T.RED))
            p.drawEllipse(QtCore.QPointF(x + 24, yy + 9), 5, 5)
            p.setPen(QtGui.QColor(T.TEXT))
            p.setFont(T.font(14, 700))
            p.drawText(QtCore.QPointF(x + 38, yy + 14), f"{s.get('id')}")
            p.setPen(QtGui.QColor("#cbd5e1"))
            p.setFont(T.font(14))
            p.drawText(QtCore.QPointF(x + 84, yy + 14), M.where(s))
            if self.model and s.get("id") in self.model.first_seen and self.model.t_start is not None:
                p.setPen(QtGui.QColor(T.MUTED))
                p.setFont(T.font(12, mono=True))
                p.drawText(QtCore.QRectF(x, yy, w - 18, 18), Qt.AlignRight | Qt.AlignVCenter,
                           M.fmt_clock(self.model.first_seen[s["id"]] - self.model.t_start))
            yy += 26
        if len(surv) > 6:
            yy -= 4
        yy += 12
        p.setPen(QtGui.QPen(T.colour(T.BORDER), 1))
        p.drawLine(QtCore.QPointF(x + 16, yy), QtCore.QPointF(x + w - 16, yy))
        p.setPen(QtGui.QColor(T.MUTED))
        p.setFont(T.font(11, 700))
        p.drawText(QtCore.QPointF(x + 18, yy + 22), "EVENTS")
        yy += 34
        if not evs:
            p.setPen(QtGui.QColor(T.DIM))
            p.setFont(T.font(12.5))
            p.drawText(QtCore.QPointF(x + 18, yy + 12), "—")
        for e in reversed(evs):
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QColor(T.EVENT_COLOURS.get(e.kind, T.MUTED)))
            p.drawEllipse(QtCore.QPointF(x + 22, yy + 7), 3.5, 3.5)
            p.setPen(QtGui.QColor("#cbd5e1"))
            p.setFont(T.font(12.5))
            fm = QtGui.QFontMetrics(p.font())
            p.drawText(QtCore.QPointF(x + 34, yy + 12), fm.elidedText(e.text, Qt.ElideRight, int(w - 52)))
            yy += 22

    def _telemetry(self, p, st):
        pose = st.get("pose") or {}
        sl = ((st.get("health") or {}).get("slam") or {})
        m = self.model
        spd = m.speed() if m else None
        home = m.home_distance() if m else None
        alt = None
        lp = self.px4.get("vehicle_local_position")
        if lp is not None and getattr(lp, "z", None) is not None:
            alt = -float(lp.z)
        items = [("CELL", str(pose.get("grid") or "—")),
                 ("X", classic.fmt(pose.get("x_m"), 2, " m")), ("Y", classic.fmt(pose.get("y_m"), 2, " m")),
                 ("HDG", f"{compass_heading(pose['yaw_deg']):.0f}°" if pose else "—"),
                 ("SPEED", classic.fmt(spd, 2, " m/s")), ("HOME", classic.fmt(home, 1, " m")),
                 ("ALT", classic.fmt(alt, 2, " m")), ("SLAM", classic.fmt(sl.get("score"), 2))]
        cw = 92.0
        w = cw * len(items) + 24
        pip = self.live.pip_rect()
        x = max(pip.right() + 18, (self.width() - w) / 2)
        if x + w > self.width() - 150:
            x = max(pip.right() + 18, self.width() - 150 - w)
        y = self.height() - 18 - 74
        rect = QtCore.QRectF(x, y, w, 74)
        self._panel(p, rect, 225)
        for i, (lab, val) in enumerate(items):
            cx = x + 12 + i * cw
            if i:
                p.setPen(QtGui.QPen(T.colour(T.BORDER), 1))
                p.drawLine(QtCore.QPointF(cx, y + 16), QtCore.QPointF(cx, y + 58))
            p.setPen(QtGui.QColor(T.MUTED))
            p.setFont(T.font(10.5, 700))
            p.drawText(QtCore.QRectF(cx, y + 10, cw, 16), Qt.AlignCenter, lab)
            p.setPen(QtGui.QColor(T.TEXT if val != "—" else T.DIM))
            p.setFont(T.font(18, 700))
            p.drawText(QtCore.QRectF(cx, y + 28, cw, 34), Qt.AlignCenter, val)

    def _compass(self, p, st):
        pose = st.get("pose")
        R = 58.0
        c = QtCore.QPointF(self.width() - 18 - R - 8, self.height() - 18 - R - 8)
        p.setPen(QtGui.QPen(T.colour("#2a3a58", 220), 1))
        p.setBrush(T.colour("#0a1220", 225))
        p.drawEllipse(c, R + 8, R + 8)
        hdg = compass_heading(pose["yaw_deg"]) if pose else 0.0
        # ring turns so the drone's heading is up
        for d in range(0, 360, 15):
            a = math.radians(d - hdg - 90)
            r0 = R - (9 if d % 90 == 0 else 5)
            p.setPen(QtGui.QPen(T.colour(T.MUTED, 220 if d % 90 == 0 else 120), 1.6 if d % 90 == 0 else 1))
            p.drawLine(QtCore.QPointF(c.x() + r0 * math.cos(a), c.y() + r0 * math.sin(a)),
                       QtCore.QPointF(c.x() + R * math.cos(a), c.y() + R * math.sin(a)))
        for d, lab in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            a = math.radians(d - hdg - 90)
            q = QtCore.QPointF(c.x() + (R - 20) * math.cos(a), c.y() + (R - 20) * math.sin(a))
            p.setPen(QtGui.QColor(T.RED if lab == "N" else "#cbd5e1"))
            p.setFont(T.font(11, 800))
            p.drawText(QtCore.QRectF(q.x() - 8, q.y() - 8, 16, 16), Qt.AlignCenter, lab)
        # home direction on the ring
        hb = self.model.home_bearing_deg() if self.model else None
        hd = self.model.home_distance() if self.model else None
        if pose and hb is not None and hd is not None and hd > 0.3:
            a = math.radians(compass_heading(hb) - hdg - 90)
            q = QtCore.QPointF(c.x() + (R + 1) * math.cos(a), c.y() + (R + 1) * math.sin(a))
            p.setPen(QtGui.QPen(QtGui.QColor("#0b1220"), 1.5))
            p.setBrush(QtGui.QColor(T.YELLOW))
            p.drawEllipse(q, 8, 8)
            p.setPen(QtGui.QColor("#0b1220"))
            p.setFont(T.font(9, 800))
            p.drawText(QtCore.QRectF(q.x() - 8, q.y() - 8, 16, 16), Qt.AlignCenter, "H")
        # drone, nose up
        if pose:
            classic.draw_quad_icon(p, c.x(), c.y(), math.pi / 2, 28)
        p.setPen(QtGui.QColor(T.TEXT))
        p.setFont(T.font(12, 800))
        p.drawText(QtCore.QRectF(c.x() - 30, c.y() + 14, 60, 16), Qt.AlignCenter, f"{hdg:.0f}°" if pose else "—")

    def _pip_caption(self, p):
        r = self.live.pip_rect()
        p.setPen(QtGui.QPen(T.colour(T.ACCENT, 170), 1.5))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(QtCore.QRectF(r).adjusted(-1, -1, 1, 1), 10, 10)
        lab = "MAP" if not self.live.map_is_main else "CAMERA"
        rounded_label(p, r.left() + 10, r.top() + 16, f"{lab}  ·  click to swap", T.colour("#0a1220", 210),
                      fg="#cbd5e1", px=11, pad=8)

    def _toasts(self, p):
        now = time.time()
        self.toasts = [t for t in self.toasts if t[0] > now]
        y = 30.0
        for exp, head, detail in self.toasts[-2:]:
            a = max(0.0, min(1.0, (exp - now) / 0.6))
            w = 440.0
            rect = QtCore.QRectF((self.width() - w) / 2, y, w, 70)
            p.setOpacity(a)
            p.setPen(QtGui.QPen(QtGui.QColor(T.GREEN), 2))
            p.setBrush(T.colour("#052e1c", 235))
            p.drawRoundedRect(rect, 16, 16)
            p.setPen(QtGui.QColor(T.GREEN))
            p.setFont(T.font(20, 800))
            p.drawText(QtCore.QRectF(rect.left(), rect.top() + 8, w, 30), Qt.AlignCenter, head)
            p.setPen(QtGui.QColor("#d1fae5"))
            p.setFont(T.font(13))
            p.drawText(QtCore.QRectF(rect.left(), rect.top() + 38, w, 22), Qt.AlignCenter, detail)
            p.setOpacity(1.0)
            y += 80


class LiveView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.camera = classic.CameraView()
        self.camera.setParent(self)
        self.map = ProMapView()
        self.map.overlays = False
        self.map.setParent(self)
        self.hud = Hud(self)
        self.map_is_main = True
        self.user_swapped = False
        for w in (self.camera, self.map):
            w.installEventFilter(self)
            w.setCursor(Qt.ArrowCursor)
        self._layout()

    # ---------------------------------------------------------------- layout --
    def pip_rect(self) -> QtCore.QRect:
        w = int(max(280, min(420, self.width() * 0.24)))
        h = int(w * 0.72)
        return QtCore.QRect(18, self.height() - 18 - h, w, h)

    def _layout(self):
        main, pip = (self.map, self.camera) if self.map_is_main else (self.camera, self.map)
        main.setGeometry(self.rect())
        pip.setGeometry(self.pip_rect())
        self.map.compact = not self.map_is_main
        main.lower()
        pip.raise_()
        self.hud.setGeometry(self.rect())
        self.hud.raise_()
        pip.setCursor(Qt.PointingHandCursor)
        main.setCursor(Qt.ArrowCursor)

    def resizeEvent(self, _):
        self._layout()

    def swap(self):
        self.user_swapped = True
        self.map_is_main = not self.map_is_main
        self._layout()
        self.update_views()

    def eventFilter(self, obj, ev):
        pip = self.camera if self.map_is_main else self.map
        if obj is pip and ev.type() == QtCore.QEvent.MouseButtonPress:
            self.swap()
            return True
        return False

    # ------------------------------------------------------------------ data --
    def set_data(self, state, mp, version, model, link_up, image, px4, truth_pose=None):
        if image is not None:
            self.camera.set_image(image)
        if state:
            cam = state.get("camera") or {}
            self.camera.objects = cam.get("objects") or []
            self.camera.cam_size = (cam.get("width", 640), cam.get("height", 480))
        self.map.set_data(state, mp, version, truth_pose)
        video = self.camera.qimage is not None and self.camera.img_age is not None and self.camera.img_age < 3.0
        if not self.user_swapped and self.map_is_main == video:      # auto: video main when there is video
            self.map_is_main = not video
            self._layout()
        self.hud.state, self.hud.model, self.hud.link_up, self.hud.px4 = state, model, link_up, px4 or {}
        self.update_views()

    def update_views(self):
        self.camera.update()
        self.map.update()
        self.hud.update()
