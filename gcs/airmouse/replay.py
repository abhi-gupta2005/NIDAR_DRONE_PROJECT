"""Logs screen: list of flight records, replay with a timeline (events marked), speed control, report export."""

from __future__ import annotations

import bisect
import time
from pathlib import Path
from typing import Callable, List, Optional

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    from python_qt_binding import QtCore, QtGui, QtWidgets  # type: ignore

from gcs.airmouse import model as M
from gcs.airmouse import record as RC
from gcs.airmouse import report as R
from gcs.airmouse import theme as T
from gcs.airmouse.widgets import Card, ProMapView, icon

Qt = QtCore.Qt
SPEEDS = (1, 4, 16, 64)


def analyse(rec: RC.FlightRecord) -> M.MissionModel:
    """Run the same MissionModel the Fly screen uses over the whole record (events, first-seen, path)."""
    mdl = M.MissionModel()
    for t, st in rec.states:
        mdl.update(t, st, 0.0)
    mdl.events = [e for e in mdl.events if "mission server link" not in e.text]     # no link in a replay
    return mdl


class Timeline(QtWidgets.QWidget):
    seeked = QtCore.pyqtSignal(float) if hasattr(QtCore, "pyqtSignal") else QtCore.Signal(float)

    def __init__(self):
        super().__init__()
        self.t0, self.t1, self.t = 0.0, 1.0, 0.0
        self.events: List[M.Event] = []
        self.t_start = self.t_end = None
        self.setFixedHeight(54)
        self.setCursor(Qt.PointingHandCursor)

    def set_record(self, t0, t1, events, t_start, t_end):
        self.t0, self.t1 = t0, max(t1, t0 + 1e-3)
        self.events, self.t_start, self.t_end = events, t_start, t_end
        self.t = t0
        self.update()

    def set_time(self, t):
        self.t = t
        self.update()

    def _x(self, t):
        return 14 + (t - self.t0) / (self.t1 - self.t0) * (self.width() - 28)

    def _t(self, x):
        return self.t0 + (x - 14) / max(1, self.width() - 28) * (self.t1 - self.t0)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        y = 30.0
        track = QtCore.QRectF(14, y - 3, self.width() - 28, 6)
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QColor("#1b2840"))
        p.drawRoundedRect(track, 3, 3)
        if self.t_start is not None:
            a, b = self._x(self.t_start), self._x(self.t_end if self.t_end is not None else self.t1)
            p.setBrush(T.colour(T.ACCENT, 45))
            p.drawRoundedRect(QtCore.QRectF(a, y - 3, b - a, 6), 3, 3)
        p.setBrush(QtGui.QColor(T.ACCENT))
        p.drawRoundedRect(QtCore.QRectF(14, y - 3, self._x(self.t) - 14, 6), 3, 3)
        for e in self.events:
            if e.kind in ("info",):
                continue
            x = self._x(e.t)
            col = QtGui.QColor(T.EVENT_COLOURS.get(e.kind, T.MUTED))
            if e.kind == "survivor":
                p.setBrush(col)
                p.setPen(QtGui.QPen(QtGui.QColor("#0b1220"), 1.2))
                p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x, y - 20), QtCore.QPointF(x + 6, y - 13),
                                               QtCore.QPointF(x, y - 6), QtCore.QPointF(x - 6, y - 13)]))
            else:
                p.setPen(QtGui.QPen(col, 2))
                p.drawLine(QtCore.QPointF(x, y + 6), QtCore.QPointF(x, y + 14))
        x = self._x(self.t)
        p.setPen(QtGui.QPen(QtGui.QColor("#0b1220"), 2))
        p.setBrush(QtGui.QColor("white"))
        p.drawEllipse(QtCore.QPointF(x, y), 9, 9)

    def mousePressEvent(self, ev):
        self.seeked.emit(min(self.t1, max(self.t0, self._t(ev.pos().x()))))

    def mouseMoveEvent(self, ev):
        if ev.buttons() & Qt.LeftButton:
            self.seeked.emit(min(self.t1, max(self.t0, self._t(ev.pos().x()))))


class ReplayPage(QtWidgets.QWidget):
    def __init__(self, roots: Callable[[], List[Path]], report_dir: Callable[[], Path], drone: str):
        super().__init__()
        self.roots, self.report_dir, self.drone = roots, report_dir, drone
        self.rec: Optional[RC.FlightRecord] = None
        self.model: Optional[M.MissionModel] = None
        self.path_t: List[float] = []
        self.t = 0.0
        self.playing = False
        self.speed = 16
        self.truth_layout = None

        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(20, 18, 20, 16)
        lay.setSpacing(14)
        # records list
        self.list = QtWidgets.QListWidget()
        self.list.itemClicked.connect(lambda it: self.load(it.data(Qt.UserRole)))
        b_ref = QtWidgets.QPushButton("Refresh")
        b_ref.clicked.connect(self.refresh_list)
        b_open = QtWidgets.QPushButton("Open file…")
        b_open.clicked.connect(self._open_file)
        lb = QtWidgets.QHBoxLayout()
        lb.addWidget(b_ref)
        lb.addWidget(b_open)
        lw = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(lw)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(self.list, 1)
        lv.addLayout(lb)
        left = Card("Flight records", lw)
        left.setFixedWidth(300)
        lay.addWidget(left)
        # map + timeline
        self.map = ProMapView()
        self.timeline = Timeline()
        self.timeline.seeked.connect(self.seek)
        self.b_play = QtWidgets.QPushButton()
        self.b_play.setIcon(icon("play", 20))
        self.b_play.setFixedSize(46, 40)
        self.b_play.clicked.connect(self.toggle)
        self.speed_btns = []
        ctr = QtWidgets.QHBoxLayout()
        ctr.setSpacing(8)
        ctr.addWidget(self.b_play)
        for s in SPEEDS:
            b = QtWidgets.QPushButton(f"{s}×")
            b.setObjectName("seg")
            b.setCheckable(True)
            b.setChecked(s == self.speed)
            b.clicked.connect(lambda _, s=s: self.set_speed(s))
            self.speed_btns.append(b)
            ctr.addWidget(b)
        ctr.addSpacing(12)
        self.time_lab = QtWidgets.QLabel("--:-- / --:--")
        self.time_lab.setFont(T.font(16, 700, mono=True))
        ctr.addWidget(self.time_lab)
        ctr.addStretch(1)
        self.b_report = QtWidgets.QPushButton("  Export report for this flight")
        self.b_report.setIcon(icon("export", 18, "white"))
        self.b_report.setObjectName("primary")
        self.b_report.clicked.connect(self.export_report)
        ctr.addWidget(self.b_report)
        mid = QtWidgets.QWidget()
        mv = QtWidgets.QVBoxLayout(mid)
        mv.setContentsMargins(0, 0, 0, 0)
        mv.setSpacing(8)
        mv.addWidget(self.map, 1)
        mv.addWidget(self.timeline)
        mv.addLayout(ctr)
        self.title = QtWidgets.QLabel("No flight loaded")
        self.title.setFont(T.font(13, 600))
        self.title.setStyleSheet(f"color:{T.MUTED};")
        lay.addWidget(Card("Replay", mid, margins=(12, 12, 12, 12), right=self.title), 1)
        # moment panel + events
        self.moment = QtWidgets.QLabel()
        self.moment.setTextFormat(Qt.RichText)
        self.moment.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.moment.setWordWrap(True)
        self.events = QtWidgets.QListWidget()
        self.events.setStyleSheet("QListWidget::item { padding:5px 8px; margin:1px 2px; border-radius:6px; }")
        self.events.itemClicked.connect(lambda it: self.seek(it.data(Qt.UserRole)))
        rw = QtWidgets.QWidget()
        rv = QtWidgets.QVBoxLayout(rw)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.setSpacing(10)
        rv.addWidget(self.moment)
        ev_lab = QtWidgets.QLabel("EVENTS · click to jump")
        ev_lab.setObjectName("cardTitle")
        rv.addWidget(ev_lab)
        rv.addWidget(self.events, 1)
        right = Card("At this moment", rw)
        right.setFixedWidth(330)
        lay.addWidget(right)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self._wall = None
        self.note = ""

    # ---------------------------------------------------------------- records --
    def refresh_list(self):
        self.list.clear()
        for r in RC.list_records(self.roots()):
            meta, summ = r["meta"], r["summary"]
            when = time.strftime("%d %b %Y · %H:%M", time.localtime(meta.get("t", r["mtime"])))
            dur = M.fmt_clock(summ.get("duration_s")) if summ.get("duration_s") is not None else "—"
            n = len(summ.get("survivors") or []) if summ else None
            src = (meta.get("source") or "live").upper()
            it = QtWidgets.QListWidgetItem(f"{when}\n{src} · {dur}" + (f" · {n} survivors" if n is not None else "")
                                           + f"\n{r['folder'].name}")
            it.setData(Qt.UserRole, str(r["path"]))
            it.setToolTip(str(r["path"]))
            self.list.addItem(it)
        if not self.list.count():
            it = QtWidgets.QListWidgetItem("No flight records yet.\nThe Fly screen records every\nconnected flight.")
            it.setFlags(Qt.NoItemFlags)
            self.list.addItem(it)

    def _open_file(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open flight record", str(self.roots()[0]),
                                                      "Flight records (flight.jsonl.gz)")
        if fn:
            self.load(fn)

    def load(self, path):
        if not path:
            return
        rec = RC.FlightRecord.load(path)
        if not rec.states:
            self.title.setText("This record has no mission data")
            return
        self.rec, self.model = rec, analyse(rec)
        self.path_t = [t for t, _, _ in self.model.path]
        src = (rec.meta.get("source") or "live").upper()
        when = time.strftime("%d %b %Y %H:%M", time.localtime(rec.meta.get("t", rec.t0)))
        self.title.setText(f"{src} · {when} · {M.fmt_clock(rec.duration)}" + ("" if rec.complete else " · cut off"))
        self.note = rec.meta.get("note") or ""
        self.timeline.set_record(rec.t0, rec.t1, self.model.events, self.model.t_start, self.model.t_end)
        self.events.clear()
        for e in self.model.events:
            if e.kind == "info":
                continue
            it = QtWidgets.QListWidgetItem(f"{M.fmt_clock(e.t - rec.t0)}   {e.text}")
            it.setForeground(QtGui.QBrush(QtGui.QColor(T.EVENT_COLOURS.get(e.kind, T.TEXT))))
            it.setData(Qt.UserRole, e.t)
            self.events.addItem(it)
        self.map.path = []
        self.seek(rec.t0)

    # --------------------------------------------------------------- playback --
    def set_speed(self, s):
        self.speed = s
        for b, v in zip(self.speed_btns, SPEEDS):
            b.setChecked(v == s)

    def toggle(self):
        if not self.rec:
            return
        self.playing = not self.playing
        if self.playing and self.t >= self.rec.t1:
            self.t = self.rec.t0
        self.b_play.setIcon(icon("pause" if self.playing else "play", 20))
        self._wall = time.time()
        if self.playing:
            self.timer.start(50)
        else:
            self.timer.stop()

    def _tick(self):
        now = time.time()
        dt = now - (self._wall or now)
        self._wall = now
        t = self.t + dt * self.speed
        if t >= self.rec.t1:
            t = self.rec.t1
            self.toggle()
        self.seek(t)

    def seek(self, t: float):
        if not self.rec:
            return
        self.t = t
        st, _ = self.rec.state_at(t)
        mp, k = self.rec.map_at(t)
        self.map.set_data(st, mp, k)
        n = bisect.bisect_right(self.path_t, t)
        self.map.path = [(x, y) for _, x, y in self.model.path[:n]]
        self.map.update()
        self.timeline.set_time(t)
        self.time_lab.setText(f"{M.fmt_clock(t - self.rec.t0)} / {M.fmt_clock(self.rec.duration)}")
        self._moment(st, t)

    def _moment(self, st, t):
        st = st or {}
        ex = st.get("explorer") or {}
        pose = st.get("pose") or {}
        mdl = self.model
        mt = max(0.0, min(t, mdl.t_end or t) - mdl.t_start) if mdl.t_start is not None and t >= mdl.t_start else 0.0
        found = [(sid, ft) for sid, ft in mdl.first_seen.items() if ft <= t]
        dist = 0.0
        n = bisect.bisect_right(self.path_t, t)
        pts = mdl.path[:n]
        for a, b in zip(pts, pts[1:]):
            dist += ((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
        rows = [("mission time", M.fmt_clock(mt)), ("brain", ex.get("state") or "off"),
                ("cell", str(pose.get("grid") or "—")), ("distance", f"{dist:.0f} m")]
        surv = "".join(f"<span style='color:#fca5a5; font-weight:700'>{sid}</span> {mdl.where_seen.get(sid, '')}"
                       f" <span style='color:{T.MUTED}'>{M.fmt_clock(ft - mdl.t_start) if mdl.t_start else ''}</span><br>"
                       for sid, ft in found) or f"<span style='color:{T.DIM}'>none yet</span>"
        plan = M.plan_text(ex)
        self.moment.setText(
            "<table cellspacing=0 cellpadding=4 width='100%'>"
            + "".join(f"<tr><td style='color:{T.MUTED}'>{k}</td><td align=right style='font-size:15px; "
                      f"font-weight:700'>{v}</td></tr>" for k, v in rows) + "</table>"
            + (f"<p style='color:{T.PINK}; margin-top:6px'>{plan}</p>" if plan else "")
            + f"<p style='color:{T.MUTED}; font-size:11px; font-weight:700; letter-spacing:1px; margin-top:10px'>"
              f"SURVIVORS FOUND · {len(found)}</p><p style='font-size:14px'>{surv}</p>"
            + (f"<p style='color:{T.DIM}; font-size:11px'>{self.note}</p>" if self.note else ""))

    def export_report(self):
        if not self.rec:
            return
        st, _ = self.rec.state_at(self.rec.t1)
        mp, k = self.rec.map_at(self.rec.t1)
        d = self.report_dir() / f"report_{self.rec.path.parent.name}"
        snaps = {p.stem[5:]: p for p in self.rec.path.parent.glob("snap_*.jpg")}
        src = f"recorded flight {self.rec.path.parent.name}" + (f" — {self.note}" if self.note else "")
        out = R.build_report(d, self.drone, st, mp, k, self.model, snaps, src, self.truth_layout)
        self.title.setText(f"Report saved: {out['pdf']}")
        QtWidgets.QApplication.clipboard().setText(str(out["pdf"]))
