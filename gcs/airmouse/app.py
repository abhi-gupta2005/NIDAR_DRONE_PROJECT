"""Main window: top bar, navigation rail, four screens; the 10 Hz refresh loop; flight recording."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path
from typing import Dict, Optional

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    from python_qt_binding import QtCore, QtGui, QtWidgets  # type: ignore

from gcs import nidar_gcs as classic
from gcs.airmouse import checks as C
from gcs.airmouse import model as M
from gcs.airmouse import record as RC
from gcs.airmouse import theme as T
from gcs.airmouse.live import BRAIN_TEXT, LiveView
from gcs.airmouse.preflight import ChecklistPage
from gcs.airmouse.replay import ReplayPage
from gcs.airmouse.survivors import SurvivorsPage
from gcs.airmouse.widgets import NavRail, TopBar, age_level

Qt = QtCore.Qt
REPO = M.REPO
PAGES = ("fly", "checklist", "survivors", "logs")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, source, ros, truth_layout: Optional[dict], drone: str, record_dir: Optional[Path],
                 records_root: Path, reports_dir: Path, demo: bool = False):
        super().__init__()
        self.source, self.ros, self.truth_layout = source, ros, truth_layout
        self.drone = drone.upper()
        self.demo = demo
        self.record_dir, self.records_root, self.reports_dir = record_dir, records_root, reports_dir
        self.recorder: Optional[RC.Recorder] = None
        self.model = M.MissionModel("PX4 route" if os.environ.get("NIDAR_BRAIN_MODE", "").lower() == "route" else "off")
        self.snapshots: Dict[str, Path] = {}
        self._last_state = None
        self._last_map_ver = None
        self._last_check = 0.0
        self.setWindowTitle(f"{self.drone} — NIDAR AirMouse")
        self.setWindowIcon(QtGui.QIcon(classic.drone_pixmap(64)))
        self.resize(1600, 960)
        self.setMinimumSize(1280, 760)

        self.top = TopBar(self.drone)
        self.rail = NavRail([("Fly", "fly", "Live flight view (F1)"), ("Checklist", "check", "Pre-flight checklist (F2)"),
                             ("Survivors", "survivor", "Survivors and report (F3)"),
                             ("Logs", "logs", "Flight records and replay (F4)")])
        self.stack = QtWidgets.QStackedWidget()
        self.live = LiveView()
        self.live.map.drone_name = self.drone
        self.live.map.truth_layout = truth_layout
        self.checklist = ChecklistPage()
        self.survivors = SurvivorsPage(lambda: self.reports_dir, self.drone, self.source_note)
        self.replay = ReplayPage(self.record_roots, lambda: self.reports_dir, self.drone)
        self.replay.truth_layout = truth_layout
        self.replay.map.truth_layout = truth_layout
        for w in (self.live, self.checklist, self.survivors, self.replay):
            self.stack.addWidget(w)
        self.rail.changed.connect(self.show_page)

        body = QtWidgets.QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(self.rail)
        body.addWidget(self.stack, 1)
        central = QtWidgets.QWidget()
        cv = QtWidgets.QVBoxLayout(central)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(0)
        cv.addWidget(self.top)
        cv.addLayout(body, 1)
        self.setCentralWidget(central)
        self.status = self.statusBar()
        self.setStyleSheet(T.STYLE)
        for i, key in enumerate((Qt.Key_F1, Qt.Key_F2, Qt.Key_F3, Qt.Key_F4)):
            QtWidgets.QShortcut(QtGui.QKeySequence(key), self, activated=lambda i=i: self.show_page(i))
        QtWidgets.QShortcut(QtGui.QKeySequence(Qt.Key_Space), self, activated=self._space)
        QtWidgets.QShortcut(QtGui.QKeySequence(Qt.Key_F11), self,
                            activated=lambda: self.showNormal() if self.isFullScreen() else self.showFullScreen())
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.replay.refresh_list()

    # ------------------------------------------------------------------ helpers --
    def record_roots(self):
        roots = [self.records_root, REPO / "logs"]
        if isinstance(self.record_dir, Path):
            roots.append(self.record_dir.parent)
        return roots

    def source_note(self) -> str:
        if self.demo and self.source is not None:
            return "DEMO — " + (self.source.rec.meta.get("note") or "recorded flight played as if live")
        if self.source is None:
            return "No Pi address given (--pi-host): nothing to show yet."
        return f"Live from the Pi mission server {self.source.host}:{self.source.port}"

    def show_page(self, i: int):
        self.stack.setCurrentIndex(i)
        self.rail.select(i)
        if i == 2:
            self.survivors.refresh(force=True)
        if i == 3:
            self.replay.refresh_list()

    def _space(self):
        if self.stack.currentIndex() == 0:
            self.live.swap()
        elif self.stack.currentIndex() == 3:
            self.replay.toggle()

    # ------------------------------------------------------------------ refresh --
    def refresh(self):
        src = self.source
        state, mp, ver = src.snapshot() if src else (None, None, 0)
        now = time.time()
        t = src.sim_time() if self.demo else now
        link_age = (now - src.last_rx) if src is not None and src.connected and src.last_rx else None
        fresh = state is not None and state is not self._last_state
        self._last_state = state if state is not None else self._last_state
        new_events = self.model.update(t, state if fresh else None, link_age)
        img = cmd = truth = None
        px4 = {}
        if self.ros:
            img, cmd, truth, px4 = self.ros.snapshot()
        px4_msgs = {k: v[0] for k, v in px4.items()}
        truth_pose = truth[:3] if truth and now - truth[3] < 2.0 else None
        st = state if self.model.link_up else None
        # flight record (live only)
        if not self.demo and src is not None and self.record_dir is not False:
            if fresh and self.recorder is None:
                folder = self.record_dir or (self.records_root / time.strftime("%Y%m%d_%H%M%S"))
                self.recorder = RC.Recorder(folder, {"source": "live", "drone": self.drone,
                                                     "endpoint": f"{src.host}:{src.port}"})
            if self.recorder is not None:
                if fresh:
                    self.recorder.add_state(t, state)
                if mp is not None and ver != self._last_map_ver:
                    self.recorder.add_map(t, mp)
        self._last_map_ver = ver
        # survivor snapshots + toasts
        for e in new_events:
            if e.kind == "survivor":
                sid = e.text.split()[1]
                self.live.hud.toast(f"SURVIVOR {sid} FOUND", e.text.split(" found at ")[-1])
                cam = self.live.camera
                if cam.qimage is not None and cam.img_age is not None and cam.img_age < 1.5:
                    folder = self.recorder.folder if self.recorder else (self.reports_dir / "snapshots")
                    folder.mkdir(parents=True, exist_ok=True)
                    path = folder / f"snap_{sid}.jpg"
                    cam.grab().save(str(path), "JPG", 90)
                    self.snapshots[sid] = path
        # screens
        page = self.stack.currentIndex()
        self.live.set_data(st, mp, ver, self.model, self.model.link_up, img, px4_msgs, truth_pose) if page == 0 \
            else self.live.map.set_data(st, mp, ver, truth_pose)
        if page == 1 and now - self._last_check > 0.5:
            self._last_check = now
            self._update_checklist(st, link_age, px4_msgs)
        self.survivors.set_data(st, mp, ver, self.model, self.snapshots, self.truth_layout)
        if page == 2:
            self.survivors.refresh()
        self._update_top(st, link_age, px4_msgs)

    def _update_checklist(self, st, link_age, px4):
        ep = f"{self.source.host}:{self.source.port}" if self.source else ""
        img_age = self.live.camera.img_age if self.live.camera.qimage is not None else None
        checks = C.run_checks(st, self.model.link_up, link_age, ep, self.ros is not None, img_age, px4)
        self.checklist.set_checks(checks, C.detect_mode(px4))

    def _update_top(self, st, link_age, px4):
        top = self.top
        h = (st or {}).get("health") or {}
        top.meters["LINK"].set(*age_level(link_age if self.model.link_up else None, 0.5, 2.0),
                               "no link" if not self.model.link_up else f"{link_age:.1f} s")
        for key, hk in (("LIDAR", "lidar"), ("CAMERA", "perception")):
            a = (h.get(hk) or {}).get("age_s")
            top.meters[key].set(*age_level(a), "no data" if a is None else f"age {a:.2f} s")
        sl = h.get("slam") or {}
        if sl.get("status") == "LOST":
            top.meters["SLAM"].set(1, T.RED, "LOST")
        else:
            top.meters["SLAM"].set(*age_level(sl.get("age_s")), f"{sl.get('status', 'no data')}")
        top.meters["PX4"].set(4 if px4 else 0, T.GREEN if px4 else T.GREY, "telemetry" if px4 else "not connected")
        bs = px4.get("battery_status")
        rem = getattr(bs, "remaining", None) if bs is not None else None
        top.battery.set(f"{rem * 100:.0f}%" if rem is not None and rem >= 0 else "—",
                        (T.GREEN if rem > 0.5 else T.AMBER if rem > 0.3 else T.RED) if rem is not None and rem >= 0
                        else T.MUTED)
        top.survivors.set(str(len(M.confirmed(st))))
        top.clock.set(M.fmt_clock(self.model.mission_time()))
        top.rec.set("DEMO" if self.demo else ("REC" if self.recorder else ""))
        ex = (st or {}).get("explorer") or {}
        if not self.model.link_up:
            where = f"{self.source.host}:{self.source.port}" if self.source else "no --pi-host given"
            top.status.set("NO LINK", f"waiting for the Pi mission server ({where})", T.RED)
        elif sl.get("status") == "LOST":
            top.status.set("SLAM LOST", "position not trusted — the brain holds", T.RED)
        elif ex:
            head, col = BRAIN_TEXT.get(ex.get("state"), (str(ex.get("state")), T.GREY))
            detail = M.plan_text(ex) if ex.get("state") in ("EXPLORE", "RETURN") else (
                ex.get("hold_reason") or ex.get("return_reason") or "")
            top.status.set(("AUTONOMOUS · " + head) if ex.get("state") == "EXPLORE" else head, detail, col)
        elif px4:
            top.status.set("PX4 ROUTE", "PX4 flies the fixed route (the brain is not used)", T.ACCENT)
        else:
            top.status.set("CONNECTED", f"SLAM {sl.get('status', 'no data')} · brain off", T.ACCENT)
        rec = f"recording to {self.recorder.folder}" if self.recorder else ("demo playback" if self.demo else "not recording")
        src = ("DEMO playback of a recorded flight" if self.demo else
               (f"Pi {self.source.host}:{self.source.port}" if self.source else "no --pi-host given"))
        self.status.showMessage(f"{src}   ·   ROS 2 {'on' if self.ros else 'off'}   ·   {rec}   ·   "
                                "watch-only: sends no commands   ·   F1–F4 screens · Space swap/play · F11 full screen")

    def closeEvent(self, ev):
        if self.recorder is not None:
            self.recorder.close({"mission_time_s": self.model.mission_time(), "distance_m": self.model.distance})
        super().closeEvent(ev)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="NIDAR AirMouse ground station (watch-only).")
    ap.add_argument("--pi-host", default=os.environ.get("NIDAR_PI_HOST", ""), help="Pi address (mission server)")
    ap.add_argument("--port", type=int, default=int(os.environ.get("NIDAR_MISSION_PORT", "5020")))
    ap.add_argument("--layout", default=None, help="arena layout JSON (ground-truth overlay in simulation)")
    ap.add_argument("--no-ros", action="store_true", help="do not use ROS 2 (no camera video / PX4 telemetry)")
    ap.add_argument("--drone-name", default=classic.DRONE_NAME)
    ap.add_argument("--record-dir", default=None, help="save this flight's record here (default gcs_records/<time>)")
    ap.add_argument("--no-record", action="store_true", help="do not record the flight")
    ap.add_argument("--records", default=str(REPO / "gcs_records"), help="folder of flight records")
    ap.add_argument("--reports", default=str(REPO / "gcs_reports"))
    ap.add_argument("--demo", default=None, metavar="RECORD", help="play a flight record as if it were live")
    ap.add_argument("--demo-speed", type=float, default=8.0)
    ap.add_argument("--demo-start", type=float, default=0.0, help="start the demo at this fraction (0..1)")
    ap.add_argument("--page", choices=PAGES, default="fly")
    ap.add_argument("--replay", default=None, metavar="RECORD", help="open the Logs screen with this record")
    ap.add_argument("--replay-at", type=float, default=None, help="replay position, fraction 0..1")
    ap.add_argument("--select", default=None, help="survivor ID to select on the Survivors screen")
    ap.add_argument("--size", default="1600x960")
    ap.add_argument("--screenshot", default=None, help="save a screenshot after --duration s and exit")
    ap.add_argument("--duration", type=float, default=4.0)
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("NIDAR AirMouse")
    source, demo = None, False
    if args.demo:
        source = RC.ReplaySource(RC.FlightRecord.load(args.demo), args.demo_speed, args.demo_start)
        demo = True
    elif args.pi_host:
        source = classic.MissionClient(args.pi_host, args.port)
        source.start()
    ros = None
    if not args.no_ros:
        try:
            ros = classic.RosLink()
        except Exception as exc:                           # ROS not sourced / not installed
            print(f"[AirMouse GCS] ROS 2 not available ({exc}); no camera video / PX4 telemetry", flush=True)
    record_dir = False if (args.no_record or demo) else (Path(args.record_dir) if args.record_dir else None)
    win = MainWindow(source, ros, classic.load_truth(args.layout), args.drone_name, record_dir,
                     Path(args.records), Path(args.reports), demo)
    if demo and args.demo_start > 0:                     # fast-forward: events, path, clock as if watched from the start
        for t, st in source.rec.states:
            if t >= source.offset:
                break
            win.model.update(t, st, 0.0)
    w, h = (int(v) for v in args.size.lower().split("x"))
    win.resize(w, h)
    win.show_page(PAGES.index(args.page))
    if args.replay:
        win.replay.load(args.replay)
        if args.replay_at is not None and win.replay.rec:
            r = win.replay.rec
            win.replay.seek(r.t0 + args.replay_at * r.duration)
    if args.select:
        win.survivors.selected = args.select
    win.show()
    if args.screenshot:
        def snap():
            win.refresh()
            if args.select:
                win.survivors.refresh(force=True)
            win.grab().save(args.screenshot)
            app.quit()
        QtCore.QTimer.singleShot(int(args.duration * 1000), snap)
    for sig in (signal.SIGINT, signal.SIGTERM):          # Ctrl+C in the launcher: close the record properly
        signal.signal(sig, lambda *_: app.quit())
    rc = app.exec_()
    if win.recorder is not None:
        win.recorder.close({"mission_time_s": win.model.mission_time(), "distance_m": win.model.distance})
    if hasattr(source, "stop"):
        source.stop()
    if ros:
        ros.shutdown()
    return rc
