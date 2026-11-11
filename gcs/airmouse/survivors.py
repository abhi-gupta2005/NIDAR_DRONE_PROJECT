"""Survivors screen: summary tiles, survivor table, map with the selected survivor, snapshot, report export."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Callable, Dict, Optional

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    from python_qt_binding import QtCore, QtGui, QtWidgets  # type: ignore

from gcs.airmouse import model as M
from gcs.airmouse import report as R
from gcs.airmouse import theme as T
from gcs.airmouse.widgets import Card, ProMapView, StatTile, icon

Qt = QtCore.Qt
COLS = ["ID", "Cell", "x (m)", "y (m)", "± (m)", "Conf.", "Seen", "Found at", "Status"]


class SurvivorsPage(QtWidgets.QWidget):
    def __init__(self, report_dir: Callable[[], Path], drone: str, source_note: Callable[[], str]):
        super().__init__()
        self.report_dir, self.drone, self.source_note = report_dir, drone, source_note
        self.state = self.mp = None
        self.version = 0
        self.model: Optional[M.MissionModel] = None
        self.snapshots: Dict[str, Path] = {}
        self.truth_layout = None
        self.selected: Optional[str] = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(20, 18, 20, 16)
        lay.setSpacing(14)
        tiles = QtWidgets.QHBoxLayout()
        tiles.setSpacing(14)
        self.t_conf = StatTile("Survivors confirmed", T.GREEN)
        self.t_cand = StatTile("Candidates", T.AMBER)
        self.t_time = StatTile("Mission time", T.ACCENT)
        self.t_dist = StatTile("Distance flown", T.VIOLET)
        self.t_area = StatTile("Area mapped", "#2dd4bf")
        for t in (self.t_conf, self.t_cand, self.t_time, self.t_dist, self.t_area):
            tiles.addWidget(t)
        lay.addLayout(tiles)

        self.table = QtWidgets.QTableWidget(0, len(COLS))
        self.table.setHorizontalHeaderLabels(COLS)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        hh.setDefaultAlignment(Qt.AlignCenter)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(40)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.itemSelectionChanged.connect(self._on_select)
        self.cands = QtWidgets.QCheckBox("Show candidates")
        self.cands.toggled.connect(lambda _: self.refresh(force=True))
        btns = QtWidgets.QHBoxLayout()
        btns.addWidget(self.cands)
        btns.addStretch(1)
        self.b_csv = QtWidgets.QPushButton("  Export CSV")
        self.b_csv.setIcon(icon("export", 18))
        self.b_pdf = QtWidgets.QPushButton("  Export PDF report")
        self.b_pdf.setObjectName("primary")
        self.b_pdf.setIcon(icon("export", 18, "white"))
        self.b_csv.clicked.connect(self.export_csv)
        self.b_pdf.clicked.connect(self.export_pdf)
        btns.addWidget(self.b_csv)
        btns.addWidget(self.b_pdf)
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(10)
        lv.addWidget(self.table, 1)
        self.note = QtWidgets.QLabel("")
        self.note.setStyleSheet(f"color:{T.MUTED};")
        self.note.setWordWrap(True)
        lv.addWidget(self.note)
        lv.addLayout(btns)

        self.map = ProMapView()
        self.detail_img = QtWidgets.QLabel()
        self.detail_img.setFixedSize(220, 150)
        self.detail_img.setAlignment(Qt.AlignCenter)
        self.detail_img.setStyleSheet(f"background:#0a1120; border:1px solid {T.BORDER}; border-radius:10px;"
                                      f"color:{T.DIM};")
        self.detail = QtWidgets.QLabel("Select a survivor in the table.")
        self.detail.setTextFormat(Qt.RichText)
        self.detail.setWordWrap(True)
        self.detail.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        drow = QtWidgets.QHBoxLayout()
        drow.setSpacing(14)
        drow.addWidget(self.detail_img)
        drow.addWidget(self.detail, 1)
        dbox = QtWidgets.QWidget()
        dbox.setLayout(drow)

        split = QtWidgets.QSplitter(Qt.Horizontal)
        leftcol = QtWidgets.QSplitter(Qt.Vertical)
        leftcol.addWidget(Card("Survivors", left))
        leftcol.addWidget(Card("Selected survivor", dbox))
        leftcol.setSizes([520, 220])
        split.addWidget(leftcol)
        split.addWidget(Card("Map", self.map, margins=(10, 12, 10, 10)))
        split.setSizes([700, 820])
        lay.addWidget(split, 1)
        self._last = 0.0
        self._note_hold = 0.0

    # ------------------------------------------------------------------ data --
    def set_data(self, state, mp, version, mdl: M.MissionModel, snapshots: Dict[str, Path], truth_layout=None):
        self.state, self.mp, self.version, self.model, self.snapshots = state, mp, version, mdl, snapshots
        self.truth_layout = truth_layout
        self.map.truth_layout = truth_layout

    def refresh(self, force: bool = False):
        if not force and time.time() - self._last < 0.5:
            return
        self._last = time.time()
        st, mdl = self.state, self.model or M.MissionModel()
        self.map.set_data(st, self.mp, self.version)
        self.map.path = [(x, y) for _, x, y in mdl.path] or self.map.path
        survivors = (st or {}).get("survivors") or []
        conf = [s for s in survivors if s.get("status") == "CONFIRMED"]
        self.t_conf.set(str(len(conf)), ", ".join(str(s.get("id")) for s in conf[:8]) or "none yet")
        self.t_cand.set(str(len(survivors) - len(conf)), "seen, not yet confirmed")
        self.t_time.set(M.fmt_clock(mdl.mission_time()), "since the drone started" if mdl.t_start else "not started")
        self.t_dist.set(f"{mdl.distance:.0f} m", "flown path (SLAM)")
        area = R.map_area_m2(self.mp)
        self.t_area.set(f"{area:.0f} m²" if area is not None else "—", "known floor + walls in the SLAM map")
        rows = R.survivor_rows(st, mdl, include_candidates=self.cands.isChecked())
        sel = self.selected
        self.table.blockSignals(True)
        self.table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            vals = [str(row["id"]), row["cell"], R._f(row["x_m"]), R._f(row["y_m"]), R._f(row["std_m"]),
                    R._f(row["confidence"]), str(row["observations"] or "—"), row["found_at"] or "—",
                    row["status"].title()]
            for c, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(v)
                it.setTextAlignment(Qt.AlignCenter)
                if c in (0, 1):
                    it.setFont(T.font(14, 700))
                    it.setForeground(QtGui.QBrush(QtGui.QColor("#fca5a5" if row["status"] == "CONFIRMED" else T.AMBER)))
                self.table.setItem(r, c, it)
            if row["id"] == sel:
                self.table.selectRow(r)
        self.table.blockSignals(False)
        if time.time() > self._note_hold:
            self.note.setText(self.source_note())
        self.map.highlight = sel
        self._show_detail()
        self.map.update()

    def _on_select(self):
        items = self.table.selectedItems()
        self.selected = items[0].text() if items else None
        self.map.highlight = self.selected
        self._show_detail()
        self.map.update()

    def _show_detail(self):
        sid = self.selected
        s = next((s for s in (self.state or {}).get("survivors") or [] if s.get("id") == sid), None)
        if s is None:
            self.detail.setText(f"<span style='color:{T.MUTED}'>Select a survivor in the table.</span>")
            self.detail_img.setPixmap(QtGui.QPixmap())
            self.detail_img.setText("no snapshot")
            return
        snap = self.snapshots.get(sid)
        pm = QtGui.QPixmap(str(snap)) if snap else QtGui.QPixmap()
        if not pm.isNull():
            self.detail_img.setPixmap(pm.scaled(self.detail_img.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.detail_img.setPixmap(QtGui.QPixmap())
            self.detail_img.setText("no snapshot\n(no camera video\nwhen confirmed)")
        mdl = self.model
        fs = mdl.first_seen.get(sid) if mdl else None
        found = M.fmt_clock(fs - mdl.t_start) if fs is not None and mdl and mdl.t_start is not None else "—"
        meth = ", ".join(f"{k} {v}" for k, v in (s.get("methods") or {}).items()) or "—"
        aliases = ", ".join(s.get("aliases") or []) or "—"
        self.detail.setText(
            f"<div style='font-size:22px; font-weight:800; color:#fca5a5'>{sid} &nbsp;·&nbsp; {M.where(s)}</div>"
            f"<table cellspacing=0 cellpadding=3 style='color:{T.TEXT}; font-size:13px; margin-top:6px'>"
            f"<tr><td style='color:{T.MUTED}'>position</td><td>x {R._f(s.get('x_m'))} m &nbsp; y {R._f(s.get('y_m'))} m"
            f" &nbsp; ± {R._f(s.get('std_m'))} m</td></tr>"
            f"<tr><td style='color:{T.MUTED}'>found at</td><td>{found} mission time</td></tr>"
            f"<tr><td style='color:{T.MUTED}'>seen</td><td>{s.get('observations', '—')} times · confidence "
            f"{R._f(s.get('confidence'))}</td></tr>"
            f"<tr><td style='color:{T.MUTED}'>method</td><td>{meth}</td></tr>"
            f"<tr><td style='color:{T.MUTED}'>also seen as</td><td>{aliases}</td></tr></table>")

    # ---------------------------------------------------------------- export --
    def _out(self) -> Path:
        d = self.report_dir() / f"report_{time.strftime('%Y%m%d_%H%M%S')}"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def export_csv(self):
        d = self._out()
        R.write_csv(d / "survivors.csv", R.survivor_rows(self.state, self.model or M.MissionModel(), True))
        self._done(d / "survivors.csv")

    def export_pdf(self):
        d = self._out()
        out = R.build_report(d, self.drone, self.state, self.mp, self.version, self.model or M.MissionModel(),
                             self.snapshots, self.source_note(), self.truth_layout)
        self._done(out["pdf"])

    def _done(self, path: Path):
        self.note.setText(f"<span style='color:{T.GREEN}'>Saved: {path} (path copied)</span>")
        self._note_hold = time.time() + 20
        QtWidgets.QApplication.clipboard().setText(str(path))
