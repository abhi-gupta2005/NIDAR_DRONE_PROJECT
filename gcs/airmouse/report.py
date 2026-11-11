"""Mission report: survivors.csv, map.png and report.pdf (Qt's PDF writer, no extra packages)."""

from __future__ import annotations

import csv
import html
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np

try:
    from PyQt5 import QtCore, QtGui
except ImportError:
    from python_qt_binding import QtCore, QtGui  # type: ignore

from gcs.airmouse import model as M
from gcs.airmouse.widgets import ProMapView


def map_area_m2(mp: Optional[dict]) -> Optional[float]:
    if not mp or mp.get("grid") is None:
        return None
    g = mp["grid"]
    return float(np.count_nonzero(g >= 0)) * float(mp.get("resolution_m", 0.1)) ** 2


def render_map(state, mp, version, path, truth_layout=None, size=(1100, 1000), drone="") -> QtGui.QImage:
    view = ProMapView()
    view.drone_name = drone or view.drone_name
    view.truth_layout = truth_layout
    view.resize(*size)
    view.set_data(state, mp, version)
    view.path = list(path)
    img = QtGui.QImage(size[0], size[1], QtGui.QImage.Format_ARGB32)
    img.fill(QtGui.QColor("#070b14"))
    view.render(img)
    return img


def survivor_rows(state, mdl: M.MissionModel, include_candidates=False):
    rows = []
    for s in (state or {}).get("survivors") or []:
        if s.get("status") != "CONFIRMED" and not include_candidates:
            continue
        fs = mdl.first_seen.get(s.get("id"))
        rows.append({"id": s.get("id"), "status": s.get("status"), "cell": M.where(s), "x_m": s.get("x_m"),
                     "y_m": s.get("y_m"), "std_m": s.get("std_m"), "confidence": s.get("confidence"),
                     "observations": s.get("observations"),
                     "found_at": M.fmt_clock(fs - mdl.t_start) if fs is not None and mdl.t_start is not None
                     else ""})
    return rows


def write_csv(path: Path, rows) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["id", "status", "cell", "x_m", "y_m", "std_m", "confidence", "observations", "found_at_mission_time"])
        for r in rows:
            w.writerow([r["id"], r["status"], r["cell"], r["x_m"], r["y_m"], r["std_m"], r["confidence"],
                        r["observations"], r["found_at"]])


def _f(v, nd=2):
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return "—"


def build_report(out_dir: Path, drone: str, state: Optional[dict], mp: Optional[dict], version: int,
                 mdl: M.MissionModel, snapshots: Dict[str, Path], source_note: str,
                 truth_layout=None) -> Dict[str, Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = survivor_rows(state, mdl)
    cands = [s for s in (state or {}).get("survivors") or [] if s.get("status") != "CONFIRMED"]
    write_csv(out_dir / "survivors.csv", survivor_rows(state, mdl, include_candidates=True))
    img = render_map(state, mp, version, [(x, y) for _, x, y in mdl.path], truth_layout, drone=drone)
    img.save(str(out_dir / "map.png"))
    ex = (state or {}).get("explorer") or {}
    area = map_area_m2(mp)
    when = time.strftime("%d %b %Y, %H:%M", time.localtime(mdl.t_first or time.time()))
    summary = [("Survivors confirmed", str(len(rows))), ("Candidates (not confirmed)", str(len(cands))),
               ("Mission time", M.fmt_clock(mdl.mission_time())), ("Distance flown", f"{mdl.distance:.0f} m"),
               ("Area mapped", f"{area:.0f} m²" if area is not None else "—"),
               ("Brain at the end", f"{ex.get('state') or 'off'}"
                + (f" — {ex.get('return_reason')}" if ex.get("return_reason") else ""))]
    doc = QtGui.QTextDocument()
    doc.addResource(QtGui.QTextDocument.ImageResource, QtCore.QUrl("map.png"), img)
    snaps_html = ""
    for sid, sp in snapshots.items():
        si = QtGui.QImage(str(sp))
        if si.isNull():
            continue
        doc.addResource(QtGui.QTextDocument.ImageResource, QtCore.QUrl(f"snap_{sid}"), si)
        snaps_html += (f"<td style='padding:6px'><img src='snap_{sid}' width='230'><br>"
                       f"<b>{html.escape(sid)}</b> — camera view when confirmed</td>")
    tr = "".join(f"<tr><td style='color:#475569'>{html.escape(k)}</td><td><b>{html.escape(v)}</b></td></tr>"
                 for k, v in summary)
    srows = "".join(
        f"<tr><td><b>{html.escape(str(r['id']))}</b></td><td>{html.escape(r['cell'])}</td><td>{_f(r['x_m'])}</td>"
        f"<td>{_f(r['y_m'])}</td><td>{_f(r['std_m'])}</td><td>{_f(r['confidence'])}</td>"
        f"<td>{r['observations'] if r['observations'] is not None else '—'}</td><td>{r['found_at'] or '—'}</td></tr>"
        for r in rows) or "<tr><td colspan=8>No survivor confirmed.</td></tr>"
    doc.setHtml(f"""
<html><body style="font-family:'Inter','Segoe UI','DejaVu Sans'; color:#0f172a; font-size:10pt">
<p style="color:#0891b2; font-size:9pt; letter-spacing:2px"><b>NIDAR AIRMOUSE · MISSION REPORT</b></p>
<h1 style="margin:0">{html.escape(drone)}</h1>
<p style="color:#475569">{html.escape(when)} · {html.escape(source_note)}</p>
<table cellspacing=0 cellpadding=5 width='100%' style='border:1px solid #cbd5e1'>{tr}</table>
<h2>Map</h2>
<p><img src="map.png" width="640"></p>
<p style="color:#475569; font-size:8.5pt">SLAM occupancy map in the arena frame (x east, y north, metres), flown path,
survivors with their grid cell. Generated by the watch-only ground station.</p>
<h2>Survivors</h2>
<table cellspacing=0 cellpadding=5 width='100%' border=1 style='border-color:#cbd5e1'>
<tr style='background:#e2e8f0'><th>ID</th><th>Cell</th><th>x (m)</th><th>y (m)</th><th>± (m)</th><th>Conf.</th>
<th>Seen</th><th>Found at</th></tr>{srows}</table>
{"<h2>Snapshots</h2><table><tr>" + snaps_html + "</tr></table>" if snaps_html else ""}
</body></html>""")
    pdf_path = out_dir / "report.pdf"
    writer = QtGui.QPdfWriter(str(pdf_path))
    writer.setPageSize(QtGui.QPageSize(QtGui.QPageSize.A4))
    writer.setPageMargins(QtCore.QMarginsF(14, 14, 14, 14), QtGui.QPageLayout.Millimeter)
    writer.setResolution(150)
    writer.setTitle(f"{drone} mission report")
    doc.setPageSize(QtCore.QSizeF(writer.width(), writer.height()))
    doc.print_(writer)
    return {"pdf": pdf_path, "csv": out_dir / "survivors.csv", "map": out_dir / "map.png"}
