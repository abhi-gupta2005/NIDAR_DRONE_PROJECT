"""
AirMouse ground station app (gcs/airmouse/, 2026-11-10), offscreen Qt where needed:
  * pre-flight checks: healthy -> READY; SLAM lost / no link / missing fusion -> NOT READY;
    PX4 telemetry is required only in PX4 mode;
  * MissionModel: one event per survivor, link loss named, mission clock, speed, flown distance;
  * flight records: decimated to 2 Hz, round trip, a record cut off by a crash still loads;
  * the window: all four screens with a demo source, CSV + PDF export, replay seek;
  * (slow) tools/make_demo_record.py writes a record the app can load.
Watch-only: nothing here sends a command anywhere.
"""

from __future__ import annotations

import base64
import gzip
import math
import os
import zlib

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from gcs.airmouse import checks as C  # noqa: E402
from gcs.airmouse import model as M  # noqa: E402
from gcs.airmouse import record as RC  # noqa: E402


def state(t=100.0, x=7.9, y=-0.75, grid="PAD", ex_state="EXPLORE", survivors=(), slam_status="TRACKING",
          perception_age=0.2):
    return {"type": "state", "timestamp": t,
            "pose": {"x_m": x, "y_m": y, "yaw_deg": 90.0, "grid": grid, "trusted": slam_status == "TRACKING"},
            "start_pose": [7.9, -0.75, 90.0], "grid": {"pitch_m": 1.05, "wall_m": 0.05, "cells": 14, "bounded": True},
            "health": {"lidar": {"age_s": 0.1, "points": 500}, "perception": {"age_s": perception_age, "frame": 5},
                       "v18": {"age_s": 0.3, "fused": 1},
                       "slam": {"age_s": 0.1, "fps": 7.5, "score": 0.8, "status": slam_status}},
            "explorer": {"state": ex_state, "plan_id": 3,
                         "plan": {"waypoints": [[8.0, 1.0]], "goal": [8.0, 1.0], "end": "look",
                                  "reason": "explore: 10 frontier cells left -> goal (1, 0)"}, "events": []},
            "survivors": list(survivors), "camera": {"width": 640, "height": 480, "objects": []}}


def surv(sid="S1", cell="E3", x=4.7, y=2.7, status="CONFIRMED"):
    return {"id": sid, "status": status, "grid": cell, "x_m": x, "y_m": y, "std_m": 0.1, "observations": 9,
            "inliers": 8, "methods": {"lidar": 9}, "confidence": 0.8, "aliases": []}


def by_name(checks):
    return {c.name: c for c in checks}


# ------------------------------------------------------------------ checks --
def test_healthy_rig_mission_is_ready():
    cs = C.run_checks(state(), True, 0.2, "pi:5020", ros_on=True, image_age=0.1, px4={}, mode="rig")
    assert C.verdict(cs)[0] == "ready", [(c.name, c.status) for c in cs if c.status != "pass"]
    assert by_name(cs)["PX4 telemetry"].status == "na"


def test_slam_lost_and_no_link_block_the_mission():
    cs = C.run_checks(state(slam_status="LOST"), True, 0.2, mode="rig")
    assert by_name(cs)["SLAM tracking"].status == "fail" and C.verdict(cs)[0] == "blocked"
    cs = C.run_checks(state(), False, None, mode="rig")
    n = by_name(cs)
    assert n["Pi mission server"].status == "fail" and n["LiDAR scans reaching the Pi"].status == "fail"


def test_missing_camera_pipeline_blocks_and_says_where_to_look():
    cs = C.run_checks(state(perception_age=None), True, 0.2, mode="rig")
    c = by_name(cs)["Camera detections reaching the Pi"]
    assert c.status == "fail" and c.required and "nidar_yolo" in c.hint


def test_px4_telemetry_required_only_in_px4_mode():
    cs = C.run_checks(state(), True, 0.2, mode="px4")
    assert by_name(cs)["PX4 telemetry"].status == "fail" and C.verdict(cs)[0] == "blocked"
    assert by_name(cs)["Explorer brain"].status == "na"


def test_drone_away_from_pad_is_only_a_warning():
    cs = C.run_checks(state(grid="E3"), True, 0.2, ros_on=True, image_age=0.1, mode="rig")
    assert by_name(cs)["Drone on the take-off pad"].status == "warn"
    assert C.verdict(cs)[0] == "warn"


# ------------------------------------------------------------------- model --
def test_model_events_clock_speed_distance():
    m = M.MissionModel()
    m.update(0.0, state(0.0), 0.1)
    assert m.t_start == 0.0                                  # brain EXPLORE = mission started
    for k in range(1, 11):                                   # 1 m at 0.5 m/s
        ss = [surv()] if k >= 5 else []
        m.update(k * 0.2, state(k * 0.2, x=7.9, y=-0.75 + 0.1 * k, grid="H1", survivors=ss), 0.1)
    kinds = [e.text for e in m.events if e.kind == "survivor"]
    assert kinds == ["Survivor S1 found at E3"]
    assert m.speed() == pytest.approx(0.5, rel=0.05) and m.distance == pytest.approx(1.0, abs=0.06)
    assert m.mission_time() == pytest.approx(2.0)
    m.update(2.2, None, 9.0)                                 # nothing new for 9 s: link lost
    assert any(e.kind == "alert" and "LOST" in e.text for e in m.events)
    m.update(2.4, state(2.4, ex_state="COMPLETE", y=0.25), 0.1)
    m.update(3.0, state(3.0, ex_state="COMPLETE", y=0.25), 0.1)
    assert m.t_end == 2.4 and m.mission_time(10.0) == pytest.approx(2.4)
    assert any("COMPLETE" in e.text for e in m.events)


def test_alerts_worst_first():
    a = M.alerts(state(slam_status="LOST", perception_age=None), True)
    assert a[0][0] == "alert" and "SLAM" in a[0][1] and any("Camera" in t for _, t in a)
    assert M.alerts(None, False)[0][0] == "alert"


# ----------------------------------------------------------------- records --
def map_msg(t):
    g = np.full((20, 30), -1, np.int8)
    g[5:15, 5:25] = 0
    g[5, 5:25] = 100
    return {"type": "map", "timestamp": t, "resolution_m": 0.1, "origin_m": [-1.0, -1.0], "shape": [20, 30],
            "start_pose": [7.9, -0.75, 90.0], "data": base64.b64encode(zlib.compress(g.tobytes())).decode()}


def make_record(folder, n=50, dt=0.1):
    rec = RC.Recorder(folder, {"source": "test", "drone": "T"})
    for k in range(n):
        t = 1000.0 + k * dt
        ss = [surv()] if k > n // 2 else []
        rec.add_state(t, state(t, y=-0.75 + 0.02 * k, grid="H1" if k else "PAD", survivors=ss))
        if k % 10 == 0:
            rec.add_map(t, map_msg(t))
    rec.close()
    return rec


def test_record_round_trip_at_2_hz(tmp_path):
    make_record(tmp_path / "f1")
    r = RC.FlightRecord.load(tmp_path / "f1")
    assert r.complete and len(r.states) == 10 and len(r.maps) == 5     # 5 s of 10 Hz -> 2 Hz
    assert r.meta["source"] == "test" and r.duration == pytest.approx(4.5)
    st, k = r.state_at(1002.05)
    assert k == 4 and st["timestamp"] == pytest.approx(1002.0)
    mp, _ = r.map_at(1002.05)
    assert mp["grid"].shape == (20, 30) and mp["grid"][5, 10] == 100
    recs = RC.list_records([tmp_path])
    assert len(recs) == 1 and recs[0]["summary"]["survivors"] == ["S1"]


def test_record_cut_off_by_a_crash_still_loads(tmp_path):
    make_record(tmp_path / "f2")
    raw = (tmp_path / "f2" / RC.RECORD_NAME).read_bytes()
    (tmp_path / "f2" / RC.RECORD_NAME).write_bytes(raw[: len(raw) * 2 // 3])
    r = RC.FlightRecord.load(tmp_path / "f2")
    assert not r.complete and 0 < len(r.states) < 10


def test_replay_source_plays_the_record_as_live(tmp_path):
    make_record(tmp_path / "f3")
    src = RC.ReplaySource(RC.FlightRecord.load(tmp_path / "f3"), speed=1.0, start_frac=0.5)
    st, mp, k = src.snapshot()
    assert st is not None and mp is not None and src.connected and src.last_rx is not None


# --------------------------------------------------------------------- app --
@pytest.fixture(scope="module")
def qapp():
    QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_window_all_screens_and_exports(qapp, tmp_path):
    from gcs.airmouse.app import MainWindow
    make_record(tmp_path / "records" / "f4", n=80)
    rec = RC.FlightRecord.load(tmp_path / "records" / "f4")
    src = RC.ReplaySource(rec, speed=1.0, start_frac=0.9)
    win = MainWindow(src, None, None, "test drone", False, tmp_path / "records", tmp_path / "reports", demo=True)
    win.resize(1500, 900)
    for t, st in rec.states:
        if t < src.offset:
            win.model.update(t, st, 0.0)
    for page in range(4):
        win.show_page(page)
        win.refresh()
        assert not win.grab().isNull()
    assert win.top.survivors.value == "1" and win.top.rec.mode == "DEMO"
    assert "AUTONOMOUS" in win.top.status.head
    win.show_page(2)
    win.survivors.refresh(force=True)
    assert win.survivors.table.rowCount() == 1 and win.survivors.table.item(0, 1).text() == "E3"
    win.survivors.export_csv()
    win.survivors.export_pdf()
    pdfs = list((tmp_path / "reports").glob("report_*/report.pdf"))
    assert pdfs and pdfs[0].read_bytes()[:4] == b"%PDF"
    assert "S1,CONFIRMED,E3" in next((tmp_path / "reports").glob("report_*/survivors.csv")).read_text()
    # replay
    win.show_page(3)
    win.replay.load(str(tmp_path / "records" / "f4" / RC.RECORD_NAME))
    win.replay.seek(rec.t0 + 0.75 * rec.duration)
    assert win.replay.map.path and any(e.kind == "survivor" for e in win.replay.model.events)
    assert win.replay.list.count() == 1
    win.close()


def test_live_view_swaps_camera_and_map(qapp):
    from gcs.airmouse.live import LiveView, compass_heading
    v = LiveView()
    v.resize(1200, 800)
    v.set_data(state(), None, 0, M.MissionModel(), True, None, {})
    assert v.map_is_main                                     # no video: the map is the main view
    v.swap()
    assert not v.map_is_main and v.map.compact
    assert compass_heading(90.0) == pytest.approx(0.0) and compass_heading(0.0) == pytest.approx(90.0)


@pytest.mark.slow
def test_demo_record_generator(tmp_path):
    import importlib.util
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location("mdr", root / "tools/make_demo_record.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main(["--out", str(tmp_path / "demo")]) == 0
    r = RC.FlightRecord.load(tmp_path / "demo")
    assert r.meta["source"] == "demo" and "No camera pipeline" in r.meta["note"]
    last, _ = r.state_at(r.t1)
    assert len(M.confirmed(last)) == 6 and r.duration > 600
    with gzip.open(tmp_path / "demo" / RC.RECORD_NAME, "rt") as f:
        assert '"flight_control":"NONE (read-only)"' in f.read()
