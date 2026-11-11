"""
Ground station (gcs/nidar_gcs.py, 2026-11-02e), offscreen Qt:
  * the map-image transform puts SLAM cells where survivor_mapper.compose() puts them;
  * against a REAL mission server (loopback) the window connects, lists the confirmed
    survivor with its grid cell and logs the event; screenshot export works.
Not covered here: ROS 2 subscriptions (camera, PX4) - checked in WSL.
"""

from __future__ import annotations

import math
import os
import threading
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
from PyQt5 import QtCore  # noqa: E402

from gcs import nidar_gcs as g  # noqa: E402
from src.mission import nidar_mission_server as ms  # noqa: E402
from src.mission.survivor_mapper import CameraModel, Grid, compose  # noqa: E402
from src.simulation.arena_feed import project_bbox  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_map_image_transform_matches_compose(app):
    mp = {"start_pose": [7.9, -0.75, 90.0], "resolution_m": 0.1, "origin_m": [-2.0, -3.0]}
    T = g.MapView.image_to_arena(mp)
    for u, v in ((0, 0), (10, 0), (0, 20), (37, 12)):
        q = T.map(QtCore.QPointF(u, v))
        x, y, _ = compose((7.9, -0.75, math.pi / 2), (-2.0 + 0.1 * u, -3.0 + 0.1 * v, 0.0))
        assert (q.x(), q.y()) == pytest.approx((x, y), abs=1e-9)


@pytest.mark.network
def test_window_shows_survivor_from_live_mission_server(app, tmp_path, monkeypatch):
    import json
    files = {k: tmp_path / os.path.basename(str(v)) for k, v in ms.FILES.items()}
    monkeypatch.setattr(ms, "FILES", files)
    state = ms.MissionState(Grid(), (7.9, -0.75, math.pi / 2), CameraModel(height_m=0.82))
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    for k in range(7):
        t = time.time()
        for f, d in ((files["slam"], {"timestamp": t, "frame": k + 1, "pose": {"x_m": 1.0, "y_m": 0.0, "yaw_deg": 0.0},
                                      "statistics": {"processing_fps": 7.7}, "match": {"score": 0.8}}),
                     (files["v18"], {"timestamp": t, "frame": k + 1, "camera_objects": [
                         {"id": 1, "class": "person", "confidence": 0.8, "bbox": box, "current_observation": True}],
                         "fused_objects": [{"camera_id": 1, "range_m": 2.0}]})):
            f.write_text(json.dumps(d))
        state.step(time.time())
        time.sleep(0.2)
    stop, ready, bound = threading.Event(), threading.Event(), []
    threading.Thread(target=ms.serve, args=(state, stop, "127.0.0.1", 0, ready, bound), daemon=True).start()
    assert ready.wait(2)
    client = g.MissionClient("127.0.0.1", bound[0])
    client.start()
    win = g.MainWindow(client, None, None, tmp_path / "reports")
    win.show()
    t0 = time.time()
    while time.time() - t0 < 3.0 and win.table.rowCount() == 0:
        app.processEvents()
        win.refresh()
        time.sleep(0.05)
    try:
        assert client.connected
        assert win.table.rowCount() == 1
        assert win.table.item(0, 0).text() == "S1" and win.table.item(0, 2).text() == "H3"
        assert "SURVIVOR S1 found at grid H3" in win.log.toPlainText()
        win.export_report()
        out = next((tmp_path / "reports").iterdir())
        assert (out / "map.png").stat().st_size > 1000 and "H3" in (out / "survivors.csv").read_text()
    finally:
        client.stop()
        stop.set()
        win.close()


def test_unknown_arena_shows_the_brain_plan(app, tmp_path, monkeypatch):
    """2026-11-09: no layout (arena unknown) + the explorer service running. The mission server
    passes the brain's plan in the arena frame; the window fits the view to the map, draws the
    plan, shows the BRAIN pill and logs the brain's events."""
    import json
    import numpy as np
    files = {k: tmp_path / os.path.basename(str(v)) for k, v in ms.FILES.items()}
    monkeypatch.setattr(ms, "FILES", files)
    layout, grid, start, cam = ms.load_config()                     # NIDAR_ARENA_LAYOUT not set
    assert not grid.bounded and grid.label(-3.0, -4.0) == "--"
    state = ms.MissionState(grid, start, cam)
    assert state.mapper.inside_arena(-30.0, 12.0)
    g_ = np.full((400, 400), -1, dtype=np.int8)
    g_[180:220, 170:230] = 0
    g_[180:220, 170] = 100
    np.save(files["grid"], g_)
    t = time.time()
    files["slam"].write_text(json.dumps({"timestamp": t, "frame": 1, "pose": {"x_m": -1.0, "y_m": 0.5, "yaw_deg": 30},
                                         "match": {"score": 0.9, "accepted": True}, "status": "TRACKING",
                                         "map": {"resolution_m": 0.1, "origin_m": [-20.0, -20.0]}}))
    files["explorer"].write_text(json.dumps({
        "timestamp": t, "state": "EXPLORE", "hold_reason": None, "plan_id": 3, "stats": {"looks": 2},
        "drone_radius_m": 0.25, "events": ["plan 3: explore: 120 frontier cells left -> goal (2.00, 1.00)"],
        "plan": {"waypoints": [[-1.0, 0.5], [1.0, 0.5], [2.0, 1.0]], "end": "look", "goal": [2.0, 1.0],
                 "reason": "explore"}}))
    state.step(time.time())
    snap, mp = state.messages()
    ex = snap["explorer"]
    assert ex["state"] == "EXPLORE" and ex["plan"]["goal"] == [2.0, 1.0] and snap["grid"]["bounded"] is False

    class FakeClient:
        host, port, connected, error, last_rx = "fake", 0, True, "", time.time()

        def snapshot(self):
            m = dict(mp)
            m["grid"] = np.frombuffer(__import__("zlib").decompress(__import__("base64").b64decode(m["data"])),
                                      dtype=np.int8).reshape(m["shape"])
            return snap, m, 1
    win = g.MainWindow(FakeClient(), None, None, tmp_path / "reports")
    win.resize(1500, 950)
    win.show()
    for _ in range(3):
        win.refresh()
        app.processEvents()
    x0, x1, y0, y1 = win.map.bounds()
    assert x0 < -3.0 and x1 > 3.0 and y0 < -2.0 and y1 > 2.0            # fitted to the mapped area
    assert "EXPLORE" in win.header.pills["BRAIN"].text()
    assert "BRAIN plan 3" in win.log.toPlainText()
    img = win.map.grab().toImage()
    T = win.map.world_transform()
    q = T.map(QtCore.QPointF(2.0, 1.0))                                 # goal ring is magenta
    found = False
    for dx in range(-11, 12):
        c = img.pixelColor(int(q.x()) + dx, int(q.y()))
        if c.red() > 180 and c.blue() > 180 and c.green() < 160:
            found = True
    assert found
    win.map.grab().save(str(tmp_path / "brain_view.png"))


def test_brain_pill_says_px4_route_instead_of_off(app, tmp_path, monkeypatch):
    """PX4 runs fly the preset route (the brain cannot fly PX4 yet): the pill says so instead of 'off'."""
    win = g.MainWindow(None, None, None, tmp_path / "reports")
    win._update_header({"health": {}}, False, {})
    assert "off" in win.header.pills["BRAIN"].text()
    monkeypatch.setenv("NIDAR_BRAIN_MODE", "route")
    win._update_header({"health": {}}, False, {})
    assert "PX4 route" in win.header.pills["BRAIN"].text()
