"""
Pi mission server (src/mission/nidar_mission_server.py, 2026-11-02e): reads the
SLAM V6 / V18 / perception files, maps survivors, serves NIDAR_MISSION_V1 to
ground stations. Tested with files written the way the real processes write
them, and over loopback TCP.
"""

from __future__ import annotations

import base64
import json
import math
import socket
import threading
import time
import zlib
from pathlib import Path

import numpy as np
import pytest

from src.mission import nidar_mission_server as ms
from src.mission.survivor_mapper import CameraModel, Grid
from src.simulation.arena_feed import project_bbox

START = (7.9, -0.75, math.pi / 2)


@pytest.fixture()
def files(tmp_path, monkeypatch):
    f = {k: tmp_path / Path(v).name for k, v in ms.FILES.items()}
    monkeypatch.setattr(ms, "FILES", f)
    monkeypatch.setattr(ms, "OUT_DIR", tmp_path / "mission")
    return f


def write(path, data):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data))
    tmp.replace(path)


def slam_state(frame, x, y, yaw_deg, t):
    return {"protocol": "NIDAR_SLAM_V6", "timestamp": t, "frame": frame,
            "pose": {"x_m": x, "y_m": y, "yaw_deg": yaw_deg}, "match": {"score": 0.8, "accepted": True},
            "statistics": {"processing_fps": 7.7, "accepted": frame, "rejected": 0},
            "map": {"resolution_m": 0.1, "origin_m": [-20.0, -20.0], "shape": [400, 400]}, "lidar_source": "GAZEBO_LIDAR"}


def v18_json(frame, t, box, rng):
    cam = {"id": 1, "class": "person", "confidence": 0.8, "bbox": box, "current_observation": True}
    return {"timestamp": t, "frame": frame, "lidar_source": "GAZEBO_LIDAR", "camera_objects": [cam],
            "fused_objects": [{"camera_id": 1, "range_m": rng, "class": "person"}],
            "persistent_objects": [{"state": "ACTIVE"}]}


def run_frames(state, files, n=8):
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    for k in range(n):
        t = time.time()
        write(files["slam"], slam_state(k + 1, 1.0, 0.0, 0.0, t))       # 1 m forward of the pad = north
        write(files["v18"], v18_json(k + 1, t, box, 2.0))
        write(files["perception"], {"frame": k + 1, "camera": {"width": 640, "height": 480, "source": "gazebo_sim"},
                                    "objects": [{}]})
        state.step(time.time())
        time.sleep(0.2)


def test_pose_and_survivor_in_arena_frame(files):
    state = ms.MissionState(Grid(), START, CameraModel(height_m=0.82))
    run_frames(state, files)
    snap, _ = state.messages()
    assert snap["pose"]["x_m"] == pytest.approx(7.9, abs=1e-6) and snap["pose"]["y_m"] == pytest.approx(0.25)
    assert snap["pose"]["grid"] == "H1"
    conf = [s for s in snap["survivors"] if s["status"] == "CONFIRMED"]
    assert len(conf) == 1 and conf[0]["grid"] == "H3"            # 1 m + 2.2 m north of the pad
    assert conf[0]["y_m"] == pytest.approx(-0.75 + 1.0 + 2.2, abs=0.05)
    assert snap["camera"]["objects"][0]["bbox"] == pytest.approx(project_bbox(2.2, 0.0, "standing", 0.82)[0])
    assert snap["health"]["slam"]["fps"] == 7.7 and snap["health"]["v18"]["active"] == 1


def test_stale_v18_or_missing_pose_maps_nothing(files):
    state = ms.MissionState(Grid(), START, CameraModel(height_m=0.82))
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    write(files["v18"], v18_json(1, time.time() - 5.0, box, 2.0))   # 5 s old
    state.step(time.time())
    write(files["v18"], v18_json(2, time.time(), box, 2.0))         # fresh, but no SLAM pose yet
    state.step(time.time())
    assert state.mapper.measurements == 0 and state.skipped_no_pose == 1


def test_map_message_roundtrip(files):
    grid = np.full((400, 400), -1, dtype=np.int8)
    grid[200:210, 190:195] = 100
    grid[195:215, 185:200][grid[195:215, 185:200] < 0] = 0
    np.save(files["grid"], grid)
    state = ms.MissionState(Grid(), START, CameraModel())
    write(files["slam"], slam_state(1, 0, 0, 0, time.time()))
    state.step(time.time())
    _, mp = state.messages()
    arr = np.frombuffer(zlib.decompress(base64.b64decode(mp["data"])), dtype=np.int8).reshape(mp["shape"])
    assert mp["shape"] == [20, 15] and (arr == 100).sum() == 50
    assert mp["origin_m"] == pytest.approx([-20.0 + 185 * 0.1, -20.0 + 195 * 0.1])


@pytest.mark.network
def test_ground_station_receives_state_and_map(files):
    np.save(files["grid"], np.zeros((400, 400), dtype=np.int8))
    state = ms.MissionState(Grid(), START, CameraModel(height_m=0.82))
    run_frames(state, files, n=2)
    stop, ready, bound = threading.Event(), threading.Event(), []
    th = threading.Thread(target=ms.serve, args=(state, stop, "127.0.0.1", 0, ready, bound), daemon=True)
    th.start()
    assert ready.wait(2)
    types = []
    with socket.create_connection(("127.0.0.1", bound[0]), timeout=3) as s:
        buf = b""
        t0 = time.time()
        while time.time() - t0 < 2.0 and len(types) < 4:
            buf += s.recv(1 << 20)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                types.append(json.loads(line)["type"])
    stop.set()
    assert types[0] == "map" and "state" in types
    ms.write_outputs(state)
    assert json.loads((ms.OUT_DIR / "survivors.json").read_text())["grid"]["cells"] == 14


def test_no_survivors_placed_while_slam_is_lost(files):
    state = ms.MissionState(Grid(), START, CameraModel(height_m=0.82))
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    for k in range(8):
        t = time.time()
        st = slam_state(k + 1, 1.0, 0.0, 0.0, t)
        st["status"] = "LOST"
        st["match"] = {"score": 0.0, "accepted": False, "reject_streak": 12 + k}
        write(files["slam"], st)
        write(files["v18"], v18_json(k + 1, t, box, 2.0))
        state.step(time.time())
        time.sleep(0.05)
    snap, _ = state.messages()
    assert state.mapper.measurements == 0 and state.skipped_bad_pose >= 7
    assert snap["health"]["slam"]["status"] == "LOST" and snap["pose"]["trusted"] is False


def test_measurement_journal_is_written_for_offline_replay(files):
    state = ms.MissionState(Grid(), START, CameraModel(height_m=0.82))
    assert state.journal_path.parent == ms.OUT_DIR
    run_frames(state, files)
    ms.write_outputs(state)
    rows = state.journal_path.read_text().splitlines()
    assert rows[0].split(",")[:3] == ["t", "outcome", "survivor"]
    assert len(rows) == 1 + 8 and all(",INLIER," in r for r in rows[1:])
    assert ms.write_journal(state) == 0                    # drained: nothing written twice
    snap, _ = state.messages()
    assert {"edge_clipped", "ghosts_absorbed"} <= set(snap["health"]["mapper"])


def test_pose_on_the_launch_pad_is_labelled_pad(files):
    """2026-11-09 run: the drone back on the pad showed 'Cell OUT' (the pad is outside the grid)."""
    state = ms.MissionState(Grid(), START, CameraModel(height_m=0.82))
    write(files["slam"], slam_state(1, 0.2, 0.0, 0.0, time.time()))
    state.step(time.time())
    assert state.messages()[0]["pose"]["grid"] == "PAD"
