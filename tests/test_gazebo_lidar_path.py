"""
Pi side of the Gazebo LiDAR path (2026-11-02), tested without Gazebo:

  arena ray-caster (stands in for gpu_lidar + ros_gz_bridge + relay)
    -> NIDAR_LIDAR_V1 over TCP  -> nidar_lidar_receiver.serve()
    -> nidar_latest_scan.json  -> GazeboScanFileBackend.next_scan()
    -> LidarScan (NIDAR angles) -> LidarObstacleDetector

What these tests prove: the wire format, the ROS(CCW) -> NIDAR(clockwise)
angle conversion, freshness/staleness behaviour and that a detector sees
a survivor at the right bearing. What they do NOT prove: that Gazebo's
gpu_lidar, ros_gz_bridge or the WSL relay produce these messages -- that is
checked in WSL with the runbook in gazebo/README.md.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from src.simulation import arena_raycast as ar
from src.core import lidar_gazebo_backend as gb
from src.communication import nidar_lidar_receiver as rx

INC = 2.0 * math.pi / ar.SAMPLES


def ros_index(body_deg: float) -> int:
    return int(round((math.radians(body_deg) + math.pi) / INC)) % ar.SAMPLES


@pytest.fixture(scope="module")
def arena():
    path = ar.default_layout(1)
    if not path.exists():
        pytest.skip("gazebo/layouts/nidar_airmouse_arena_s1.json not generated")
    return ar.load_arena(path)


def entry_pose(arena):
    x, y = arena.layout["entry"]["gap_center"]
    return x, y + 0.5, math.pi / 2                      # inside cell H1, facing north


# ----------------------------------------------------------- ray-caster ---

def test_raycast_matches_arena_geometry(arena):
    x, y, yaw = entry_pose(arena)
    r, labels = ar.cast(arena, x, y, yaw, height_m=1.0)
    pitch, wall = arena.layout["grid"]["pitch_m"], arena.layout["grid"]["wall_m"]
    # wall line between rows 0 and 1 is at y = pitch + wall/2; its south face at y = pitch
    assert r[ros_index(0)] == pytest.approx(pitch - y, abs=0.01)
    assert labels[ros_index(0)] == "wall"
    assert math.isinf(r[ros_index(180)]), "behind = the open entry gap, nothing within 12 m"


def test_scan_plane_above_walls_sees_nothing(arena):
    x, y, yaw = entry_pose(arena)
    r, _ = ar.cast(arena, x, y, yaw, height_m=arena.wall_height_m + 0.1)
    assert np.all(np.isinf(r))


# ------------------------------------------------------ angle convention --

def test_ros_ccw_scan_becomes_nidar_clockwise(arena):
    """A wall 0.5 m to the drone's LEFT must appear at NIDAR 270 deg, y < 0 (left)."""
    ranges = np.full(ar.SAMPLES, np.inf)
    ranges[ros_index(90)] = 0.5                          # ROS +90 = left
    ranges[ros_index(-30)] = 2.0                         # ROS -30 = 30 deg right
    msg = ar.laserscan_message(ranges, seq=1)
    msg["received"] = time.time()
    scan = gb.ros_scan_to_lidar_scan(msg, scan_id=0)
    left = np.nanargmin(np.where(scan.valid, scan.distance_m, np.nan))
    assert scan.angle_deg[left] == pytest.approx(270.0, abs=0.5)
    xs, ys = scan.to_xy()
    order = np.argsort(np.hypot(xs, ys))
    assert xs[order[0]] == pytest.approx(0.0, abs=0.01) and ys[order[0]] == pytest.approx(-0.5, abs=0.01)
    assert scan.angle_deg[ros_index(-30)] == pytest.approx(30.0, abs=0.5)
    assert scan.valid.sum() == 2, "inf / null returns are invalid, not zero-range points"


def test_null_and_out_of_range_samples_are_invalid():
    ranges = [None, 0.01, 1.0, 13.0, float("inf")] + [1.0] * 10
    msg = ar.laserscan_message([1.0] * 15, seq=1)
    msg["ranges"] = ranges
    scan = gb.ros_scan_to_lidar_scan(msg, 0, timestamp=0.0)
    assert list(scan.valid[:5]) == [False, False, True, False, False]
    assert np.isnan(scan.distance_m[0]) and np.isnan(scan.distance_m[3])


# -------------------------------------------------------- receiver (TCP) --

@pytest.fixture()
def receiver(tmp_path):
    out = tmp_path / "nidar_latest_scan.json"
    stop, ready, bound, stats = threading.Event(), threading.Event(), [], rx.Stats()
    th = threading.Thread(target=rx.serve, kwargs=dict(host="127.0.0.1", port=0, path=out, stop=stop,
                                                     stats=stats, ready=ready, bound=bound), daemon=True)
    th.start()
    assert ready.wait(2.0)
    yield out, bound[0], stats
    stop.set()
    th.join(2.0)


def wait_for(cond, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


@pytest.mark.network
def test_receiver_stores_valid_scans_and_rejects_bad_lines(arena, receiver):
    out, port, stats = receiver
    x, y, yaw = entry_pose(arena)
    r, _ = ar.cast(arena, x, y, yaw)
    with socket.create_connection(("127.0.0.1", port), timeout=2.0) as s:
        for seq in range(1, 6):
            s.sendall((json.dumps(ar.laserscan_message(r, seq=seq)) + "\n").encode())
        s.sendall(b"not json\n")
        s.sendall((json.dumps({"protocol": "NIDAR_PERCEPTION_V1", "seq": 9}) + "\n").encode())
        bad_inc = ar.laserscan_message(r, seq=10)
        bad_inc["angle_increment"] = 3.0
        s.sendall((json.dumps(bad_inc) + "\n").encode())
        assert wait_for(lambda: stats.received == 5 and stats.rejected == 3)
    saved = json.loads(out.read_text())
    assert saved["seq"] == 5 and saved["protocol"] == "NIDAR_LIDAR_V1"
    assert abs(saved["received"] - time.time()) < 5.0
    assert len(saved["ranges"]) == ar.SAMPLES


@pytest.mark.network
def test_scan_split_across_tcp_segments_is_reassembled(arena, receiver):
    out, port, stats = receiver
    line = (json.dumps(ar.laserscan_message(ar.cast(arena, *entry_pose(arena))[0], seq=42)) + "\n").encode()
    with socket.create_connection(("127.0.0.1", port), timeout=2.0) as s:
        for i in range(0, len(line), 333):
            s.sendall(line[i:i + 333])
            time.sleep(0.002)
        assert wait_for(lambda: stats.received == 1)
    assert json.loads(out.read_text())["seq"] == 42


# --------------------------------------------------- backend freshness ----

def test_backend_returns_each_scan_once_then_raises_stale(tmp_path, arena):
    path = tmp_path / "scan.json"
    r, _ = ar.cast(arena, *entry_pose(arena))
    rx.store_scan(ar.laserscan_message(r, seq=1), path)
    be = gb.GazeboScanFileBackend(path, timeout_s=0.3)
    s1 = be.next_scan()
    assert s1.scan_id == 0 and s1.valid.sum() > 100
    t0 = time.monotonic()
    with pytest.raises(gb.LidarStaleError):
        be.next_scan()                                   # same scan must not be returned twice
    assert 0.25 <= time.monotonic() - t0 < 1.0
    rx.store_scan(ar.laserscan_message(r, seq=2), path)
    assert be.next_scan().scan_id == 1


def test_backend_ignores_scan_file_left_from_previous_run(tmp_path, arena):
    path = tmp_path / "scan.json"
    msg = ar.laserscan_message(ar.cast(arena, *entry_pose(arena))[0], seq=7)
    msg["received"] = time.time() - 60.0                 # an hour-old file would look the same
    path.write_text(json.dumps(msg))
    be = gb.GazeboScanFileBackend(path, timeout_s=0.2)
    with pytest.raises(gb.LidarStaleError):
        be.next_scan()
    assert be.rejected_old == 1


def test_backend_without_any_file_raises_stale(tmp_path):
    with pytest.raises(gb.LidarStaleError):
        gb.GazeboScanFileBackend(tmp_path / "missing.json", timeout_s=0.1).next_scan()


def test_backend_factory_is_env_selected(monkeypatch):
    monkeypatch.delenv("NIDAR_LIDAR_BACKEND", raising=False)
    assert gb.create_backend().name == "STL-19P_SIMULATOR"
    monkeypatch.setenv("NIDAR_LIDAR_BACKEND", "gazebo")
    assert gb.create_backend().name == "GAZEBO_LIDAR"
    monkeypatch.setenv("NIDAR_LIDAR_BACKEND", "SERIAL")
    with pytest.raises(RuntimeError):
        gb.create_backend()


# ------------------------------------------ end to end: TCP -> detector ---

def standing_survivor_view(arena, standoff=1.2):
    """Sensor pose inside the survivor's room, looking at a standing survivor."""
    for s in arena.survivors:
        if s["posture"] != "standing":
            continue
        for k in range(16):
            a = 2 * math.pi * k / 16
            x, y = s["x"] + standoff * math.cos(a), s["y"] + standoff * math.sin(a)
            yaw = math.atan2(s["y"] - y, s["x"] - x)
            r, labels = ar.cast(arena, x, y, yaw, height_m=1.0)
            if labels[ros_index(0)] == s["id"]:        # clear line of sight
                return s, (x, y, yaw)
    pytest.skip("no clear view of a standing survivor in this layout")


@pytest.mark.network
def test_detector_sees_standing_survivor_at_true_bearing_through_tcp(arena, receiver):
    from lidar_obstacle_detector import LidarObstacleDetector, ObstacleDetectorConfig
    out, port, stats = receiver
    s, (x, y, yaw) = standing_survivor_view(arena)
    yaw_off = yaw - math.radians(25.0)                   # survivor ends up 25 deg to the LEFT
    r, _ = ar.cast(arena, x, y, yaw_off, height_m=1.0)
    with socket.create_connection(("127.0.0.1", port), timeout=2.0) as sock:
        sock.sendall((json.dumps(ar.laserscan_message(r, seq=1)) + "\n").encode())
        assert wait_for(lambda: stats.received == 1)
    scan = gb.GazeboScanFileBackend(out, timeout_s=1.0).next_scan()
    obstacles = LidarObstacleDetector(ObstacleDetectorConfig()).detect(scan)
    true_range, true_bearing = ar.bearing_to(x, y, yaw_off, s["x"], s["y"])
    assert true_bearing == pytest.approx(-25.0, abs=0.01)
    nidar_true = true_bearing % 360.0                    # -25 -> 335 (clockwise convention)
    near = [o for o in obstacles if abs((o.bearing_deg - nidar_true + 180) % 360 - 180) < 6.0]
    assert near, f"no LiDAR cluster near {nidar_true:.1f} deg: {[round(o.bearing_deg, 1) for o in obstacles]}"
    best = min(near, key=lambda o: abs(o.distance_m - (true_range - 0.2)))
    assert best.distance_m == pytest.approx(true_range - 0.2, abs=0.15), "range to the body surface"
