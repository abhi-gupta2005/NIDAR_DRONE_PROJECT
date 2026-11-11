"""
WSL-side Gazebo helpers (gazebo/ros2/*.py), ROS-independent parts (2026-11-02):

  * nidar_lidar_relay.scan_to_wire + LatestLineSender  -> the REAL Pi receiver
    -> GazeboScanFileBackend  (relay and Pi agree on the wire format)
  * nidar_camera_relay.LatestFrameServer -> the REAL Windows GazeboCameraCapture
  * nidar_rig_driver.RouteFollower driven on a kinematic model through the
    generated arena: reaches the goal and never touches a wall.

The ROS subscriptions/publishers themselves (rclpy, ros_gz_bridge, Gazebo)
are not exercised here -- see gazebo/README.md for the WSL checks.
"""

from __future__ import annotations

import importlib.util
import json
import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from src.simulation import arena_raycast as ar
from src.core import lidar_gazebo_backend as gb
from src.communication import nidar_lidar_receiver as rx

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "gazebo" / "ros2" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def arena():
    path = ar.default_layout(1)
    if not path.exists():
        pytest.skip("arena layout not generated")
    return ar.load_arena(path)


def fake_laserscan(ranges, stamp=12.5):
    n = len(ranges)
    inc = 2 * math.pi / n
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=int(stamp), nanosec=int((stamp % 1) * 1e9)),
                               frame_id="x500_nidar_airmouse_0/nidar_lidar_link/nidar_lidar"),
        angle_min=-math.pi, angle_max=-math.pi + inc * (n - 1), angle_increment=inc,
        range_min=0.15, range_max=12.0, scan_time=0.1, time_increment=0.0,
        ranges=[float(r) for r in ranges])


def test_scan_to_wire_matches_receiver_contract(arena):
    relay = load("nidar_lidar_relay")
    r, _ = ar.cast(arena, 7.9, 0.5, math.pi / 2)
    r = list(r)
    r[3] = float("-inf")
    r[4] = float("nan")
    wire = relay.scan_to_wire(fake_laserscan(r), seq=3, now=100.0)
    assert wire["protocol"] == gb.PROTOCOL == rx.PROTOCOL
    assert wire["stamp"] == pytest.approx(12.5) and wire["scan_rate_hz"] == pytest.approx(10.0)
    assert wire["ranges"][3] is None and wire["ranges"][4] is None
    json.dumps(wire, allow_nan=False)                     # strict JSON: no NaN/Infinity tokens
    ref = ar.laserscan_message(r, seq=3)
    for k in ("angle_min", "angle_max", "angle_increment"):
        assert wire[k] == pytest.approx(ref[k])
    gb.ros_scan_to_lidar_scan(dict(wire, received=time.time()), 0)       # backend accepts it


@pytest.mark.network
def test_lidar_relay_to_pi_receiver_to_backend(arena, tmp_path):
    relay = load("nidar_lidar_relay")
    out = tmp_path / "scan.json"
    stop, ready, bound, stats = threading.Event(), threading.Event(), [], rx.Stats()
    th = threading.Thread(target=rx.serve, kwargs=dict(host="127.0.0.1", port=0, path=out, stop=stop,
                                                     stats=stats, ready=ready, bound=bound), daemon=True)
    th.start()
    assert ready.wait(2)
    sender = relay.LatestLineSender("127.0.0.1", bound[0])
    sender.start()
    try:
        backend = gb.GazeboScanFileBackend(out, timeout_s=2.0)
        x, y, yaw = 7.9, 0.5, math.pi / 2
        r, _ = ar.cast(arena, x, y, yaw)
        for seq in range(1, 4):
            sender.submit((json.dumps(relay.scan_to_wire(fake_laserscan(r), seq)) + "\n").encode())
            scan = backend.next_scan()
        assert stats.rejected == 0
        assert scan.valid.sum() == int(np.isfinite(r).sum())
        # forward ray (ROS 0 rad = index 250) -> NIDAR 0 deg, same range
        assert scan.angle_deg[250] == pytest.approx(0.0, abs=1e-6)
        assert scan.distance_m[250] == pytest.approx(r[250], abs=1e-3)
    finally:
        sender.stop()
        stop.set()
        th.join(2)


@pytest.mark.network
def test_lidar_sender_keeps_only_the_newest_scan():
    relay = load("nidar_lidar_relay")
    s = relay.LatestLineSender("127.0.0.1", 9)          # never started: nothing is sent
    for i in range(5):
        s.submit(f"{i}\n".encode())
    assert s.dropped == 4 and s._line == b"4\n"


@pytest.mark.network
def test_camera_relay_server_feeds_windows_capture(windows_module):
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "imencode"):
        pytest.skip("real OpenCV needed")
    cam = load("nidar_camera_relay")
    server = cam.LatestFrameServer("127.0.0.1", 0)
    server.start()
    try:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:, 320:] = 255

        def feed():
            for _ in range(30):
                if server.client is not None:
                    server.submit(cam.encode_frame(img))
                    return
                time.sleep(0.05)

        threading.Thread(target=feed, daemon=True).start()
        cap = windows_module.GazeboCameraCapture(f"127.0.0.1:{server.port}", connect_timeout=3.0)
        assert cap.isOpened() and cap.get(cv2.CAP_PROP_FRAME_WIDTH) == 640
        ok, frame = cap.read()
        assert ok and frame[:, :300].mean() < 20 and frame[:, 340:].mean() > 235
        cap.release()
    finally:
        server.stop()


# ----------------------------------------------------------- rig driver ---

def drive(follower, start, dt=0.05, max_steps=60000):
    x, y, yaw = start
    trace = [(x, y, yaw)]
    for _ in range(max_steps):
        vx, vy, wz, done = follower.update(x, y, yaw)
        if done:
            return trace, True
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw += wz * dt
        trace.append((x, y, yaw))
    return trace, False


@pytest.mark.parametrize("route", ["survivor:S2", "survivor:S6", "rooms"])
def test_rig_route_reaches_goal_without_touching_walls(arena, route):
    drv = load("nidar_rig_driver")
    path = drv.build_route(arena.layout, route)
    sp = arena.layout["spawn_pose"]
    trace, done = drive(drv.RouteFollower(path, speed=0.4), (sp["x"], sp["y"], sp["yaw"]))
    assert done, f"route {route} not finished"
    end = trace[-1]
    assert math.hypot(end[0] - path[-1][0], end[1] - path[-1][1]) < 0.06
    walls = ar.Arena(arena.segments, arena.wall_height_m, [], arena.layout)
    clearance = min(ar.cast(walls, x, y, 0.0, 1.0, samples=36)[0].min() for x, y, _ in trace[::5])
    assert clearance > 0.25, f"rig came within {clearance:.2f} m of a wall"
    if route.startswith("survivor:"):
        s = next(v for v in arena.survivors if v["id"] == route.split(":")[1])
        _, brg = ar.bearing_to(*end, s["x"], s["y"])
        assert abs(brg) < 4.0, "the route must end facing the survivor"


def test_odometry_frame_handles_world_and_relative_odometry():
    drv = load("nidar_rig_driver")
    spawn = (7.9, -0.75, math.pi / 2)
    world = drv.OdomFrame(spawn)
    assert world.to_arena(7.9, -0.75, math.pi / 2) == pytest.approx((7.9, -0.75, math.pi / 2))
    assert world.relative is False
    rel = drv.OdomFrame(spawn)
    assert rel.to_arena(0.0, 0.0, 0.0) == pytest.approx((7.9, -0.75, math.pi / 2))
    x, y, yaw = rel.to_arena(1.0, 0.0, 0.0)                     # 1 m forward = 1 m north
    assert (x, y) == pytest.approx((7.9, 0.25)) and yaw == pytest.approx(math.pi / 2)
