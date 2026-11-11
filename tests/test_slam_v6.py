"""
SLAM V6 (src/core/nidar_slam_v6.py, 2026-11-02): scan-to-map matcher replacing
the V5.1 centroid matcher (SLAM-01) -- V5.1 itself is unchanged and keeps its
strict xfail in test_slam.py.

Data: the generated competition arena, ray-cast scans (500 beams, 1 cm noise)
converted through the same NIDAR_LIDAR_V1 -> LidarScan path the Pi uses, along
vehicle-like trajectories (0.4 m/s, yaw <= 0.8 rad/s, 7.7 Hz = rate measured
in Gazebo).
Measured when written: S2 route 16.9 m -> final error 0.041 m (max 0.044);
rooms routes 126-157 m in arenas s1-s3 -> final <= 0.031 m, max <= 0.142 m.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from src.simulation import arena_raycast as ar
from src.core import nidar_slam_v6 as s6
from src.core.lidar_gazebo_backend import ros_scan_to_lidar_scan

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def arena():
    path = ar.default_layout(1)
    if not path.exists():
        pytest.skip("arena layout not generated")
    return ar.load_arena(path)


def scan_at(arena, x, y, yaw, i, rng, noise=0.01):
    r, _ = ar.cast(arena, x, y, yaw, 1.0)
    r = r.copy()
    fin = np.isfinite(r)
    r[fin] += rng.normal(0, noise, fin.sum())
    msg = ar.laserscan_message(r, seq=i)
    msg["received"] = 0.0
    return s6.scan_to_points(ros_scan_to_lidar_scan(msg, i, 0.0))


def run(arena, traj, noise=0.01):
    rng = np.random.default_rng(0)
    slam = s6.SlamV6()
    x0, y0, th0 = traj[0][1:]
    errs = []
    for i, (t, x, y, yaw) in enumerate(traj):
        est = slam.process(scan_at(arena, x, y, yaw, i, rng, noise), t)
        c, s = math.cos(th0), math.sin(th0)
        ax, ay = x0 + c * est["x"] - s * est["y"], y0 + s * est["x"] + c * est["y"]
        errs.append(math.hypot(ax - x, ay - y))
    return slam, np.array(errs)


def test_body_frame_is_x_forward_y_left(arena):
    """A wall 0.5 m to the LEFT must appear at +y in the SLAM map (V5.1 mirrored it)."""
    from src.core.lidar_interface import LidarScan
    ang_cw = np.arange(0, 360, 0.72)
    d = np.full(ang_cw.shape, np.nan)
    left = np.abs(((ang_cw - 270 + 180) % 360) - 180) < 20          # NIDAR 270 deg = left
    d[left] = 0.5 / np.cos(np.radians(ang_cw[left] - 270))
    scan = LidarScan(0, 0.0, ang_cw, d, np.isfinite(d), ["x"] * len(d), 10.0, 5000)
    pts = s6.scan_to_points(scan)
    assert np.all(pts[:, 1] == pytest.approx(0.5, abs=1e-6)) and abs(pts[:, 0]).max() < 0.2
    slam = s6.SlamV6()
    slam.process(pts, 0.0)
    lev = slam.levels[-1]
    j = int((0.5 - lev.origin) / lev.res)
    i = int((0.0 - lev.origin) / lev.res)
    assert lev.L[j, i] > 0, "wall cell must be occupied at +y (left)"


def test_straight_flight_into_the_maze(arena):
    sp = arena.layout["spawn_pose"]
    traj = [(k / 7.7, sp["x"], sp["y"] + 0.05 * k, sp["yaw"]) for k in range(40)]   # 2 m north through the entry
    slam, err = run(arena, traj)
    assert err[-1] < 0.08 and err.max() < 0.12, err
    assert slam.rejected == 0


def test_bad_scan_is_rejected_and_map_not_corrupted(arena):
    sp = arena.layout["spawn_pose"]
    traj = [(k / 7.7, sp["x"], sp["y"] + 0.05 * k, sp["yaw"]) for k in range(20)]
    slam, _ = run(arena, traj)
    before = slam.levels[-1].L.copy()
    pose_before = slam.pose.copy()
    rng = np.random.default_rng(9)
    junk = rng.uniform(-6, 6, size=(300, 2))                           # nothing like the map
    r = slam.process(junk, traj[-1][0] + 0.13)
    assert r["accepted"] is False
    assert np.array_equal(before, slam.levels[-1].L), "rejected scan must not change the map"
    assert math.hypot(*(slam.pose[:2] - pose_before[:2])) < 0.1


@pytest.mark.slow
def test_route_to_survivor_s2_stays_within_10cm(arena):
    from src.simulation.arena_trajectory import route_trajectory
    slam, err = run(arena, route_trajectory(arena.layout, "survivor:S2"))
    assert err[-1] < 0.10 and err.max() < 0.15, (err[-1], err.max())
    assert slam.rejected == 0


@pytest.mark.slow
def test_slam_v6_process_with_gazebo_backend(arena, tmp_path):
    """Real process: reads Gazebo-format scans, writes monitor-compatible state + grid, survives an outage."""
    from src.simulation.arena_feed import ArenaFeeder
    from src.simulation.arena_trajectory import route_trajectory
    feeder = ArenaFeeder(arena, tmp_path)
    env = dict(os.environ, NIDAR_LIDAR_BACKEND="GAZEBO", NIDAR_SCAN_FILE=str(feeder.scan_file),
               PYTHONUNBUFFERED="1", PYTHONPATH=os.pathsep.join([str(ROOT), str(ROOT / "src" / "core")]))
    traj = route_trajectory(arena.layout, "survivor:S2")[:60]
    with open(tmp_path / "slam6.log", "w") as log:
        proc = subprocess.Popen([sys.executable, "-m", "src.core.nidar_slam_v6"], env=env, cwd=str(tmp_path),
                                stdout=log, stderr=subprocess.STDOUT)
        try:
            time.sleep(1.5)
            for t, x, y, yaw in traj:
                feeder.step((x, y, yaw), camera=False, lidar=True)
                time.sleep(0.12)
            time.sleep(1.5)
            alive = proc.poll() is None
        finally:
            proc.terminate()
            proc.wait(5)
    text = (tmp_path / "slam6.log").read_text()
    assert alive and "LiDAR stale" in text, text[-1500:]
    st = json.loads((tmp_path / "outputs/slam/state/nidar_slam_v6_state.json").read_text())
    assert st["protocol"] == "NIDAR_SLAM_V6" and st["lidar_source"] == "GAZEBO_LIDAR"
    assert {"x_m", "y_m", "yaw_deg"} <= set(st["pose"]) and "processing_fps" in st["statistics"]
    grid = np.load(tmp_path / "outputs/slam/maps/nidar_slam_v6_grid.npy")
    assert grid.shape == tuple(st["map"]["shape"]) and (grid > 60).sum() > 100
    # pose against truth (SLAM frame = start pose)
    x0, y0, th0 = traj[0][1:]
    c, s = math.cos(th0), math.sin(th0)
    px, py = st["pose"]["x_m"], st["pose"]["y_m"]
    ax, ay = x0 + c * px - s * py, y0 + s * px + c * py
    assert math.hypot(ax - traj[-1][1], ay - traj[-1][2]) < 0.15


# --------------------------------------------- 2026-11-02 Gazebo-on-Pi conditions ---

def _wifi_pi_run(arena, route, frac, proc_time, use_stamp, seed=1):
    """Scans at 7.7 Hz, delivered in Wi-Fi bursts (every 2nd one held ~120 ms), SLAM busy proc_time
    per scan and always taking the newest arrived scan -- the conditions of the first Gazebo run
    on the Pi (SLAM 5.0 Hz, 2272 of 5672 scans rejected, pose 21 m off)."""
    from src.simulation.arena_trajectory import route_trajectory
    traj = route_trajectory(arena.layout, route, rate_hz=7.7)
    traj = traj[: int(len(traj) * frac)]
    rng = np.random.default_rng(seed)
    arrive = [t + 0.03 + (0.12 if k % 2 == 0 else 0.0) + rng.uniform(0, 0.02) for k, (t, *_r) in enumerate(traj)]
    slam = s6.SlamV6()
    x0, y0, th0 = traj[0][1:]
    now, last, errs = 0.0, -1, []
    while True:
        avail = [i for i in range(last + 1, len(traj)) if arrive[i] <= now]
        if not avail:
            if last + 1 >= len(traj):
                break
            now = min(arrive[last + 1:])
            continue
        i = max(avail)
        t, x, y, yaw = traj[i]
        r, _ = ar.cast(arena, x, y, yaw, 1.0)
        r = r.copy(); fin = np.isfinite(r); r[fin] += rng.normal(0, 0.01, fin.sum())
        msg = ar.laserscan_message(r, seq=i, stamp=100.0 + t)
        msg["received"] = arrive[i]
        scan = ros_scan_to_lidar_scan(msg, i, arrive[i])
        est = slam.process(s6.scan_to_points(scan), arrive[i], scan.sensor_time if use_stamp else None)
        c, s = math.cos(th0), math.sin(th0)
        errs.append(math.hypot(x0 + c * est["x"] - s * est["y"] - x, y0 + s * est["x"] + c * est["y"] - y))
        last, now = i, max(now, arrive[i]) + proc_time
    return slam, np.array(errs)


def test_backend_passes_the_sensor_stamp():
    msg = ar.laserscan_message([1.0] * 20, seq=1, stamp=123.25)
    assert ros_scan_to_lidar_scan(dict(msg, received=5.0), 0).sensor_time == 123.25
    assert ros_scan_to_lidar_scan(dict(ar.laserscan_message([1.0] * 20, seq=1), received=5.0), 0).sensor_time is None


@pytest.mark.slow
@pytest.mark.parametrize("use_stamp", [True, False])
def test_slow_pi_and_wifi_bursts_with_room_spins(arena, use_stamp):
    """Before the 2026-11-02 fix this diverged: 13.8 m (receive times) / 18.6 m (stamps) final error."""
    slam, err = _wifi_pi_run(arena, "rooms", 0.35, 0.2, use_stamp)
    assert err[-1] < 0.15 and err.max() < 0.3, (err[-1], err.max())
    assert slam.rejected <= 0.02 * slam.scans


def test_lost_after_bad_scans_and_recovers(arena):
    sp = arena.layout["spawn_pose"]
    rng = np.random.default_rng(3)
    slam = s6.SlamV6()
    for k in range(15):
        slam.process(scan_at(arena, sp["x"], sp["y"] + 0.05 * k, sp["yaw"], k, rng), k / 7.7)
    hold = slam.pose.copy()
    for k in range(s6.LOST_AFTER):
        r = slam.process(rng.uniform(-6, 6, size=(300, 2)), (15 + k) / 7.7)
    assert r["lost"] is True and r["accepted"] is False
    assert np.allclose(slam.pose, hold), "while lost the last good pose is held"
    r = slam.process(scan_at(arena, sp["x"], sp["y"] + 0.05 * 14, sp["yaw"], 99, rng), 40 / 7.7)
    assert r["accepted"] and not r["lost"]
