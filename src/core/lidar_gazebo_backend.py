#!/usr/bin/env python3
"""
NIDAR LiDAR backend fed by the Gazebo simulation (2026-11-02).

Data path:
  Gazebo gpu_lidar  --ros_gz_bridge-->  /nidar/scan (sensor_msgs/LaserScan, WSL)
  --gazebo/ros2/nidar_lidar_relay.py--> TCP :5011 on the Pi
  --src/communication/nidar_lidar_receiver.py--> src/communication/nidar_latest_scan.json
  --GazeboScanFileBackend.next_scan()--> LidarScan (same dataclass as the simulator)

Angle convention (IMPORTANT):
  ROS LaserScan angles are counter-clockwise from the sensor's +x (forward).
  NIDAR LiDAR angles are CLOCKWISE from forward (0 = forward, 90 = right), the
  convention V18's camera/LiDAR fusion uses (camera bearing positive = right,
  see src/simulation/nidar_world.py).  So:  nidar_angle = (-ros_angle) mod 360.

Freshness:
  next_scan() returns only a scan it has not returned before, and only if the
  Pi received it less than `max_age_s` ago (default = timeout_s), so a file
  left over from an earlier run is never used. If no such scan arrives within
  `timeout_s` it raises LidarStaleError -- V18 then skips the
  cycle (its JSON stops updating, the AirMouse server reports V18_STALE and the
  bridge publishes zero); SLAM waits.

Selected with the environment variable NIDAR_LIDAR_BACKEND=GAZEBO (V18 and SLAM).
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:                                   # package-style import (SLAM)
    from src.core.lidar_interface import LidarScan
except ImportError:                    # path-style import (V18)
    from lidar_interface import LidarScan

PROTOCOL = "NIDAR_LIDAR_V1"
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCAN_FILE = _PROJECT_ROOT / "src" / "communication" / "nidar_latest_scan.json"


class LidarStaleError(RuntimeError):
    """No new LiDAR scan arrived within the timeout."""


def ros_scan_to_lidar_scan(msg: dict, scan_id: int, timestamp: Optional[float] = None) -> LidarScan:
    """Convert a NIDAR_LIDAR_V1 dict (LaserScan fields) to a NIDAR LidarScan."""
    ranges = np.array([np.nan if r is None else float(r) for r in msg["ranges"]], dtype=float)
    n = len(ranges)
    a_min = float(msg["angle_min"])
    inc = float(msg["angle_increment"])
    ros_angles = a_min + inc * np.arange(n)
    nidar_deg = np.mod(-np.degrees(ros_angles), 360.0)
    r_min = float(msg.get("range_min", 0.0))
    r_max = float(msg.get("range_max", math.inf))
    valid = np.isfinite(ranges) & (ranges >= r_min) & (ranges <= r_max)
    distance = ranges.copy()
    distance[~valid] = np.nan
    scan_rate = float(msg.get("scan_rate_hz", 10.0))
    scan = LidarScan(
        scan_id=int(scan_id),
        timestamp=float(timestamp if timestamp is not None else msg.get("received", time.time())),
        angle_deg=nidar_deg,
        distance_m=distance,
        valid=valid,
        hit_types=["Gazebo" if v else "Invalid" for v in valid],
        scan_rate_hz=scan_rate,
        measurements_per_second=int(round(n * scan_rate)),
    )
    # The sensor's own time (Gazebo sim time). SLAM V6 uses it for its motion model: the
    # Pi receive time jitters with Wi-Fi batching. None when the relay did not send one.
    try:
        stamp = float(msg.get("stamp") or 0.0)
    except (TypeError, ValueError):
        stamp = 0.0
    scan.sensor_time = stamp if stamp > 0.0 else None
    return scan


class GazeboScanFileBackend:
    """Reads the latest scan written by nidar_lidar_receiver.py."""

    name = "GAZEBO_LIDAR"

    def __init__(self, path: Optional[Path] = None, timeout_s: float = 1.0, poll_s: float = 0.005,
                 max_age_s: Optional[float] = None, clock=time.time):
        self.path = Path(path or os.environ.get("NIDAR_SCAN_FILE", DEFAULT_SCAN_FILE))
        self.timeout_s = float(timeout_s)
        self.poll_s = float(poll_s)
        # A scan older than this (Pi clock, "received" field written by the
        # receiver) is never used -- e.g. the file left over from a previous run.
        self.max_age_s = float(max_age_s if max_age_s is not None else timeout_s)
        self.clock = clock
        self.scan_id = 0
        self.rejected_old = 0
        self._last_seq = None
        self._last_key = None

    def reset(self) -> None:
        self.scan_id = 0
        self._last_seq = None
        self._last_key = None

    def _read_if_changed(self) -> Optional[dict]:
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return None
        # The receiver replaces the file atomically (new inode each time), so
        # (inode, mtime) changes on every write even with a coarse mtime clock.
        key = (st.st_ino, st.st_mtime_ns)
        if key == self._last_key:
            return None
        try:
            with self.path.open("r", encoding="utf-8") as f:
                msg = json.load(f)
        except (OSError, ValueError):
            return None                      # partially visible file: try again next poll
        self._last_key = key
        if msg.get("protocol") != PROTOCOL:
            return None
        if msg.get("seq") == self._last_seq:
            return None
        try:
            received = float(msg["received"])
        except (KeyError, TypeError, ValueError):
            return None                      # not written by nidar_lidar_receiver.py
        if self.clock() - received > self.max_age_s:
            self.rejected_old += 1
            return None
        return msg

    def next_scan(self, dt: Optional[float] = None) -> LidarScan:
        deadline = time.monotonic() + self.timeout_s
        while True:
            msg = self._read_if_changed()
            if msg is not None:
                self._last_seq = msg.get("seq")
                scan = ros_scan_to_lidar_scan(msg, self.scan_id, float(msg["received"]))
                self.scan_id += 1
                return scan
            if time.monotonic() >= deadline:
                raise LidarStaleError(
                    f"no new Gazebo LiDAR scan for {self.timeout_s:.1f} s ({self.path})")
            time.sleep(self.poll_s)


def create_backend(default: str = "SIMULATOR"):
    """Factory shared by V18 and SLAM: NIDAR_LIDAR_BACKEND = SIMULATOR | GAZEBO."""
    choice = os.environ.get("NIDAR_LIDAR_BACKEND", default).strip().upper()
    if choice == "GAZEBO":
        return GazeboScanFileBackend()
    if choice == "SIMULATOR":
        try:
            from src.core.lidar_interface import SimulatedSTL19PBackend
        except ImportError:
            from lidar_interface import SimulatedSTL19PBackend
        return SimulatedSTL19PBackend()
    raise RuntimeError(f"NIDAR_LIDAR_BACKEND={choice!r} not supported here (SIMULATOR or GAZEBO)")
