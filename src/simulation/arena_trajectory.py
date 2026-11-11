#!/usr/bin/env python3
"""
Vehicle-like trajectories through the generated arena (2026-11-02).

Runs gazebo/ros2/nidar_rig_driver.RouteFollower on a kinematic model at the
LiDAR rate measured in Gazebo (7.7 Hz), so offline SLAM / mapping tests see
the same kind of motion as the Gazebo rig: 0.4 m/s, yaw rate <= 0.8 rad/s.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from typing import List, Tuple

from src.simulation import arena_raycast as ar

_REPO = Path(__file__).resolve().parents[2]


def _driver():
    spec = importlib.util.spec_from_file_location("nidar_rig_driver_sim",
                                                  _REPO / "gazebo" / "ros2" / "nidar_rig_driver.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def route_trajectory(layout: dict, route: str, rate_hz: float = 7.7, speed: float = 0.4,
                     max_steps: int = 20000) -> List[Tuple[float, float, float, float]]:
    """[(t, x, y, yaw)] sampled at rate_hz until the route is finished."""
    drv = _driver()
    path = drv.build_route(layout, route)
    fol = drv.RouteFollower(path, speed=speed)
    sp = layout["spawn_pose"]
    x, y, yaw = sp["x"], sp["y"], sp["yaw"]
    dt = 1.0 / rate_hz
    sub = 10                                   # integrate finer than the scan rate
    out = [(0.0, x, y, yaw)]
    t = 0.0
    for _ in range(max_steps):
        done = False
        for _ in range(sub):
            vx, vy, wz, done = fol.update(x, y, yaw)
            if done:
                break
            c, s = math.cos(yaw), math.sin(yaw)
            h = dt / sub
            x += (c * vx - s * vy) * h
            y += (s * vx + c * vy) * h
            yaw = ar.wrap(yaw + wz * h) if hasattr(ar, "wrap") else (yaw + wz * h + math.pi) % (2 * math.pi) - math.pi
        t += dt
        out.append((t, x, y, yaw))
        if done:
            break
    return out
