"""
gazebo/tools/check_scan_vs_truth.py comparison logic (2026-11-02c): a scan that
matches the arena passes; a mirrored or rotated scan is reported as such.
(The ROS subscription part runs only in WSL.)
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from src.simulation import arena_raycast as ar

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def tool():
    spec = importlib.util.spec_from_file_location("check_scan", ROOT / "gazebo/tools/check_scan_vs_truth.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def scene():
    path = ar.default_layout(1)
    if not path.exists():
        pytest.skip("arena layout not generated")
    arena = ar.load_arena(path)
    x, y = ar.cell_centre(arena.layout, 3, 5)               # inside the maze, asymmetric surroundings
    yaw = 0.4
    pred, labels = ar.cast(arena, x, y, yaw, 1.0)
    return arena, (x, y, yaw), pred, labels


def noisy(r, rng, sd=0.01):
    out = np.array(r, dtype=float)
    fin = np.isfinite(out)
    out[fin] += rng.normal(0, sd, fin.sum())
    return out


def test_matching_scan_passes(tool, scene):
    arena, pose, pred, labels = scene
    meas = noisy(pred, np.random.default_rng(1))
    res = tool.compare(list(meas), pred, labels)
    assert res["best_shift_beams"] == 0
    assert tool.verdict(res).startswith("PASS"), tool.verdict(res)


def test_mirrored_scan_is_reported(tool, scene):
    arena, pose, pred, labels = scene
    n = len(pred)
    meas = noisy(pred, np.random.default_rng(2))[(n - np.arange(n)) % n]   # CW instead of CCW
    v = tool.verdict(tool.compare(list(meas), pred, labels))
    assert "MIRRORED" in v, v


def test_rotated_scan_is_reported(tool, scene):
    arena, pose, pred, labels = scene
    meas = np.roll(noisy(pred, np.random.default_rng(3)), 14)                # ~10 deg sensor yaw
    res = tool.compare(list(meas), pred, labels)
    v = tool.verdict(res)
    assert "ROTATED" in v and res["best_shift_beams"] == -14, (v, res["best_shift_beams"])


def test_wrong_pose_fails(tool, scene):
    arena, (x, y, yaw), pred, labels = scene
    other, _ = ar.cast(arena, x + 0.4, y - 0.3, yaw, 1.0)
    v = tool.verdict(tool.compare(list(noisy(other, np.random.default_rng(4))), pred, labels))
    assert v.startswith("FAIL"), v


def test_self_occlusion_is_reported(tool, scene):
    """2026-11-02 Gazebo run: the rig's nose block made 29 forward beams read 0.15-0.17 m."""
    arena, pose, pred, labels = scene
    meas = noisy(pred, np.random.default_rng(5))
    n = len(meas)
    fwd = np.arange(n // 2 - 14, n // 2 + 15)                 # +-10 deg around angle 0 (beam 250)
    meas[fwd] = 0.16
    res = tool.compare(list(meas), pred, labels)
    v = tool.verdict(res)
    assert res["near_unexplained"] == len(fwd) and "not in the arena" in v, v
    assert min(res["near_angles_deg"]) >= -10.5 and max(res["near_angles_deg"]) <= 10.5


def test_real_close_wall_is_not_self_occlusion(tool):
    path = ar.default_layout(1)
    arena = ar.load_arena(path)
    x, y = ar.cell_centre(arena.layout, 3, 5)
    pred, labels = ar.cast(arena, x + 0.30, y, 0.0, 1.0)     # 0.2 m from a wall if one is there
    res = tool.compare(list(noisy(pred, np.random.default_rng(6))), pred, labels)
    assert res["near_unexplained"] == 0
