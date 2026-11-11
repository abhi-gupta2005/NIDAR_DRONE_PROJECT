"""
Autonomous explorer (src/mission/nidar_explorer.py, 2026-11-09) -- knows nothing about the arena.

Closed loop in tests/explorer_sim.py: ray-cast LiDAR -> map built with SLAM V6's own grid code
(mode "slam" = full SLAM V6 estimate) -> Explorer -> drone flying the plans (0.4 m/s, 30 deg/s,
turning while moving) -> ground truth.

Measured 2026-11-09 (drone radius 0.25 m, 20 min mission budget, then return + land):
  arenas s1-s3 and 6 new random arenas (generator seeds 4-9): camera coverage 0.95-0.98 of the grid
  cells, 6/6 survivors in camera view in every arena, landed 0.07-0.09 m from the pad;
  full SLAM V6 (s1): coverage 0.985, 6/6, 0.08 m; with the 0.05 m map margin the drone keeps
  >= 0.31 m from every wall. The x500 (0.39 m) cannot be kept clear in 1 m corridors (0.35-0.37 m).
Bugs these tests pin: thin walls 'erased' by grazing beams (sticky walls + raw hits), a viewpoint
loop (visited viewpoints), escaping to a 'nearest safe cell' through a wall (geodesic search).
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from src.mission import nidar_explorer as ne

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))


def grid_with(walls=(), free_box=None, n=200, res=0.1, origin=-10.0):
    """int8 grid: unknown everywhere, free inside free_box (x0, y0, x1, y1), walls as rectangles."""
    g = np.full((n, n), -1, dtype=np.int8)
    def cells(x0, y0, x1, y1):
        return (slice(int((y0 - origin) / res), int((y1 - origin) / res)),
                slice(int((x0 - origin) / res), int((x1 - origin) / res)))
    if free_box:
        g[cells(*free_box)] = 0
    for w in walls:
        g[cells(*w)] = 100
    return g


def explorer_on(g, radius=0.25):
    ex = ne.Explorer(drone_radius=radius)
    ex.update_map(g, res=0.1, origin=-10.0)
    return ex


def test_planning_radius_includes_the_map_margin():
    ex = ne.Explorer(drone_radius=0.25)
    assert ex.R == pytest.approx(0.25 + ne.MAP_MARGIN_M)


def test_sticky_walls_survive_grazing_erosion():
    wall = (-0.05, -2.0, 0.05, 2.0)
    g = grid_with([wall], free_box=(-3, -3, 3, 3))
    ex = explorer_on(g)
    g2 = grid_with([], free_box=(-3, -3, 3, 3))                      # later map: the wall faded to free
    ex.update_map(g2, res=0.1, origin=-10.0)
    j, i = ex.view.cell(-0.04, 0.0)                                   # a cell inside the wall rectangle
    assert ex.view.occ[j, i] and not ex.view.free[j, i]


def test_raw_hits_block_a_wall_the_map_calls_free():
    g = grid_with([], free_box=(-3, -3, 3, 3))
    ex = explorer_on(g)
    assert ex.plan_still_safe(-1.0, 0.0, [(1.0, 0.0)])
    ex.add_hits(np.array([[0.0, y] for y in np.arange(-2, 2, 0.05)]))
    assert not ex.plan_still_safe(-1.0, 0.0, [(1.0, 0.0)])


def test_nearest_safe_cell_is_never_across_a_wall():
    # drone squeezed against a thin wall; the free side across the wall is roomier, but unreachable
    g = grid_with([(-0.05, -3.0, 0.05, 3.0)], free_box=(-0.35, -3, 3, 3))
    ex = explorer_on(g)
    start = ex.view.cell(-0.2, 0.0)
    near = ex._nearest_safe(start, ex.safe())
    assert near is None or ex.view.centre(*near)[0] < 0.0


def test_frontiers_only_inside_the_walls_box():
    # walls box spans x -2..2, y 0..4; free space also outside it (behind the "pad")
    walls = [(-2.0, 0.0, 2.0, 0.1), (-2.0, 3.9, 2.0, 4.0), (-2.1, 0.0, -2.0, 4.0), (2.0, 0.0, 2.1, 4.0)]
    g = grid_with(walls, free_box=(-1.9, -3.0, 1.9, 3.9))
    ex = explorer_on(g)
    fr = ex.targets()
    js, is_ = np.nonzero(fr)
    ys = [ex.view.centre(j, i)[1] for j, i in zip(js, is_)]
    assert ys and min(ys) >= 0.0, "no targets behind the arena's wall line"


def test_look_sweeps_only_towards_unseen_space():
    g = grid_with([], free_box=(-3, -3, 3, 3))
    ex = explorer_on(g)
    ex.seen[:] = True
    j0, j1 = ex.view.cell(1.0, -0.5)[0], ex.view.cell(1.0, 0.5)[0]
    i0, i1 = ex.view.cell(1.0, 0.0)[1], ex.view.cell(2.5, 0.0)[1]
    ex.seen[j0:j1, i0:i1] = False                                     # unseen patch straight ahead (+x)
    yaws = ex.look_yaws(0.0, 0.0, 0.0)
    assert yaws and all(abs(y) < math.radians(40) for y in yaws), yaws
    ex.seen[:] = True
    assert ex.look_yaws(0.0, 0.0, 0.0) == []


@pytest.mark.slow
def test_explores_arena_s1_with_full_slam_and_returns_home():
    import explorer_sim as es
    r = es.run(ROOT / "gazebo/layouts/nidar_airmouse_arena_s1.json", mode="slam", t_max=1800, radius=0.25)
    assert r["coverage"] >= 0.95, r["coverage"]
    assert len(r["survivors_seen"]) == 6
    assert r["min_wall_clearance"] >= 0.25, r["min_clear_at"]
    assert r["end_dist_to_pad"] < 0.2 and "returning" in r["reason"] or "landing" in r["reason"]


@pytest.mark.slow
def test_solves_a_new_random_arena(tmp_path):
    subprocess.run([sys.executable, str(ROOT / "gazebo/nidar_arena_generator.py"), "--seed", "7",
                    "--out-dir", str(tmp_path)], check=True, capture_output=True)
    import explorer_sim as es
    r = es.run(tmp_path / "layouts/nidar_airmouse_arena_s7.json", mode="map", t_max=1800, radius=0.25)
    assert r["coverage"] >= 0.95 and len(r["survivors_seen"]) == 6
    assert r["min_wall_clearance"] >= 0.25 and r["end_dist_to_pad"] < 0.2
