"""
Colourful arena worlds (gazebo/worlds/*.sdf, 2026-11-02g).

The colour scheme splits each wall into coloured visual pieces (room colours, corridor
white, outer navy). gpu_lidar sees VISUALS, so the pieces must tile every wall exactly
like the single box before (no gap, no overlap, same line/thickness/height) and the
collision boxes must stay the merged segments. Everything else that was added (floor
tiles, grid lines, apron, H pad) must stay on the floor, far below the LiDAR plane.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORLDS = sorted((ROOT / "gazebo" / "worlds").glob("nidar_airmouse_arena_s*.sdf"))


def _boxes(link, kind):
    out = {}
    for el in link.findall(kind):
        size = el.find("geometry/box/size")
        if size is None:
            continue
        out[el.get("name")] = ([float(v) for v in el.findtext("pose").split()], [float(v) for v in size.text.split()])
    return out


def _span(pose, size):
    vertical = abs(pose[5]) > 0.1
    c = pose[1] if vertical else pose[0]
    return c - size[0] / 2, c + size[0] / 2, (pose[0] if vertical else pose[1]), vertical


@pytest.mark.parametrize("world", WORLDS, ids=lambda p: p.stem)
def test_wall_pieces_tile_the_walls_exactly(world):
    link = ET.parse(world).getroot().find("world/model[@name='nidar_arena']/link")
    coll = {k[:-2]: v for k, v in _boxes(link, "collision").items() if k.startswith("wall_")}
    vis = _boxes(link, "visual")
    layout = json.loads((ROOT / "gazebo" / "layouts" / (world.stem + ".json")).read_text())
    assert len(coll) == len(layout["walls"])
    for name, (pose, size) in coll.items():
        a0, a1, line, vertical = _span(pose, size)
        pieces = sorted(_span(p, s) + (p, s) for k, (p, s) in vis.items() if k.startswith(name + "_p"))
        assert pieces, name
        assert pieces[0][0] == pytest.approx(a0, abs=2e-4) and pieces[-1][1] == pytest.approx(a1, abs=2e-4)
        for (b0, b1, bline, bvert, p, s), nxt in zip(pieces, pieces[1:] + [None]):
            assert bvert == vertical and bline == pytest.approx(line, abs=1e-4)
            assert s[1:] == pytest.approx(size[1:]) and p[2] == pytest.approx(pose[2])
            if nxt is not None:
                assert nxt[0] == pytest.approx(b1, abs=2e-4), f"{name}: gap/overlap at {b1:.3f}"


@pytest.mark.parametrize("world", WORLDS, ids=lambda p: p.stem)
def test_other_visuals_stay_on_the_floor_or_above_the_walls(world):
    w = ET.parse(world).getroot().find("world")
    link = w.find("model[@name='nidar_arena']/link")
    for v in link.findall("visual"):
        if v.get("name").startswith("wall_"):
            continue
        z = float(v.findtext("pose").split()[2])
        assert z <= 0.01 or z >= 2.4, f"{v.get('name')} at z={z}"
    scene = w.find("scene")
    assert scene.find("sky") is not None and scene.findtext("shadows") == "false"
