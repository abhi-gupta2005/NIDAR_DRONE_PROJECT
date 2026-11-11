#!/usr/bin/env python3
"""
NIDAR AirMouse arena generator -- Gazebo (gz sim) world for the mission brief.

Builds a covered, maze-like, GPS-denied indoor arena that follows the
"Track 1 - Mission Brief - NIDAR AirMouse" rules and CHECKS them:

  * total size <= 15 m x 15 m, modular uniform grid
  * corridors with >= 1.0 m CLEAR width (wall thickness subtracted)
  * standard rooms 2 m x 2 m (clear), each with one doorway
  * 8 ft (2.44 m) wall / ceiling height, top covered by a net
  * ONE designated entry/exit opening, 2 ft x 2 ft launch pad outside it
  * up to 6 survivors, each in a different room
  * every cell and every room reachable from the entry

Grid convention (used by the map / survivor tagging later):
  world frame  : Gazebo ENU, x = east, y = north, metres; origin = outer
                 south-west corner of the arena
  cell (c, r)  : column c = 0.. from west, row r = 0.. from south (entry wall)
  grid label   : column letter + row number, e.g. cell (3, 6) -> "D7"
  cell centre  : ((c + 0.5) * pitch + wall/2, (r + 0.5) * pitch + wall/2)

Outputs (default names):
  gazebo/worlds/<world>.sdf          the Gazebo world
  gazebo/layouts/<world>.json        ground truth (walls, rooms, survivors, spawn)
  gazebo/layouts/<world>.png         top-down picture with grid labels

The internal layout is randomised by --seed because the brief says the real
layout may not be disclosed; test against several seeds.

Usage:
  python gazebo/nidar_arena_generator.py --seed 1
  python gazebo/nidar_arena_generator.py --seed 7 --survivors 6 --rooms 8 --world nidar_airmouse_arena_s7
  python gazebo/nidar_arena_generator.py --seed 1 --survivor-models primitive   # no internet needed
"""

from __future__ import annotations

import argparse
import json
import math
import random
import string
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple
from xml.sax.saxutils import escape

HERE = Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
# Mission-brief limits (checked, never silently changed)
# ---------------------------------------------------------------------------
BRIEF_MAX_ARENA_M = 15.0
BRIEF_MIN_CORRIDOR_CLEAR_M = 1.0
BRIEF_ROOM_CLEAR_M = 2.0
BRIEF_HEIGHT_M = 8 * 0.3048          # 8 ft = 2.4384 m
BRIEF_LAUNCH_PAD_M = 2 * 0.3048      # 2 ft = 0.6096 m
BRIEF_MAX_SURVIVORS = 6

# Survivor models. Fuel models are downloaded once by Gazebo (needs internet the
# first time, then cached in ~/.gz/fuel). "primitive" needs nothing.
FUEL_SURVIVORS = [
    ("rescue_randy", "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue Randy", "lying"),
    ("standing_person", "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person", "standing"),
    ("casual_female", "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female", "standing"),
]

Cell = Tuple[int, int]


def grid_label(c: int, r: int) -> str:
    letters = string.ascii_uppercase
    col = letters[c] if c < 26 else letters[c // 26 - 1] + letters[c % 26]
    return f"{col}{r + 1}"


@dataclass
class Room:
    rid: str
    anchor: Cell                      # south-west cell of the 2x2 block
    cells: List[Cell]
    door: Optional[Tuple[Cell, Cell]] = None   # (room cell, outside cell)


@dataclass
class Arena:
    cells: int
    pitch: float
    wall: float
    height: float
    seed: int
    entry_col: int
    # wall present between neighbouring cells; key = frozenset({a, b})
    walls: Set[frozenset] = field(default_factory=set)
    rooms: List[Room] = field(default_factory=list)
    room_of: Dict[Cell, str] = field(default_factory=dict)
    survivors: List[dict] = field(default_factory=list)

    # -- geometry -------------------------------------------------------------
    @property
    def outer_size(self) -> float:
        return self.cells * self.pitch + self.wall

    def cell_center(self, c: int, r: int) -> Tuple[float, float]:
        o = self.wall / 2.0
        return (o + (c + 0.5) * self.pitch, o + (r + 0.5) * self.pitch)

    def in_grid(self, c: int, r: int) -> bool:
        return 0 <= c < self.cells and 0 <= r < self.cells

    def neighbours(self, c: int, r: int):
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if self.in_grid(c + dc, r + dr):
                yield (c + dc, r + dr)

    def open_between(self, a: Cell, b: Cell) -> bool:
        return frozenset((a, b)) not in self.walls

    def remove_wall(self, a: Cell, b: Cell) -> None:
        self.walls.discard(frozenset((a, b)))


# ---------------------------------------------------------------------------
# Layout generation
# ---------------------------------------------------------------------------

def generate_layout(cells: int, pitch: float, wall: float, height: float, n_rooms: int,
                    n_survivors: int, loop_factor: float, seed: int, entry_col: Optional[int],
                    max_attempts: int = 200) -> Arena:
    rng = random.Random(seed)
    for attempt in range(max_attempts):
        arena = _try_layout(rng, cells, pitch, wall, height, n_rooms, loop_factor, seed, entry_col)
        if arena is not None:
            _place_survivors(arena, rng, n_survivors)
            return arena
    raise RuntimeError("could not generate a valid layout; try fewer rooms or another seed")


def _try_layout(rng, cells, pitch, wall, height, n_rooms, loop_factor, seed, entry_col):
    entry_col = cells // 2 if entry_col is None else entry_col
    a = Arena(cells=cells, pitch=pitch, wall=wall, height=height, seed=seed, entry_col=entry_col)

    # 1. every interior edge starts as a wall
    for c in range(cells):
        for r in range(cells):
            for nb in ((c + 1, r), (c, r + 1)):
                if a.in_grid(*nb):
                    a.walls.add(frozenset(((c, r), nb)))

    # 2. rooms: 2x2 blocks, not on the entry column's first rows, 1-cell gap between rooms
    reserved = {(entry_col, 0), (entry_col, 1)}
    blocked: Set[Cell] = set()
    anchors = [(c, r) for c in range(cells - 1) for r in range(cells - 1)]
    rng.shuffle(anchors)
    for (c, r) in anchors:
        if len(a.rooms) >= n_rooms:
            break
        block = [(c, r), (c + 1, r), (c, r + 1), (c + 1, r + 1)]
        if any(x in blocked or x in reserved for x in block):
            continue
        rid = f"R{len(a.rooms) + 1}"
        room = Room(rid=rid, anchor=(c, r), cells=block)
        a.rooms.append(room)
        for x in block:
            a.room_of[x] = rid
        for dc in range(-1, 3):           # block the room plus a 1-cell ring
            for dr in range(-1, 3):
                blocked.add((c + dc, r + dr))
        # open the inside of the room
        a.remove_wall((c, r), (c + 1, r))
        a.remove_wall((c, r + 1), (c + 1, r + 1))
        a.remove_wall((c, r), (c, r + 1))
        a.remove_wall((c + 1, r), (c + 1, r + 1))
    if len(a.rooms) < n_rooms:
        return None

    # 3. corridors: randomised DFS over non-room cells; each room becomes a leaf
    #    reached through exactly one doorway
    corridor_cells = [(c, r) for c in range(cells) for r in range(cells) if (c, r) not in a.room_of]
    start = (entry_col, 0)
    visited: Set[Cell] = {start}
    stack = [start]
    rooms_done: Set[str] = set()
    while stack:
        cur = stack[-1]
        options = []
        for nb in a.neighbours(*cur):
            if nb in a.room_of:
                if a.room_of[nb] not in rooms_done:
                    options.append(nb)
            elif nb not in visited:
                options.append(nb)
        if not options:
            stack.pop()
            continue
        nb = rng.choice(options)
        a.remove_wall(cur, nb)
        if nb in a.room_of:
            rid = a.room_of[nb]
            rooms_done.add(rid)
            next(r for r in a.rooms if r.rid == rid).door = (nb, cur)
        else:
            visited.add(nb)
            stack.append(nb)
    if len(visited) != len(corridor_cells) or len(rooms_done) != len(a.rooms):
        return None

    # 4. loops / junctions: remove some corridor-corridor walls (never room walls)
    for key in list(a.walls):
        x, y = tuple(key)
        if x in a.room_of or y in a.room_of:
            continue
        if rng.random() < loop_factor:
            a.walls.discard(key)
    return a


def _place_survivors(a: Arena, rng: random.Random, n: int) -> None:
    n = min(n, BRIEF_MAX_SURVIVORS, len(a.rooms))
    rooms = rng.sample(a.rooms, n)
    for i, room in enumerate(rooms):
        cell = rng.choice(room.cells)
        cx, cy = a.cell_center(*cell)
        # keep the body inside the cell, away from the wall line
        jx, jy = rng.uniform(-0.15, 0.15) * a.pitch, rng.uniform(-0.15, 0.15) * a.pitch
        a.survivors.append({
            "id": f"S{i + 1}",
            "room": room.rid,
            "cell": list(cell),
            "grid": grid_label(*cell),
            "x": round(cx + jx, 3),
            "y": round(cy + jy, 3),
            "yaw": round(rng.uniform(-math.pi, math.pi), 3),
        })


# ---------------------------------------------------------------------------
# Validation against the brief
# ---------------------------------------------------------------------------

def validate(a: Arena) -> List[Tuple[str, bool, str]]:
    checks = []

    def add(name, ok, detail):
        checks.append((name, bool(ok), detail))

    add("arena <= 15 m x 15 m", a.outer_size <= BRIEF_MAX_ARENA_M + 1e-9,
        f"{a.outer_size:.2f} m x {a.outer_size:.2f} m")
    clear = a.pitch - a.wall
    add("corridor clear width >= 1.0 m", clear >= BRIEF_MIN_CORRIDOR_CLEAR_M - 1e-9, f"{clear:.2f} m")
    room_clear = 2 * a.pitch - a.wall
    add("room clear size ~ 2 m x 2 m", BRIEF_ROOM_CLEAR_M - 1e-9 <= room_clear <= BRIEF_ROOM_CLEAR_M + 0.15,
        f"{room_clear:.2f} m x {room_clear:.2f} m")
    add("height 8 ft (2.44 m)", abs(a.height - BRIEF_HEIGHT_M) < 0.01, f"{a.height:.3f} m")
    add("rooms each have exactly one doorway", all(r.door for r in a.rooms) and all(
        sum(1 for rc in r.cells for nb in a.neighbours(*rc)
            if nb not in r.cells and a.open_between(rc, nb)) == 1 for r in a.rooms),
        f"{len(a.rooms)} rooms")
    # reachability from the entry cell
    start = (a.entry_col, 0)
    seen = {start}
    q = deque([start])
    while q:
        cur = q.popleft()
        for nb in a.neighbours(*cur):
            if nb not in seen and a.open_between(cur, nb):
                seen.add(nb)
                q.append(nb)
    total = a.cells * a.cells
    add("every cell reachable from the entry", len(seen) == total, f"{len(seen)}/{total} cells")
    add("survivors <= 6, each in a different room",
        len(a.survivors) <= 6 and len({s['room'] for s in a.survivors}) == len(a.survivors),
        f"{len(a.survivors)} survivors in rooms {[s['room'] for s in a.survivors]}")
    add("single entry/exit opening", True, f"south wall, cell {grid_label(a.entry_col, 0)}")
    return checks


# ---------------------------------------------------------------------------
# Wall segments (merged collinear panels -> fewer Gazebo entities)
# ---------------------------------------------------------------------------

def wall_segments(a: Arena) -> List[Tuple[float, float, float, float]]:
    """Centre-line segments (x1, y1, x2, y2) in metres, merged where collinear."""
    o = a.wall / 2.0
    n = a.cells
    p = a.pitch
    # vertical lines x = o + k*p, k = 0..n ; horizontal lines y = o + k*p
    vertical: Dict[int, List[int]] = {k: [] for k in range(n + 1)}     # k -> list of rows with a wall
    horizontal: Dict[int, List[int]] = {k: [] for k in range(n + 1)}   # k -> list of cols with a wall
    for r in range(n):
        vertical[0].append(r)
        vertical[n].append(r)
        for c in range(n - 1):
            if not a.open_between((c, r), (c + 1, r)):
                vertical[c + 1].append(r)
    for c in range(n):
        if c != a.entry_col:
            horizontal[0].append(c)        # south wall with the entry gap
        horizontal[n].append(c)
        for r in range(n - 1):
            if not a.open_between((c, r), (c, r + 1)):
                horizontal[r + 1].append(c)

    def runs(idx: List[int]):
        idx = sorted(idx)
        out = []
        for i in idx:
            if out and out[-1][1] == i - 1:
                out[-1][1] = i
            else:
                out.append([i, i])
        return out

    segs = []
    for k, rows in vertical.items():
        x = o + k * p
        for r0, r1 in runs(rows):
            segs.append((x, o + r0 * p, x, o + (r1 + 1) * p))
    for k, cols in horizontal.items():
        y = o + k * p
        for c0, c1 in runs(cols):
            segs.append((o + c0 * p, y, o + (c1 + 1) * p, y))
    return segs


# ---------------------------------------------------------------------------
# SDF writer
# ---------------------------------------------------------------------------

# Colour scheme (2026-11-02g): every room has its own colour (its walls and a tinted
# floor), corridor walls warm white, outer walls navy, concrete floor with golden grid
# lines, grass outside, H launch pad, sky. Colours only: wall/collision geometry is
# identical to the plain version (the LiDAR sees the same arena).
ROOM_COLOURS = [(0.93, 0.42, 0.38), (0.18, 0.66, 0.66), (0.97, 0.70, 0.22), (0.60, 0.46, 0.86),
                (0.40, 0.75, 0.36), (0.33, 0.58, 0.94), (0.93, 0.50, 0.72), (0.97, 0.55, 0.20)]
CORRIDOR_WALL = (0.93, 0.92, 0.88)
OUTER_WALL = (0.16, 0.24, 0.40)
FLOOR = (0.70, 0.71, 0.73)
GRID_LINE = (0.98, 0.80, 0.15)
GRASS = (0.33, 0.55, 0.28)
APRON = (0.28, 0.30, 0.33)


def _rgba(c, a=1.0) -> str:
    return f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a}"


def _tint(c, k=0.55):
    return tuple(v + (1.0 - v) * k for v in c)


def wall_runs(a: Arena):
    """Same runs as wall_segments(), with panel indices: (orient, k, i0, i1); 'v' = x const."""
    n = a.cells
    vertical: Dict[int, List[int]] = {k: [] for k in range(n + 1)}
    horizontal: Dict[int, List[int]] = {k: [] for k in range(n + 1)}
    for r in range(n):
        vertical[0].append(r)
        vertical[n].append(r)
        for c in range(n - 1):
            if not a.open_between((c, r), (c + 1, r)):
                vertical[c + 1].append(r)
    for c in range(n):
        if c != a.entry_col:
            horizontal[0].append(c)
        horizontal[n].append(c)
        for r in range(n - 1):
            if not a.open_between((c, r), (c, r + 1)):
                horizontal[r + 1].append(c)
    out = []
    for orient, lines in (("v", vertical), ("h", horizontal)):
        for k, idx in lines.items():
            idx = sorted(idx)
            run = []
            for i in idx:
                if run and run[-1][1] == i - 1:
                    run[-1][1] = i
                else:
                    run.append([i, i])
            out += [(orient, k, i0, i1) for i0, i1 in run]
    return out


def _panel_colour(a: Arena, orient: str, k: int, i: int):
    sides = [(k - 1, i), (k, i)] if orient == "v" else [(i, k - 1), (i, k)]
    rooms = sorted(a.room_of[c] for c in sides if c in a.room_of)
    if rooms:
        idx = [r.rid for r in a.rooms].index(rooms[0])
        return ("room", ROOM_COLOURS[idx % len(ROOM_COLOURS)])
    if k in (0, a.cells):
        return ("outer", OUTER_WALL)
    return ("corridor", CORRIDOR_WALL)


def wall_pieces(a: Arena):
    """Coloured visual pieces that tile each wall run exactly (no overlap, no gap):
    [(run_index, x1, y1, x2, y2, colour)] -- ends extended by wall/2 only at the run ends."""
    o, p, t = a.wall / 2.0, a.pitch, a.wall
    pieces = []
    for ri, (orient, k, i0, i1) in enumerate(wall_runs(a)):
        groups = []
        for i in range(i0, i1 + 1):
            key = _panel_colour(a, orient, k, i)
            if groups and groups[-1][2] == key:
                groups[-1][1] = i
            else:
                groups.append([i, i, key])
        line = o + k * p
        for j0, j1, (_, col) in groups:
            s0 = o + j0 * p - (t / 2 if j0 == i0 else 0.0)
            s1 = o + (j1 + 1) * p + (t / 2 if j1 == i1 else 0.0)
            if orient == "v":
                pieces.append((ri, line, s0, line, s1, col))
            else:
                pieces.append((ri, s0, line, s1, line, col))
    return pieces


def _box_visual(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float, rgba: str,
                yaw: float = 0.0, extra: str = "", roughness: float = 0.8) -> str:
    return f"""
        <visual name="{name}_v"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.08 0.08 0.08 1</specular>
            <pbr><metal><metalness>0</metalness><roughness>{roughness}</roughness></metal></pbr></material>{extra}</visual>"""


def _box_collision(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float, yaw: float = 0.0) -> str:
    return f"""
        <collision name="{name}_c"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>"""


def _disc_visual(name: str, x: float, y: float, z: float, r: float, h: float, rgba: str) -> str:
    return f"""
        <visual name="{name}_v"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>"""


def _box_link(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float,
              rgba: str, collision: bool = True, yaw: float = 0.0, transparency: float = 0.0) -> str:
    coll = (f"""
        <collision name="{name}_c"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>"""
            if collision else "")
    tr = f"<transparency>{transparency}</transparency>" if transparency else ""
    return f"""
        <visual name="{name}_v"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>{tr}</visual>{coll}"""


def _survivor_sdf(s: dict, i: int, mode: str) -> str:
    name = f"survivor_{s['id']}"
    if mode == "fuel":
        key, uri, posture = FUEL_SURVIVORS[i % len(FUEL_SURVIVORS)]
        s["model"] = key
        s["posture"] = posture
        return f"""
    <include>
      <name>{name}</name>
      <uri>{escape(uri)}</uri>
      <pose>{s['x']} {s['y']} 0 0 0 {s['yaw']}</pose>
      <static>true</static>
    </include>"""
    s["model"] = "primitive_mannequin"
    s["posture"] = "standing"
    # simple standing mannequin: legs/body capsule + head sphere, high-visibility orange
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{s['x']} {s['y']} 0 0 0 {s['yaw']}</pose>
      <link name="body">
        <visual name="torso"><pose>0 0 0.85 0 0 0</pose>
          <geometry><capsule><radius>0.18</radius><length>1.2</length></capsule></geometry>
          <material><ambient>1 0.45 0 1</ambient><diffuse>1 0.45 0 1</diffuse></material></visual>
        <visual name="head"><pose>0 0 1.65 0 0 0</pose>
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
          <material><ambient>0.9 0.75 0.6 1</ambient><diffuse>0.9 0.75 0.6 1</diffuse></material></visual>
        <collision name="c"><pose>0 0 0.85 0 0 0</pose>
          <geometry><cylinder><radius>0.2</radius><length>1.7</length></cylinder></geometry></collision>
      </link>
    </model>"""


def build_sdf(a: Arena, world_name: str, survivor_mode: str, plugin_block: str = "", shadows: bool = False) -> str:
    segs = wall_segments(a)
    h = a.height
    t = a.wall
    size = a.outer_size
    visuals = []

    # walls: collision = the merged segments (unchanged); visuals = coloured pieces tiling them
    for i, (x1, y1, x2, y2) in enumerate(segs):
        length = math.hypot(x2 - x1, y2 - y1) + t        # overlap the corners
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        yaw = 0.0 if abs(y2 - y1) < 1e-9 else math.pi / 2
        visuals.append(_box_collision(f"wall_{i}", cx, cy, h / 2, length, t, h, yaw))
    for j, (ri, x1, y1, x2, y2, col) in enumerate(wall_pieces(a)):
        length = math.hypot(x2 - x1, y2 - y1)
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        yaw = 0.0 if abs(y2 - y1) < 1e-9 else math.pi / 2
        visuals.append(_box_visual(f"wall_{ri}_p{j}", cx, cy, h / 2, length, t, h, _rgba(col), yaw, roughness=0.7))

    # floor (visual only): concrete, room floors tinted in the room colour, golden grid lines
    visuals.append(_box_visual("floor", size / 2, size / 2, -0.005, size, size, 0.01, _rgba(FLOOR), roughness=0.9))
    o = t / 2
    for ri, room in enumerate(a.rooms):
        col = _tint(ROOM_COLOURS[ri % len(ROOM_COLOURS)])
        for (c, r) in room.cells:
            x, y = a.cell_center(c, r)
            visuals.append(_box_visual(f"room_{room.rid}_floor_{c}_{r}", x, y, 0.0005, a.pitch - t, a.pitch - t,
                                       0.001, _rgba(col), roughness=0.9))
    for k in range(a.cells + 1):
        v = o + k * a.pitch
        visuals.append(_box_visual(f"gx_{k}", v, size / 2, 0.002, 0.02, size, 0.002, _rgba(GRID_LINE)))
        visuals.append(_box_visual(f"gy_{k}", size / 2, v, 0.002, size, 0.02, 0.002, _rgba(GRID_LINE)))

    # ceiling net (thin, see-through, casts no shadow; with collision: the arena is covered)
    visuals.append(_box_visual("ceiling_net", size / 2, size / 2, h + 0.01, size, size, 0.02, "0.1 0.1 0.1 1",
                               extra="<transparency>0.85</transparency><cast_shadows>false</cast_shadows>"))
    visuals.append(_box_collision("ceiling_net", size / 2, size / 2, h + 0.01, size, size, 0.02))

    # entry marking, asphalt apron and the 2 ft x 2 ft H launch pad outside the entry gap
    ex, _ = a.cell_center(a.entry_col, 0)
    pad = BRIEF_LAUNCH_PAD_M
    pad_y = -0.75
    visuals.append(_box_visual("apron", ex, -1.0, 0.0005, 3.2, 1.9, 0.001, _rgba(APRON)))
    visuals.append(_box_visual("entry_marker", ex, o + 0.15, 0.003, a.pitch - t, 0.3, 0.004, "0.1 0.8 0.1 1"))
    visuals.append(_box_visual("launch_pad", ex, pad_y, 0.003, pad, pad, 0.004, "0.18 0.20 0.24 1"))
    visuals.append(_disc_visual("pad_ring", ex, pad_y, 0.0055, pad * 0.44, 0.001, "0.98 0.80 0.15 1"))
    visuals.append(_disc_visual("pad_inner", ex, pad_y, 0.0065, pad * 0.37, 0.001, "0.18 0.20 0.24 1"))
    for nm, dx, sx, sy in (("pad_h_l", -0.07, 0.035, 0.24), ("pad_h_r", 0.07, 0.035, 0.24), ("pad_h_m", 0.0, 0.14, 0.035)):
        visuals.append(_box_visual(nm, ex + dx, pad_y, 0.0075, sx, sy, 0.001, "1 1 1 1"))

    survivors = "".join(_survivor_sdf(s, i, survivor_mode) for i, s in enumerate(a.survivors))

    # indoor lights (the net blocks most daylight)
    lights = []
    for i, (lx, ly) in enumerate([(size * 0.25, size * 0.25), (size * 0.75, size * 0.25),
                                  (size * 0.25, size * 0.75), (size * 0.75, size * 0.75), (size / 2, size / 2)]):
        lights.append(f"""
    <light name="indoor_{i}" type="point">
      <pose>{lx:.2f} {ly:.2f} {h - 0.2:.2f} 0 0 0</pose>
      <diffuse>0.92 0.90 0.84 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>14</range><constant>0.35</constant><linear>0.07</linear><quadratic>0.008</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by gazebo/nidar_arena_generator.py (seed {a.seed}).
     NIDAR AirMouse arena: {size:.2f} m x {size:.2f} m, {a.cells}x{a.cells} cells of {a.pitch} m,
     walls {a.wall} m thick and {a.height:.3f} m high, {len(a.rooms)} rooms, {len(a.survivors)} survivors.
     Ground truth: gazebo/layouts/{world_name}.json -->
<sdf version="1.9">
  <world name="{world_name}">{plugin_block}
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.58 0.58 0.60 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <sky></sky>
      <shadows>{'true' if shadows else 'false'}</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 -5 0 0 0</pose>
          <geometry><box><size>1000 1000 10</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>{_rgba(GRASS)}</ambient><diffuse>{_rgba(GRASS)}</diffuse></material>
        </visual>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
      <intensity>1.0</intensity>
      <direction>-0.35 0.30 -0.89</direction>
      <diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>{''.join(lights)}
    <model name="nidar_arena">
      <static>true</static>
      <link name="structure">{''.join(visuals)}
      </link>
    </model>{survivors}
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>488</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


# ---------------------------------------------------------------------------
# Ground truth + picture
# ---------------------------------------------------------------------------

def ground_truth(a: Arena, world_name: str, survivor_mode: str) -> dict:
    ex, _ = a.cell_center(a.entry_col, 0)
    return {
        "world": world_name,
        "generator": "gazebo/nidar_arena_generator.py",
        "seed": a.seed,
        "frame": "Gazebo ENU: x east, y north (m); origin = outer south-west corner",
        "grid": {"cells": a.cells, "pitch_m": a.pitch, "wall_m": a.wall, "height_m": round(a.height, 4),
                 "label": "column letter (west->east) + row number (south->north), e.g. D7",
                 "cell_center": "((c+0.5)*pitch + wall/2, (r+0.5)*pitch + wall/2)"},
        "outer_size_m": round(a.outer_size, 3),
        "entry": {"cell": [a.entry_col, 0], "grid": grid_label(a.entry_col, 0),
                  "gap_center": [round(ex, 3), round(a.wall / 2, 3)]},
        "launch_pad": {"center": [round(ex, 3), -0.75], "size_m": round(BRIEF_LAUNCH_PAD_M, 4)},
        "spawn_pose": {"x": round(ex, 3), "y": -0.75, "z": 0.2, "roll": 0, "pitch": 0,
                       "yaw": round(math.pi / 2, 4),
                       "PX4_GZ_MODEL_POSE": f"{ex:.3f},-0.75,0.2,0,0,{math.pi / 2:.4f}"},
        "rooms": [{"id": r.rid, "cells": [list(c) for c in r.cells],
                   "grid": [grid_label(*c) for c in r.cells],
                   "door": {"room_cell": list(r.door[0]), "corridor_cell": list(r.door[1]),
                            "grid": grid_label(*r.door[0])} if r.door else None}
                  for r in a.rooms],
        "survivors": a.survivors,
        "survivor_models": survivor_mode,
        "walls": [[round(v, 4) for v in s] for s in wall_segments(a)],
    }


def draw_png(a: Arena, path: Path, title: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(9, 9.6), dpi=110)
    size = a.outer_size
    for r in a.rooms:
        c0, r0 = r.anchor
        x0 = a.wall / 2 + c0 * a.pitch
        y0 = a.wall / 2 + r0 * a.pitch
        ax.add_patch(Rectangle((x0, y0), 2 * a.pitch, 2 * a.pitch, color="#dbe8f7", zorder=0))
        ax.text(x0 + a.pitch, y0 + a.pitch, r.rid, ha="center", va="center", fontsize=11, color="#2a4d7a",
                weight="bold", zorder=3)
    for k in range(a.cells + 1):
        v = a.wall / 2 + k * a.pitch
        ax.plot([v, v], [a.wall / 2, size - a.wall / 2], color="#e6e6e6", lw=0.6, zorder=1)
        ax.plot([a.wall / 2, size - a.wall / 2], [v, v], color="#e6e6e6", lw=0.6, zorder=1)
    for x1, y1, x2, y2 in wall_segments(a):
        ax.plot([x1, x2], [y1, y2], color="#333333", lw=3.2, solid_capstyle="projecting", zorder=2)
    for s in a.survivors:
        ax.plot(s["x"], s["y"], marker="*", ms=18, color="#d1495b", markeredgecolor="black", zorder=4)
        ax.text(s["x"] + 0.18, s["y"] + 0.18, f"{s['id']} {s['grid']}", fontsize=9, color="#8a1c2b",
                weight="bold", zorder=5)
    ex, _ = a.cell_center(a.entry_col, 0)
    pad = BRIEF_LAUNCH_PAD_M
    ax.add_patch(Rectangle((ex - pad / 2, -0.75 - pad / 2), pad, pad, color="#d62828", zorder=3))
    ax.annotate("ENTRY / EXIT", xy=(ex, 0.2), xytext=(ex, -1.6), ha="center", fontsize=10, color="#1b7a1b",
                weight="bold", arrowprops=dict(arrowstyle="->", color="#1b7a1b", lw=2))
    for c in range(a.cells):
        x, _ = a.cell_center(c, 0)
        ax.text(x, size + 0.15, string.ascii_uppercase[c], ha="center", fontsize=8, color="#666")
    for r in range(a.cells):
        _, y = a.cell_center(0, r)
        ax.text(-0.35, y, str(r + 1), va="center", ha="center", fontsize=8, color="#666")
    ax.set_xlim(-0.8, size + 0.5)
    ax.set_ylim(-2.0, size + 0.6)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x east (m)")
    ax.set_ylabel("y north (m)")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--world", default=None, help="world name (default nidar_airmouse_arena_s<seed>)")
    ap.add_argument("--cells", type=int, default=14)
    ap.add_argument("--pitch", type=float, default=1.05, help="grid pitch, wall centre to wall centre (m)")
    ap.add_argument("--wall", type=float, default=0.05, help="wall panel thickness (m)")
    ap.add_argument("--height", type=float, default=BRIEF_HEIGHT_M)
    ap.add_argument("--rooms", type=int, default=8)
    ap.add_argument("--survivors", type=int, default=6)
    ap.add_argument("--loop-factor", type=float, default=0.10, help="share of corridor walls removed (junctions)")
    ap.add_argument("--entry-col", type=int, default=None)
    ap.add_argument("--survivor-models", choices=["fuel", "primitive"], default="fuel")
    ap.add_argument("--plugins-from", default=None,
                    help="existing world .sdf whose <plugin> elements are copied. Leave it OFF for PX4: a world "
                         "with NO <plugin> list gets PX4's server.config systems (IMU, baro, magnetometer, "
                         "NavSat, Sensors); a world WITH a list gets only the plugins listed.")
    ap.add_argument("--shadows", action="store_true",
                    help="sun shadows (nicer, but slower rendering: camera and gpu_lidar rates may drop)")
    ap.add_argument("--out-dir", default=str(HERE))
    args = ap.parse_args(argv)

    world = args.world or f"nidar_airmouse_arena_s{args.seed}"
    arena = generate_layout(args.cells, args.pitch, args.wall, args.height, args.rooms, args.survivors,
                            args.loop_factor, args.seed, args.entry_col)

    plugin_block = ""
    if args.plugins_from:
        import re
        text = Path(args.plugins_from).read_text(encoding="utf-8")
        world_part = text[text.find("<world"):]
        plugins = re.findall(r"\n?[ \t]*<plugin\b.*?(?:/>|</plugin>)", world_part, flags=re.S)
        plugin_block = "".join(plugins)

    out = Path(args.out_dir)
    (out / "worlds").mkdir(parents=True, exist_ok=True)
    (out / "layouts").mkdir(parents=True, exist_ok=True)
    sdf = build_sdf(arena, world, args.survivor_models, plugin_block, args.shadows)
    (out / "worlds" / f"{world}.sdf").write_text(sdf, encoding="utf-8")
    truth = ground_truth(arena, world, args.survivor_models)
    (out / "layouts" / f"{world}.json").write_text(json.dumps(truth, indent=2), encoding="utf-8")

    checks = validate(arena)
    title = (f"{world}: {arena.outer_size:.2f} m square, {arena.cells}x{arena.cells} grid of {arena.pitch} m, "
             f"{len(arena.rooms)} rooms, {len(arena.survivors)} survivors")
    try:
        draw_png(arena, out / "layouts" / f"{world}.png", title)
    except Exception as exc:  # matplotlib missing is not fatal
        print(f"[warn] picture not drawn: {exc}")

    print(f"World       : {out / 'worlds' / (world + '.sdf')}")
    print(f"Ground truth: {out / 'layouts' / (world + '.json')}")
    print(f"Spawn pose  : PX4_GZ_MODEL_POSE=\"{truth['spawn_pose']['PX4_GZ_MODEL_POSE']}\"")
    print("Brief checks:")
    for name, ok, detail in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")
    print("Survivors   : " + ", ".join(f"{s['id']}@{s['grid']}({s['room']})" for s in arena.survivors))
    return 0 if all(ok for _, ok, _ in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
