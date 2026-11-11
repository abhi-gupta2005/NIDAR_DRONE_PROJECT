#!/usr/bin/env python3
"""
2D LiDAR ray-caster over a generated competition arena (2026-11-02).

Reads gazebo/layouts/<world>.json (written by gazebo/nidar_arena_generator.py)
and produces the scan a horizontal 360-degree LiDAR would measure at a given
pose and height -- in ROS LaserScan form (angles counter-clockwise from the
sensor's forward axis), exactly what ros_gz_bridge publishes from the Gazebo
gpu_lidar and what gazebo/ros2/nidar_lidar_relay.py forwards to the Pi.

Used for:
  * tests of the Pi LiDAR path (relay format -> receiver -> GazeboScanFileBackend)
    without Gazebo running;
  * gazebo/tools/send_synthetic_scans.py, which feeds the real Pi receiver;
  * measuring what V18's camera+LiDAR fusion does in the arena geometry.

It is NOT Gazebo: no noise model, no physics; walls are thin boxes, survivors
are vertical cylinders with a posture-dependent height. Gazebo stays the
reference -- this only lets the Pi side be tested before Gazebo is involved.

Frames: arena ENU (x east, y north, metres), yaw = 0 facing east, CCW positive.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

PROTOCOL = "NIDAR_LIDAR_V1"

# STL-19P-like scan used by the Gazebo sensor model (gazebo/models/nidar_lidar_stl19p)
SAMPLES = 500
SCAN_HZ = 10.0
RANGE_MIN = 0.03
RANGE_MAX = 12.0

# Approximate survivor footprints seen by a horizontal scan plane.
# "lying" stands for the low Fuel models (Rescue Randy): only hit when the
# LiDAR plane is below LOW_SURVIVOR_HEIGHT_M.
SURVIVOR_SHAPES = {
    "standing": {"radius_m": 0.20, "height_m": 1.70},
    "lying": {"radius_m": 0.40, "height_m": 0.45},
}


@dataclass
class Circle:
    x: float
    y: float
    r: float
    height: float
    label: str


@dataclass
class Arena:
    segments: np.ndarray            # (N, 4) x1 y1 x2 y2 -- wall box edges
    wall_height_m: float
    circles: List[Circle]
    layout: dict

    @property
    def survivors(self) -> List[dict]:
        return list(self.layout.get("survivors", []))


def _wall_box_edges(x1: float, y1: float, x2: float, y2: float, half: float) -> List[Tuple[float, float, float, float]]:
    """Axis-aligned wall centre line -> the 4 edges of its box (thickness 2*half)."""
    xa, xb = sorted((x1, x2))
    ya, yb = sorted((y1, y2))
    xa, xb, ya, yb = xa - half, xb + half, ya - half, yb + half
    return [(xa, ya, xb, ya), (xb, ya, xb, yb), (xb, yb, xa, yb), (xa, yb, xa, ya)]


def load_arena(layout_path: Path, include_survivors: bool = True) -> Arena:
    layout = json.loads(Path(layout_path).read_text(encoding="utf-8"))
    half = float(layout["grid"]["wall_m"]) / 2.0
    edges = []
    for x1, y1, x2, y2 in layout["walls"]:
        edges.extend(_wall_box_edges(x1, y1, x2, y2, half))
    circles = []
    if include_survivors:
        for s in layout.get("survivors", []):
            shape = SURVIVOR_SHAPES.get(s.get("posture", "standing"), SURVIVOR_SHAPES["standing"])
            circles.append(Circle(float(s["x"]), float(s["y"]), shape["radius_m"], shape["height_m"], s["id"]))
    return Arena(np.array(edges, dtype=float), float(layout["grid"]["height_m"]), circles, layout)


def cast(arena: Arena, x: float, y: float, yaw: float, height_m: float = 1.0,
         samples: int = SAMPLES, range_max: float = RANGE_MAX,
         angle_min: float = -math.pi) -> Tuple[np.ndarray, List[Optional[str]]]:
    """
    Ranges for `samples` rays, ray i at body angle angle_min + i*2*pi/samples
    (CCW, ROS convention). Returns (ranges with inf for no return, hit labels).
    """
    inc = 2.0 * math.pi / samples
    body = angle_min + inc * np.arange(samples)
    world = yaw + body
    dx, dy = np.cos(world), np.sin(world)                      # (S,)
    best = np.full(samples, np.inf)
    labels: List[Optional[str]] = [None] * samples

    if height_m < arena.wall_height_m and len(arena.segments):
        sx1, sy1, sx2, sy2 = (arena.segments[:, k] for k in range(4))
        ex, ey = sx2 - sx1, sy2 - sy1                           # (N,)
        # Solve  p + t*d = s1 + u*e  for every ray/segment pair
        denom = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
        qx, qy = sx1[None, :] - x, sy1[None, :] - y
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qx * ey[None, :] - qy * ex[None, :]) / denom
            u = (qx * dy[:, None] - qy * dx[:, None]) / denom
        hit = (np.abs(denom) > 1e-12) & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
        t = np.where(hit, t, np.inf)
        wall_best = t.min(axis=1)
        closer = wall_best < best
        best = np.where(closer, wall_best, best)
        for i in np.nonzero(closer)[0]:
            labels[i] = "wall"

    for c in arena.circles:
        if height_m >= c.height:
            continue                                            # scan plane passes over it
        fx, fy = x - c.x, y - c.y
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - c.r * c.r)
        ok = disc >= 0.0
        tt = np.where(ok, -b - np.sqrt(np.where(ok, disc, 0.0)), np.inf)
        tt = np.where(tt > 1e-9, tt, np.inf)
        closer = tt < best
        best = np.where(closer, tt, best)
        for i in np.nonzero(closer)[0]:
            labels[i] = c.label

    best = np.where(best <= range_max, best, np.inf)
    return best, labels


def laserscan_message(ranges: Sequence[float], seq: int, stamp: float = 0.0,
                      angle_min: float = -math.pi, samples: Optional[int] = None,
                      range_min: float = RANGE_MIN, range_max: float = RANGE_MAX,
                      scan_rate_hz: float = SCAN_HZ, frame_id: str = "nidar_lidar",
                      sent: Optional[float] = None) -> dict:
    """
    The NIDAR_LIDAR_V1 wire message -- the same fields
    gazebo/ros2/nidar_lidar_relay.py copies out of a sensor_msgs/LaserScan.
    Non-finite ranges are sent as null.
    """
    n = len(ranges) if samples is None else samples
    inc = 2.0 * math.pi / n
    return {
        "protocol": PROTOCOL,
        "seq": int(seq),
        "stamp": float(stamp),
        "sent": float(sent) if sent is not None else None,
        "frame_id": frame_id,
        "angle_min": float(angle_min),
        "angle_max": float(angle_min + inc * (n - 1)),
        "angle_increment": float(inc),
        "range_min": float(range_min),
        "range_max": float(range_max),
        "scan_rate_hz": float(scan_rate_hz),
        "ranges": [round(float(r), 4) if math.isfinite(r) else None for r in ranges],
    }


def bearing_to(x: float, y: float, yaw: float, tx: float, ty: float) -> Tuple[float, float]:
    """
    (range_m, bearing_deg) of a target in the NIDAR convention:
    0 = straight ahead, positive = to the RIGHT (camera bearing sign).
    """
    rel = math.atan2(ty - y, tx - x) - yaw                    # CCW from forward
    rel = (rel + math.pi) % (2.0 * math.pi) - math.pi
    return math.hypot(tx - x, ty - y), -math.degrees(rel)


def cell_centre(layout: dict, c: int, r: int) -> Tuple[float, float]:
    g = layout["grid"]
    pitch, wall = float(g["pitch_m"]), float(g["wall_m"])
    return ((c + 0.5) * pitch + wall / 2.0, (r + 0.5) * pitch + wall / 2.0)


def cell_graph(layout: dict) -> dict:
    """
    Cell connectivity rebuilt from the wall list: two neighbouring cells are
    connected when no wall covers the midpoint of their shared edge.
    Returns {(c, r): [(c2, r2), ...]}.
    """
    g = layout["grid"]
    pitch, wall, cells = float(g["pitch_m"]), float(g["wall_m"]), int(g["cells"])
    half = wall / 2.0 + 1e-6

    def blocked(px, py):
        for x1, y1, x2, y2 in layout["walls"]:
            if abs(x1 - x2) < 1e-9:                             # vertical wall
                if abs(px - x1) <= half and min(y1, y2) - 1e-6 <= py <= max(y1, y2) + 1e-6:
                    return True
            else:                                               # horizontal wall
                if abs(py - y1) <= half and min(x1, x2) - 1e-6 <= px <= max(x1, x2) + 1e-6:
                    return True
        return False

    graph = {}
    for c in range(cells):
        for r in range(cells):
            cx, cy = cell_centre(layout, c, r)
            out = []
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                if 0 <= nc < cells and 0 <= nr < cells and not blocked(cx + dc * pitch / 2.0, cy + dr * pitch / 2.0):
                    out.append((nc, nr))
            graph[(c, r)] = out
    return graph


def bfs_cells(graph: dict, start, goal) -> List[Tuple[int, int]]:
    from collections import deque
    start, goal = tuple(start), tuple(goal)
    prev = {start: None}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            break
        for n in graph[cur]:
            if n not in prev:
                prev[n] = cur
                q.append(n)
    if goal not in prev:
        raise ValueError(f"no path {start} -> {goal}")
    path, cur = [], goal
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    return path[::-1]


def densify(points: Sequence[Tuple[float, float]], step_m: float = 0.25,
            final_yaw: Optional[float] = None) -> List[Tuple[float, float, float]]:
    """Waypoints -> (x, y, yaw) samples <= step_m apart, yaw along travel."""
    path: List[Tuple[float, float, float]] = []
    for (ax, ay), (bx, by) in zip(points[:-1], points[1:]):
        d = math.hypot(bx - ax, by - ay)
        if d < 1e-9:
            continue
        yaw = math.atan2(by - ay, bx - ax)
        n = max(1, int(math.ceil(d / step_m)))
        for k in range(n):
            f = k / n
            path.append((ax + f * (bx - ax), ay + f * (by - ay), yaw))
    last = points[-1]
    yaw = final_yaw if final_yaw is not None else (path[-1][2] if path else 0.0)
    path.append((last[0], last[1], yaw))
    return path


def _entry_points(layout: dict) -> List[Tuple[float, float]]:
    pad_x, pad_y = layout["launch_pad"]["center"]
    gx, gy = layout["entry"]["gap_center"]
    return [(pad_x, pad_y), (gx, gy)]


def survey_path(layout: dict, step_m: float = 0.25) -> List[Tuple[float, float, float]]:
    """
    Launch pad -> entry gap -> depth-first walk through EVERY cell (with
    backtracking), yaw along the direction of travel. Covers the whole arena.
    """
    graph = cell_graph(layout)
    order: List[Tuple[int, int]] = []
    seen = set()
    stack = [(tuple(layout["entry"]["cell"]), iter(graph[tuple(layout["entry"]["cell"])]))]
    seen.add(stack[0][0])
    order.append(stack[0][0])
    while stack:                                                # iterative DFS incl. backtracking
        cell, it = stack[-1]
        nxt = next((n for n in it if n not in seen), None)
        if nxt is None:
            stack.pop()
            if stack:
                order.append(stack[-1][0])
            continue
        seen.add(nxt)
        order.append(nxt)
        stack.append((nxt, iter(graph[nxt])))
    pts = _entry_points(layout) + [cell_centre(layout, c, r) for c, r in order]
    return densify(pts, step_m)


def rooms_path(layout: dict, step_m: float = 0.25, look_around: bool = True) -> List[Tuple[float, float, float]]:
    """
    Entry -> each room in nearest-first order -> back to the entry.
    With look_around (default since 2026-11-02e): from the room's door cell go
    to the room's centre, turn a full circle in 4 steps of 90 deg (so the
    camera sees every corner), and leave through the door again.
    Consecutive samples with the same x,y are turn-in-place steps
    (gazebo/ros2/nidar_rig_driver.RouteFollower waits for the yaw there).
    """
    graph = cell_graph(layout)
    cur = tuple(layout["entry"]["cell"])
    out: List[Tuple[float, float, float]] = []
    pts = _entry_points(layout) + [cell_centre(layout, *cur)]
    todo = list(layout["rooms"])
    while todo:
        lengths = [(len(bfs_cells(graph, cur, tuple(rm["door"]["room_cell"]))), i) for i, rm in enumerate(todo)]
        _, i = min(lengths)
        room = todo.pop(i)
        door = tuple(room["door"]["room_cell"])
        pts += [cell_centre(layout, c, r) for c, r in bfs_cells(graph, cur, door)[1:]]
        cur = door
        if look_around:
            centres = [cell_centre(layout, c, r) for c, r in room["cells"]]
            rc = (sum(p[0] for p in centres) / len(centres), sum(p[1] for p in centres) / len(centres))
            pts.append(rc)
            seg = densify(pts, step_m)
            out += seg if not out else seg[1:]
            yaw0 = seg[-1][2]
            for k in range(1, 5):
                out.append((rc[0], rc[1], (yaw0 + k * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi))
            pts = [rc, cell_centre(layout, *door)]
    pts += [cell_centre(layout, c, r) for c, r in bfs_cells(graph, cur, tuple(layout["entry"]["cell"]))[1:]]
    pts.append(tuple(layout["entry"]["gap_center"]))
    seg = densify(pts, step_m)
    out += seg if not out else seg[1:]
    return out


def survivor_view_path(layout: dict, survivor_id: str, standoff_cells: int = 1,
                       step_m: float = 0.25) -> List[Tuple[float, float, float]]:
    """Entry -> a cell next to the survivor (inside its room), ending yawed towards the survivor."""
    graph = cell_graph(layout)
    s = next(x for x in layout["survivors"] if x["id"] == survivor_id)
    target = tuple(s["cell"])
    route = bfs_cells(graph, tuple(layout["entry"]["cell"]), target)
    stop = route[-1 - standoff_cells] if len(route) > standoff_cells else route[0]
    route = route[: route.index(stop) + 1]
    pts = _entry_points(layout) + [cell_centre(layout, c, r) for c, r in route]
    sx, sy = pts[-1]
    return densify(pts, step_m, final_yaw=math.atan2(s["y"] - sy, s["x"] - sx))


def default_layout(seed: int = 1) -> Path:
    return Path(__file__).resolve().parents[2] / "gazebo" / "layouts" / f"nidar_airmouse_arena_s{seed}.json"
