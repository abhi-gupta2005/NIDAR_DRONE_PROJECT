#!/usr/bin/env python3
"""
NIDAR autonomous explorer -- the "brain" (2026-11-09). Runs on the Raspberry Pi next to SLAM V6.

Knows NOTHING about the arena in advance (no size, no walls, no rooms, no survivors). From the
live SLAM V6 occupancy grid and pose only, it decides where the drone flies next:

  1. targets  = frontiers of two kinds: LiDAR frontiers (known free cells next to unknown space)
                and CAMERA frontiers (free cells the camera has seen next to free cells it has
                not seen -- survivors are found by the camera, not the LiDAR). A frontier touches
                the space it leads to, so no wall can be between them (a room behind a wall is
                reached through its door). Only inside the area enclosed by the walls seen so far
                (it never wanders off into open space outside the arena, e.g. behind the pad);
  2. viewpoint = a safe cell within FRONTIER_REACH_M (through free space) of a frontier, with the
                best (path cost - information) trade-off;
  3. path     = Dijkstra on known free cells that keep >= DRONE_RADIUS from every wall, with a
                cost that pulls the path to the middle of corridors and doors; then shortened
                into straight legs (line of sight on the same safe cells);
  4. at the viewpoint: turn a full circle so the camera sees all around;
  5. nothing left to explore or look at -> fly back to the take-off point -> land.

Frames: the SLAM frame (origin = take-off point, x = initial heading, y left, metres).
Grid: SLAM V6 export, int8, -1 unknown, 0..100 occupancy %, cell (row j, col i) has its
lower-left corner at (origin + i*res, origin + j*res).

No flight control here: it produces plans (straight legs + an end action) for the flight
interface (gazebo/ros2 rig driver / PX4 flyer in follow mode), which keeps its own guards.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------- tuning --
DRONE_RADIUS_M = 0.25        # physical radius (prop tips) of the drone; x500 = 0.39 (too wide for 1 m corridors)
MAP_MARGIN_M = 0.05          # 0.1 m map cells blur walls by up to half a cell: planning radius = radius + this
PREFERRED_CLEARANCE_M = 0.60 # paths are pulled towards this distance from walls (corridor middle)
CLEARANCE_WEIGHT = 20.0      # how strongly (cost multiplier at zero extra clearance)
STRAIGHT_LEG_EXTRA_M = 0.10  # a straight leg may only cross cells with clearance >= radius + this
                             # (map cells blur walls by up to 0.05 m; otherwise: more turn points)
OCC_PCT = 65                 # >= : wall
FREE_PCT = 35                # <  : free (between: uncertain, treated as wall for clearance)
CAMERA_RANGE_M = 5.0         # camera "has looked at" a cell within this range (a 1.7 m person is ~150 px tall)
CAMERA_TARGET_CLEAR_M = 0.2  # unseen cells hugging a wall are not camera targets (rays miss them; a person
                             # there is seen anyway: its body reaches >= 0.2 m out from the wall)
CAMERA_HFOV = math.radians(70.0)
FRONTIER_REACH_M = 0.8       # a viewpoint must be this close to a frontier, measured through free space
INFO_R_M = 1.0               # information = frontier cells within this square radius
INFO_WEIGHT_M = 0.15         # metres of path worth one frontier cell of information
MIN_TARGET_CELLS = 8         # ignore frontier specks smaller than this (cells)
WALL_BOX_MARGIN_M = 0.0      # targets only inside the bounding box of the walls seen so far
ARRIVE_EXHAUST_R_M = 1.2     # after looking from a viewpoint, frontiers this close (through free space) that
                             # remain are given up (not reachable / not visible for this drone)
VISITED_R_M = 0.4            # a viewpoint already looked from is not chosen again
MAX_PLAN_CELLS = 250_000
LOOK_BIN_DEG = 10            # directions checked for unseen space when planning the look at a viewpoint
MISSION_TIME_S = 20 * 60     # after this: return to the take-off point and land


@dataclass
class Plan:
    waypoints: List[Tuple[float, float]]
    end: str                            # "look" (turn through look_yaws) | "none" | "land"
    goal: Tuple[float, float]
    reason: str
    cost_m: float = 0.0
    look_yaws: List[float] = field(default_factory=list)   # absolute yaws to turn through, in order


@dataclass
class GridView:
    occ: np.ndarray                     # bool, wall/uncertain
    free: np.ndarray                    # bool, known free
    unknown: np.ndarray                 # bool
    res: float
    origin: float

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor((y - self.origin) / self.res)), int(math.floor((x - self.origin) / self.res))

    def centre(self, j: int, i: int) -> Tuple[float, float]:
        return self.origin + (i + 0.5) * self.res, self.origin + (j + 0.5) * self.res

    @classmethod
    def from_int8(cls, grid: np.ndarray, res: float = 0.1, origin: float = -20.0) -> "GridView":
        g = np.asarray(grid)
        unknown = g < 0
        occ = (g >= FREE_PCT) & ~unknown
        free = (g >= 0) & (g < FREE_PCT)
        return cls(occ=occ, free=free, unknown=unknown, res=float(res), origin=float(origin))


def distance_to(mask: np.ndarray, res: float) -> np.ndarray:
    """Euclidean distance (m) from every cell centre to the nearest True cell centre."""
    if not mask.any():
        return np.full(mask.shape, np.inf, dtype=np.float32)
    src = np.where(mask, 0, 255).astype(np.uint8)
    try:
        import cv2
        d = cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    except ImportError:                                      # pragma: no cover (Pi and laptop have cv2)
        from scipy.ndimage import distance_transform_edt
        d = distance_transform_edt(src > 0)
    return d.astype(np.float32) * res


def box_count(mask: np.ndarray, r_cells: int) -> np.ndarray:
    """Number of True cells within a (2r+1)^2 square around every cell (integral image)."""
    m = mask.astype(np.int32)
    ii = np.pad(m.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    n0, n1 = m.shape
    j0 = np.clip(np.arange(n0) - r_cells, 0, n0)
    j1 = np.clip(np.arange(n0) + r_cells + 1, 0, n0)
    i0 = np.clip(np.arange(n1) - r_cells, 0, n1)
    i1 = np.clip(np.arange(n1) + r_cells + 1, 0, n1)
    return (ii[j1][:, i1] - ii[j0][:, i1] - ii[j1][:, i0] + ii[j0][:, i0])


def line_cells(j0, i0, j1, i1):
    """Cells on the straight line between two cells (Bresenham, inclusive)."""
    cells = []
    dj, di = abs(j1 - j0), abs(i1 - i0)
    sj, si = (1 if j1 > j0 else -1), (1 if i1 > i0 else -1)
    err = di - dj
    j, i = j0, i0
    while True:
        cells.append((j, i))
        if j == j1 and i == i1:
            return cells
        e2 = 2 * err
        if e2 > -dj:
            err -= dj
            i += si
        if e2 < di:
            err += di
            j += sj


class Explorer:
    def __init__(self, drone_radius: float = DRONE_RADIUS_M, camera_range: float = CAMERA_RANGE_M,
                 mission_time_s: float = MISSION_TIME_S):
        self.radius = float(drone_radius)
        self.R = float(drone_radius) + MAP_MARGIN_M                # planning radius
        self.mission_time_s = float(mission_time_s)
        self.t_start: Optional[float] = None
        self.cam_range = float(camera_range)
        self.seen: Optional[np.ndarray] = None          # camera coverage (bool, same grid)
        self.sticky: Optional[np.ndarray] = None        # cells that were clearly walls at some time
        self.hits: Optional[np.ndarray] = None          # cells with a raw LiDAR hit (always obstacles)
        self.given_up: Optional[np.ndarray] = None       # targets abandoned (bool)
        self.visited: List[Tuple[float, float]] = []      # viewpoints already looked from
        self.home: Optional[Tuple[float, float]] = None
        self.view: Optional[GridView] = None
        self.clear: Optional[np.ndarray] = None
        self.state = "EXPLORE"                           # EXPLORE | RETURN | DONE
        self.last_goal: Optional[Tuple[float, float]] = None
        self.stats = {"plans": 0, "looks": 0, "given_up_cells": 0}

    # ------------------------------------------------------------ inputs --
    def update_map(self, grid: np.ndarray, res: float = 0.1, origin: float = -20.0) -> None:
        g = np.asarray(grid)
        if self.seen is None or self.seen.shape != g.shape:
            self.seen = np.zeros(g.shape, dtype=bool)
            self.given_up = np.zeros(g.shape, dtype=bool)
            self.sticky = np.zeros(g.shape, dtype=bool)
        # Sticky walls: a cell that was clearly a wall stays a wall for planning. Beams grazing a thin
        # (5 cm) wall pass through the 0.1 m cells that contain it and slowly mark it free again
        # (seen 2026-11-09: a path along the outside of the south wall, 0.28 m from it).
        self.sticky |= g >= OCC_PCT
        if self.hits is None or self.hits.shape != g.shape:
            self.hits = np.zeros(g.shape, dtype=bool)
        self.view = GridView.from_int8(g, res, origin)
        self._apply_obstacles()

    def _apply_obstacles(self) -> None:
        self.view.occ |= self.sticky | self.hits
        self.view.free &= ~(self.sticky | self.hits)
        self.clear = distance_to(self.view.occ, self.view.res)
        self._hits_dirty = False

    def add_hits(self, points_xy: np.ndarray) -> None:
        """Raw LiDAR hit points (SLAM frame, Nx2): every hit is a real surface, so its cell is an obstacle
        for planning even if the averaged map has not (yet) marked it -- a thin wall seen only at grazing
        angles stays 'free' in the log-odds map (seen 2026-11-09: a leg 0.06 m from such a wall)."""
        if self.view is None or len(points_xy) == 0:
            return
        v = self.view
        j = np.floor((points_xy[:, 1] - v.origin) / v.res).astype(int)
        i = np.floor((points_xy[:, 0] - v.origin) / v.res).astype(int)
        ok = (j >= 0) & (j < self.hits.shape[0]) & (i >= 0) & (i < self.hits.shape[1])
        new = ~self.hits[j[ok], i[ok]]
        if new.any():
            self.hits[j[ok], i[ok]] = True
            self._hits_dirty = True

    def refresh(self) -> None:
        if self.view is not None and getattr(self, "_hits_dirty", False):
            self._apply_obstacles()

    def plan_still_safe(self, x: float, y: float, waypoints: Sequence[Tuple[float, float]], start_index: int = 0) -> bool:
        """Re-check the rest of a plan against the newest map and hits (call ~1 Hz while flying it)."""
        self.refresh()
        safe = self.view.free & (self.clear >= self.R)
        pts = [(x, y)] + list(waypoints[start_index:])
        for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
            for c in line_cells(*self.view.cell(ax, ay), *self.view.cell(bx, by)):
                if not safe[c]:
                    return False
        return True

    def observe(self, x: float, y: float, yaw: float, n_rays: int = 48) -> None:
        """Mark what the camera sees now: rays across its field of view, stopped by walls (vectorised)."""
        v = self.view
        if v is None:
            return
        if self.home is None:
            self.home = (x, y)
        steps = int(self.cam_range / v.res)
        a = yaw + np.linspace(-CAMERA_HFOV / 2, CAMERA_HFOV / 2, n_rays)
        s = np.arange(1, steps + 1) * v.res
        px = x + np.cos(a)[:, None] * s[None, :]
        py = y + np.sin(a)[:, None] * s[None, :]
        j = np.floor((py - v.origin) / v.res).astype(int)
        i = np.floor((px - v.origin) / v.res).astype(int)
        n0, n1 = v.occ.shape
        inside = (j >= 0) & (j < n0) & (i >= 0) & (i < n1)
        jc, ic = np.clip(j, 0, n0 - 1), np.clip(i, 0, n1 - 1)
        blocked = v.occ[jc, ic] | ~inside
        first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), steps)
        keep = np.arange(steps)[None, :] < first[:, None]
        self.seen[jc[keep], ic[keep]] = True

    # ----------------------------------------------------------- planning --
    def safe(self) -> np.ndarray:
        return self.view.free & (self.clear >= self.R)

    def wall_box(self):
        js, is_ = np.nonzero(self.view.occ)
        if len(js) < 20:
            return None
        m = int(round(WALL_BOX_MARGIN_M / self.view.res))
        return js.min() - m, js.max() + m, is_.min() - m, is_.max() + m

    def targets(self) -> np.ndarray:
        """Frontier cells (bool): LiDAR frontiers + camera frontiers, inside the walls' box, not given up."""
        v = self.view

        def touches(mask):
            out = np.zeros_like(mask)
            out[1:, :] |= mask[:-1, :]
            out[:-1, :] |= mask[1:, :]
            out[:, 1:] |= mask[:, :-1]
            out[:, :-1] |= mask[:, 1:]
            return out

        lidar_frontier = v.free & touches(v.unknown)
        camera_frontier = v.free & self.seen & touches(v.free & ~self.seen & (self.clear >= CAMERA_TARGET_CLEAR_M))
        box = self.wall_box()
        inside = np.zeros_like(v.free)
        if box is not None:
            j0, j1, i0, i1 = box
            inside[max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] = True
        return (lidar_frontier | camera_frontier) & inside & ~self.given_up

    def near_through_free(self, mask: np.ndarray, radius_m: float) -> np.ndarray:
        """Cells within radius of `mask`, moving only through known free cells (geodesic dilation)."""
        reach = mask.copy()
        passable = self.view.free
        for _ in range(int(round(radius_m / self.view.res))):
            grown = reach.copy()
            grown[1:, :] |= reach[:-1, :]
            grown[:-1, :] |= reach[1:, :]
            grown[:, 1:] |= reach[:, :-1]
            grown[:, :-1] |= reach[:, 1:]
            reach = grown & passable
        return reach

    def dijkstra(self, start_cell, passable, cost_mult):
        """Costs (m) from start over passable cells (8-connected); returns (dist, parent) dicts on a crop."""
        js, is_ = np.nonzero(passable)
        if len(js) == 0:
            return None
        j0, j1 = max(js.min() - 1, 0), min(js.max() + 2, passable.shape[0])
        i0, i1 = max(is_.min() - 1, 0), min(is_.max() + 2, passable.shape[1])
        P = passable[j0:j1, i0:i1]
        C = cost_mult[j0:j1, i0:i1]
        H, W = P.shape
        sj, si = start_cell[0] - j0, start_cell[1] - i0
        if not (0 <= sj < H and 0 <= si < W) or not P[sj, si]:
            return None
        dist = np.full(H * W, np.inf)
        parent = np.full(H * W, -1, dtype=np.int64)
        s = sj * W + si
        dist[s] = 0.0
        heap = [(0.0, s)]
        res = self.view.res
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        Pf, Cf = P.ravel(), C.ravel()
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            uj, ui = divmod(u, W)
            for dj, di, L in steps:
                vj, vi = uj + dj, ui + di
                if 0 <= vj < H and 0 <= vi < W:
                    vv = vj * W + vi
                    if Pf[vv]:
                        if dj and di and not (Pf[uj * W + vi] and Pf[vj * W + ui]):
                            continue                        # no corner cutting
                        nd = d + L * res * 0.5 * (Cf[u] + Cf[vv])
                        if nd < dist[vv]:
                            dist[vv] = nd
                            parent[vv] = u
                            heapq.heappush(heap, (nd, vv))
        return {"dist": dist.reshape(H, W), "parent": parent, "off": (j0, i0), "shape": (H, W)}

    def cost_field(self, start_xy):
        safe = self.safe()
        extra = np.clip(PREFERRED_CLEARANCE_M - self.clear, 0.0, None) / max(PREFERRED_CLEARANCE_M - self.R, 1e-3)
        mult = 1.0 + CLEARANCE_WEIGHT * np.minimum(extra, 1.0) ** 2
        sc = self.view.cell(*start_xy)
        if not (0 <= sc[0] < safe.shape[0] and 0 <= sc[1] < safe.shape[1]):
            return None
        if not safe[sc]:
            sc = self._nearest_safe(sc, safe)
            if sc is None:
                return None
        return self.dijkstra(sc, safe, mult)

    def _nearest_safe(self, cell, safe, max_r=8):
        """Nearest safe cell reachable from `cell` through known free cells (never across a wall)."""
        from collections import deque
        free = self.view.free | safe
        n0, n1 = safe.shape
        seen = {tuple(cell)}
        q = deque([(tuple(cell), 0)])
        while q:
            (j, i), d = q.popleft()
            if safe[j, i]:
                return (j, i)
            if d >= max_r:
                continue
            for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nj, ni = j + dj, i + di
                if 0 <= nj < n0 and 0 <= ni < n1 and (nj, ni) not in seen and free[nj, ni]:
                    seen.add((nj, ni))
                    q.append(((nj, ni), d + 1))
        return None

    def path_to(self, field_, goal_cell) -> List[Tuple[float, float]]:
        j0, i0 = field_["off"]
        H, W = field_["shape"]
        u = (goal_cell[0] - j0) * W + (goal_cell[1] - i0)
        cells = []
        while u >= 0:
            uj, ui = divmod(int(u), W)
            cells.append((uj + j0, ui + i0))
            u = field_["parent"][u]
        cells.reverse()
        return self.simplify(cells)

    def simplify(self, cells) -> List[Tuple[float, float]]:
        """String pulling: keep a waypoint only where the straight line would leave the safe area,
        then snap the turning points to the cell centres (legs run through the corridor middle)."""
        if not cells:
            return []
        safe = self.safe() & (self.clear >= self.R + STRAIGHT_LEG_EXTRA_M)
        keep = [cells[0]]
        k = 0
        while k < len(cells) - 1:
            nxt = k + 1
            for m in range(len(cells) - 1, k, -1):
                if all(safe[c] or c in (cells[k], cells[m]) for c in line_cells(*cells[k], *cells[m])):
                    nxt = m
                    break
            keep.append(cells[nxt])
            k = nxt
        return [self.view.centre(*c) for c in keep]

    def plan(self, x: float, y: float, yaw: float, t: Optional[float] = None) -> Optional[Plan]:
        """Next plan from the current pose, or None when the mission is complete (landed at home)."""
        if self.view is None:
            return None
        self.refresh()
        if self.home is None:
            self.home = (x, y)
        if t is not None:
            self.t_start = t if self.t_start is None else self.t_start
            if self.state == "EXPLORE" and t - self.t_start > self.mission_time_s:
                self.state = "RETURN"
                self.return_reason = f"mission time {self.mission_time_s / 60:.0f} min used"
        self.stats["plans"] += 1
        field_ = self.cost_field((x, y))
        if field_ is None:
            return Plan([], "none", (x, y), "no safe cell around the drone: hold", 0.0)
        if self.state == "EXPLORE":
            p = self._explore_plan(field_, (x, y))
            if p is not None:
                return p
            self.state = "RETURN"
            self.return_reason = "nothing left to explore"
        if self.state == "RETURN":
            hc = self.view.cell(*self.home)
            j0, i0 = field_["off"]
            H, W = field_["shape"]
            hj, hi = hc[0] - j0, hc[1] - i0
            safe = self.safe()
            if not (0 <= hj < H and 0 <= hi < W and np.isfinite(field_["dist"][hj, hi])):
                near = self._nearest_safe(hc, safe)
                if near is not None:
                    hj, hi = near[0] - j0, near[1] - i0
            if 0 <= hj < H and 0 <= hi < W and np.isfinite(field_["dist"][hj, hi]):
                wps = self.path_to(field_, (hj + j0, hi + i0))
                if wps and (hj + j0, hi + i0) == tuple(hc):
                    wps[-1] = self.home                    # land on the take-off point, not its cell centre
                if math.hypot(x - self.home[0], y - self.home[1]) < 0.3:
                    self.state = "DONE"
                    return Plan([], "land", self.home, "exploration complete: landing at the take-off point")
                return Plan(wps, "land", self.home, f"{getattr(self, 'return_reason', 'done')}: returning to the "
                            "take-off point", float(field_["dist"][hj, hi]))
            self.state = "DONE"
            return Plan([], "land", (x, y), "home not reachable on known free cells: landing here")
        return None

    def _explore_plan(self, field_, pos) -> Optional[Plan]:
        frontier = self.targets()
        v = self.view
        info = box_count(frontier, int(round(INFO_R_M / v.res))).astype(float)
        reach = self.near_through_free(frontier, FRONTIER_REACH_M)
        j0, i0 = field_["off"]
        H, W = field_["shape"]
        dist = field_["dist"]
        box = self.wall_box()
        inside = np.zeros_like(reach)
        if box is not None:
            bj0, bj1, bi0, bi1 = box
            inside[max(bj0, 0):bj1 + 1, max(bi0, 0):bi1 + 1] = True
        I = np.where(reach & inside, info, 0.0)[j0:j0 + H, i0:i0 + W]       # viewpoints inside the walls too
        I[I < MIN_TARGET_CELLS] = 0.0
        rv = int(round(VISITED_R_M / v.res))
        for vx, vy in self.visited:                        # never look from the same spot twice
            cj, ci = v.cell(vx, vy)
            I[max(cj - j0 - rv, 0):max(cj - j0 + rv + 1, 0), max(ci - i0 - rv, 0):max(ci - i0 + rv + 1, 0)] = 0.0
        score = np.where(np.isfinite(dist) & (I > 0), dist - INFO_WEIGHT_M * np.minimum(I, 80.0), np.inf)
        if not np.isfinite(score).any():
            return None
        k = int(np.argmin(score))
        gj, gi = divmod(k, W)
        goal = v.centre(gj + j0, gi + i0)
        wps = self.path_to(field_, (gj + j0, gi + i0))
        self.last_goal = goal
        arrive_yaw = math.atan2(wps[-1][1] - wps[-2][1], wps[-1][0] - wps[-2][0]) if len(wps) >= 2 else None
        yaws = self.look_yaws(goal[0], goal[1], arrive_yaw)
        return Plan(wps, "look" if yaws else "none", goal, f"explore: {int(frontier.sum())} frontier cells left",
                    float(dist[gj, gi]), yaws)

    def look_yaws(self, x: float, y: float, arrive_yaw: Optional[float]) -> List[float]:
        """Yaws to turn through at a viewpoint so the camera sees the not-yet-seen free space around it.
        Only the arc that contains unseen space is swept (the LiDAR sees all round anyway); [] = no look."""
        v = self.view
        nb = int(360 / LOOK_BIN_DEG)
        steps = int(self.cam_range / v.res)
        need = []
        n0, n1 = v.occ.shape
        for b in range(nb):
            a = math.radians(b * LOOK_BIN_DEG)
            ca, sa = math.cos(a), math.sin(a)
            hit = False
            for k in range(2, steps + 1):
                j, i = v.cell(x + ca * k * v.res, y + sa * k * v.res)
                if not (0 <= j < n0 and 0 <= i < n1) or v.occ[j, i]:
                    break
                if v.free[j, i] and not self.seen[j, i] and not self.given_up[j, i] \
                        and self.clear[j, i] >= CAMERA_TARGET_CLEAR_M:
                    hit = True
                    break
            if hit:
                need.append(a)
        if not need:
            return []
        need.sort()
        if len(need) == 1:
            gaps = [(2 * math.pi, 0)]
        else:
            gaps = [((need[(k + 1) % len(need)] - need[k]) % (2 * math.pi), k) for k in range(len(need))]
        gap, k = max(gaps)
        start = need[(k + 1) % len(need)]                 # sweep CCW from start ...
        arc = (2 * math.pi - gap) if len(need) > 1 else 0.0
        half = CAMERA_HFOV / 2 * 0.8
        first, sweep = start + half, max(0.0, arc - 2 * half)  # ... with the camera's own width at both ends
        if arc <= 2 * half:
            first, sweep = start + arc / 2, 0.0
        if arrive_yaw is not None and abs(math.atan2(math.sin(first + sweep - arrive_yaw),
                                                     math.cos(first + sweep - arrive_yaw))) < \
                abs(math.atan2(math.sin(first - arrive_yaw), math.cos(first - arrive_yaw))):
            first, sweep = first + sweep, -sweep              # start at the end nearer to the arrival heading
        n = max(1, int(math.ceil(abs(sweep) / (math.pi / 2))))
        return [math.atan2(math.sin(first + sweep * q / n), math.cos(first + sweep * q / n)) for q in range(n + 1)] \
            if sweep else [math.atan2(math.sin(first), math.cos(first))]

    def arrived_and_looked(self, x: float, y: float) -> None:
        """Call after the full-circle look at a viewpoint: frontiers still this close (through free
        space) are given up -- e.g. a gap too narrow for this drone, or space hidden in a corner."""
        self.stats["looks"] += 1
        self.visited.append((x, y))
        if self.view is None:
            return
        frontier = self.targets()
        here = np.zeros_like(frontier)
        j, i = self.view.cell(x, y)
        if 0 <= j < here.shape[0] and 0 <= i < here.shape[1]:
            here[j, i] = True
        near = self.near_through_free(here, ARRIVE_EXHAUST_R_M)
        gone = frontier & near
        self.given_up |= gone
        self.stats["given_up_cells"] += int(gone.sum())
