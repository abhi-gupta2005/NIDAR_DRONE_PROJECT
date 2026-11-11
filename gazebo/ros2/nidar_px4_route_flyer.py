#!/usr/bin/env python3
"""
NIDAR PX4 route flyer -- SIMULATION ONLY (2026-11-09).

Flies the PX4 x500_nidar_airmouse drone through the Gazebo arena in OFFBOARD
position mode, so the whole NIDAR chain (LiDAR -> Pi SLAM, camera -> YOLO ->
V18 -> survivor mapper -> ground station) runs on a flying PX4 drone instead
of the rig. Same idea as nidar_rig_driver.py, but PX4 does the flying.

  pad -> entry -> every room: corridor to the cell outside the door, enter the
  door cell, look around 4 x 90 deg, back out -> entry -> pad -> LAND

Why it is built this way (x500 vs the arena):
  * the x500's prop tips reach 0.39 m from its centre; a corridor gives 0.475 m
    from the centreline to the wall face. So the drone only flies straight
    lines between cell centres and stops to turn on the spot at each corner
    (no corner cutting), at a slow speed (default 0.3 m/s);
  * a survivor can stand right inside a door (arenas s2/s3: 0.10 and 0.16 m
    from the door-cell centre). The drone enters a room only as far as its
    own LiDAR shows >= 0.45 m free ahead, looks around from there if >= 0.40 m
    is free all round, then backs out along the same line. It never uses the
    survivors' true positions.

Safety (this file can arm a vehicle, so):
  1. it refuses to run unless the Gazebo arena world is running on THIS
     machine (gz topic /world/<world>/clock) - a real drone has no such world;
  2. altitude fixed (default 0.64 m above take-off = LiDAR plane 1.0 m, camera
     0.82 m like the rig), max 1.5 m; speed <= 0.6 m/s; yaw <= 30 deg/s;
  3. every setpoint must lie inside the arena or the pad->entry corridor;
  4. LiDAR guard: obstacle < 0.30 m anywhere, or an unexpected obstacle ahead
     in a corridor, or tracking error > 0.8 m, or a turn/leg taking 15 s longer
     than planned (PX4 not following), or no PX4 position for 2 s -> LAND where
     it is (gaps up to 2 s: PX4 holds the last setpoint; up to 8 s if the LiDAR stopped too = Gazebo paused);
  5. Ctrl+C at any time -> LAND command (sent before ROS shuts down);
  6. if PX4 leaves OFFBOARD (you typed a mode at pxh>, or a failsafe), this
     script stops commanding at once and does not fight it.
It never touches nidar_bridge / px4_setpoint_sim (their ALLOW_ARM and
ALLOW_OFFBOARD stay False): NIDAR perception does NOT steer this drone.

Run in WSL (after MicroXRCEAgent, run_px4.sh, the sensor bridge, relays, Pi, YOLO, GCS):
    source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
    python3 $NIDAR_REPO/gazebo/ros2/nidar_px4_route_flyer.py --seed 1 --route rooms
    python3 $NIDAR_REPO/gazebo/ros2/nidar_px4_route_flyer.py --seed 1 --route survivor:S2
    python3 $NIDAR_REPO/gazebo/ros2/nidar_px4_route_flyer.py --seed 1 --dry-run
Log: logs/px4_flight_<time>.csv (PX4's position in the arena frame).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from src.simulation import arena_raycast as ar  # noqa: E402

# ------------------------------------------------------------------ limits --
ALT_DEFAULT_M = 0.64          # x500_nidar_airmouse: LiDAR 0.36 m, camera 0.18 m above ground when landed
ALT_MAX_M = 1.5               # walls 2.44 m, net above
SPEED_DEFAULT, SPEED_MAX = 0.3, 0.6
YAW_RATE = math.radians(30.0)
CLIMB_RATE = 0.3
ARRIVE_M = 0.08
YAW_TOL = math.radians(8.0)
LOOK_DWELL_S = 1.5
FACE_DWELL_S = 3.0
TRACK_PAUSE_M = 0.45          # carrot waits while the drone lags this far behind (with velocity feed-forward
                              # PX4 lags little; without it ~0.32 m at 0.3 m/s -> stop-and-go, seen 2026-11-09)
TRACK_ABORT_M = 0.80
CROSS_TRACK_ABORT_M = 0.30    # sideways off the centreline: walls are 0.475 m away, prop tips 0.39 m
STOP_AHEAD_M = 0.45           # probe moves stop when something is this close ahead
TURN_CLEAR_M = 0.40           # turn on the spot only with this much room all round
EMERGENCY_M = 0.30            # anything this close: land
AHEAD_SECTOR = math.radians(50.0)
SETTLE_M = 0.06               # turn on the spot only once back within this of the turn point (F33: 09:55 crash
                              # turned 0.11 m overshot, 0.39 m from a wall = prop reach)
SCAN_SETTLE_S = 1.5           # after a turn, start the leg only when the heading was steady this long: the LiDAR
                              # arrives up to ~1 s behind PX4's heading (10:44: a scan taken mid-turn showed the
                              # side wall 0.43 m "ahead" -> false LAND)
SETTLE_MAX_S = 12.0           # cannot settle at a turn point for this long -> land
FENCE_MARGIN_M = 0.30
POSE_GAP_HOLD_S = 0.5          # PX4 position older than this: hold the last setpoint (WSL/DDS hiccup)
POSE_GAP_LAND_S = 2.0          # older than this: land (2026-11-09: 0.5 s was too strict under WSL load)
SIM_PAUSE_MAX_S = 8.0          # ... unless the LiDAR stopped too: the whole simulation is paused (nothing moves);
                               # wait up to this long. 2026-11-09: Gazebo in WSL pauses 1.7-2.8 s every ~35 s
                               # (rig and PX4 runs; the rig moved 0.01 m in a 2.5 s gap at 0.2 m/s), and one
                               # 2.01 s pause landed a PX4 flight for nothing.


RESUME_GRACE_S = 2.0           # after such a pause the LiDAR comes back first; PX4's position must follow within this
                               # (2026-11-09 10:27: PX4 0.2 s behind the LiDAR after a pause -> landed for nothing)


def gap_action(pose_age: float, scan_age: float, paused_in_gap: bool = False,
               since_lidar_back: float = None) -> str:
    """'ok' | 'hold' | 'land' for a gap in PX4's position (pure; tested). A gap while the LiDAR keeps
    coming means PX4's data is really missing; a gap with the LiDAR silent too means Gazebo paused, and
    when Gazebo resumes the LiDAR is back a moment before PX4's position."""
    if pose_age <= POSE_GAP_HOLD_S:
        return "ok"
    if pose_age > SIM_PAUSE_MAX_S:
        return "land"
    if scan_age > POSE_GAP_HOLD_S:                       # paused right now
        return "hold"
    if paused_in_gap and since_lidar_back is not None and since_lidar_back <= RESUME_GRACE_S:
        return "hold"                                    # just resumed: give PX4 its moment
    return "hold" if pose_age <= POSE_GAP_LAND_S else "land"
NO_PROGRESS_S = 15.0           # no movement / no turning for this long while commanded: PX4 not following -> land


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------- frames -------
@dataclass
class ArenaNed:
    """Arena (ENU: x east, y north, yaw CCW from east) <-> PX4 local NED, anchored at take-off.
    The drone takes off from the launch pad (= layout spawn pose); PX4's local origin is
    wherever its estimator started, so the take-off NED position (n0, e0, d0) is the anchor."""
    pad_x: float
    pad_y: float
    n0: float = 0.0
    e0: float = 0.0
    d0: float = 0.0

    def to_ned(self, x: float, y: float, alt: float) -> Tuple[float, float, float]:
        return self.n0 + (y - self.pad_y), self.e0 + (x - self.pad_x), self.d0 - alt

    def to_arena(self, n: float, e: float, d: float) -> Tuple[float, float, float]:
        return self.pad_x + (e - self.e0), self.pad_y + (n - self.n0), self.d0 - d

    @staticmethod
    def yaw_to_ned(yaw_enu: float) -> float:
        return wrap(math.pi / 2 - yaw_enu)

    @staticmethod
    def yaw_to_enu(yaw_ned: float) -> float:
        return wrap(math.pi / 2 - yaw_ned)


# ------------------------------------------------------------- route --------
@dataclass
class Task:
    kind: str                                   # translate | turn | dwell | land
    target: Optional[Tuple[float, float]] = None
    yaw: Optional[float] = None                 # turn: absolute arena yaw
    seconds: float = 0.0
    probe: bool = False                         # translate inside a room: may stop early
    exc: Optional[int] = None                   # excursion id
    idx: int = 0                                # point index inside the excursion
    ret: bool = False                           # excursion return leg
    look: bool = False                          # part of an excursion's look
    face: Optional[Tuple[float, float]] = None  # turn towards this point (from wherever the drone is)


def _centre(layout, cell):
    return ar.cell_centre(layout, int(cell[0]), int(cell[1]))


def _compress(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Drop repeated points and merge collinear runs into one straight leg."""
    pts = []
    for p in points:
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
            pts.append((float(p[0]), float(p[1])))
    out = pts[:2]
    for p in pts[2:]:
        a, b = out[-2], out[-1]
        cross = (b[0] - a[0]) * (p[1] - b[1]) - (b[1] - a[1]) * (p[0] - b[0])
        dot = (b[0] - a[0]) * (p[0] - b[0]) + (b[1] - a[1]) * (p[1] - b[1])
        if abs(cross) < 1e-6 and dot > 0:
            out[-1] = p
        else:
            out.append(p)
    return out


def _corridor_legs(points) -> List[Task]:
    return [Task("translate", target=p) for p in _compress(points)[1:]]


def _excursion(layout, eid: int, outside, cells, look: str, face_xy=None) -> List[Task]:
    """outside cell -> room cells (probe legs) -> look -> the same way back to outside."""
    pts = [_centre(layout, outside)] + [_centre(layout, c) for c in cells]
    tasks = [Task("translate", target=pts[i], probe=True, exc=eid, idx=i) for i in range(1, len(pts))]
    if look == "around":
        for k in range(1, 5):
            tasks.append(Task("turn", yaw=k * math.pi / 2, exc=eid, look=True))   # relative, resolved later
            tasks.append(Task("dwell", seconds=LOOK_DWELL_S, exc=eid, look=True))
    else:
        tasks.append(Task("turn", face=face_xy, exc=eid, look=True))
        tasks.append(Task("dwell", seconds=FACE_DWELL_S, exc=eid, look=True))
    for j in range(len(pts) - 2, -1, -1):
        tasks.append(Task("translate", target=pts[j], probe=True, exc=eid, idx=j, ret=True))
    return tasks


def build_route(layout: dict, route: str = "rooms") -> Tuple[List[Task], dict]:
    graph = ar.cell_graph(layout)
    pad = tuple(layout["launch_pad"]["center"])
    gap = tuple(layout["entry"]["gap_center"])
    entry = tuple(layout["entry"]["cell"])
    tasks: List[Task] = []
    tasks += _corridor_legs([pad, gap, _centre(layout, entry)])
    cur = entry
    eid = 0
    if route == "rooms":
        todo = list(layout["rooms"])
        while todo:
            lens = [(len(ar.bfs_cells(graph, cur, tuple(r["door"]["corridor_cell"]))), i) for i, r in enumerate(todo)]
            room = todo.pop(min(lens)[1])
            outside = tuple(room["door"]["corridor_cell"])
            cells = ar.bfs_cells(graph, cur, outside)
            tasks += _corridor_legs([_centre(layout, c) for c in cells])
            tasks += _excursion(layout, eid, outside, [tuple(room["door"]["room_cell"])], "around")
            eid += 1
            cur = outside
    elif route.startswith("survivor:"):
        sid = route.split(":", 1)[1]
        s = next(x for x in layout["survivors"] if x["id"] == sid)
        room = next(r for r in layout["rooms"] if r["id"] == s["room"])
        outside = tuple(room["door"]["corridor_cell"])
        door = tuple(room["door"]["room_cell"])
        tasks += _corridor_legs([_centre(layout, c) for c in ar.bfs_cells(graph, cur, outside)])
        inside = ar.bfs_cells(graph, door, tuple(s["cell"]))          # door ... survivor's cell
        stop = inside[:-1] if len(inside) > 1 else inside              # cell before the survivor (or the door)
        tasks += _excursion(layout, eid, outside, stop, "face", (s["x"], s["y"]))
        cur = outside
    else:
        raise ValueError(f"unknown route {route!r} (rooms | survivor:<ID>)")
    tasks += _corridor_legs([_centre(layout, c) for c in ar.bfs_cells(graph, cur, entry)] + [gap, pad])
    tasks.append(Task("land"))
    info = {"pad": pad, "gap": gap, "excursions": eid if route == "rooms" else 1}
    return tasks, info


def route_length(tasks: Sequence[Task], start) -> float:
    total, p = 0.0, start
    for t in tasks:
        if t.kind == "translate":
            total += math.hypot(t.target[0] - p[0], t.target[1] - p[1])
            p = t.target
    return total


class Fence:
    """Allowed area: the arena interior (FENCE_MARGIN_M from the outer walls) or the pad->entry corridor."""

    def __init__(self, layout: dict):
        g = layout["grid"]
        self.size = g["cells"] * g["pitch_m"] + g["wall_m"]
        self.pad = layout["launch_pad"]["center"]
        self.gap = layout["entry"]["gap_center"]

    def inside(self, x: float, y: float) -> bool:
        m = FENCE_MARGIN_M
        if m - 1e-9 <= x <= self.size - m + 1e-9 and m - 1e-9 <= y <= self.size - m + 1e-9:
            return True
        return abs(x - self.gap[0]) <= 0.12 and self.pad[1] - 0.35 <= y <= m + 0.3


# ------------------------------------------------------------- LiDAR guard ---
def scan_min(ranges, angle_min: float, angle_inc: float, range_min: float, direction: Optional[float] = None,
             sector: float = AHEAD_SECTOR) -> float:
    """Smallest valid range (ROS LaserScan: angle 0 = forward, CCW), optionally within +-sector of `direction`."""
    best = math.inf
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min:
            continue
        if direction is not None and abs(wrap(angle_min + i * angle_inc - direction)) > sector:
            continue
        best = min(best, r)
    return best


# ------------------------------------------------------------- executor -----
class RouteExecutor:
    """Pure state machine: (time, PX4 position in the arena frame, heading, LiDAR) -> setpoint.
    No ROS here, so it can be tested with a simulated vehicle."""

    def __init__(self, tasks: List[Task], fence: Fence, start_xy, alt: float = ALT_DEFAULT_M,
                 speed: float = SPEED_DEFAULT, start_yaw: float = math.pi / 2):
        if not 0.2 <= alt <= ALT_MAX_M:
            raise ValueError(f"altitude {alt} m outside 0.2..{ALT_MAX_M} m")
        if not 0.05 <= speed <= SPEED_MAX:
            raise ValueError(f"speed {speed} m/s outside 0.05..{SPEED_MAX} m/s")
        self.tasks = list(tasks)
        self.fence = fence
        self.alt, self.speed = alt, speed
        self.sp = [float(start_xy[0]), float(start_xy[1])]
        self.sp_alt = 0.0
        self.sp_yaw = start_yaw
        self.phase = "TAKEOFF"
        self.i = 0
        self.sub = None              # translate: align | move ; turn: turn ; dwell: t_end
        self.leg_start = None
        self.leg_s = 0.0
        self.turn_target = None
        self.t_last = None
        self.stable_since = None
        self.events: List[str] = []
        self.abort_reason: Optional[str] = None
        self.blocked: List[int] = []
        self.deadline: Optional[float] = None
        self.vel = (0.0, 0.0, 0.0)          # feed-forward velocity (arena frame) sent with the setpoint
        self._prog_ref = None               # (x, y, heading) when progress was last seen
        self._hdg = []                      # (time, heading) of the last 3 s
        self._pos = []                      # (time, x, y) of the last 1 s
        self._wait_since = None             # waiting to settle at a turn point since

    # -- helpers --
    def _event(self, text):
        self.events.append(text)

    def _yaw_ramp(self, target, dt):
        err = wrap(target - self.sp_yaw)
        step = YAW_RATE * dt
        self.sp_yaw = target if abs(err) <= step else wrap(self.sp_yaw + math.copysign(step, err))
        return abs(wrap(target - self.sp_yaw)) < 1e-9

    def _settled(self, now, pos) -> bool:
        """On the turn point and not drifting? Checked all through a turn on the spot: never yaw while overshot
        towards a wall (the yaw setpoint holds until the drone is back)."""
        old = [p for p in self._pos if now - p[0] >= 0.5]
        still = bool(old) and math.hypot(pos[0] - old[-1][1], pos[1] - old[-1][2]) < 0.03
        if math.hypot(pos[0] - self.sp[0], pos[1] - self.sp[1]) < SETTLE_M and still:
            self._wait_since = None
            return True
        self._wait_since = self._wait_since or now
        if now - self._wait_since > SETTLE_MAX_S:
            self._abort(f"could not settle at the turn point in {SETTLE_MAX_S:.0f} s")
        return False

    def _heading_steady(self, now) -> bool:
        h = [x for x in self._hdg if now - x[0] <= SCAN_SETTLE_S]
        if not self._hdg or now - self._hdg[0][0] < SCAN_SETTLE_S or not h:
            return False
        cur = self._hdg[-1][1]
        return all(abs(wrap(a - cur)) < math.radians(3.0) for _, a in h)

    def _abort(self, reason):
        self.phase, self.abort_reason = "ABORT", reason
        self._event(f"ABORT: {reason}")

    def _cut_excursion(self, task: Task, clear_all_round: bool):
        """Blocked on probe leg `task`: drop the rest of the way in; look from here if there is room;
        come back to the excursion's previous point (on the same line)."""
        keep = []
        for t in self.tasks[self.i + 1:]:
            if t.exc == task.exc and t.kind == "translate" and not t.ret:
                continue                                # deeper into the room: skipped
            if t.exc == task.exc and t.look and not clear_all_round:
                continue                                # not enough room to turn here
            if t.exc == task.exc and t.ret and t.idx >= task.idx:
                continue                                # return points we never passed
            keep.append(t)
        self.tasks = self.tasks[: self.i + 1] + keep
        self.blocked.append(task.exc)

    # -- main step --
    def step(self, now: float, pos, alt_now: float, heading: float, scan=None) -> dict:
        """pos/alt/heading: PX4 estimate in the arena frame. scan: callable(direction or None) -> min range."""
        dt = 0.0 if self.t_last is None else max(0.0, min(0.2, now - self.t_last))
        self.t_last = now
        self.vel = (0.0, 0.0, 0.0)
        self._hdg = [x for x in self._hdg if now - x[0] <= 3.0] + [(now, heading)]
        self._pos = [x for x in self._pos if now - x[0] <= 1.0] + [(now, pos[0], pos[1])]
        if self.phase in ("ABORT", "LAND", "DONE"):
            return self._out()
        if scan is not None and scan(None) < EMERGENCY_M and self.phase != "TAKEOFF":
            self._abort(f"obstacle {scan(None):.2f} m from the LiDAR")
            return self._out()
        err = math.hypot(pos[0] - self.sp[0], pos[1] - self.sp[1])
        if err > TRACK_ABORT_M:
            self._abort(f"tracking error {err:.2f} m")
            return self._out()

        if self.phase == "TAKEOFF":
            if self.sp_alt < self.alt:
                self.vel = (0.0, 0.0, CLIMB_RATE)
            self.sp_alt = min(self.alt, self.sp_alt + CLIMB_RATE * dt)
            if abs(alt_now - self.alt) < 0.08 and self.sp_alt >= self.alt:
                self.stable_since = self.stable_since or now
                if now - self.stable_since >= 1.0:
                    self.phase = "ROUTE"
                    self._event(f"airborne at {alt_now:.2f} m")
            else:
                self.stable_since = None
            return self._out()

        if self.deadline is not None:
            ref = self._prog_ref
            if ref is None or math.hypot(pos[0] - ref[0], pos[1] - ref[1]) > 0.05 or \
                    abs(wrap(heading - ref[2])) > math.radians(3):
                self._prog_ref = (pos[0], pos[1], heading)          # moving or turning: reset the watchdog
                self.deadline = now + NO_PROGRESS_S
            elif now > self.deadline:
                self._abort(f"no progress on task {self.i} ({self.tasks[self.i].kind}) for {NO_PROGRESS_S:.0f} s: "
                            "PX4 not following")
                return self._out()
        while self.i < len(self.tasks):
            t = self.tasks[self.i]
            if t.kind == "land":
                self.phase = "LAND"
                self._event("route finished: landing")
                return self._out()
            if t.kind == "dwell":
                if self.sub is None:
                    self.sub = now + t.seconds
                if now < self.sub:
                    return self._out()
                self._next()
                continue
            if t.kind == "turn":
                if self.sub is None:
                    if not self._settled(now, pos):
                        return self._out()
                    if t.face is not None:
                        self.turn_target = math.atan2(t.face[1] - pos[1], t.face[0] - pos[0])
                    elif t.look:
                        base = self._look_base(t)
                        self.turn_target = wrap(base + t.yaw)
                    else:
                        self.turn_target = t.yaw
                    if scan is not None and scan(None) < TURN_CLEAR_M:
                        self._event(f"no room to turn ({scan(None):.2f} m): look skipped")
                        self._skip_look(t)
                        continue
                    self.sub = "turn"
                    self.deadline, self._prog_ref = now + NO_PROGRESS_S, (pos[0], pos[1], heading)
                if not self._settled(now, pos):
                    return self._out()                          # drifted off the point: hold the yaw
                if self._yaw_ramp(self.turn_target, dt) and abs(wrap(heading - self.turn_target)) < YAW_TOL:
                    self._next()
                    continue
                return self._out()
            if t.kind == "translate":
                tx, ty = t.target
                if not self.fence.inside(tx, ty):
                    self._abort(f"route point ({tx:.2f}, {ty:.2f}) outside the fence")
                    return self._out()
                if self.sub is None:
                    self.leg_start = (self.sp[0], self.sp[1])
                    self.leg_s = 0.0
                    d = math.hypot(tx - self.sp[0], ty - self.sp[1])
                    if d < 1e-6:
                        self._next()
                        continue
                    self.leg_dir = math.atan2(ty - self.sp[1], tx - self.sp[0])
                    # out of a room: back out facing the room (no turn in a doorway); else face the way
                    self.leg_yaw = self.sp_yaw if t.ret else self.leg_dir
                    self.sub = "align"
                    self._turned = False
                    self.deadline, self._prog_ref = now + NO_PROGRESS_S, (pos[0], pos[1], heading)
                if self.sub == "align":
                    turning = abs(wrap(self.leg_yaw - self.sp_yaw)) > 1e-9 or abs(wrap(heading - self.leg_yaw)) >= YAW_TOL
                    if turning and not self._settled(now, pos):
                        return self._out()                      # on the corner point, not drifting: then yaw
                    self._turned = self._turned or turning
                    if self._yaw_ramp(self.leg_yaw, dt) and abs(wrap(heading - self.leg_yaw)) < YAW_TOL:
                        self.sub = "settle" if self._turned else "move"
                    return self._out()
                if self.sub == "settle":                        # heading steady, so the LiDAR is current
                    if not self._heading_steady(now):
                        return self._out()
                    self.sub = "move"
                # move
                sx, sy = self.leg_start
                L = math.hypot(tx - sx, ty - sy)
                ux, uy = (tx - sx) / L, (ty - sy) / L
                cross = abs((pos[0] - sx) * uy - (pos[1] - sy) * ux)
                if cross > CROSS_TRACK_ABORT_M:
                    self._abort(f"{cross:.2f} m off the centreline")
                    return self._out()
                ahead = scan(wrap(self.leg_dir - heading)) if scan is not None else math.inf
                if ahead < STOP_AHEAD_M and self.leg_s < L - ARRIVE_M:
                    if not t.probe:
                        self._abort(f"unexpected obstacle {ahead:.2f} m ahead in a corridor")
                        return self._out()
                    # stop where the drone is (projected on the line) and finish the leg here
                    along = max(0.0, min(L, (pos[0] - sx) * ux + (pos[1] - sy) * uy))
                    self.sp = [sx + along * ux, sy + along * uy]
                    self._event(f"probe stopped {ahead:.2f} m before an obstacle")
                    self._cut_excursion(t, clear_all_round=(scan(None) >= TURN_CLEAR_M))
                    self._next()
                    return self._out()
                if err < TRACK_PAUSE_M and self.leg_s < L:
                    self.leg_s = min(L, self.leg_s + self.speed * dt)
                    if self.leg_s < L:
                        self.vel = (self.speed * ux, self.speed * uy, 0.0)
                self.sp = [sx + self.leg_s * ux, sy + self.leg_s * uy]
                if self.leg_s >= L and math.hypot(pos[0] - tx, pos[1] - ty) < ARRIVE_M:
                    self.sp = [tx, ty]
                    self._next()
                    continue
                return self._out()
            raise ValueError(t.kind)
        self.phase = "LAND"
        return self._out()

    def _look_base(self, t: Task) -> float:
        """Heading on arrival for a look-around: remembered at the first look turn of the excursion."""
        if getattr(self, "_look_exc", None) != t.exc:
            self._look_exc, self._look_yaw0 = t.exc, self.sp_yaw
        return self._look_yaw0

    def _skip_look(self, t: Task):
        self.tasks = self.tasks[: self.i] + [x for x in self.tasks[self.i:] if not (x.exc == t.exc and x.look)]
        self.sub = None

    def _next(self):
        self.i += 1
        self.sub = None
        self.deadline = None
        self._prog_ref = None

    def _out(self) -> dict:
        x, y = self.sp
        if self.phase not in ("ABORT",) and not self.fence.inside(x, y):
            self._abort(f"setpoint ({x:.2f}, {y:.2f}) outside the fence")
        return {"phase": self.phase, "x": x, "y": y, "alt": self.sp_alt, "yaw": self.sp_yaw,
                "vx": self.vel[0], "vy": self.vel[1], "vz": self.vel[2], "task": self.i, "abort": self.abort_reason}


# ------------------------------------------------------------- ROS node -----
PX4_INPUTS = ("offboard_control_mode", "trajectory_setpoint", "vehicle_command")
ACK_RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED",
              5: "IN_PROGRESS", 6: "CANCELLED"}
CMD_NAME = {176: "SET MODE OFFBOARD", 400: "ARM/DISARM", 21: "LAND"}


def pick_px4_topic(names_types, direction: str, base: str):
    """PX4's real topic for `base`: '/fmu/<direction>/<base>' or its versioned form '<...>_vN' (highest N).
    PX4 >= 1.16 adds _vN to versioned messages (seen 2026-11-09: /fmu/out/vehicle_status_v4)."""
    import re
    pat = re.compile(rf"^/fmu/{direction}/{re.escape(base)}(?:_v(\d+))?$")
    best = None
    for name, types in names_types:
        m = pat.match(name)
        if m:
            ver = int(m.group(1) or 0)
            if best is None or ver > best[0]:
                best = (ver, name, types[0] if types else None)
    return None if best is None else (best[1], best[2])


def gazebo_world_running(world: str, topics_text: Optional[str] = None) -> bool:
    if topics_text is None:
        try:
            topics_text = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=10).stdout
        except (OSError, subprocess.TimeoutExpired):
            return False
    return f"/world/{world}/clock" in topics_text.split()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--route", default="rooms", help="rooms | survivor:<ID>")
    ap.add_argument("--alt", type=float, default=ALT_DEFAULT_M, help="m above take-off (0.64 = LiDAR plane 1.0 m)")
    ap.add_argument("--speed", type=float, default=SPEED_DEFAULT)
    ap.add_argument("--dry-run", action="store_true", help="print the route and exit (no ROS, no PX4)")
    args = ap.parse_args(argv)

    world = f"nidar_airmouse_arena_s{args.seed}"
    layout = json.loads((REPO / "gazebo" / "layouts" / f"{world}.json").read_text())
    tasks, info = build_route(layout, args.route)
    fence = Fence(layout)
    bad = [t.target for t in tasks if t.kind == "translate" and not fence.inside(*t.target)]
    length = route_length(tasks, info["pad"])
    turns = sum(1 for t in tasks if t.kind == "turn")
    print(f"[PX4FLY] {world}: route {args.route}, {length:.1f} m of straight legs, {turns} look turns, "
          f"alt {args.alt:.2f} m, {args.speed:.2f} m/s, ~{(length / args.speed + 6 * turns) / 60:.1f} min", flush=True)
    if bad:
        print(f"[PX4FLY] REFUSED: route points outside the fence: {bad[:3]}", flush=True)
        return 2
    if args.dry_run:
        for k, t in enumerate(tasks):
            print(f"  {k:3d} {t.kind:9s} {t.target or ''} {'probe' if t.probe else ''} "
                  f"{'return' if t.ret else ''} {'look' if t.look else ''}")
        return 0
    if not gazebo_world_running(world):
        print(f"[PX4FLY] REFUSED: Gazebo world {world} is not running on this machine. "
              "This script only flies the simulated drone (start run_px4.sh first).", flush=True)
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import LaserScan
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
    NAV_OFFBOARD, ARMED = 14, 2

    class Flyer(Node):
        def __init__(self):
            super().__init__("nidar_px4_route_flyer")
            self.pubs = {}                                           # base -> (publisher, message class, topic)
            self.acks = []
            self.create_subscription(LaserScan, "/nidar/scan", self.on_scan, qos_profile_sensor_data)
            self.lp = self.status = self.scan = None
            self.scan_t = 0.0
            self.gap_paused, self.lidar_back_t = False, None
            self.subscribed = set()
            self.state = "WAIT_PX4"
            self.t_state = time.time()
            self.t_start = time.time()
            self.frame = ArenaNed(*info["pad"])
            self.exec = None
            self.last_cmd = 0.0
            self.land_sent = 0
            self.done = False
            self.n_events = 0
            self.last_out = None
            self.in_gap = False
            self.max_gap = 0.0
            self.n_gaps = 0
            out = REPO / "logs"
            out.mkdir(exist_ok=True)
            self.log_path = out / time.strftime("px4_flight_%Y%m%d_%H%M%S.csv")
            self.log_f = open(self.log_path, "w", newline="")
            self.log = csv.writer(self.log_f)
            self.log.writerow(["wall_time", "x_m", "y_m", "alt_m", "yaw_rad", "phase", "task"])
            self.create_timer(0.05, self.tick)
            self.create_timer(1.0, self.discover)

        # -- inputs --
        def discover(self):
            graph = self.get_topic_names_and_types()
            defaults = {"offboard_control_mode": OffboardControlMode, "trajectory_setpoint": TrajectorySetpoint,
                        "vehicle_command": VehicleCommand}
            for base in PX4_INPUTS:
                if base in self.pubs:
                    continue
                found = pick_px4_topic(graph, "in", base)
                waited = time.time() - self.t_start > 10
                if found is None and not waited:
                    continue
                name, tname = found if found else (f"/fmu/in/{base}", None)
                cls = defaults[base]
                if tname:
                    try:
                        cls = get_message(tname)
                    except (AttributeError, ModuleNotFoundError, ValueError):
                        pass
                self.pubs[base] = (self.create_publisher(cls, name, 10), cls, name)
                if found:                                            # one severity per call site (rclpy rule)
                    self.get_logger().info(f"PX4 input {name}")
                else:
                    self.get_logger().warning(f"PX4 input {name} (not seen in the ROS graph: guessed)")
            for name, types in graph:
                if name in self.subscribed:
                    continue
                for prefix, cb in (("/fmu/out/vehicle_local_position", self.on_lp),
                                   ("/fmu/out/vehicle_status", self.on_status),
                                   ("/fmu/out/vehicle_command_ack", self.on_ack),
                                   ("/fmu/out/failsafe_flags", self.on_flags)):
                    if name.startswith(prefix) and "groundtruth" not in name:
                        try:
                            cls = get_message(types[0])
                        except (AttributeError, ModuleNotFoundError, ValueError):
                            continue
                        self.create_subscription(cls, name, cb, px4_qos)
                        self.subscribed.add(name)
                        self.get_logger().info(f"subscribed {name}")

        def on_lp(self, m):
            self.lp = (m, time.time())

        def on_status(self, m):
            self.status = (m, time.time())

        def on_flags(self, m):
            self.flags = m

        def flags_set(self):
            """Names of PX4 failsafe/arming flags that are True (why PX4 may refuse to arm)."""
            m = getattr(self, "flags", None)
            if m is None:
                return "not published"
            on = [k for k, t in m.get_fields_and_field_types().items()
                  if t == "boolean" and getattr(m, k, False) and not k.startswith("mode_req")]
            return ", ".join(on) or "none"

        def on_ack(self, m):
            txt = f"PX4 reply to {CMD_NAME.get(m.command, m.command)}: {ACK_RESULT.get(m.result, m.result)}"
            if not self.acks or self.acks[-1] != txt:
                self.acks.append(txt)
                if m.result == 0:                                    # one severity per call site (rclpy rule)
                    self.get_logger().info(txt)
                else:
                    self.get_logger().warning(txt)

        def on_scan(self, m):
            self.scan = m
            self.scan_t = time.time()

        def scan_fn(self):
            s = self.scan
            if s is None:
                return None
            return lambda direction: scan_min(s.ranges, s.angle_min, s.angle_increment, max(s.range_min, 0.16),
                                              direction)

        # -- outputs --
        def us(self):
            return int(self.get_clock().now().nanoseconds / 1000)

        def ready_to_publish(self):
            return all(b in self.pubs for b in PX4_INPUTS)

        def heartbeat(self):
            pub, cls, _ = self.pubs["offboard_control_mode"]
            m = cls()
            m.timestamp = self.us()
            m.position, m.velocity, m.acceleration, m.attitude, m.body_rate = True, False, False, False, False
            pub.publish(m)

        def setpoint(self, x, y, alt, yaw, vx=0.0, vy=0.0, vz=0.0):
            n, e, d = self.frame.to_ned(x, y, alt)
            pub, cls, _ = self.pubs["trajectory_setpoint"]
            m = cls()
            m.timestamp = self.us()
            nan = float("nan")
            m.position = [float(n), float(e), float(d)]
            m.velocity = [float(vy), float(vx), float(-vz)]           # feed-forward, NED (north = arena +y)
            m.acceleration = [nan, nan, nan]
            m.jerk = [nan, nan, nan]
            m.yaw = float(ArenaNed.yaw_to_ned(yaw))
            m.yawspeed = nan
            pub.publish(m)

        def command(self, cmd, p1=0.0, p2=0.0):
            pub, cls, _ = self.pubs["vehicle_command"]
            m = cls()
            m.timestamp = self.us()
            m.command, m.param1, m.param2 = int(cmd), float(p1), float(p2)
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external = True
            pub.publish(m)

        def land(self, reason):
            self.get_logger().warn(f"LAND: {reason}")
            for _ in range(3):
                self.command(21)                                     # VEHICLE_CMD_NAV_LAND
            self.state, self.t_state = "LANDING", time.time()

        # -- state --
        def armed(self):
            return self.status is not None and self.status[0].arming_state == ARMED

        def offboard(self):
            return self.status is not None and self.status[0].nav_state == NAV_OFFBOARD

        def arena_pose(self):
            m = self.lp[0]
            x, y, alt = self.frame.to_arena(m.x, m.y, m.z)
            return (x, y), alt, ArenaNed.yaw_to_enu(m.heading)

        def tick(self):
            now = time.time()
            if self.state == "WAIT_PX4":
                if self.lp and self.status and self.lp[0].xy_valid and self.lp[0].z_valid and self.ready_to_publish():
                    m = self.lp[0]
                    self.frame.n0, self.frame.e0, self.frame.d0 = m.x, m.y, m.z
                    pos, alt, hdg = self.arena_pose()
                    self.exec = RouteExecutor(tasks, fence, info["pad"], args.alt, args.speed, start_yaw=hdg)
                    self.get_logger().info(f"PX4 ready; take-off point = launch pad, heading {math.degrees(hdg):.0f} deg")
                    self.state, self.t_state = "PRESTREAM", now
                elif now - self.t_state > 30:
                    self.get_logger().error("no PX4 position after 30 s (MicroXRCEAgent + run_px4.sh running?)")
                    self.done = True
                return
            if self.state in ("PRESTREAM", "ARMING", "FLY"):
                age = now - self.lp[1] if self.lp else math.inf
                if age > POSE_GAP_HOLD_S:
                    if self.state != "FLY":
                        return
                    self.max_gap = max(self.max_gap, age)
                    scan_age = now - self.scan_t
                    if scan_age > 1.0:                                  # the LiDAR stopped too: Gazebo paused
                        self.gap_paused, self.lidar_back_t = True, None
                    elif self.gap_paused and self.lidar_back_t is None:
                        self.lidar_back_t = now
                    back = None if self.lidar_back_t is None else now - self.lidar_back_t
                    if gap_action(age, scan_age, self.gap_paused, back) == "land":
                        self.land(f"no PX4 position for {age:.1f} s"
                                  + (" (LiDAR still arriving: PX4 data lost)" if now - self.scan_t <= POSE_GAP_HOLD_S
                                     else " (whole simulation stopped)"))
                        return
                    if not self.in_gap:
                        self.in_gap = True
                        self.n_gaps += 1
                    if self.last_out is not None:               # keep PX4 in OFFBOARD, holding
                        self.heartbeat()
                        self.setpoint(*self.last_out)
                    return
                self.gap_paused, self.lidar_back_t = False, None
                if self.in_gap:
                    self.in_gap = False
                    self.get_logger().info(f"PX4 position back (gap {self.max_gap:.2f} s max so far, "
                                           f"{self.n_gaps} gaps)")
                pos, alt, hdg = self.arena_pose()
                self.log.writerow([f"{now:.3f}", f"{pos[0]:.4f}", f"{pos[1]:.4f}", f"{alt:.3f}", f"{hdg:.4f}",
                                   self.exec.phase, self.exec.i])
                if now - getattr(self, "last_flush", 0.0) > 1.0:
                    self.last_flush = now
                    self.log_f.flush()
            if self.state == "PRESTREAM":
                self.heartbeat()
                self.setpoint(*info["pad"], 0.0, self.exec.sp_yaw)
                if now - self.t_state > 1.5:
                    self.state, self.t_state = "ARMING", now
                return
            if self.state == "ARMING":
                self.heartbeat()
                self.setpoint(*info["pad"], 0.0, self.exec.sp_yaw)
                if self.armed() and self.offboard():
                    self.get_logger().info("ARMED in OFFBOARD: taking off")
                    self.state, self.t_state = "FLY", now
                elif now - self.t_state > 15:
                    st = self.status[0] if self.status else None
                    self.get_logger().error(
                        "PX4 did not arm / enter OFFBOARD in 15 s. "
                        f"nav_state={getattr(st, 'nav_state', '?')} arming_state={getattr(st, 'arming_state', '?')}; "
                        f"inputs: {', '.join(v[2] for v in self.pubs.values())}; "
                        f"PX4 replies: {self.acks[-4:] or 'none (commands not reaching PX4?)'}; "
                        f"PX4 failsafe flags set: {self.flags_set()}. "
                        "Also look for 'Arming denied' / 'Preflight' lines at pxh>.")
                    self.command(400, 0.0)                           # disarm (still on the ground)
                    self.done = True
                elif now - self.last_cmd > 1.0:
                    self.last_cmd = now
                    self.command(176, 1.0, 6.0)                      # DO_SET_MODE: custom, OFFBOARD
                    self.command(400, 1.0)                           # ARM
                return
            if self.state == "FLY":
                if not self.offboard():
                    self.get_logger().warn("PX4 left OFFBOARD (pilot or failsafe): stopping, PX4 is in charge")
                    self.done = True
                    return
                pos, alt, hdg = self.arena_pose()
                out = self.exec.step(now, pos, alt, hdg, self.scan_fn())
                for e in self.exec.events[self.n_events:]:
                    self.get_logger().info(e)
                self.n_events = len(self.exec.events)
                if out["phase"] == "ABORT":
                    self.land(out["abort"])
                    return
                if out["phase"] == "LAND":
                    self.land("route finished")
                    return
                self.heartbeat()
                self.last_out = (out["x"], out["y"], out["alt"], out["yaw"])     # held in gaps with v = 0
                self.setpoint(*self.last_out, out["vx"], out["vy"], out["vz"])
                return
            if self.state == "LANDING":
                if not self.armed():
                    self.get_logger().info(f"landed and disarmed. PX4 position gaps > {POSE_GAP_HOLD_S} s: "
                                           f"{self.n_gaps} (max {self.max_gap:.2f} s). Log: {self.log_path}")
                    self.done = True
                elif now - self.t_state > 120:
                    self.get_logger().error("still armed 120 s after LAND: check the pxh> console")
                    self.done = True
                elif now - self.last_cmd > 2.0 and not self.offboard():
                    self.last_cmd = now                              # PX4 is landing on its own
                elif now - self.last_cmd > 2.0:
                    self.last_cmd = now
                    self.command(21)

    node = Flyer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        if node.armed():
            node.land("Ctrl+C")
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
        print("\n[PX4FLY] stopped (LAND sent if the drone was armed)", flush=True)
    finally:
        node.log_f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
