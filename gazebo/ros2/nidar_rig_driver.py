#!/usr/bin/env python3
"""
NIDAR sensor-rig driver -- WSL side (2026-11-02). NO PX4, no flight control.

Moves the nidar_rig model (gazebo/models/nidar_rig) along a route computed
from the arena's ground-truth layout, so camera + LiDAR data from a KNOWN
path reach the pipeline. Ground truth (odometry) is logged to CSV for
checking V18 ranges and SLAM poses afterwards.

  /nidar/rig/odom (nav_msgs/Odometry)  --> RouteFollower --> /nidar/rig/cmd_vel (geometry_msgs/Twist, body frame)

Run in WSL (ROS 2 sourced, rig world + ros_gz_bridge running):
    python3 gazebo/ros2/nidar_rig_driver.py --seed 1 --route survivor:S2
    python3 gazebo/ros2/nidar_rig_driver.py --seed 1 --route rooms --speed 0.4
    python3 gazebo/ros2/nidar_rig_driver.py --seed 1 --route survey

Routes:  survey (every cell, ~410 m) | rooms (into each room, look around 360 deg) | survivor:<ID> (stop facing it)
Ctrl+C or the end of the route publishes a zero velocity.

AUTONOMOUS mode (2026-11-09): --brain PI_HOST[:5030] -- no route, no layout knowledge. The rig flies
the plans of the explorer service on the Pi (src/mission/nidar_explorer_service.py, NIDAR_NAV_V1):
    python3 gazebo/ros2/nidar_rig_driver.py --brain 10.218.70.147
Plans arrive in the SLAM frame and are flown in it: odometry is mapped to SLAM's frame by an alignment
taken from the brain's SLAM pose while the rig stands still (start it on the pad), then kept up to date
from every brain pose (2 Hz, only while the rig is hardly turning). Guards: no brain / no plan /
odometry stale -> stand still; LiDAR obstacle < 0.30 m in the direction of motion -> stop + BLOCKED.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
import queue
import socket
import threading
from typing import Callable, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from src.simulation import arena_raycast as ar  # noqa: E402


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def build_route(layout: dict, route: str, step_m: float = 0.25) -> List[Tuple[float, float, float]]:
    if route == "survey":
        return ar.survey_path(layout, step_m)
    if route == "rooms":
        return ar.rooms_path(layout, step_m)
    if route.startswith("survivor:"):
        return ar.survivor_view_path(layout, route.split(":", 1)[1], step_m=step_m)
    raise ValueError(f"unknown route {route!r} (survey | rooms | survivor:<ID>)")


class RouteFollower:
    """
    Carrot-following on a dense path for a holonomic body.
    update(x, y, yaw) -> (vx_body, vy_body, yaw_rate, done)
    """

    def __init__(self, path: Sequence[Tuple[float, float, float]], speed: float = 0.4,
                 reach_m: float = 0.15, yaw_gain: float = 1.5, max_yaw_rate: float = 0.8,
                 slow_radius_m: float = 0.5):
        self.path = list(path)
        self.speed = float(speed)
        self.reach = float(reach_m)
        self.k_yaw = float(yaw_gain)
        self.max_wz = float(max_yaw_rate)
        self.slow_r = float(slow_radius_m)
        self.idx = 0

    def update(self, x: float, y: float, yaw: float) -> Tuple[float, float, float, bool]:
        last = len(self.path) - 1
        while self.idx < last:
            px, py, pyaw = self.path[self.idx]
            if math.hypot(px - x, py - y) >= self.reach:
                break
            nx, ny, _ = self.path[self.idx + 1]
            turn_in_place = math.hypot(nx - px, ny - py) < 1e-6 or (
                self.idx > 0 and math.hypot(px - self.path[self.idx - 1][0], py - self.path[self.idx - 1][1]) < 1e-6)
            if turn_in_place and abs(wrap(pyaw - yaw)) > math.radians(8.0):
                break                                   # turn-in-place step: wait for the heading
            self.idx += 1
        tx, ty, tyaw = self.path[self.idx]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        yaw_err = wrap(tyaw - yaw)
        if self.idx == last and dist < 0.05 and abs(yaw_err) < math.radians(3.0):
            return 0.0, 0.0, 0.0, True
        if self.idx < last:
            v = self.speed * min(1.0, dist / self.reach)    # no overshoot around turn-in-place points
        else:
            v = min(self.speed, self.speed * dist / self.slow_r)
        if dist > 1e-6:
            vxw, vyw = v * dx / dist, v * dy / dist
        else:
            vxw = vyw = 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        vx_b = c * vxw + s * vyw
        vy_b = -s * vxw + c * vyw
        wz = max(-self.max_wz, min(self.max_wz, self.k_yaw * yaw_err))
        return vx_b, vy_b, wz, False


class OdomFrame:
    """
    Maps the odometry pose into the arena frame. If the first odometry pose
    is ~0 while the rig was spawned elsewhere, odometry is taken as relative
    to the spawn pose (covers both OdometryPublisher conventions).
    """

    def __init__(self, spawn: Tuple[float, float, float]):
        self.spawn = spawn
        self.relative: Optional[bool] = None

    def to_arena(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        sx, sy, syaw = self.spawn
        if self.relative is None:
            self.relative = math.hypot(x, y) < 0.05 and math.hypot(sx, sy) > 0.5
        if not self.relative:
            return x, y, yaw
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * x - s * y, sy + s * x + c * y, wrap(syaw + yaw)


# ------------------------------------------------------------ brain-follow mode --
NAV_PROTOCOL = "NIDAR_NAV_V1"
NAV_PORT = 5030
STOP_AHEAD_M = 0.30          # LiDAR range in the direction of motion that stops the rig (centre to surface)
AHEAD_SECTOR = math.radians(35.0)
ALIGN_STILL_M = 0.02         # the rig must not have moved more than this over ALIGN_STILL_S to align
ALIGN_STILL_S = 1.0
MIN_LEG_M = 0.10             # waypoints closer than this to the previous point are skipped (no spurious turns)
# Alignment is kept up to date from every brain pose (2026-11-09 run: a fixed alignment taken at start-up carried
# SLAM's early 0.7 deg yaw error; 13 m from the pad that put the rig 0.16 m off its plan -- closest wall 0.22 m)
REALIGN_GAIN = 0.3           # blend factor per pose message (2 Hz -> time constant ~1.5 s)
REALIGN_MAX_YAW_RATE = math.radians(6.0)   # use a pose only if the rig was hardly turning around the scan time
REALIGN_REJECT_M = 0.5       # bigger jumps than these are not blended in (SLAM jump / outlier)
REALIGN_REJECT_RAD = math.radians(5.0)
LINK_LATENCY_S = 0.02        # Pi -> WSL message delay (LAN)


def ahead_min(ranges, angle_min: float, angle_inc: float, range_min: float, direction: float,
              sector: float = AHEAD_SECTOR) -> float:
    """Smallest valid LiDAR range within +-sector of `direction` (body frame, ROS: 0 = forward, CCW)."""
    best = math.inf
    for k, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < max(range_min, 0.05):
            continue
        if abs(wrap(angle_min + k * angle_inc - direction)) <= sector:
            best = min(best, r)
    return best


class Align:
    """odometry = A o slam. First from one (odometry pose, SLAM pose) pair taken while standing still,
    then blended towards every new pair (BrainFollower.realign) so the rig flies in SLAM's frame."""

    def __init__(self, odom_pose, slam_pose):
        ox, oy, oyaw = odom_pose
        sx, sy, syaw = slam_pose
        self.yaw = wrap(oyaw - syaw)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.tx, self.ty = ox - (c * sx - s * sy), oy - (s * sx + c * sy)

    def blend(self, odom_pose, slam_pose, k: float) -> Tuple[float, float]:
        """Move a fraction k towards the alignment this pair implies; returns the (metres, rad) disagreement."""
        m = Align(odom_pose, slam_pose)
        dyaw = wrap(m.yaw - self.yaw)
        before = self.xy(slam_pose[0], slam_pose[1])
        dist = math.hypot(before[0] - odom_pose[0], before[1] - odom_pose[1])
        self.yaw = wrap(self.yaw + k * dyaw)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        sx, sy = slam_pose[0], slam_pose[1]
        tx_m, ty_m = odom_pose[0] - (c * sx - s * sy), odom_pose[1] - (s * sx + c * sy)
        self.tx += k * (tx_m - self.tx)
        self.ty += k * (ty_m - self.ty)
        return dist, dyaw

    def inv(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        """odometry pose -> SLAM pose."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.tx, y - self.ty
        return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - self.yaw)

    def xy(self, x: float, y: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.tx + c * x - s * y, self.ty + s * x + c * y

    def heading(self, yaw: float) -> float:
        return wrap(yaw + self.yaw)


def plan_to_path(start, waypoints_arena, look_yaws_arena, step_m: float = 0.25):
    """Dense (x, y, yaw) path for RouteFollower + the waypoint index each point leads to.
    Legs face their direction of travel (camera forward); the look is a series of turn-in-place points."""
    x, y, yaw = start
    path, wp_of = [(x, y, yaw)], [0]
    px, py, pyaw = x, y, yaw
    for k, (wx, wy) in enumerate(waypoints_arena):
        L = math.hypot(wx - px, wy - py)
        if L < MIN_LEG_M:
            continue
        h = math.atan2(wy - py, wx - px)
        n = max(1, int(math.ceil(L / step_m)))
        for q in range(1, n + 1):
            path.append((px + (wx - px) * q / n, py + (wy - py) * q / n, h))
            wp_of.append(k)
        px, py, pyaw = wx, wy, h
    if len(path) == 1:                                  # already there: one point to settle on
        path.append((px, py, pyaw))
        wp_of.append(max(len(waypoints_arena) - 1, 0))
    for a in look_yaws_arena:
        path.append((px, py, a))
        wp_of.append(max(len(waypoints_arena) - 1, 0))
    return path, wp_of


class BrainFollower:
    """Pure (no ROS, no sockets): NIDAR_NAV_V1 messages + rig pose + LiDAR -> body velocities + replies.
    Tested in tests/test_rig_brain_follow.py."""

    def __init__(self, speed: float = 0.4, stop_m: float = STOP_AHEAD_M):
        self.speed = float(speed)
        self.stop_m = float(stop_m)
        self.align: Optional[Align] = None
        self.pending: Optional[dict] = None         # plan waiting for alignment
        self.follower: Optional[RouteFollower] = None
        self.plan: Optional[dict] = None
        self.wp_of: List[int] = []
        self.state = "WAIT"                         # WAIT | FLYING | HOLD | COMPLETE
        self.reason = "waiting for the brain"
        self.outbox: List[dict] = []
        self.last_status = 0.0
        self.history: List[Tuple[float, float, float, float]] = []   # (t, x, y, yaw) odometry, last 3 s
        self.realigned = 0
        self.realign_rejected = 0
        self.lidar_wait = False
        self.events: List[str] = []

    def _stop(self, state: str, reason: str) -> None:
        self.follower, self.plan, self.state, self.reason = None, None, state, reason
        self.events.append(f"{state}: {reason}")

    def standing_still(self, now: float) -> bool:
        old = [h for h in self.history if now - h[0] >= ALIGN_STILL_S]
        if not old:
            return False
        t0, x0, y0 = old[-1][:3]
        return all(math.hypot(h[1] - x0, h[2] - y0) <= ALIGN_STILL_M for h in self.history if h[0] >= t0)

    def odom_at(self, ts: float):
        """Odometry pose at local time ts (interpolated), or None if outside the history or while turning."""
        h = self.history
        k = next((i for i in range(1, len(h)) if h[i][0] >= ts), None)
        if k is None or h[0][0] > ts:
            return None
        win = [q for q in h if abs(q[0] - ts) <= 0.3]
        for a, b in zip(win[:-1], win[1:]):
            if b[0] - a[0] > 1e-3 and abs(wrap(b[3] - a[3])) / (b[0] - a[0]) > REALIGN_MAX_YAW_RATE:
                return None                                # timing error would become a yaw error
        a, b = h[k - 1], h[k]
        f = 0.0 if b[0] - a[0] < 1e-6 else (ts - a[0]) / (b[0] - a[0])
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), wrap(a[3] + f * wrap(b[3] - a[3])))

    def realign(self, slam_pose, ts: float) -> None:
        o = self.odom_at(ts)
        if o is None:
            return
        m = Align(o, slam_pose)
        if abs(wrap(m.yaw - self.align.yaw)) > REALIGN_REJECT_RAD or \
                math.hypot(*(a - b for a, b in zip(self.align.xy(*slam_pose[:2]), o[:2]))) > REALIGN_REJECT_M:
            self.realign_rejected += 1
            if self.realign_rejected in (1, 10, 100):
                self.events.append(f"SLAM pose disagrees with odometry by > {REALIGN_REJECT_M} m / "
                                   f"{math.degrees(REALIGN_REJECT_RAD):.0f} deg -- not used ({self.realign_rejected}x)")
            return
        self.align.blend(o, slam_pose, REALIGN_GAIN)
        self.realigned += 1

    def on_message(self, msg: dict, pose, now: float) -> None:
        kind = msg.get("type")
        if kind == "pose" or (kind == "plan" and msg.get("pose")):
            sp = (msg["x"], msg["y"], msg["yaw"]) if kind == "pose" else tuple(msg["pose"])
            if self.align is None and pose is not None and self.standing_still(now):
                self.align = Align(pose, sp)
                self.events.append(f"aligned: SLAM origin = odom ({self.align.tx:.2f}, {self.align.ty:.2f}), "
                                   f"yaw {math.degrees(self.align.yaw):.1f} deg")
            elif self.align is not None and kind == "pose":
                self.realign(sp, now - float(msg.get("age") or 0.0) - LINK_LATENCY_S)
        if kind == "plan":
            self.pending = msg
        elif kind == "cancel":
            if self.plan is not None and msg.get("id") == self.plan.get("id"):
                self._stop("HOLD", f"plan {msg.get('id')} cancelled by the brain: {msg.get('reason')}")
            if self.pending is not None and msg.get("id") == self.pending.get("id"):
                self.pending = None
        elif kind == "hold":
            self.pending = None
            if self.state != "HOLD" or self.reason != f"brain: {msg.get('reason')}":
                self._stop("HOLD", f"brain: {msg.get('reason')}")

    def disconnected(self) -> None:
        self.pending = None
        if self.state != "COMPLETE":
            self._stop("WAIT", "brain link lost: standing still")

    def _start(self, msg: dict, pose_slam) -> None:
        # the path stays in the SLAM frame; the rig's pose is converted with the CURRENT alignment every cycle
        wps = [(float(x), float(y)) for x, y in msg.get("waypoints", [])]
        yaws = [float(a) for a in msg.get("look_yaws", [])]
        path, self.wp_of = plan_to_path(pose_slam, wps, yaws)
        self.follower = RouteFollower(path, speed=self.speed)
        self.plan, self.state = msg, "FLYING"
        self.reason = f"plan {msg['id']}: {msg.get('reason', '')}"
        self.events.append(f"plan {msg['id']} ({msg.get('end')}): {len(wps)} waypoints, "
                           f"{len(yaws)} look headings -- {msg.get('reason', '')}")

    def update(self, pose, now: float, ahead: Optional[Callable[[float], Optional[float]]] = None
               ) -> Tuple[float, float, float]:
        """pose = rig pose (odometry frame) or None if stale; ahead(direction_body) -> LiDAR range (m),
        or None when there is no fresh scan."""
        if pose is None:
            return 0.0, 0.0, 0.0
        self.history.append((now, pose[0], pose[1], pose[2]))
        self.history = [h for h in self.history if now - h[0] <= 3 * ALIGN_STILL_S]
        if self.align is None:
            return 0.0, 0.0, 0.0
        pose_s = self.align.inv(*pose)                      # where the rig is in SLAM's frame (the plan's frame)
        if self.pending is not None:
            self._start(self.pending, pose_s)
            self.pending = None
        if self.follower is None or self.plan is None:
            return 0.0, 0.0, 0.0
        pid = self.plan["id"]
        vx, vy, wz, done = self.follower.update(*pose_s)    # body-frame velocities: the same in both frames
        wp = self.wp_of[min(self.follower.idx, len(self.wp_of) - 1)]
        if done:
            end = self.plan.get("end")
            self.outbox.append({"type": "status", "id": pid, "state": "DONE", "wp": len(self.plan["waypoints"])})
            if end == "land":
                self._stop("COMPLETE", "mission complete: back at the take-off point "
                           "(rig: a real drone lands here)")
            else:
                self._stop("HOLD", f"plan {pid} done; waiting for the next plan")
            return 0.0, 0.0, 0.0
        v = math.hypot(vx, vy)
        if ahead is not None and v > 0.05:
            r = ahead(math.atan2(vy, vx))
            if r is None:                                 # no fresh LiDAR: wait (not a BLOCKED path)
                if not self.lidar_wait:
                    self.lidar_wait = True
                    self.events.append("no fresh LiDAR scan: standing still until it is back")
                return 0.0, 0.0, 0.0
            self.lidar_wait = False
            if r < self.stop_m:
                self.outbox.append({"type": "status", "id": pid, "state": "BLOCKED", "wp": wp})
                self._stop("HOLD", f"LiDAR: obstacle {r:.2f} m ahead -- stopped, plan {pid} BLOCKED")
                return 0.0, 0.0, 0.0
        if now - self.last_status >= 1.0:
            self.last_status = now
            self.outbox.append({"type": "status", "id": pid, "state": "FLYING", "wp": wp})
        return vx, vy, wz

    def drain(self) -> List[dict]:
        out, self.outbox = self.outbox, []
        return out


class NavClient:
    """TCP client to the explorer service; reconnects every 2 s. Newline JSON."""

    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.inbox: "queue.Queue[dict]" = queue.Queue()
        self.sock = None
        self.lock = threading.Lock()
        self.halt = threading.Event()
        self.lost = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.halt.is_set():
            try:
                s = socket.create_connection(self.addr, timeout=3.0)
            except OSError:
                time.sleep(2.0)
                continue
            s.settimeout(1.0)
            with self.lock:
                self.sock = s
            print(f"[RIG] brain connected: {self.addr[0]}:{self.addr[1]}", flush=True)
            buf = b""
            while not self.halt.is_set():
                try:
                    chunk = s.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        self.inbox.put(json.loads(line))
                    except ValueError:
                        pass
            with self.lock:
                self.sock = None
            try:
                s.close()
            except OSError:
                pass
            self.lost.set()
            print("[RIG] brain link lost; reconnecting ...", flush=True)
            time.sleep(1.0)

    @property
    def connected(self) -> bool:
        return self.sock is not None

    def send(self, msg: dict) -> None:
        data = (json.dumps(dict(msg, protocol=NAV_PROTOCOL)) + "\n").encode()
        with self.lock:
            if self.sock is not None:
                try:
                    self.sock.sendall(data)
                except OSError:
                    pass

    def drain(self) -> List[dict]:
        out = []
        while True:
            try:
                out.append(self.inbox.get_nowait())
            except queue.Empty:
                return out

    def close(self):
        self.halt.set()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1, help="arena seed (gazebo/layouts/nidar_airmouse_arena_s<seed>.json)")
    ap.add_argument("--layout", default=None, help="explicit layout JSON (overrides --seed)")
    ap.add_argument("--route", default="survivor:S2")
    ap.add_argument("--brain", default=None, metavar="HOST[:PORT]",
                    help="autonomous: fly the explorer service's plans (Pi, port 5030); --route is ignored")
    ap.add_argument("--speed", type=float, default=0.4, help="m/s (rig only; not a flight speed)")
    ap.add_argument("--truth-csv", default=None, help="default logs/rig_truth_<time>.csv")
    args = ap.parse_args(argv)

    layout_path = Path(args.layout) if args.layout else ar.default_layout(args.seed)
    brain = nav = None
    if args.brain:
        # the layout is used ONLY to express the ground-truth log in the arena frame (for scoring);
        # the brain gets nothing from it
        host, _, port = args.brain.partition(":")
        layout = json.loads(layout_path.read_text(encoding="utf-8")) if layout_path.exists() else None
        sp = layout["spawn_pose"] if layout else {"x": 0.0, "y": 0.0, "yaw": 0.0}
        brain = BrainFollower(speed=args.speed)
        nav = NavClient(host, int(port or NAV_PORT))
        follower = None
        path = []
        print(f"[RIG] AUTONOMOUS: following the explorer at {host}:{port or NAV_PORT} (no route, no map given)",
              flush=True)
    else:
        layout = json.loads(layout_path.read_text(encoding="utf-8"))
        path = build_route(layout, args.route)
        sp = layout["spawn_pose"]
        follower = RouteFollower(path, speed=args.speed)
        length = sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))
        print(f"[RIG] {layout_path.name}: route {args.route}, {length:.1f} m, ~{length / args.speed / 60:.1f} min",
              flush=True)
    frame = OdomFrame((sp["x"], sp["y"], sp["yaw"]))

    truth_path = Path(args.truth_csv) if args.truth_csv else REPO / "logs" / f"rig_truth_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    truth_path.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    class RigDriver(Node):
        def __init__(self):
            super().__init__("nidar_rig_driver")
            self.pub = self.create_publisher(Twist, "/nidar/rig/cmd_vel", 10)
            self.create_subscription(Odometry, "/nidar/rig/odom", self.on_odom, 10)
            self.scan = None
            self.scan_t = 0.0
            if brain is not None:
                self.create_subscription(LaserScan, "/nidar/scan", self.on_scan, qos_profile_sensor_data)
            self.pose = None
            self.last_odom = 0.0
            self.done = False
            self.csv_file = open(truth_path, "w", newline="")
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(["wall_time", "x_m", "y_m", "yaw_rad", "route_index"])
            self.create_timer(0.05, self.control)
            self.last_print = 0.0
            self.n_ev = 0

        def on_odom(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.pose = frame.to_arena(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            self.last_odom = time.time()
            idx = follower.idx if follower is not None else (brain.plan or {}).get("id", 0)
            self.csv.writerow([f"{self.last_odom:.3f}", f"{self.pose[0]:.4f}", f"{self.pose[1]:.4f}",
                               f"{self.pose[2]:.4f}", idx])

        def on_scan(self, msg):
            self.scan, self.scan_t = msg, time.time()

        def ahead(self, direction):
            m = self.scan
            if m is None or time.time() - self.scan_t > 0.5:
                return None                               # no fresh LiDAR: the follower stands still
            return ahead_min(m.ranges, m.angle_min, m.angle_increment, m.range_min, direction)

        def send(self, vx, vy, wz):
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
            self.pub.publish(t)

        def control(self):
            now = time.time()
            fresh = self.pose is not None and now - self.last_odom <= 0.5
            if brain is not None:
                self.control_brain(now, fresh)
                return
            if not fresh:
                self.send(0.0, 0.0, 0.0)                 # no fresh odometry: hold still
                if now - self.last_print > 2.0:
                    self.last_print = now
                    print("[RIG] waiting for /nidar/rig/odom (is the rig bridge running?)", flush=True)
                return
            vx, vy, wz, done = follower.update(*self.pose)
            self.send(vx, vy, wz)
            if done and not self.done:
                self.done = True
                print(f"[RIG] route finished at x={self.pose[0]:.2f} y={self.pose[1]:.2f} "
                      f"yaw={math.degrees(self.pose[2]):.0f} deg; holding (Ctrl+C to exit)", flush=True)
            if now - self.last_print > 2.0:
                self.last_print = now
                print(f"[RIG] {follower.idx}/{len(path) - 1}  x={self.pose[0]:.2f} y={self.pose[1]:.2f} "
                      f"yaw={math.degrees(self.pose[2]):.0f}", flush=True)

        def control_brain(self, now, fresh):
            if nav.lost.is_set():
                nav.lost.clear()
                brain.disconnected()
            pose = self.pose if fresh else None
            for msg in nav.drain():
                brain.on_message(msg, pose, now)
            vx, vy, wz = brain.update(pose, now, self.ahead)
            self.send(vx, vy, wz)
            for msg in brain.drain():
                nav.send(msg)
            for e in brain.events[self.n_ev:]:
                print(f"[RIG] {e}", flush=True)
            self.n_ev = len(brain.events)
            if now - self.last_print > 3.0:
                self.last_print = now
                where = f"x={self.pose[0]:.2f} y={self.pose[1]:.2f} yaw={math.degrees(self.pose[2]):.0f}" \
                    if self.pose else "no odometry yet (is run_rig.sh running?)"
                link = "brain UP" if nav.connected else "brain: connecting"
                al = "aligned" if brain.align else "not aligned (keep the rig still on the pad)"
                print(f"[RIG] {brain.state:8s} {link} | {al} | {where}", flush=True)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RigDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if nav is not None:
            nav.close()
        if rclpy.ok():
            for _ in range(3):
                node.send(0.0, 0.0, 0.0)
                time.sleep(0.05)
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[RIG] stopped; ground truth: {truth_path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
