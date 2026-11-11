#!/usr/bin/env python3
"""
NIDAR explorer service -- the autonomous "brain" on the Raspberry Pi (2026-11-09).

Reads what SLAM V6 writes (pose + state, occupancy grid, the LiDAR scan it used) and runs the
Explorer (src/mission/nidar_explorer.py). Sends flight plans to ONE flight interface over TCP:

    protocol NIDAR_NAV_V1, port 5030 (NIDAR_NAV_PORT), newline-separated JSON, SLAM frame
    brain -> flight:  {"type":"plan","id":7,"frame":"slam","waypoints":[[x,y],...],"end":"look|none|land",
                       "look_yaws":[rad,...],"goal":[x,y],"reason":"..."}
                      {"type":"cancel","id":7,"reason":"..."}      (stop where you are; a new plan follows)
                      {"type":"hold","reason":"..."}               (no plan: hover / stand still)
                      {"type":"pose","x":..,"y":..,"yaw":..,"t":..,"age":..}  (SLAM pose, 2 Hz; age = how old
                       its scan is, s: the flight interface keeps its own frame aligned to SLAM's with it)
    flight -> brain:  {"type":"status","id":7,"state":"FLYING|DONE|BLOCKED|ABORTED","wp":2}
                      (wp = index of the next waypoint not yet reached; FLYING about once a second)

The flight interface (gazebo/ros2/nidar_rig_driver.py --brain, later the PX4 flyer) keeps its own
guards (LiDAR stop, tracking, Ctrl+C). The brain sends nothing while SLAM is LOST or stale (HOLD).
Only cells known free and >= drone radius + 0.05 m from any wall are ever planned through.

Run on the Pi (second SSH window, after start_nidar_pi.sh with NIDAR_SLAM=V6):
    cd ~/drone_project && source .venv/bin/activate
    python src/mission/nidar_explorer_service.py --radius 0.25 --mission-min 20
Writes outputs/mission/explorer_state.json (shown by the mission server / ground station).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parents[2]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))
from src.mission.nidar_explorer import Explorer, Plan, DRONE_RADIUS_M, MISSION_TIME_S  # noqa: E402

PROTOCOL = "NIDAR_NAV_V1"
PORT = int(os.environ.get("NIDAR_NAV_PORT", "5030"))
FILES = {
    "slam": PROJECT_DIR / "outputs/slam/state/nidar_slam_v6_state.json",
    "grid": PROJECT_DIR / "outputs/slam/maps/nidar_slam_v6_grid.npy",
    "scan": Path(os.environ.get("NIDAR_SCAN_FILE", PROJECT_DIR / "src/communication/nidar_latest_scan.json")),
}
OUT = PROJECT_DIR / "outputs/mission/explorer_state.json"
SLAM_STALE_S = 1.0
CHECK_PLAN_S = 1.0
MIN_SCORE_FOR_HITS = 0.5
POSE_EVERY_S = 0.5
REPLAN_WHILE_HOLDING_S = 1.0     # planning costs ~0.05 s on a PC, ~1 s on a Pi 3B+: not every loop
MAX_BLOCKED_STREAK = 5           # this many plans in a row stopped by the LiDAR guard -> HOLD for good (operator)


def read_json(path: Path) -> Optional[dict]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def scan_points(scan: dict) -> np.ndarray:
    """NIDAR_LIDAR_V1 (ROS LaserScan fields) -> Nx2 body-frame points (x forward, y left)."""
    r = np.array([np.nan if v is None else float(v) for v in scan.get("ranges", [])])
    a = float(scan["angle_min"]) + float(scan["angle_increment"]) * np.arange(len(r))
    ok = np.isfinite(r) & (r >= max(0.15, float(scan.get("range_min", 0.0)))) & \
        (r <= min(10.0, float(scan.get("range_max", 12.0))))
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


class Brain:
    """Pure decision loop (no sockets): inputs -> outgoing messages. Tested in tests/test_explorer_service.py."""

    def __init__(self, explorer: Explorer):
        self.ex = explorer
        self.plan: Optional[Plan] = None
        self.plan_id = 0
        self.wp = 0
        self.last_check = 0.0
        self.holding: Optional[str] = None
        self.complete = False
        self.scans = []                       # recent scans (received time, dict)
        self.grid_mtime = None
        self.pose = None
        self.plan_msg: Optional[dict] = None
        self.last_pose_msg = 0.0
        self.next_plan_t = 0.0
        self.blocked_streak = 0
        self.stuck = False
        self.events: List[str] = []

    def _hold(self, reason) -> List[dict]:
        if self.holding == reason:
            return []
        self.holding = reason
        self.events.append(f"HOLD: {reason}")
        out = []
        if self.plan is not None:
            out.append({"type": "cancel", "id": self.plan_id, "reason": reason})
            self.plan = None
        self.plan_msg = {"type": "hold", "reason": reason}
        out.append(self.plan_msg)
        return out

    def resend(self) -> List[dict]:
        """A (re)connected flight interface gets the current plan (same id) or hold again."""
        return [self.plan_msg] if self.plan_msg is not None and not self.complete else []

    def feed_scan(self, scan: Optional[dict]) -> None:
        if scan and "received" in scan and (not self.scans or scan["received"] != self.scans[-1][0]):
            self.scans.append((float(scan["received"]), scan))
            self.scans = self.scans[-30:]

    def step(self, now: float, slam: Optional[dict], grid: Optional[np.ndarray], client_msgs: List[dict],
             connected: bool = True) -> List[dict]:
        out: List[dict] = []
        if grid is not None:
            mp = (slam or {}).get("map") or {}
            org = mp.get("origin_m") or [-20.0, -20.0]
            self.ex.update_map(grid, res=float(mp.get("resolution_m", 0.1)), origin=float(org[0]))
        if not slam or now - float(slam.get("timestamp", 0)) > SLAM_STALE_S:
            return out + self._hold("SLAM stale or missing")
        m = slam.get("match") or {}
        if slam.get("status") == "LOST" or m.get("accepted") is False:
            return out + self._hold("SLAM lost: not trusting the position")
        if self.ex.view is None:
            return out + self._hold("waiting for the first map")
        p = slam["pose"]
        x, y, yaw = float(p["x_m"]), float(p["y_m"]), math.radians(float(p["yaw_deg"]))
        self.pose = (x, y, yaw)
        if now - self.last_pose_msg >= POSE_EVERY_S:
            self.last_pose_msg = now
            st_ = slam.get("scan_timestamp")
            out.append({"type": "pose", "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4), "t": st_,
                        "age": None if st_ is None else round(max(0.0, now - float(st_)), 3)})
        # raw hits: only the scan SLAM itself used for this pose (no smearing while turning)
        st = slam.get("scan_timestamp")
        if st is not None and float(m.get("score", 0.0)) >= MIN_SCORE_FOR_HITS:
            for rt, sc in self.scans:
                if abs(rt - float(st)) < 0.005:
                    pts = scan_points(sc)
                    c, s = math.cos(yaw), math.sin(yaw)
                    self.ex.add_hits(pts @ np.array([[c, s], [-s, c]]) + np.array([x, y]))
                    break
        self.ex.observe(x, y, yaw)
        for msg in client_msgs:
            if msg.get("type") != "status" or msg.get("id") != self.plan_id or self.plan is None:
                continue
            self.wp = int(msg.get("wp", self.wp))
            state = msg.get("state")
            if state == "DONE":
                self.blocked_streak = 0
                if self.plan.end == "land":
                    self.complete = True
                    self.events.append("MISSION COMPLETE: landed at the take-off point")
                else:
                    self.ex.arrived_and_looked(x, y)
                self.plan = self.plan_msg = None
            elif state in ("BLOCKED", "ABORTED"):
                self.blocked_streak += 1
                self.events.append(f"plan {self.plan_id} {state} by the flight interface: replanning")
                self.ex.arrived_and_looked(x, y)
                self.plan = self.plan_msg = None
                if self.blocked_streak >= MAX_BLOCKED_STREAK:      # offline SLAM test: 14 729 BLOCKED in a row
                    self.stuck = True
        if self.stuck:
            return out + self._hold(f"stuck: {self.blocked_streak} plans in a row stopped by the LiDAR guard "
                                    "-- operator needed")
        if self.complete or not connected:
            return out
        if self.plan is not None and now - self.last_check >= CHECK_PLAN_S:
            self.last_check = now
            if not self.ex.plan_still_safe(x, y, self.plan.waypoints, self.wp):
                out.append({"type": "cancel", "id": self.plan_id, "reason": "new obstacle on the path"})
                self.events.append(f"plan {self.plan_id} cancelled: new obstacle on the path")
                self.plan = self.plan_msg = None
        if self.plan is None and now >= self.next_plan_t:
            p_ = self.ex.plan(x, y, yaw, now)
            if p_ is None:
                self.complete = True
                return out
            if not p_.waypoints and p_.end != "land":
                self.next_plan_t = now + REPLAN_WHILE_HOLDING_S
                return out + self._hold(p_.reason)
            self.holding = None
            self.plan_id += 1
            self.plan, self.wp = p_, 0
            self.last_check = now
            self.events.append(f"plan {self.plan_id}: {p_.reason} -> goal ({p_.goal[0]:.2f}, {p_.goal[1]:.2f})")
            self.plan_msg = {"type": "plan", "id": self.plan_id, "frame": "slam",
                             "waypoints": [[round(a, 3), round(b, 3)] for a, b in p_.waypoints],
                             "end": p_.end, "look_yaws": [round(a, 4) for a in p_.look_yaws],
                             "goal": [round(p_.goal[0], 3), round(p_.goal[1], 3)], "reason": p_.reason,
                             "pose": [round(x, 3), round(y, 3), round(yaw, 4)]}
            out.append(self.plan_msg)
        return out

    def state_json(self, now: float) -> dict:
        return {"protocol": PROTOCOL, "timestamp": now, "state": "COMPLETE" if self.complete else
                ("HOLD" if self.holding else self.ex.state), "hold_reason": self.holding,
                "plan_id": self.plan_id, "pose": self.pose,
                "plan": None if self.plan is None else {"waypoints": self.plan.waypoints, "end": self.plan.end,
                                                         "goal": self.plan.goal, "reason": self.plan.reason},
                "stats": self.ex.stats, "drone_radius_m": self.ex.radius, "events": self.events[-10:],
                "return_reason": getattr(self.ex, "return_reason", None)}


class Link:
    """One flight-interface client at a time; newline JSON both ways."""

    def __init__(self, host: str, port: int):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(1)
        self.srv.settimeout(0.5)
        self.port = self.srv.getsockname()[1]
        self.conn = None
        self.inbox: "queue.Queue[dict]" = queue.Queue()
        self.lock = threading.Lock()
        self.halt = threading.Event()
        self.new_client = threading.Event()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while not self.halt.is_set():
            try:
                conn, addr = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self.lock:
                if self.conn:
                    try:
                        self.conn.close()
                    except OSError:
                        pass
                self.conn = conn
            self.new_client.set()
            print(f"[NAV] flight interface connected: {addr}", flush=True)
            threading.Thread(target=self._read, args=(conn,), daemon=True).start()

    def _read(self, conn):
        buf = b""
        try:
            conn.settimeout(1.0)
            while not self.halt.is_set():
                try:
                    chunk = conn.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        self.inbox.put(json.loads(line))
                    except ValueError:
                        pass
        except OSError:
            pass
        with self.lock:
            if self.conn is conn:
                self.conn = None
        print("[NAV] flight interface disconnected", flush=True)

    @property
    def connected(self) -> bool:
        return self.conn is not None

    def send(self, msg: dict) -> None:
        data = (json.dumps(dict(msg, protocol=PROTOCOL)) + "\n").encode()
        with self.lock:
            if self.conn:
                try:
                    self.conn.sendall(data)
                except OSError:
                    self.conn = None

    def drain(self) -> List[dict]:
        out = []
        while True:
            try:
                out.append(self.inbox.get_nowait())
            except queue.Empty:
                return out

    def close(self):
        self.halt.set()
        try:
            self.srv.close()
        except OSError:
            pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--radius", type=float, default=DRONE_RADIUS_M, help="drone radius to the prop tips (m)")
    ap.add_argument("--mission-min", type=float, default=MISSION_TIME_S / 60, help="then return and land")
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args(argv)
    brain = Brain(Explorer(drone_radius=args.radius, mission_time_s=args.mission_min * 60))
    link = Link("0.0.0.0", args.port)
    print("=" * 78, flush=True)
    print(f"NIDAR EXPLORER ({PROTOCOL}) port {link.port} | drone radius {args.radius} m (+0.05 map margin) | "
          f"mission {args.mission_min:.0f} min", flush=True)
    print("Knows nothing about the arena: explores, looks, returns to the take-off point, lands.", flush=True)
    print("=" * 78, flush=True)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    last_write = last_print = 0.0
    n_ev = 0
    try:
        while True:
            now = time.time()
            if link.new_client.is_set():                  # a (re)connected client gets the current plan again
                link.new_client.clear()
                for msg in brain.resend():
                    link.send(msg)
            grid = None
            try:
                mt = FILES["grid"].stat().st_mtime
                if mt != brain.grid_mtime:
                    brain.grid_mtime = mt
                    grid = np.load(FILES["grid"])
            except (OSError, ValueError):
                pass
            brain.feed_scan(read_json(FILES["scan"]))
            for msg in brain.step(now, read_json(FILES["slam"]), grid, link.drain(), link.connected):
                link.send(msg)
            for e in brain.events[n_ev:]:
                print(f"[NAV] {e}", flush=True)
            n_ev = len(brain.events)
            if now - last_write > 0.5:
                last_write = now
                tmp = OUT.with_suffix(".tmp")
                tmp.write_text(json.dumps(brain.state_json(now), default=float), encoding="utf-8")
                tmp.replace(OUT)
            if now - last_print > 10:
                last_print = now
                print(f"[NAV] state {brain.state_json(now)['state']} | client {'UP' if link.connected else 'none'} | "
                      f"plans {brain.plan_id} | looks {brain.ex.stats['looks']}", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        link.send({"type": "hold", "reason": "explorer stopped"})
        link.close()
        print("[NAV] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
