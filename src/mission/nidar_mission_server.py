#!/usr/bin/env python3
"""
NIDAR mission server -- Raspberry Pi (2026-11-02e).

Collects what the Pi knows and serves it to the ground station (gcs/nidar_gcs.py):
  * drone pose from SLAM V6 (converted to the ARENA frame with the start pose),
  * the SLAM occupancy map,
  * survivors: V18 camera/LiDAR measurements + SLAM pose -> src/mission/survivor_mapper.py,
  * the latest camera detections (boxes for the camera view overlay),
  * pipeline health: perception, V18, LiDAR, SLAM ages and rates.

Protocol NIDAR_MISSION_V1: TCP (default 5020), newline-delimited JSON, any number
of ground stations. Messages:
  {"type": "state", ...}   5 Hz
  {"type": "map", ...}     when the SLAM grid file changes (<= 1 Hz) and on connect
Files written: outputs/mission/survivors.json, outputs/mission/mission_state.json.

Configuration (environment):
  NIDAR_MISSION_PORT     5020
  NIDAR_ARENA_LAYOUT     gazebo/layouts/<world>.json -> start pose + grid from the layout
  NIDAR_START_POSE       "x,y,yaw_deg" arena pose of the drone when SLAM started (overrides the layout)
  NIDAR_CAMERA_HEIGHT_M  camera height above the floor for ground ranging (default 0.82 = 1.0 m flight)
  NIDAR_CAMERA_HFOV_DEG  70
  NIDAR_PROJECT_DIR      project root (default: this file's project)
Read-only towards the flight side: it never sends commands.
"""

from __future__ import annotations

import base64
import csv
import json
import math
import os
import signal
import socket
import sys
import threading
import time
import zlib
from collections import deque
from pathlib import Path
from typing import Optional

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np  # noqa: E402

from src.mission.survivor_mapper import CameraModel, Grid, SurvivorMapper, compose  # noqa: E402

PROTOCOL = "NIDAR_MISSION_V1"
PROJECT_DIR = Path(os.environ.get("NIDAR_PROJECT_DIR", _ROOT))
PORT = int(os.environ.get("NIDAR_MISSION_PORT", "5020"))
FILES = {
    "perception": PROJECT_DIR / "src/communication/nidar_latest_perception.json",
    "scan": Path(os.environ.get("NIDAR_SCAN_FILE", PROJECT_DIR / "src/communication/nidar_latest_scan.json")),
    "v18": PROJECT_DIR / "nidar_v18_headless_persistent_fusion.json",
    "slam": PROJECT_DIR / "outputs/slam/state/nidar_slam_v6_state.json",
    "grid": PROJECT_DIR / "outputs/slam/maps/nidar_slam_v6_grid.npy",
    "explorer": PROJECT_DIR / "outputs/mission/explorer_state.json",    # autonomous brain (2026-11-09)
}
EXPLORER_STALE_S = 5.0     # 2026-11-09 rig run: the Pi brain sometimes paused > 2 s -> BRAIN flickered 'off'
OUT_DIR = PROJECT_DIR / "outputs/mission"
STALE_S = 1.0
MIN_POSE_SCORE = 0.30          # survivors are mapped only with a trusted SLAM pose (2026-11-02f)
STATE_HZ = 5.0
LOOP_HZ = 10.0
PATH_STEP_M = 0.05
PAD_RADIUS_M = 0.6           # pose within this of the take-off point is labelled PAD (was "OUT")


def read_json(path: Path) -> Optional[dict]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def file_age(path: Path, now: float) -> Optional[float]:
    try:
        return max(0.0, now - path.stat().st_mtime)
    except OSError:
        return None


def load_config():
    layout = None
    lp = os.environ.get("NIDAR_ARENA_LAYOUT", "").strip()
    if lp:
        p = Path(lp)
        if not p.is_absolute():
            p = PROJECT_DIR / p
        layout = json.loads(p.read_text(encoding="utf-8"))
    grid = Grid.from_layout(layout) if layout else Grid(bounded=False)     # no layout: arena unknown
    start = (0.0, 0.0, 0.0)
    if layout:
        sp = layout["spawn_pose"]
        start = (float(sp["x"]), float(sp["y"]), float(sp["yaw"]))
    env_start = os.environ.get("NIDAR_START_POSE", "").strip()
    if env_start:
        x, y, yaw_deg = [float(v) for v in env_start.split(",")]
        start = (x, y, math.radians(yaw_deg))
    cam = CameraModel(hfov_deg=float(os.environ.get("NIDAR_CAMERA_HFOV_DEG", "70")),
                      height_m=float(os.environ.get("NIDAR_CAMERA_HEIGHT_M", "0.82")))
    return layout, grid, start, cam


class MissionState:
    def __init__(self, grid: Grid, start, cam: CameraModel, layout: Optional[dict] = None):
        self.grid, self.start, self.layout = grid, start, layout
        self.mapper = SurvivorMapper(cam, grid, start)
        self.poses = deque(maxlen=300)              # (t, slam_x, slam_y, slam_yaw, trusted)
        self.path = []                              # arena frame [x, y]
        self.last_v18_frame = None
        self.last_slam_frame = None
        self.lock = threading.Lock()
        self.snapshot = {}
        self.map_msg = None
        self.map_mtime = None
        self.skipped_no_pose = 0
        self.skipped_bad_pose = 0
        self.slam_status = "NO DATA"
        self.journal_path = OUT_DIR / time.strftime("measurements_%Y%m%d_%H%M%S.csv")

    def pose_at(self, t: float):
        if not self.poses:
            return None
        best = min(self.poses, key=lambda p: abs(p[0] - t))
        return best if abs(best[0] - t) <= 0.5 else None

    @staticmethod
    def trusted(slam: dict) -> bool:
        """SLAM pose good enough to place survivors: tracking, match accepted, decent score."""
        m = slam.get("match") or {}
        if slam.get("status") == "LOST" or m.get("accepted") is False:
            return False
        try:
            return float(m.get("score", 1.0)) >= MIN_POSE_SCORE
        except (TypeError, ValueError):
            return False

    def step(self, now: float) -> None:
        slam = read_json(FILES["slam"])
        if slam and slam.get("frame") != self.last_slam_frame:
            self.last_slam_frame = slam.get("frame")
            try:
                p = slam["pose"]
                rec = (float(slam["timestamp"]), float(p["x_m"]), float(p["y_m"]),
                       math.radians(float(p["yaw_deg"])), self.trusted(slam))
                self.poses.append(rec)
                self.slam_status = slam.get("status", "TRACKING")
                ax, ay, _ = compose(self.start, rec[1:4])
                if not self.path or math.hypot(ax - self.path[-1][0], ay - self.path[-1][1]) >= PATH_STEP_M:
                    self.path.append([round(ax, 3), round(ay, 3)])
            except (KeyError, TypeError, ValueError):
                pass
        v18 = read_json(FILES["v18"])
        perception = read_json(FILES["perception"])
        if perception and isinstance(perception.get("camera"), dict):
            try:
                self.mapper.camera.width = float(perception["camera"]["width"])
                self.mapper.camera.height = float(perception["camera"]["height"])
            except (KeyError, TypeError, ValueError):
                pass
        v18_age = None
        if v18:
            try:
                v18_age = now - float(v18.get("timestamp", 0))
            except (TypeError, ValueError):
                v18_age = None
        if v18 and v18.get("frame") != self.last_v18_frame and v18_age is not None and v18_age < STALE_S:
            self.last_v18_frame = v18.get("frame")
            pose = self.pose_at(float(v18["timestamp"]))
            if pose is None:
                self.skipped_no_pose += 1
            elif not pose[4]:
                self.skipped_bad_pose += 1                  # SLAM lost / poor match: do not place survivors
            else:
                self.mapper.update(v18.get("camera_objects") or [], v18.get("fused_objects") or [],
                                   pose[1:4], float(v18["timestamp"]))
        self._build_snapshot(now, slam, v18, perception, v18_age)
        self._maybe_load_map(slam)

    def _build_snapshot(self, now, slam, v18, perception, v18_age) -> None:
        pose = None
        if self.poses:
            t, x, y, yaw, _trusted = self.poses[-1]
            ax, ay, ayaw = compose(self.start, (x, y, yaw))
            pose = {"x_m": round(ax, 3), "y_m": round(ay, 3), "yaw_deg": round(math.degrees(ayaw), 2),
                    "grid": ("PAD" if math.hypot(ax - self.start[0], ay - self.start[1]) < PAD_RADIUS_M
                             else self.grid.label(ax, ay)), "age_s": round(now - t, 2), "trusted": _trusted,
                    "slam_status": self.slam_status,
                    "slam": {"x_m": round(x, 3), "y_m": round(y, 3), "yaw_deg": round(math.degrees(yaw), 2)}}
        scan = read_json(FILES["scan"])
        health = {
            "perception": {
                "frame": perception.get("frame") if perception else None,
                "age_s": _r(file_age(FILES["perception"], now)),
                "objects": len(perception.get("objects", [])) if perception else 0,
                "source": (perception.get("camera") or {}).get("source") if perception else None,
            },
            "v18": {
                "age_s": _r(v18_age), "lidar_source": v18.get("lidar_source") if v18 else None,
                "active": sum(1 for o in (v18.get("persistent_objects") or []) if o.get("state") == "ACTIVE") if v18 else 0,
                "fused": len(v18.get("fused_objects") or []) if v18 else 0,
            },
            "lidar": {"age_s": _r(now - float(scan["received"])) if scan and "received" in scan else None,
                      "seq": scan.get("seq") if scan else None,
                      "points": len(scan.get("ranges", [])) if scan else 0},
            "slam": {"age_s": _r(now - float(slam["timestamp"])) if slam else None,
                     "fps": (slam.get("statistics") or {}).get("processing_fps") if slam else None,
                     "score": (slam.get("match") or {}).get("score") if slam else None,
                     "accepted": (slam.get("statistics") or {}).get("accepted") if slam else None,
                     "rejected": (slam.get("statistics") or {}).get("rejected") if slam else None,
                     "lidar_source": slam.get("lidar_source") if slam else None,
                     "status": (slam.get("status") or "TRACKING") if slam else "NO DATA",
                     "reject_streak": (slam.get("match") or {}).get("reject_streak") if slam else None},
            "mapper": {"measurements": self.mapper.measurements, "lidar_rejected_f22": self.mapper.lidar_rejected,
                       "outside_arena_rejected": self.mapper.outside_rejected,
                       "edge_clipped": self.mapper.edge_clipped, "ghosts_absorbed": self.mapper.ghosts,
                       "skipped_no_pose": self.skipped_no_pose, "skipped_bad_pose": self.skipped_bad_pose,
                       "merges": [list(m) for m in self.mapper.merges[-20:]]},
        }
        explorer = self._explorer(now)
        objects = []
        if v18 and v18_age is not None and v18_age < STALE_S:
            for o in v18.get("camera_objects") or []:
                if o.get("current_observation") is False:
                    continue
                objects.append({"id": o.get("id"), "class": o.get("class"), "confidence": o.get("confidence"),
                                "bbox": o.get("bbox")})
        latest = [{"x_m": round(m.x, 3), "y_m": round(m.y, 3), "method": m.method}
                  for m in self.mapper.last_measurements]
        snap = {
            "type": "state", "protocol": PROTOCOL, "timestamp": now, "pose": pose,
            "path_tail": self.path[-300:], "path_len": len(self.path),
            "survivors": self.mapper.report(), "measurements": latest,
            "camera": {"width": self.mapper.camera.width, "height": self.mapper.camera.height, "objects": objects},
            "health": health, "start_pose": [self.start[0], self.start[1], math.degrees(self.start[2])],
            "grid": {"pitch_m": self.grid.pitch_m, "wall_m": self.grid.wall_m, "cells": self.grid.cells,
                     "bounded": self.grid.bounded},
            "explorer": explorer,
            "flight_control": "NONE (read-only)",
        }
        with self.lock:
            self.snapshot = snap

    def _explorer(self, now: float) -> Optional[dict]:
        """The autonomous brain's state (nidar_explorer_service.py), plan converted to the arena frame."""
        ex = read_json(FILES["explorer"])
        try:
            if not ex or now - float(ex.get("timestamp", 0)) > EXPLORER_STALE_S:
                return None

            def arena(xy):
                ax, ay, _ = compose(self.start, (float(xy[0]), float(xy[1]), 0.0))
                return [round(ax, 3), round(ay, 3)]
            plan = ex.get("plan")
            return {"state": ex.get("state"), "hold_reason": ex.get("hold_reason"), "plan_id": ex.get("plan_id"),
                    "plan": None if not plan else {"waypoints": [arena(w) for w in plan.get("waypoints") or []],
                                                   "goal": arena(plan["goal"]), "end": plan.get("end"),
                                                   "reason": plan.get("reason")},
                    "looks": (ex.get("stats") or {}).get("looks"), "radius_m": ex.get("drone_radius_m"),
                    "return_reason": ex.get("return_reason"),
                    "events": list(ex.get("events") or [])[-10:]}
        except (KeyError, TypeError, ValueError, IndexError):
            return None

    def _maybe_load_map(self, slam) -> None:
        try:
            mtime = FILES["grid"].stat().st_mtime_ns
        except OSError:
            return
        if mtime == self.map_mtime:
            return
        try:
            grid = np.load(FILES["grid"])
        except (OSError, ValueError):
            return                                  # file being replaced: next loop
        self.map_mtime = mtime
        meta = (slam or {}).get("map") or {}
        res = float(meta.get("resolution_m", 0.1))
        origin = meta.get("origin_m", [-grid.shape[1] * res / 2, -grid.shape[0] * res / 2])
        known = np.argwhere(grid >= 0)
        if len(known):
            r0, c0 = known.min(axis=0)
            r1, c1 = known.max(axis=0) + 1
        else:
            r0 = c0 = 0
            r1, c1 = 1, 1
        crop = np.ascontiguousarray(grid[r0:r1, c0:c1].astype(np.int8))
        msg = {"type": "map", "protocol": PROTOCOL, "timestamp": time.time(), "resolution_m": res,
               "origin_m": [float(origin[0]) + c0 * res, float(origin[1]) + r0 * res],
               "shape": [int(crop.shape[0]), int(crop.shape[1])], "frame": "SLAM (start pose)",
               "start_pose": [self.start[0], self.start[1], math.degrees(self.start[2])],
               "encoding": "int8 row-major (row = +y), zlib, base64; -1 unknown, 0..100 occupied %",
               "data": base64.b64encode(zlib.compress(crop.tobytes(), 6)).decode("ascii"),
               "path": self.path[-2000:]}
        with self.lock:
            self.map_msg = msg

    def messages(self):
        with self.lock:
            return self.snapshot, self.map_msg


def _r(v, nd=2):
    return None if v is None else round(float(v), nd)


class ClientThread(threading.Thread):
    def __init__(self, conn, addr, state: MissionState, stop: threading.Event):
        super().__init__(daemon=True)
        self.conn, self.addr, self.state, self.stop = conn, addr, state, stop

    def run(self):
        print(f"[MISSION] ground station connected: {self.addr}", flush=True)
        sent_map = None
        try:
            self.conn.settimeout(5.0)
            while not self.stop.is_set():
                snap, mp = self.state.messages()
                if mp is not None and mp is not sent_map:
                    self.conn.sendall((json.dumps(mp, separators=(",", ":")) + "\n").encode())
                    sent_map = mp
                if snap:
                    self.conn.sendall((json.dumps(snap, separators=(",", ":")) + "\n").encode())
                time.sleep(1.0 / STATE_HZ)
        except OSError as exc:
            print(f"[MISSION] {self.addr} gone: {exc}", flush=True)
        finally:
            try:
                self.conn.close()
            except OSError:
                pass


def serve(state: MissionState, stop: threading.Event, host: str = "0.0.0.0", port: int = PORT,
          ready: threading.Event = None, bound: list = None) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    srv.settimeout(0.3)
    if bound is not None:
        bound.append(srv.getsockname()[1])
    if ready is not None:
        ready.set()
    try:
        while not stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            ClientThread(conn, addr, state, stop).start()
    finally:
        srv.close()


JOURNAL_COLUMNS = ["t", "outcome", "survivor", "assoc", "dist_to_centre", "camera_id", "conf", "bbox",
                   "lidar_m", "ground_m", "lidar_rejected", "method", "range_m", "bearing_deg",
                   "pose_x", "pose_y", "pose_yaw_deg", "x", "y"]


def write_journal(state: MissionState) -> int:
    """Append the mapper's new measurement rows to outputs/mission/measurements_<start time>.csv
    (every camera measurement with its inputs and what the mapper did with it: offline replay)."""
    rows = state.mapper.drain_journal()
    if not rows:
        return 0
    path = state.journal_path
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=JOURNAL_COLUMNS, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerows(rows)
    return len(rows)


def write_outputs(state: MissionState) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    write_journal(state)
    snap, _ = state.messages()
    if not snap:
        return
    for name, data in (("survivors.json", {"timestamp": snap["timestamp"], "survivors": snap["survivors"],
                                            "start_pose": snap["start_pose"], "grid": snap["grid"]}),
                       ("mission_state.json", snap)):
        tmp = OUT_DIR / (name + ".tmp")
        tmp.write_text(json.dumps(data, indent=1), encoding="utf-8")
        tmp.replace(OUT_DIR / name)


def main() -> int:
    layout, grid, start, cam = load_config()
    state = MissionState(grid, start, cam, layout)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print("=" * 78, flush=True)
    print(f"NIDAR MISSION SERVER ({PROTOCOL}) port {PORT} | project {PROJECT_DIR}", flush=True)
    print(f"start pose (arena): x={start[0]:.2f} y={start[1]:.2f} yaw={math.degrees(start[2]):.1f} deg | "
          f"grid {grid.cells}x{grid.cells} pitch {grid.pitch_m} m | camera height {cam.height_m} m", flush=True)
    print("needs: SLAM V6 (NIDAR_SLAM=V6), V18, receivers. Flight control: none.", flush=True)
    print("=" * 78, flush=True)
    threading.Thread(target=serve, args=(state, stop), daemon=True).start()
    last_write, last_print, n_conf = 0.0, 0.0, 0
    try:
        while not stop.is_set():
            now = time.time()
            state.step(now)
            if now - last_write >= 1.0:
                write_outputs(state)
                last_write = now
            snap, _ = state.messages()
            conf = [s for s in snap.get("survivors", []) if s["status"] == "CONFIRMED"]
            if len(conf) != n_conf:
                n_conf = len(conf)
                s = conf[-1]
                print(f"[MISSION] SURVIVOR {s['id']} at {s['grid']} (x={s['x_m']:.2f} y={s['y_m']:.2f}, "
                      f"{s['observations']} obs, {s['methods']})", flush=True)
            if now - last_print >= 5.0:
                last_print = now
                p = snap.get("pose") or {}
                h = snap.get("health", {})
                print(f"[MISSION] pose {p.get('grid', '-')} ({p.get('x_m', '-')}, {p.get('y_m', '-')}) | "
                      f"survivors {n_conf} confirmed | SLAM age {h.get('slam', {}).get('age_s')} s | "
                      f"V18 age {h.get('v18', {}).get('age_s')} s", flush=True)
            time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        write_outputs(state)
        print("[MISSION] stopped; survivors written to outputs/mission/survivors.json", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
