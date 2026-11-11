#!/usr/bin/env python3
"""
NIDAR SLAM V6 -- 2-D LiDAR SLAM with a scan-to-map matcher (2026-11-02).

Why a new version: SLAM V5.1's ScanMatcher estimates motion from the shift of
the point-cloud centroid (SLAM-01), so its pose does not follow a moving
sensor (arena test: 6.8 m travelled -> 0.44 m estimated). V5.1 is kept
unchanged; start_nidar_pi.sh runs V6 when NIDAR_SLAM=V6.

Method (the approach of Hector SLAM, Kohlbrecher et al. 2011, built for
LiDAR-only robots/drones without wheel odometry):
  * occupancy grids at 3 resolutions (0.05 / 0.10 / 0.20 m), log-odds,
    each updated with every accepted scan (hits at endpoints, free space
    along the beams);
  * each new scan is aligned to the MAP (not to the previous scan) by
    Gauss-Newton on the bilinearly interpolated occupancy, coarse to fine,
    starting from a constant-velocity prediction;
  * the match is accepted only if enough endpoints land on occupied map
    cells; otherwise the prediction is kept and the map is not updated.

Frames: SLAM frame = the vehicle's pose at the first scan (x forward,
y LEFT, z up, yaw counter-clockwise). NIDAR LiDAR angles are clockwise
(0 = forward, 90 = right), so beam angle_ccw = -angle_deg.
(V5.1 used the clockwise angles directly as x/y, i.e. a mirrored map.)

Outputs (atomic writes, relative to the working directory like V5.1):
  outputs/slam/state/nidar_slam_v6_state.json   every scan (schema readable by
                                                integration_monitor.py)
  outputs/slam/maps/nidar_slam_v6_grid.npy      0.10 m occupancy (int8: -1 unknown,
                                                0..100), every MAP_EXPORT_SEC
  outputs/slam/trajectories/nidar_slam_v6_trajectory.csv

LiDAR source: NIDAR_LIDAR_BACKEND = SIMULATOR (default) | GAZEBO.
Flight control: none. This process only reads scans and writes files.
"""

from __future__ import annotations

import csv
import json
import math
import os
import signal
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

VERSION = "NIDAR_SLAM_V6"

# ------------------------------------------------------------------ config --
LEVEL_RESOLUTIONS = (0.20, 0.10, 0.05)     # coarse -> fine
MAP_HALF_SIZE_M = 20.0                     # map covers +-20 m around the start
EXPORT_LEVEL_RES = 0.10
MIN_RANGE_M = 0.15
MAX_RANGE_M = 10.0
MAX_POINTS = 240                           # endpoints used for matching (Pi 3B+ budget)
ITERATIONS = (6, 6, 8)                     # Gauss-Newton iterations per level
L_OCC = 0.9                                # log-odds increment for a hit
L_FREE = -0.35                             # log-odds increment for free space
L_MIN, L_MAX = -2.0, 3.5
M_SATURATION = 0.9                         # log-odds giving "fully occupied" for matching (one hit)
FREE_STOP_CELLS = 2                        # stop free-space updates this many cells before the hit
UPDATE_MIN_MOVE_M = 0.04
UPDATE_MIN_TURN_RAD = math.radians(2.0)
MIN_SCORE = 0.20                           # mean map value at endpoints to accept a match
GOOD_SCORE = 0.45                          # a match this good needs no second hypothesis
MAP_UPDATE_MIN_SCORE = 0.35                # only clearly good matches write into the map
FREE_MAX_M = 6.0                           # free-space updates only along the first 6 m of a beam
V_MAX = 1.5                                # m/s   plausible vehicle speed (motion gate)
W_MAX = 1.5                                # rad/s plausible yaw rate
YAW_SEARCH = (math.radians(24.0), math.radians(3.0))      # coarse yaw search around each hypothesis
WIDE_SEARCH = (math.radians(45.0), math.radians(4.5), 0.30, 0.15)   # last resort: yaw, step, xy, step
LOST_AFTER = 10                            # consecutive rejected scans -> state "LOST"
WARMUP_SCANS = 5                           # first scans: vehicle on the pad, accepted to seed the map
MAP_EXPORT_SEC = 1.0

STATE_FILE = Path("outputs/slam/state/nidar_slam_v6_state.json")
GRID_FILE = Path("outputs/slam/maps/nidar_slam_v6_grid.npy")
TRAJ_FILE = Path("outputs/slam/trajectories/nidar_slam_v6_trajectory.csv")


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw], dtype=float)


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def scan_to_points(scan, min_range: float = MIN_RANGE_M, max_range: float = MAX_RANGE_M) -> np.ndarray:
    """NIDAR LidarScan (clockwise degrees) -> Nx2 points in the body frame (x fwd, y left)."""
    d = np.asarray(scan.distance_m, dtype=float).reshape(-1)
    a = np.asarray(scan.angle_deg, dtype=float).reshape(-1)
    n = min(len(d), len(a))
    d, a = d[:n], a[:n]
    valid = np.isfinite(d) & np.isfinite(a) & (d >= min_range) & (d <= max_range)
    if hasattr(scan, "valid"):
        valid &= np.asarray(scan.valid, dtype=bool).reshape(-1)[:n]
    ccw = -np.radians(a[valid])
    return np.column_stack([d[valid] * np.cos(ccw), d[valid] * np.sin(ccw)])


class GridLevel:
    """One occupancy grid (log-odds) with bilinear map access for matching."""

    def __init__(self, res: float, half_size_m: float = MAP_HALF_SIZE_M):
        self.res = float(res)
        self.n = int(round(2 * half_size_m / res))
        self.origin = -half_size_m                    # world coord of cell (0,0) corner, both axes
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self._M = np.zeros((self.n, self.n), dtype=np.float32)   # kept in step with L (changed cells only)

    # map value used for matching: 0 = free/unknown, 1 = surely occupied
    def M(self) -> np.ndarray:
        return self._M

    def to_cells(self, pts: np.ndarray) -> np.ndarray:
        return (pts - self.origin) / self.res - 0.5          # continuous, cell centres at integers

    def interp(self, pts_world: np.ndarray):
        """Bilinear value and world-frame gradient at points."""
        M = self.M()
        c = self.to_cells(pts_world)
        x0 = np.floor(c[:, 0]).astype(int)
        y0 = np.floor(c[:, 1]).astype(int)
        inside = (x0 >= 0) & (y0 >= 0) & (x0 < self.n - 1) & (y0 < self.n - 1)
        x0c = np.clip(x0, 0, self.n - 2)
        y0c = np.clip(y0, 0, self.n - 2)
        fx = c[:, 0] - x0
        fy = c[:, 1] - y0
        p00 = M[y0c, x0c]
        p10 = M[y0c, x0c + 1]
        p01 = M[y0c + 1, x0c]
        p11 = M[y0c + 1, x0c + 1]
        val = fy * (fx * p11 + (1 - fx) * p01) + (1 - fy) * (fx * p10 + (1 - fx) * p00)
        dx = fy * (p11 - p01) + (1 - fy) * (p10 - p00)
        dy = fx * (p11 - p10) + (1 - fx) * (p01 - p00)
        val = np.where(inside, val, 0.0)
        grad = np.column_stack([np.where(inside, dx, 0.0), np.where(inside, dy, 0.0)]) / self.res
        return val, grad

    def update(self, origin_xy: np.ndarray, endpoints: np.ndarray) -> None:
        """Hits at endpoints, free space along each beam (stops FREE_STOP_CELLS before the hit)."""
        if len(endpoints) == 0:
            return
        o = np.asarray(origin_xy, dtype=float)
        vec = endpoints - o
        dist = np.hypot(vec[:, 0], vec[:, 1])
        step = self.res
        free_len = np.minimum(np.maximum(dist - FREE_STOP_CELLS * self.res, 0.0), FREE_MAX_M)
        nmax = int(np.ceil(free_len.max() / step)) if len(free_len) else 0
        if nmax > 0:
            t = np.arange(nmax) * step                                  # (S,)
            mask = t[None, :] < free_len[:, None]                         # (N,S)
            unit = vec / np.maximum(dist, 1e-9)[:, None]
            fx = o[0] + unit[:, 0:1] * t[None, :]
            fy = o[1] + unit[:, 1:2] * t[None, :]
            ci = np.floor((fx[mask] - self.origin) / self.res).astype(int)
            cj = np.floor((fy[mask] - self.origin) / self.res).astype(int)
            ok = (ci >= 0) & (cj >= 0) & (ci < self.n) & (cj < self.n)
            flat = cj[ok] * self.n + ci[ok]
            self.L.reshape(-1)[flat] += L_FREE      # fancy-index += applies once per cell even with repeats
        else:
            flat = np.empty(0, dtype=int)
        hi = np.floor((endpoints[:, 0] - self.origin) / self.res).astype(int)
        hj = np.floor((endpoints[:, 1] - self.origin) / self.res).astype(int)
        ok = (hi >= 0) & (hj >= 0) & (hi < self.n) & (hj < self.n)
        hflat = hj[ok] * self.n + hi[ok]
        self.L.reshape(-1)[hflat] += L_OCC
        changed = np.concatenate([flat, hflat])
        Lf, Mf = self.L.reshape(-1), self._M.reshape(-1)
        Lf[changed] = np.clip(Lf[changed], L_MIN, L_MAX)
        Mf[changed] = np.clip(Lf[changed] / M_SATURATION, 0.0, 1.0)

    def export_int8(self) -> np.ndarray:
        """-1 unknown, 0..100 occupancy probability (%)."""
        p = 1.0 / (1.0 + np.exp(-self.L))
        out = np.round(p * 100).astype(np.int8)
        out[np.abs(self.L) < 1e-3] = -1
        return out


def transform(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return points @ R.T + pose[:2]


class HectorMatcher:
    @staticmethod
    def search(level: GridLevel, points: np.ndarray, start: np.ndarray, yaw_half: float, yaw_step: float,
               xy_half: float = 0.0, xy_step: float = 1.0) -> np.ndarray:
        """Brute-force the best start pose (mean map value) on a coarse level: yaw (+ optional x,y) grid."""
        yaws = start[2] + np.arange(-yaw_half, yaw_half + 1e-9, yaw_step)
        offs = np.arange(-xy_half, xy_half + 1e-9, xy_step) if xy_half > 0 else np.zeros(1)
        dx, dy = np.meshgrid(offs, offs)
        dx, dy = dx.ravel(), dy.ravel()
        c, s = np.cos(yaws), np.sin(yaws)                                       # (K,)
        rx = c[:, None] * points[None, :, 0] - s[:, None] * points[None, :, 1]  # (K,N)
        ry = s[:, None] * points[None, :, 0] + c[:, None] * points[None, :, 1]
        X = start[0] + dx[:, None, None] + rx[None, :, :]                       # (T,K,N)
        Y = start[1] + dy[:, None, None] + ry[None, :, :]
        val, _ = level.interp(np.column_stack([X.ravel(), Y.ravel()]))
        score = val.reshape(len(dx), len(yaws), -1).mean(axis=2)
        ti, ki = np.unravel_index(int(np.argmax(score)), score.shape)
        return np.array([start[0] + dx[ti], start[1] + dy[ti], yaws[ki]])

    def match(self, levels: List[GridLevel], points: np.ndarray, guess: np.ndarray,
              iterations=ITERATIONS) -> Tuple[np.ndarray, float]:
        pose = guess.astype(float).copy()
        for level, n_it in zip(levels, iterations):
            for _ in range(n_it):
                S = transform(points, pose)
                val, grad = level.interp(S)
                c, s = math.cos(pose[2]), math.sin(pose[2])
                dSdth = np.column_stack([-s * points[:, 0] - c * points[:, 1],
                                         c * points[:, 0] - s * points[:, 1]])
                J = np.column_stack([grad[:, 0], grad[:, 1], np.sum(grad * dSdth, axis=1)])
                r = 1.0 - val
                H = J.T @ J
                g = J.T @ r
                if not np.isfinite(H).all() or np.trace(H) < 1e-9:
                    break
                H = H + np.eye(3) * (1e-3 * np.trace(H) + 1e-9)          # damping: weak directions stay put
                try:
                    delta = np.linalg.solve(H, g)
                except np.linalg.LinAlgError:
                    break
                step = math.hypot(delta[0], delta[1])
                lim = 2.0 * level.res
                if step > lim:
                    delta[:2] *= lim / step
                delta[2] = max(-0.2, min(0.2, delta[2]))
                pose = pose + delta
                if step < 1e-4 and abs(delta[2]) < 1e-4:
                    break
        val, _ = levels[-1].interp(transform(points, pose))
        return pose, float(np.mean(val))


@dataclass
class SlamV6:
    levels: List[GridLevel] = field(default_factory=lambda: [GridLevel(r) for r in LEVEL_RESOLUTIONS])
    matcher: HectorMatcher = field(default_factory=HectorMatcher)
    pose: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))       # per second
    last_t: Optional[float] = None
    last_update_pose: Optional[np.ndarray] = None
    scans: int = 0
    accepted: int = 0
    rejected: int = 0
    last_score: float = 0.0
    reject_streak: int = 0

    def _plausible(self, pose: np.ndarray, dt: float) -> bool:
        h = max(dt, 0.1)
        return (math.hypot(*(pose[:2] - self.pose[:2])) <= V_MAX * h + 0.15
                and abs(wrap(pose[2] - self.pose[2])) <= W_MAX * h + 0.10)

    def _hypothesis(self, pts: np.ndarray, start: np.ndarray, dt: float, wide: bool = False):
        lvl = self.levels[0]
        if wide:
            yh, ys, xh, xs = WIDE_SEARCH
            g = self.matcher.search(lvl, pts, start, yh, ys, xh, xs)
        else:
            g = self.matcher.search(lvl, pts, start, *YAW_SEARCH)
        pose, score = self.matcher.match(self.levels, pts, g)
        pose[2] = wrap(pose[2])
        return score >= MIN_SCORE and self._plausible(pose, dt), score, pose

    def process(self, points: np.ndarray, t: float, t_sensor: Optional[float] = None) -> dict:
        """
        One scan (body-frame points). t = wall/receive time; t_sensor = the sensor's own
        timestamp (Gazebo stamp), used for the motion model when given: receive times over
        Wi-Fi arrive in bursts (pairs ~15 ms apart, then gaps), which wrecked the velocity
        estimate in the first Gazebo run on the Pi (2026-11-02).
        """
        self.scans += 1
        if len(points) > MAX_POINTS:
            idx = np.linspace(0, len(points) - 1, MAX_POINTS).astype(int)
            pts = points[idx]
        else:
            pts = points
        tm = t_sensor if t_sensor is not None and t_sensor > 0 else t
        dt = 0.0 if self.last_t is None else max(0.0, min(1.0, tm - self.last_t))
        guess = self.pose + self.velocity * dt
        guess[2] = wrap(guess[2])
        accepted = False
        if self.last_update_pose is None:
            new_pose, score, accepted = self.pose.copy(), 1.0, True       # first scan defines the frame
        elif len(pts) < 20:
            new_pose, score = self.pose.copy(), 0.0
        elif self.scans <= WARMUP_SCANS:
            new_pose, score = self.matcher.match(self.levels, pts, guess)
            accepted = True                                               # map still thin: no gate yet
        else:
            # hypotheses: constant-velocity prediction, then "did not move", then a wide search
            best = None
            for start, wide in ((guess, False), (self.pose, False), (self.pose, True)):
                ok, sc, pose = self._hypothesis(pts, start, dt, wide)
                if ok and (best is None or sc > best[0]):
                    best = (sc, pose)
                if best is not None and best[0] >= GOOD_SCORE:
                    break
            if best is not None:
                score, new_pose, accepted = best[0], best[1], True
            else:
                new_pose, score = self.pose.copy(), 0.0                   # hold the last good pose
        new_pose[2] = wrap(new_pose[2])
        if accepted and dt >= 0.05 and self.last_t is not None:
            v = (new_pose - self.pose) / dt
            v[2] = wrap(new_pose[2] - self.pose[2]) / dt
            sp = math.hypot(v[0], v[1])
            if sp > V_MAX:
                v[:2] *= V_MAX / sp
            v[2] = max(-W_MAX, min(W_MAX, v[2]))
            self.velocity = 0.6 * v + 0.4 * self.velocity
        elif not accepted:
            self.velocity = np.zeros(3)
        self.pose = new_pose
        self.last_t = tm
        self.last_score = score
        if accepted:
            self.accepted += 1
            self.reject_streak = 0
        else:
            self.rejected += 1
            self.reject_streak = getattr(self, "reject_streak", 0) + 1
        good = accepted and (self.scans <= WARMUP_SCANS or score >= MAP_UPDATE_MIN_SCORE)
        if good and (self.last_update_pose is None or self.scans <= WARMUP_SCANS
                     or math.hypot(*(self.pose[:2] - self.last_update_pose[:2])) >= UPDATE_MIN_MOVE_M
                     or abs(wrap(self.pose[2] - self.last_update_pose[2])) >= UPDATE_MIN_TURN_RAD):
            world = transform(points, self.pose)
            for lev in self.levels:
                lev.update(self.pose[:2], world)
            self.last_update_pose = self.pose.copy()
        return {"x": float(self.pose[0]), "y": float(self.pose[1]), "yaw": float(self.pose[2]),
                "score": float(score), "accepted": bool(accepted), "lost": self.lost,
                "reject_streak": int(getattr(self, "reject_streak", 0))}

    @property
    def lost(self) -> bool:
        return getattr(self, "reject_streak", 0) >= LOST_AFTER

    def export_level(self) -> GridLevel:
        return min(self.levels, key=lambda l: abs(l.res - EXPORT_LEVEL_RES))


# ------------------------------------------------------------- process ----

RUNNING = True


def _stop(signum, frame):
    global RUNNING
    RUNNING = False


def _atomic_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    tmp.replace(path)


def _atomic_npy(path: Path, arr: np.ndarray) -> None:
    tmp = path.with_name(path.stem + ".tmp.npy")
    np.save(tmp, arr)
    tmp.replace(path)


def main() -> int:
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    from src.core.lidar_gazebo_backend import create_backend, LidarStaleError

    for p in (STATE_FILE, GRID_FILE, TRAJ_FILE):
        p.parent.mkdir(parents=True, exist_ok=True)
    lidar = create_backend()
    slam = SlamV6()
    print("=" * 80, flush=True)
    print(f"{VERSION}  scan-to-map SLAM  | LiDAR {lidar.name} | levels {LEVEL_RESOLUTIONS} m", flush=True)
    print(f"state {STATE_FILE}  grid {GRID_FILE}  | flight control: NONE", flush=True)
    print("=" * 80, flush=True)
    traj = open(TRAJ_FILE, "w", newline="")
    tw = csv.writer(traj)
    tw.writerow(["wall_time", "scan", "x_m", "y_m", "yaw_deg", "score", "accepted"])
    t_start = time.time()
    last_export = 0.0
    last_print = 0.0
    try:
        while RUNNING:
            try:
                scan = lidar.next_scan()
            except LidarStaleError as exc:
                print(f"[SLAM6] LiDAR stale: {exc}", flush=True)
                continue
            t = float(getattr(scan, "timestamp", time.time()))
            pts = scan_to_points(scan)
            r = slam.process(pts, t, getattr(scan, "sensor_time", None))
            now = time.time()
            fps = slam.scans / max(now - t_start, 1e-3)
            lev = slam.export_level()
            state = {
                "protocol": VERSION, "timestamp": now, "frame": slam.scans,
                "lidar_source": lidar.name, "scan_timestamp": t,
                "pose": {"x_m": r["x"], "y_m": r["y"], "yaw_deg": math.degrees(r["yaw"])},
                "frame_convention": "start pose: x forward, y left, yaw CCW",
                "match": {"score": round(r["score"], 3), "accepted": r["accepted"],
                          "reject_streak": r["reject_streak"]},
                "status": "LOST" if r["lost"] else "TRACKING",
                "statistics": {"processing_fps": round(fps, 2), "scans": slam.scans,
                               "accepted": slam.accepted, "rejected": slam.rejected, "points": int(len(pts))},
                "map": {"file": str(GRID_FILE), "resolution_m": lev.res, "origin_m": [lev.origin, lev.origin],
                        "shape": [lev.n, lev.n], "exported": last_export},
                "flight_control": "DISABLED",
            }
            if now - last_export >= MAP_EXPORT_SEC:
                _atomic_npy(GRID_FILE, lev.export_int8())
                last_export = now
                state["map"]["exported"] = now
                traj.flush()
            _atomic_json(STATE_FILE, state)
            tw.writerow([f"{now:.3f}", slam.scans, f"{r['x']:.4f}", f"{r['y']:.4f}",
                         f"{math.degrees(r['yaw']):.2f}", f"{r['score']:.3f}", int(r["accepted"])])
            if now - last_print >= 2.0:
                last_print = now
                print(f"[SLAM6] scan {slam.scans} pose x={r['x']:+.2f} y={r['y']:+.2f} "
                      f"yaw={math.degrees(r['yaw']):+.1f} score={r['score']:.2f} "
                      f"acc={slam.accepted} rej={slam.rejected} {fps:.1f} Hz", flush=True)
    finally:
        traj.close()
        try:
            _atomic_npy(GRID_FILE, slam.export_level().export_int8())
        except Exception as exc:                       # pragma: no cover
            print(f"[SLAM6] final map export failed: {exc}", flush=True)
        print(f"[SLAM6] stopped after {slam.scans} scans (accepted {slam.accepted}, "
              f"rejected {slam.rejected})", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
