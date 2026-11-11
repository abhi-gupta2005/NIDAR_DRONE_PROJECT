#!/usr/bin/env python3
"""
Survivor mapper (2026-11-02): camera detections + LiDAR range + SLAM pose ->
survivor positions in the arena and their grid cells (e.g. "E3").

Per camera frame (V18 JSON):
  * camera bearing from the bounding box with a PINHOLE model (V18 uses a
    linear pixel->angle model; up to ~2 deg off at 25 deg);
  * range, two independent ways:
      LIDAR  - V18's fused range (LiDAR cluster matched to the camera bearing);
      GROUND - from the box's bottom edge: feet/body on a flat floor, camera at
               a known height (needs the bottom edge visible, not clipped);
  * the LiDAR value is rejected (F22: a low survivor below the LiDAR plane,
    fused with the wall behind it) when
      - both ranges exist and disagree by > max(0.5 m, 35 %), or
      - the box's top edge, placed at the LiDAR range, is below the LiDAR
        plane: then the camera object cannot be what the LiDAR beam hit.
    With only one source, that one is used.
  * body-frame point -> SLAM frame (pose at that time) -> arena frame (start pose).
Over time, measurements are clustered into survivors; a survivor is CONFIRMED
after >= MIN_OBS observations spread over >= MIN_SPAN_S seconds.

Keeping the same ID for the same person (2026-11-02f):
  * a camera track (the Windows stable ID) that fed survivor S2 in the last 3 s keeps
    feeding S2 even if its position estimate wanders up to 2 m;
  * entries that end up closer than 0.75 m are merged; the OLDER label survives and
    the other is reported as an alias ("S7 -> S5");
  * measurements outside the arena (+0.25 m) are rejected: a survivor cannot be there,
    so it means a bad pose or a bad range, not a new person.
Re-identifying a person seen again much later relies on the SLAM pose being right.

Robust position (2026-11-02g, after the Gazebo run where one person was confirmed 1 m off,
in the wrong cell, with a 0.88 m spread):
  * a survivor's position is no longer the plain mean of every measurement: it is the
    median, refined by the mean of the measurements within 0.5 m of it (inliers); wrong
    LiDAR returns, half-body boxes and wandering track estimates become outliers instead
    of pulling the dot into the next cell;
  * an entry is CONFIRMED only when >= 5 INLIERS over >= 0.8 s agree within 0.45 m
    (spread); a wide, disagreeing cluster stays a candidate until it settles;
  * every measurement (also the rejected ones) is kept in a journal so the mission
    server can write outputs/mission/measurements_<time>.csv for offline replay.

Frames: body x forward / y left; NIDAR bearings are positive to the RIGHT.
Grid labels follow gazebo/layouts/*.json: column letter (west->east) + row
number (south->north), cell c covers [c*pitch + wall/2, (c+1)*pitch + wall/2).
No flight control here: this only produces a list of positions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

SURVIVOR_CLASSES = ("person",)
LIDAR_CENTRE_OFFSET_M = 0.20      # LiDAR measures the body surface; centre is further away
GROUND_CENTRE_OFFSET_M = 0.10     # box bottom = feet of a standing person
GROUND_CENTRE_OFFSET_LOW_M = 0.35 # box wider than tall (lying/sitting): bottom = near edge of the body
LIDAR_ABOVE_CAMERA_M = 0.18       # x500_nidar_airmouse / nidar_rig: LiDAR plane 0.18 m above the camera
TOP_BELOW_PLANE_MARGIN_M = 0.10
CLIP_MARGIN_PX = 3.0
SKIP_EDGE_CLIPPED = True          # box cut by the left/right image edge: its centre is not the body centre
BODY_HALF_WIDTH_M = 0.22          # standing person, for the centre of an edge-clipped box (needs LiDAR range)
F22_ABS_M, F22_REL = 0.5, 0.35    # LiDAR vs ground range disagreement that rejects the LiDAR value
GROUND_MIN_M, GROUND_MAX_M = 0.4, 8.0
LIDAR_MIN_M, LIDAR_MAX_M = 0.2, 10.0
ASSOC_GATE_M = 0.8
TRACK_GATE_M = 2.0                # same camera track (Windows stable ID) seen < TRACK_MEMORY_S ago
TRACK_MEMORY_S = 3.0
MERGE_DIST_M = 0.75               # two survivor entries closer than this are the same person
ARENA_MARGIN_M = -0.05            # survivors are INSIDE the outer walls (inner face at 0.05 m): a point in or
                                  # behind an outer wall is a range error (LiDAR hit the wall behind a low body)
MIN_OBS = 5                       # inliers needed to confirm
MIN_SPAN_S = 0.8
INLIER_RADIUS_M = 0.5             # measurements farther than this from the robust centre are outliers
MAX_CONFIRM_SPREAD_M = 0.45       # inlier spread (RMS distance) allowed for a confirmation
MIN_INLIER_RATIO = 0.6            # and most of the entry's measurements must agree (no 50/50 split)
MAX_POINTS = 400                  # measurements kept per survivor (newest)
RAY_TOL_M = 0.30                  # a measurement "looks through" another entry if its camera ray passes this close
GHOST_MAX_DIST_M = 2.5            # only entries this close can be range ghosts of each other
GHOST_INDEPENDENT_MAX = 0.30      # <= 30 % of an entry's views see it apart from the other entry -> ghost
MIN_CONFIDENCE = 0.30


@dataclass
class CameraModel:
    width: float = 640.0
    height: float = 480.0
    hfov_deg: float = 70.0
    height_m: float = 0.82          # camera height above the floor

    @property
    def f(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg / 2.0))

    def bearing_deg(self, u: float) -> float:
        """Pinhole bearing, positive = right of the optical axis."""
        return math.degrees(math.atan2(u - self.width / 2.0, self.f))

    def ground_range(self, u: float, v_bottom: float) -> Optional[Tuple[float, float]]:
        """(range_m, bearing_deg) of the floor point under the box's bottom-centre, or None."""
        dv = v_bottom - self.height / 2.0
        if dv <= 2.0 or v_bottom >= self.height - CLIP_MARGIN_PX:
            return None                                    # at/above the horizon, or feet cut off
        fwd = self.height_m * self.f / dv
        lat = fwd * (u - self.width / 2.0) / self.f
        return math.hypot(fwd, lat), math.degrees(math.atan2(lat, fwd))


@dataclass
class Grid:
    pitch_m: float = 1.05
    wall_m: float = 0.05
    cells: int = 14
    bounded: bool = True          # False: arena unknown (no layout) -- no cell labels, no "outside" rejection

    def label(self, x: float, y: float) -> str:
        if not self.bounded:
            return "--"
        c = math.floor((x - self.wall_m / 2.0) / self.pitch_m)
        r = math.floor((y - self.wall_m / 2.0) / self.pitch_m)
        if not (0 <= c < self.cells and 0 <= r < self.cells):
            return "OUT"
        return f"{chr(ord('A') + c)}{r + 1}"

    @classmethod
    def from_layout(cls, layout: dict) -> "Grid":
        g = layout["grid"]
        return cls(float(g["pitch_m"]), float(g["wall_m"]), int(g["cells"]))


def compose(start: Sequence[float], local: Sequence[float]) -> Tuple[float, float, float]:
    """start (arena pose of the SLAM origin) o local (SLAM pose) -> arena pose."""
    sx, sy, sth = start
    x, y, th = local
    c, s = math.cos(sth), math.sin(sth)
    return sx + c * x - s * y, sy + s * x + c * y, (sth + th + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Measurement:
    t: float
    x: float
    y: float
    method: str               # LIDAR | GROUND
    range_m: float
    bearing_deg: float
    confidence: float
    lidar_rejected: bool = False
    camera_id: Optional[int] = None
    row: Optional[dict] = None            # journal row (inputs of this measurement)
    pose_x: float = 0.0                   # drone position when it was taken (arena frame)
    pose_y: float = 0.0


@dataclass
class Survivor:
    """One person: all its measurements, a robust centre and its inlier statistics."""
    sx: float = 0.0                       # kept for compatibility (unused)
    sy: float = 0.0
    n: int = 0                            # all observations ever added
    first_t: float = 0.0
    last_t: float = 0.0
    methods: Dict[str, int] = field(default_factory=dict)
    best_conf: float = 0.0
    label: Optional[str] = None           # S1, S2 ... once confirmed
    aliases: List[str] = field(default_factory=list)   # labels merged into this one
    pts: List[Tuple[float, float, float, float, float]] = field(default_factory=list)  # (t, x, y, drone_x, drone_y)
    _cx: float = 0.0
    _cy: float = 0.0
    _spread: float = 0.0
    _n_in: int = 0
    _span_in: float = 0.0

    @property
    def x(self) -> float:
        return self._cx

    @property
    def y(self) -> float:
        return self._cy

    @property
    def std(self) -> float:
        """Spread of the inliers (RMS distance from the centre), metres."""
        return self._spread

    @property
    def inliers(self) -> int:
        return self._n_in

    @property
    def inlier_span(self) -> float:
        return self._span_in

    def _refit(self) -> None:
        xs = sorted(p[1] for p in self.pts)
        ys = sorted(p[2] for p in self.pts)
        k = len(xs)
        cx = 0.5 * (xs[(k - 1) // 2] + xs[k // 2])
        cy = 0.5 * (ys[(k - 1) // 2] + ys[k // 2])
        inl = []
        for _ in range(3):
            near = [p for p in self.pts if math.hypot(p[1] - cx, p[2] - cy) <= INLIER_RADIUS_M]
            if not near:
                break                                     # no consensus around the median
            inl = near
            cx = sum(p[1] for p in inl) / len(inl)
            cy = sum(p[2] for p in inl) / len(inl)
        self._cx, self._cy = cx, cy
        self._n_in = len(inl)
        ref = inl or self.pts
        self._spread = math.sqrt(sum((p[1] - cx) ** 2 + (p[2] - cy) ** 2 for p in ref) / len(ref))
        ts = [p[0] for p in inl]
        self._span_in = (max(ts) - min(ts)) if ts else 0.0

    def add(self, m: Measurement) -> None:
        if self.n == 0:
            self.first_t = m.t
        self.n += 1
        self.pts.append((m.t, m.x, m.y, m.pose_x, m.pose_y))
        if len(self.pts) > MAX_POINTS:
            del self.pts[0]
        self.last_t = m.t
        self.methods[m.method] = self.methods.get(m.method, 0) + 1
        self.best_conf = max(self.best_conf, m.confidence)
        self._refit()

    def inlier_points(self):
        return [p for p in self.pts if math.hypot(p[1] - self._cx, p[2] - self._cy) <= INLIER_RADIUS_M]

    def independent_fraction(self, other: "Survivor") -> float:
        """Share of this entry's inlier views whose camera ray does NOT pass through `other`.
        A range error (wrong LiDAR return, wrong floor point) moves a measurement ALONG the ray:
        an entry made only of such views is a ghost of the person on that ray."""
        pts = self.inlier_points()[-80:]
        if not pts:
            return 1.0
        indep = 0
        for _, mx, my, px, py in pts:
            dx, dy = mx - px, my - py
            n = math.hypot(dx, dy)
            if n < 1e-6:
                indep += 1
                continue
            ux, uy = dx / n, dy / n
            cx, cy = other.x - px, other.y - py
            along = cx * ux + cy * uy
            perp = abs(cx * uy - cy * ux)
            if not (along > 0.2 and perp <= RAY_TOL_M):
                indep += 1
        return indep / len(pts)

    def ready_to_confirm(self) -> bool:
        return (self._n_in >= MIN_OBS and self._span_in >= MIN_SPAN_S
                and self._spread <= MAX_CONFIRM_SPREAD_M
                and self._n_in >= MIN_INLIER_RATIO * len(self.pts))

    @property
    def confirmed(self) -> bool:
        return self.label is not None

    def absorb(self, other: "Survivor", take_points: bool = True) -> None:
        """Merge `other` into this entry. take_points=False for a range ghost: its (wrong)
        positions must not pull this entry's centre."""
        self.n += other.n
        if take_points:
            self.pts = sorted(self.pts + other.pts)[-MAX_POINTS:]
        self.first_t = min(self.first_t, other.first_t)
        self.last_t = max(self.last_t, other.last_t)
        for k, v in other.methods.items():
            self.methods[k] = self.methods.get(k, 0) + v
        self.best_conf = max(self.best_conf, other.best_conf)
        self.aliases += other.aliases + ([other.label] if other.label else [])
        self._refit()


def _label_rank(sv: Survivor):
    """Older/confirmed entries win a merge: labelled before unlabelled, then lower number, then older."""
    num = int(sv.label[1:]) if sv.label else 10 ** 9
    return (sv.label is None, num, sv.first_t)


class SurvivorMapper:
    def __init__(self, camera: CameraModel = None, grid: Grid = None,
                 start_pose: Sequence[float] = (0.0, 0.0, 0.0), classes: Iterable[str] = SURVIVOR_CLASSES):
        self.camera = camera or CameraModel()
        self.grid = grid or Grid()
        self.start_pose = tuple(start_pose)
        self.classes = set(classes)
        self.survivors: List[Survivor] = []
        self.next_label = 1
        self.measurements = 0
        self.lidar_rejected = 0
        self.outside_rejected = 0
        self.edge_clipped = 0
        self.ghosts = 0                                  # entries absorbed as range ghosts of another
        self.merges: List[Tuple[str, str]] = []          # (absorbed label, kept label)
        self.last_measurements: List[Measurement] = []
        self._tracks: Dict[object, Tuple[Survivor, float]] = {}
        self.journal: List[dict] = []                       # every measurement + outcome (mission server drains it)
        self.max_journal = 5000

    def inside_arena(self, x: float, y: float) -> bool:
        if not self.grid.bounded:
            return True                                       # arena unknown (2026-11-09): nothing to clip to
        size = self.grid.cells * self.grid.pitch_m + self.grid.wall_m
        m = ARENA_MARGIN_M
        return -m <= x <= size + m and -m <= y <= size + m

    # ---------------------------------------------------------- per frame --
    def measure(self, camera_objects: Sequence[dict], fused_objects: Sequence[dict],
                slam_pose: Sequence[float], t: float) -> List[Measurement]:
        """One V18 camera frame -> measurements (arena frame)."""
        fused_by_cam = {}
        for f in fused_objects or []:
            if f.get("camera_id") is not None:
                fused_by_cam[f.get("camera_id")] = f
        pose = compose(self.start_pose, slam_pose)
        out: List[Measurement] = []
        for obj in camera_objects or []:
            if str(obj.get("class", "")).lower() not in self.classes:
                continue
            if obj.get("current_observation") is False or obj.get("spatial_status") == "PREDICTED":
                continue                                    # Windows tracker's predicted box, not seen now
            try:
                conf = float(obj.get("confidence", 0.0))
                x1, y1, x2, y2 = [float(v) for v in obj["bbox"]]
            except (KeyError, TypeError, ValueError):
                continue
            if conf < MIN_CONFIDENCE:
                continue
            clip_l = x1 <= CLIP_MARGIN_PX
            clip_r = x2 >= self.camera.width - CLIP_MARGIN_PX
            u = 0.5 * (x1 + x2)
            if SKIP_EDGE_CLIPPED and (clip_l or clip_r):
                # The box centre is pulled towards the image centre. A standing body (tall box)
                # with a LiDAR range: place the centre one body half-width inside the visible
                # edge. Otherwise the bearing is unknown: skip the frame.
                f_l = fused_by_cam.get(obj.get("id"))
                try:
                    r_l = float(f_l["range_m"]) if f_l is not None else None
                except (KeyError, TypeError, ValueError):
                    r_l = None
                if clip_l != clip_r and (y2 - y1) > (x2 - x1) and r_l and LIDAR_MIN_M <= r_l <= LIDAR_MAX_M:
                    half_px = self.camera.f * BODY_HALF_WIDTH_M / (r_l + LIDAR_CENTRE_OFFSET_M)
                    u = (x2 - half_px) if clip_l else (x1 + half_px)
                else:
                    self.edge_clipped += 1
                    self._log({"t": round(t, 3), "camera_id": obj.get("id"), "conf": round(conf, 3),
                               "bbox": "%.0f %.0f %.0f %.0f" % (x1, y1, x2, y2), "outcome": "EDGE_CLIPPED"})
                    continue
            cam_brg = self.camera.bearing_deg(u)
            ground = self.camera.ground_range(u, y2)
            lidar = None
            f = fused_by_cam.get(obj.get("id"))
            if f is not None:
                try:
                    lr = float(f["range_m"])
                    if LIDAR_MIN_M <= lr <= LIDAR_MAX_M:
                        lidar = lr
                except (KeyError, TypeError, ValueError):
                    pass
            g = ground[0] if ground and GROUND_MIN_M <= ground[0] <= GROUND_MAX_M else None
            rejected = False
            if lidar is not None and g is not None and abs(lidar - g) > max(F22_ABS_M, F22_REL * g):
                rejected = True                              # F22: LiDAR hit something else (wall behind)
            if lidar is not None and not rejected:
                top_z = self.camera.height_m - (y1 - self.camera.height / 2.0) * lidar / self.camera.f
                if y1 > 1.0 and top_z < self.camera.height_m + LIDAR_ABOVE_CAMERA_M - TOP_BELOW_PLANE_MARGIN_M:
                    rejected = True                          # object entirely below the LiDAR plane
            if rejected:
                lidar = None
                self.lidar_rejected += 1
            if lidar is not None:
                rng, method = lidar + LIDAR_CENTRE_OFFSET_M, "LIDAR"
            elif g is not None:
                low = (x2 - x1) > (y2 - y1)
                rng = g + (GROUND_CENTRE_OFFSET_LOW_M if low else GROUND_CENTRE_OFFSET_M)
                method = "GROUND"
            else:
                continue
            b = math.radians(cam_brg)
            bx, by = rng * math.cos(b), -rng * math.sin(b)  # bearing right-positive -> y left
            c, s = math.cos(pose[2]), math.sin(pose[2])
            mx, my = pose[0] + c * bx - s * by, pose[1] + s * bx + c * by
            row = {"t": round(t, 3), "pose_x": round(pose[0], 3), "pose_y": round(pose[1], 3),
                   "pose_yaw_deg": round(math.degrees(pose[2]), 2), "camera_id": obj.get("id"),
                   "conf": round(conf, 3), "bbox": "%.0f %.0f %.0f %.0f" % (x1, y1, x2, y2),
                   "lidar_m": None if f is None else f.get("range_m"), "ground_m": None if g is None else round(g, 3),
                   "lidar_rejected": rejected, "method": method, "range_m": round(rng, 3),
                   "bearing_deg": round(cam_brg, 2), "x": round(mx, 3), "y": round(my, 3)}
            if not self.inside_arena(mx, my):
                self.outside_rejected += 1
                self._log(dict(row, outcome="OUTSIDE_ARENA"))
                continue
            out.append(Measurement(t, mx, my, method, rng, cam_brg, conf, rejected, obj.get("id"), row,
                                   pose[0], pose[1]))
        self.last_measurements = out
        return out

    def update(self, camera_objects, fused_objects, slam_pose, t: float) -> List[Measurement]:
        ms = self.measure(camera_objects, fused_objects, slam_pose, t)
        for m in ms:
            self.measurements += 1
            best = None
            tr = self._tracks.get(m.camera_id) if m.camera_id is not None else None
            if tr is not None and t - tr[1] <= TRACK_MEMORY_S and tr[0] in self.survivors \
                    and math.hypot(tr[0].x - m.x, tr[0].y - m.y) <= TRACK_GATE_M:
                best = tr[0]                                  # same camera track -> same survivor
            if best is None:
                bd = ASSOC_GATE_M
                for sv in self.survivors:
                    d = math.hypot(sv.x - m.x, sv.y - m.y)
                    if d < bd:
                        best, bd = sv, d
            via = "track" if best is not None and tr is not None and best is tr[0] else "nearest"
            if best is None:
                best = Survivor()
                self.survivors.append(best)
                via = "new"
            best.add(m)
            if m.camera_id is not None:
                self._tracks[m.camera_id] = (best, t)
            if best.label is None and best.ready_to_confirm():
                best.label = f"S{self.next_label}"
                self.next_label += 1
            d = math.hypot(best.x - m.x, best.y - m.y)
            self._log(dict(getattr(m, "row", {}), outcome="INLIER" if d <= INLIER_RADIUS_M else "OUTLIER",
                           survivor=best.label or f"cand@{best.x:.2f},{best.y:.2f}", assoc=via,
                           dist_to_centre=round(d, 3)))
        self._merge_close()
        return ms

    def _log(self, row: dict) -> None:
        self.journal.append(row)
        if len(self.journal) > self.max_journal:
            del self.journal[: len(self.journal) - self.max_journal]

    def drain_journal(self) -> List[dict]:
        rows, self.journal = self.journal, []
        return rows

    def _ghost_pair(self, a: Survivor, b: Survivor):
        """(keep, drop) if one entry is a range ghost of the other, else None."""
        d = math.hypot(a.x - b.x, a.y - b.y)
        if d < MERGE_DIST_M or d > GHOST_MAX_DIST_M:
            return None
        fa, fb = a.independent_fraction(b), b.independent_fraction(a)
        if min(fa, fb) > GHOST_INDEPENDENT_MAX:
            return None                                  # both seen apart from each other: two people
        # the entry that is mostly seen "through" the other is the ghost; if both are, the
        # one with more independent support (then more inliers) is the person
        if (fa, a.inliers) >= (fb, b.inliers):
            return a, b
        return b, a

    def _merge_close(self) -> None:
        merged = True
        while merged:
            merged = False
            for i in range(len(self.survivors)):
                for j in range(i + 1, len(self.survivors)):
                    a, b = self.survivors[i], self.survivors[j]
                    if math.hypot(a.x - b.x, a.y - b.y) >= MERGE_DIST_M:
                        pair = self._ghost_pair(a, b)
                        if pair is None:
                            continue
                        keep, drop = pair
                        self.ghosts += 1
                        ghost = True
                    else:
                        keep, drop = (a, b) if _label_rank(a) <= _label_rank(b) else (b, a)
                        ghost = False
                    if ghost and drop.label and not keep.label:
                        keep.label, drop.label = drop.label, None     # the person keeps the ID it was given
                    keep.absorb(drop, take_points=not ghost)
                    if drop.label:
                        self.merges.append((drop.label, keep.label or "?"))
                    if keep.label is None and keep.ready_to_confirm():
                        keep.label = f"S{self.next_label}"
                        self.next_label += 1
                    self.survivors.remove(drop)
                    for k, (sv, tt) in list(self._tracks.items()):
                        if sv is drop:
                            self._tracks[k] = (keep, tt)
                    merged = True
                    break
                if merged:
                    break

    # ------------------------------------------------------------ report --
    def report(self, include_candidates: bool = True) -> List[dict]:
        rows = []
        for sv in sorted(self.survivors, key=lambda s: (s.label is None, s.first_t)):
            if not include_candidates and not sv.confirmed:
                continue
            rows.append({
                "id": sv.label or "?", "status": "CONFIRMED" if sv.confirmed else "CANDIDATE",
                "grid": self.grid.label(sv.x, sv.y), "x_m": round(sv.x, 3), "y_m": round(sv.y, 3),
                "std_m": round(sv.std, 3), "observations": sv.n, "inliers": sv.inliers, "methods": dict(sv.methods),
                "first_seen": sv.first_t, "last_seen": sv.last_t, "confidence": round(sv.best_conf, 3),
                "aliases": list(sv.aliases),
            })
        return rows
