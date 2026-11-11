#!/usr/bin/env python3
"""
NIDAR co-simulated world (camera + LiDAR from ONE scene).

Why this exists
---------------
The existing STL-19P simulator (nidar_lidar_simulator.py) generates LiDAR
scans of a fixed room, and the Windows webcam generates camera packets of a
completely different scene.  V18 fusion therefore could only ever be
exercised on data that does not describe the same objects.

This module generates BOTH data products from a single 2-D world so that the
real receiver / V18 / SLAM / bridge code can be tested against ground truth:

  * NIDAR_PERCEPTION_V1 camera packets (same schema as nidar_v11_windows.py)
  * LidarScan objects (same dataclass as src/core/lidar_interface.py)

Frame conventions (documented here because the project never had them written
down -- see docs/NIDAR_PROJECT_REGISTER.md, "Coordinate frames"):

  World frame   : x east, y north, yaw counter-clockwise from +x (radians).
  Body frame    : forward = direction of yaw, right = forward rotated -90 deg.
  Bearing       : angle from body-forward, POSITIVE TO THE RIGHT (clockwise seen
                  from above), degrees.  This is the convention the camera code
                  in V18 uses (object right of image centre -> positive bearing).
  LiDAR angle   : identical to bearing -- 0 deg = forward, increasing clockwise.
                  With this convention lidar_obstacle_detector.py reports
                  bearing_deg == true bearing (mod 360) and V18's camera/LiDAR
                  gate works as intended.  On real hardware the STL-19P 0-degree
                  mark and rotation direction MUST be calibrated to this.

Nothing in this file modifies or depends on the camera/LiDAR hardware code
paths; it only produces data in the formats the existing code consumes.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from src.core.lidar_interface import LidarScan


# ---------------------------------------------------------------------------
# Camera model (matches the constants hard-coded in V18 / the Windows script)
# ---------------------------------------------------------------------------

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_HFOV_DEG = 70.0          # V18: CAMERA_HFOV_DEG
CAMERA_SOURCE = "nidar_world_sim"

# LiDAR model (matches nidar_lidar_simulator.py)
LIDAR_SCAN_HZ = 10.0
LIDAR_SAMPLES_PER_SCAN = 500
LIDAR_MAX_RANGE_M = 12.0
LIDAR_MIN_RANGE_M = 0.03


def focal_length_px(width: int = CAMERA_WIDTH, hfov_deg: float = CAMERA_HFOV_DEG) -> float:
    """Pinhole focal length for a given horizontal field of view."""
    return (width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))


# ---------------------------------------------------------------------------
# World description
# ---------------------------------------------------------------------------

@dataclass
class WorldObject:
    """A cylinder-shaped object (people, pillars, boxes...)."""

    name: str
    label: str                       # camera class label ("person", "chair"...)
    radius_m: float
    height_m: float
    # position(t) -> (x, y) in the world frame.  Static objects use a constant.
    position: Callable[[float], Tuple[float, float]]
    camera_visible: bool = True      # False = LiDAR sees it, camera never does
    lidar_visible: bool = True       # False = camera sees it, LiDAR never does
    stable_id: int = 0               # assigned by World

    def at(self, t: float) -> Tuple[float, float]:
        return self.position(t)


@dataclass
class SensorPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0                 # radians, CCW from world +x

    def forward(self) -> Tuple[float, float]:
        return math.cos(self.yaw), math.sin(self.yaw)

    def right(self) -> Tuple[float, float]:
        # forward rotated by -90 degrees
        return math.sin(self.yaw), -math.cos(self.yaw)


@dataclass
class CameraNoise:
    center_sigma_px: float = 3.0
    size_sigma_ratio: float = 0.04
    dropout_probability: float = 0.0     # probability a visible object is missed
    confidence_range: Tuple[float, float] = (0.45, 0.90)


@dataclass
class LidarNoise:
    noise_scale: float = 1.0             # same model as nidar_lidar_simulator
    invalid_probability: float = 0.008


@dataclass
class Room:
    """Axis-aligned rectangular room [0, width] x [0, height]."""

    width: float
    height: float


def static(x: float, y: float) -> Callable[[float], Tuple[float, float]]:
    return lambda t: (x, y)


def linear_path(start: Tuple[float, float], end: Tuple[float, float],
                duration_s: float, loop: bool = False) -> Callable[[float], Tuple[float, float]]:
    """Move from start to end in duration_s seconds (then hold, or loop back and forth)."""

    def pos(t: float) -> Tuple[float, float]:
        if duration_s <= 0:
            return end
        if loop:
            phase = (t % (2 * duration_s)) / duration_s
            s = phase if phase <= 1.0 else 2.0 - phase
        else:
            s = max(0.0, min(1.0, t / duration_s))
        return (start[0] + (end[0] - start[0]) * s,
                start[1] + (end[1] - start[1]) * s)

    return pos


# ---------------------------------------------------------------------------
# Ground-truth helpers
# ---------------------------------------------------------------------------

@dataclass
class Observation:
    """Ground truth of one object relative to the sensor at time t."""

    obj: WorldObject
    range_m: float
    bearing_deg: float               # positive = right of forward
    forward_m: float
    right_m: float
    in_camera_fov: bool


def relative_observation(pose: SensorPose, obj: WorldObject, t: float,
                         hfov_deg: float = CAMERA_HFOV_DEG) -> Observation:
    ox, oy = obj.at(t)
    dx, dy = ox - pose.x, oy - pose.y
    fx, fy = pose.forward()
    rx, ry = pose.right()
    forward = dx * fx + dy * fy
    right = dx * rx + dy * ry
    rng = math.hypot(dx, dy)
    bearing = math.degrees(math.atan2(right, forward))
    half = hfov_deg / 2.0
    in_fov = forward > 0.05 and abs(bearing) <= half
    return Observation(obj, rng, bearing, forward, right, in_fov)


# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------

class World:
    """
    A 2-D scene with a (possibly moving) sensor and cylinder objects.

    Usage:
        world = World(room=Room(8, 6), seed=1)
        world.add(WorldObject("Person", "person", 0.28, 1.7, linear_path(...)))
        packet = world.camera_packet(frame=1, t=0.0)
        scan   = world.lidar_scan(scan_id=0, t=0.0)
    """

    def __init__(self, room: Optional[Room] = None, seed: Optional[int] = 0,
                 camera_noise: Optional[CameraNoise] = None,
                 lidar_noise: Optional[LidarNoise] = None,
                 camera_height_m: float = 1.2):
        self.room = room
        self.objects: List[WorldObject] = []
        self.pose = SensorPose()
        self.pose_fn: Optional[Callable[[float], SensorPose]] = None
        self.camera_noise = camera_noise or CameraNoise()
        self.lidar_noise = lidar_noise or LidarNoise()
        self.camera_height_m = camera_height_m
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        self._next_id = 1

    # -- scene construction -------------------------------------------------

    def add(self, obj: WorldObject) -> WorldObject:
        obj.stable_id = self._next_id
        self._next_id += 1
        self.objects.append(obj)
        return obj

    def set_sensor_path(self, pose_fn: Callable[[float], SensorPose]) -> None:
        """Make the sensor move: pose_fn(t) -> SensorPose."""
        self.pose_fn = pose_fn

    def sensor_pose(self, t: float) -> SensorPose:
        if self.pose_fn is not None:
            self.pose = self.pose_fn(t)
        return self.pose

    # -- ground truth -------------------------------------------------------

    def truth(self, t: float) -> List[Observation]:
        pose = self.sensor_pose(t)
        return [relative_observation(pose, o, t) for o in self.objects]

    def camera_visible_observations(self, t: float) -> List[Observation]:
        """
        Ground-truth objects the camera can actually see at time t: inside the
        horizontal FOV, camera_visible, not behind the sensor and not occluded
        (an object whose bearing lies inside the angular extent of a nearer
        object is hidden).  Used both for packet generation and for scoring.
        """
        pose = self.sensor_pose(t)
        visible: List[Observation] = []
        for obj in self.objects:
            if not obj.camera_visible:
                continue
            ob = relative_observation(pose, obj, t)
            if ob.in_camera_fov and ob.range_m > obj.radius_m:
                visible.append(ob)

        visible.sort(key=lambda o: o.range_m)
        kept: List[Observation] = []
        for ob in visible:
            occluded = False
            for near in kept:
                half_width_deg = math.degrees(math.atan2(near.obj.radius_m, near.range_m))
                if abs(ob.bearing_deg - near.bearing_deg) < half_width_deg:
                    occluded = True
                    break
            if not occluded:
                kept.append(ob)
        return kept

    # -- camera -------------------------------------------------------------

    def camera_packet(self, frame: int, t: float, timestamp: Optional[float] = None) -> Dict:
        """
        Build a NIDAR_PERCEPTION_V1 packet exactly like nidar_v11_windows.py.

        Only objects inside the horizontal FOV and not occluded (simple
        angular-overlap occlusion: nearer object wins) are exported.  Every
        exported object is reported CONFIRMED / OBSERVED like a stable track.
        """
        f_px = focal_length_px()
        cx, cy = CAMERA_WIDTH / 2.0, CAMERA_HEIGHT / 2.0
        ts = timestamp if timestamp is not None else t

        kept = self.camera_visible_observations(t)

        objects = []
        for ob in kept:
            if self.rng.random() < self.camera_noise.dropout_probability:
                continue
            # Pinhole projection of the object centre and size.
            u = cx + f_px * math.tan(math.radians(ob.bearing_deg))
            # Vertical: camera horizontal, object centre at height_m/2.
            dz = self.camera_height_m - ob.obj.height_m / 2.0
            v = cy + f_px * dz / max(ob.forward_m, 0.05)
            w = f_px * (2.0 * ob.obj.radius_m) / max(ob.forward_m, 0.05)
            h = f_px * ob.obj.height_m / max(ob.forward_m, 0.05)

            u += self.rng.gauss(0.0, self.camera_noise.center_sigma_px)
            v += self.rng.gauss(0.0, self.camera_noise.center_sigma_px)
            w *= 1.0 + self.rng.gauss(0.0, self.camera_noise.size_sigma_ratio)
            h *= 1.0 + self.rng.gauss(0.0, self.camera_noise.size_sigma_ratio)

            x1 = max(0.0, u - w / 2.0)
            x2 = min(float(CAMERA_WIDTH - 1), u + w / 2.0)
            y1 = max(0.0, v - h / 2.0)
            y2 = min(float(CAMERA_HEIGHT - 1), v + h / 2.0)
            if x2 <= x1 or y2 <= y1:
                continue
            ccx, ccy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            lo, hi = self.camera_noise.confidence_range
            conf = round(self.rng.uniform(lo, hi), 4)

            objects.append(self._object_record(ob.obj, frame, ts, (x1, y1, x2, y2),
                                               (ccx, ccy), conf))

        return {
            "protocol": "NIDAR_PERCEPTION_V1",
            "frame": int(frame),
            "timestamp": float(ts),
            "camera": {
                "width": CAMERA_WIDTH,
                "height": CAMERA_HEIGHT,
                "source": CAMERA_SOURCE,
            },
            "objects": objects,
        }

    @staticmethod
    def _object_record(obj: WorldObject, frame: int, ts: float, box, center, conf) -> Dict:
        """Same keys and types as object_record() in nidar_v11_windows.py."""
        x1, y1, x2, y2 = box
        ccx, ccy = center
        return {
            "frame": int(frame),
            "timestamp": float(ts),
            "id": int(obj.stable_id),
            "class": obj.label,
            "state": "CONFIRMED",
            "confidence": float(conf),
            "tracker_id": int(obj.stable_id),
            "bbox": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
            "center": [round(ccx, 2), round(ccy, 2)],
            "predicted_bbox": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
            "predicted_center": [round(ccx, 2), round(ccy, 2)],
            "normalized_center": [round(ccx / CAMERA_WIDTH * 2.0 - 1.0, 4),
                                  round(ccy / CAMERA_HEIGHT * 2.0 - 1.0, 4)],
            "bbox_size": [round(x2 - x1, 2), round(y2 - y1, 2)],
            "velocity_px_frame": [0.0, 0.0],
            "motion_confidence": 1.0,
            "track_stability_score": 95.0,
            "observation_age_frames": 0,
            "prediction_horizon_frames": 0,
            "prediction_budget_frames": 0,
            "prediction_reliability": "MEASURED",
            "spatial_status": "OBSERVED",
            "confirmed": True,
            "age_frames": int(frame),
            "missed_frames": 0,
            "max_missed_frames": 0,
            "recoveries": 0,
            "tracker_switches": 0,
            "first_frame": 1,
            "last_seen_frame": int(frame),
            "state_since_frame": 1,
            "current_observation": True,
        }

    # -- LiDAR --------------------------------------------------------------

    def lidar_scan(self, scan_id: int, t: float, timestamp: Optional[float] = None) -> LidarScan:
        """
        Ray-cast a 360 degree scan from the sensor pose.

        angle_deg[k] is the LiDAR angle (0 = forward, clockwise positive) and
        distance_m[k] the measured range with the same noise model as
        nidar_lidar_simulator.make_scan().
        """
        pose = self.sensor_pose(t)
        angles = np.linspace(0.0, 360.0, LIDAR_SAMPLES_PER_SCAN, endpoint=False)
        fx, fy = pose.forward()
        rx, ry = pose.right()
        a = np.deg2rad(angles)
        # ray direction in the world frame = forward*cos(a) + right*sin(a)
        dir_x = fx * np.cos(a) + rx * np.sin(a)
        dir_y = fy * np.cos(a) + ry * np.sin(a)

        dist = np.full(angles.shape, LIDAR_MAX_RANGE_M, dtype=float)
        hits = np.array(["None"] * len(angles), dtype=object)

        if self.room is not None:
            wall_d = self._ray_room(pose, dir_x, dir_y)
            closer = wall_d < dist
            dist[closer] = wall_d[closer]
            hits[closer] = "Wall"

        for obj in self.objects:
            if not obj.lidar_visible:
                continue
            ox, oy = obj.at(t)
            d = self._ray_circle(pose, dir_x, dir_y, ox, oy, obj.radius_m)
            closer = d < dist
            dist[closer] = d[closer]
            hits[closer] = obj.name

        sigma = (0.006 + 0.0015 * dist) * self.lidar_noise.noise_scale
        dist = dist + self.np_rng.normal(0.0, 1.0, size=dist.shape) * sigma
        dist = np.clip(dist, LIDAR_MIN_RANGE_M, LIDAR_MAX_RANGE_M)
        invalid = self.np_rng.random(dist.shape) < self.lidar_noise.invalid_probability
        dist[invalid] = np.nan
        hits[invalid] = "Invalid"
        # Rays that hit nothing are reported as max range by the STL-19P model.
        valid = np.isfinite(dist) & (dist >= LIDAR_MIN_RANGE_M) & (dist <= LIDAR_MAX_RANGE_M)
        dist_out = dist.copy()
        dist_out[~valid] = np.nan

        return LidarScan(
            scan_id=int(scan_id),
            timestamp=float(timestamp if timestamp is not None else t),
            angle_deg=angles,
            distance_m=dist_out,
            valid=valid,
            hit_types=list(hits),
            scan_rate_hz=LIDAR_SCAN_HZ,
            measurements_per_second=int(LIDAR_SCAN_HZ * LIDAR_SAMPLES_PER_SCAN),
        )

    def _ray_room(self, pose: SensorPose, dir_x: np.ndarray, dir_y: np.ndarray) -> np.ndarray:
        d = np.full(dir_x.shape, np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            for bound, origin, direction in (
                (self.room.width, pose.x, dir_x),
                (0.0, pose.x, dir_x),
                (self.room.height, pose.y, dir_y),
                (0.0, pose.y, dir_y),
            ):
                tt = (bound - origin) / direction
                tt = np.where((direction != 0) & (tt > 0), tt, np.inf)
                d = np.minimum(d, tt)
        return d

    @staticmethod
    def _ray_circle(pose: SensorPose, dir_x: np.ndarray, dir_y: np.ndarray,
                    cx: float, cy: float, radius: float) -> np.ndarray:
        ox, oy = pose.x - cx, pose.y - cy
        b = 2.0 * (ox * dir_x + oy * dir_y)
        c = ox * ox + oy * oy - radius * radius
        disc = b * b - 4.0 * c
        d = np.full(dir_x.shape, np.inf)
        ok = disc >= 0
        root = np.sqrt(np.where(ok, disc, 0.0))
        t1 = (-b - root) / 2.0
        t2 = (-b + root) / 2.0
        t1 = np.where(ok & (t1 > 0), t1, np.inf)
        t2 = np.where(ok & (t2 > 0), t2, np.inf)
        return np.minimum(d, np.minimum(t1, t2))


# ---------------------------------------------------------------------------
# Ready-made scenes
# ---------------------------------------------------------------------------

def indoor_scene(seed: int = 0, person_speed_mps: float = 0.4) -> World:
    """
    8 m x 6 m room, sensor at (1.0, 3.0) looking along +x (east).

    One person walks across the camera field of view at ~4 m range from
    right to left and back; a chair and a pillar are static LiDAR-only
    clutter (the camera is not expected to classify them).
    """
    world = World(room=Room(8.0, 6.0), seed=seed)
    world.pose = SensorPose(1.0, 3.0, 0.0)
    span = 2.8
    duration = (2 * span) / max(person_speed_mps, 0.01)
    world.add(WorldObject("Person", "person", 0.28, 1.70,
                          linear_path((5.0, 3.0 - span / 2), (5.0, 3.0 + span / 2),
                                      duration, loop=True)))
    world.add(WorldObject("Chair", "chair", 0.35, 0.9, static(3.0, 1.0)))
    world.add(WorldObject("Pillar", "pillar", 0.30, 2.5, static(6.5, 5.2),
                          camera_visible=False))
    return world


def two_person_scene(seed: int = 0) -> World:
    """Two people crossing paths at different ranges -- identity stress test."""
    world = World(room=Room(10.0, 8.0), seed=seed)
    world.pose = SensorPose(1.0, 4.0, 0.0)
    world.add(WorldObject("PersonA", "person", 0.28, 1.70,
                          linear_path((4.0, 2.5), (4.0, 5.5), 10.0, loop=True)))
    world.add(WorldObject("PersonB", "person", 0.28, 1.65,
                          linear_path((6.5, 5.5), (6.5, 2.5), 10.0, loop=True)))
    return world


def open_field_scene(seed: int = 0) -> World:
    """No walls (outdoor), one static person at 5 m slightly right of centre."""
    world = World(room=None, seed=seed)
    world.pose = SensorPose(0.0, 0.0, 0.0)
    world.add(WorldObject("Person", "person", 0.28, 1.70, static(5.0, -0.8)))
    return world


def moving_sensor_scene(seed: int = 0, speed_mps: float = 0.5) -> World:
    """
    Static room, the SENSOR moves along +x at speed_mps -- for SLAM tests.
    Ground truth pose is available through world.sensor_pose(t).
    """
    world = World(room=Room(10.0, 8.0), seed=seed)
    world.add(WorldObject("Pillar1", "pillar", 0.30, 2.5, static(3.0, 1.5), camera_visible=False))
    world.add(WorldObject("Pillar2", "pillar", 0.30, 2.5, static(7.0, 6.5), camera_visible=False))
    world.add(WorldObject("Box", "box", 0.45, 1.0, static(5.5, 2.0), camera_visible=False))
    world.set_sensor_path(lambda t: SensorPose(2.0 + speed_mps * t, 4.0, 0.0))
    return world
