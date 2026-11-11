#!/usr/bin/env python3

import csv
import json
import math
import os
import signal
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from pathlib import Path

from src.core.lidar_interface import (
    SimulatedSTL19PBackend,
    SerialSTL19PBackend,
)


# ============================================================
# LIDAR BACKEND
# ============================================================
# Physical LiDAR is not connected yet, so simulation remains
# the active backend. The SLAM pipeline receives the same
# LidarScan interface regardless of the eventual source.
# ============================================================

LIDAR_BACKEND = "SIMULATOR"

# Available backends:
#   SIMULATOR -> simulated STL-19P data
#   SERIAL    -> physical STL-19P UART interface
#
# Keep SIMULATOR until the physical LiDAR is connected and
# its packet protocol has been verified.

SUPPORTED_LIDAR_BACKENDS = {
    "SIMULATOR",
    "SERIAL",
}


def create_lidar_backend():
    """Create the selected LiDAR backend."""

    backend_name = LIDAR_BACKEND.upper().strip()

    if backend_name not in SUPPORTED_LIDAR_BACKENDS:
        raise RuntimeError(
            f"Unsupported LiDAR backend: {backend_name}. "
            f"Supported: {sorted(SUPPORTED_LIDAR_BACKENDS)}"
        )

    if backend_name == "SIMULATOR":
        backend = SimulatedSTL19PBackend()
    else:
        backend = SerialSTL19PBackend()

    return backend



# ============================================================
# NIDAR SLAM V5
# Stable Headless 2D SLAM
#
# Features:
#   - STL-19P simulator
#   - LiDAR point filtering
#   - scan matching
#   - motion sanity validation
#   - pose smoothing
#   - pose confidence
#   - trajectory history
#   - occupancy grid
#   - loop-closure foundation
#   - JSON state output
#   - CSV trajectory output
#   - PGM occupancy map
#
# Flight control: DISABLED
# Display: DISABLED
# Webcam: NOT REQUIRED
# ============================================================


VERSION = "NIDAR_SLAM_V5_1"


# ============================================================
# CONFIGURATION
# ============================================================

MAP_SIZE_M = 20.0
MAP_RESOLUTION_M = 0.05

MAP_CELLS = int(MAP_SIZE_M / MAP_RESOLUTION_M)
MAP_HALF_M = MAP_SIZE_M / 2.0

MIN_RANGE_M = 0.10
MAX_RANGE_M = 8.0

MAX_SCAN_POINTS = 500

MAX_TRANSLATION_PER_FRAME_M = 0.15
MAX_ROTATION_PER_FRAME_DEG = 5.0

MAX_MATCH_ERROR_M = 0.20

POSE_ALPHA = 0.35

LOOP_MIN_DISTANCE_M = 0.50
LOOP_MAX_DISTANCE_M = 3.0
LOOP_MAX_YAW_DEG = 30.0

LOOP_MATCH_ERROR_M = 0.10

PRINT_EVERY = 10

JSON_OUTPUT = os.path.expanduser(
    "outputs/slam/state/nidar_slam_v5_1_state.json"
)

CSV_OUTPUT = os.path.expanduser(
    "outputs/slam/trajectories/nidar_slam_v5_1_trajectory.csv"
)

PGM_OUTPUT = os.path.expanduser(
    "outputs/slam/maps/nidar_slam_v5_1_map.pgm"
)


# ============================================================
# GLOBAL SHUTDOWN
# ============================================================

RUNNING = True


def signal_handler(signum, frame):
    global RUNNING

    if RUNNING:
        print("\nShutdown requested...")

    RUNNING = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# ============================================================
# DATA STRUCTURES
# ============================================================

@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class MotionEstimate:
    dx: float
    dy: float
    dyaw: float
    error: float
    valid: bool


@dataclass
class TrajectoryEntry:
    frame: int
    timestamp: float
    x: float
    y: float
    yaw_deg: float
    dx: float
    dy: float
    dyaw_deg: float
    match_error: float
    match_ok: bool
    pose_confidence: float
    loop_detected: bool


# ============================================================
# ANGLE UTILITIES
# ============================================================

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def angle_difference(a, b):
    return normalize_angle(a - b)


# ============================================================
# SCAN EXTRACTION
# ============================================================

def extract_xy(scan):
    """
    Convert the simulator LiDAR scan into Nx2 XY points.

    The current simulator exposes distance and angle arrays.
    """

    distance = np.asarray(scan.distance_m, dtype=np.float64).reshape(-1)
    angle = np.asarray(scan.angle_deg, dtype=np.float64).reshape(-1)

    n = min(len(distance), len(angle))

    if n == 0:
        return np.empty((0, 2), dtype=np.float64)

    distance = distance[:n]
    angle = angle[:n]

    valid = (
        np.isfinite(distance)
        & np.isfinite(angle)
        & (distance >= MIN_RANGE_M)
        & (distance <= MAX_RANGE_M)
    )

    distance = distance[valid]
    angle = angle[valid]

    if len(distance) == 0:
        return np.empty((0, 2), dtype=np.float64)

    angle_rad = np.deg2rad(angle)

    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)

    points = np.column_stack((x, y))

    return points


# ============================================================
# POINT DOWNSAMPLING
# ============================================================

def downsample_points(points, max_points=MAX_SCAN_POINTS):

    if len(points) <= max_points:
        return points

    step = len(points) / max_points

    indices = (
        np.arange(max_points) * step
    ).astype(int)

    indices = np.clip(
        indices,
        0,
        len(points) - 1
    )

    return points[indices]


# ============================================================
# ROBUST CENTROID
# ============================================================

def robust_centroid(points):

    if len(points) == 0:
        return np.zeros(2)

    center = np.median(points, axis=0)

    distances = np.linalg.norm(
        points - center,
        axis=1
    )

    if len(distances) < 10:
        return center

    threshold = np.percentile(
        distances,
        80
    )

    good = distances <= threshold

    if np.count_nonzero(good) < 5:
        return center

    return np.mean(
        points[good],
        axis=0
    )


# ============================================================
# SIMPLE SCAN MATCHER
# ============================================================

class ScanMatcher:

    def __init__(self):

        self.previous_scan = None

    def match(
        self,
        previous_points,
        current_points
    ):

        if (
            previous_points is None
            or current_points is None
        ):
            return MotionEstimate(
                0.0,
                0.0,
                0.0,
                float("inf"),
                False
            )

        if (
            len(previous_points) < 20
            or len(current_points) < 20
        ):
            return MotionEstimate(
                0.0,
                0.0,
                0.0,
                float("inf"),
                False
            )

        previous_center = robust_centroid(
            previous_points
        )

        current_center = robust_centroid(
            current_points
        )

        delta = (
            current_center
            - previous_center
        )

        dx = float(delta[0])
        dy = float(delta[1])

        # Estimate rotational change from
        # covariance orientation.

        def principal_angle(points):

            centered = (
                points
                - np.mean(points, axis=0)
            )

            covariance = (
                centered.T @ centered
            )

            try:
                eigenvalues, eigenvectors = np.linalg.eigh(
                    covariance
                )

                vector = eigenvectors[
                    :, np.argmax(eigenvalues)
                ]

                return math.atan2(
                    vector[1],
                    vector[0]
                )

            except Exception:
                return 0.0

        previous_angle = principal_angle(
            previous_points
        )

        current_angle = principal_angle(
            current_points
        )

        dyaw = normalize_angle(
            current_angle
            - previous_angle
        )

        # Simulator scans can produce
        # unstable principal-axis flips.
        # Limit the estimate to the configured
        # physical motion range.

        if abs(dyaw) > math.radians(
            MAX_ROTATION_PER_FRAME_DEG
        ):
            dyaw = 0.0

        # Calculate a simple correspondence error.

        count = min(
            len(previous_points),
            len(current_points)
        )

        if count > 250:
            count = 250

        p = previous_points[:count]
        c = current_points[:count]

        if len(p) > 0:

            distance_error = np.linalg.norm(
                p - c,
                axis=1
            )

            error = float(
                np.median(distance_error)
            )

        else:
            error = float("inf")

        valid = True

        if abs(dx) > MAX_TRANSLATION_PER_FRAME_M:
            valid = False

        if abs(dy) > MAX_TRANSLATION_PER_FRAME_M:
            valid = False

        if abs(dyaw) > math.radians(
            MAX_ROTATION_PER_FRAME_DEG
        ):
            valid = False

        if error > MAX_MATCH_ERROR_M:
            valid = False

        return MotionEstimate(
            dx,
            dy,
            dyaw,
            error,
            valid
        )


# ============================================================
# POSE ESTIMATOR
# ============================================================

class PoseEstimator:

    def __init__(self):

        self.pose = Pose()

        self.confidence = 1.0

    def update(
        self,
        motion,
        accepted
    ):

        if not accepted:

            self.confidence *= 0.97

            return self.pose

        c = math.cos(
            self.pose.yaw
        )

        s = math.sin(
            self.pose.yaw
        )

        world_dx = (
            motion.dx * c
            - motion.dy * s
        )

        world_dy = (
            motion.dx * s
            + motion.dy * c
        )

        new_x = (
            self.pose.x
            + world_dx
        )

        new_y = (
            self.pose.y
            + world_dy
        )

        new_yaw = normalize_angle(
            self.pose.yaw
            + motion.dyaw
        )

        # Smooth pose.

        alpha = POSE_ALPHA

        self.pose.x = (
            alpha * new_x
            + (1.0 - alpha)
            * self.pose.x
        )

        self.pose.y = (
            alpha * new_y
            + (1.0 - alpha)
            * self.pose.y
        )

        self.pose.yaw = normalize_angle(
            alpha * new_yaw
            + (1.0 - alpha)
            * self.pose.yaw
        )

        # Confidence follows match quality.

        quality = 1.0 - min(
            motion.error
            / MAX_MATCH_ERROR_M,
            1.0
        )

        self.confidence = (
            0.80 * self.confidence
            + 0.20 * quality
        )

        self.confidence = float(
            np.clip(
                self.confidence,
                0.0,
                1.0
            )
        )

        return self.pose


# ============================================================
# OCCUPANCY GRID
# ============================================================

class OccupancyGrid:

    def __init__(
        self,
        size_m,
        resolution
    ):

        self.size_m = size_m

        self.resolution = resolution

        self.cells = int(
            size_m / resolution
        )

        self.grid = np.zeros(
            (
                self.cells,
                self.cells
            ),
            dtype=np.int16
        )

    def world_to_grid(
        self,
        x,
        y
    ):

        gx = int(
            (x + MAP_HALF_M)
            / self.resolution
        )

        gy = int(
            (y + MAP_HALF_M)
            / self.resolution
        )

        return gx, gy

    def update(
        self,
        points,
        pose
    ):

        if len(points) == 0:
            return

        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)

        world_x = (
            points[:, 0] * c
            - points[:, 1] * s
            + pose.x
        )

        world_y = (
            points[:, 0] * s
            + points[:, 1] * c
            + pose.y
        )

        gx = (
            (world_x + MAP_HALF_M)
            / self.resolution
        ).astype(int)

        gy = (
            (world_y + MAP_HALF_M)
            / self.resolution
        ).astype(int)

        valid = (
            (gx >= 0)
            & (gx < self.cells)
            & (gy >= 0)
            & (gy < self.cells)
        )

        gx = gx[valid]
        gy = gy[valid]

        if len(gx) == 0:
            return

        self.grid[
            gy,
            gx
        ] += 1

        self.grid = np.clip(
            self.grid,
            0,
            100
        )

    def occupied_count(self):

        return int(
            np.count_nonzero(
                self.grid > 0
            )
        )

    def save_pgm(self, path):

        occupied = self.grid.astype(
            np.float32
        )

        image = np.full(
            occupied.shape,
            255,
            dtype=np.uint8
        )

        occupied_mask = occupied > 0

        if np.any(occupied_mask):

            max_value = max(
                1.0,
                float(
                    occupied[
                        occupied_mask
                    ].max()
                )
            )

            darkness = (
                occupied[
                    occupied_mask
                ] / max_value
            )

            image[
                occupied_mask
            ] = (
                255
                - darkness * 220
            ).astype(
                np.uint8
            )

        # PGM coordinate orientation.

        image = np.flipud(
            image
        )

        with open(
            path,
            "wb"
        ) as f:

            header = (
                f"P5\n"
                f"{self.cells} "
                f"{self.cells}\n"
                f"255\n"
            ).encode()

            f.write(header)

            f.write(
                image.tobytes()
            )


# ============================================================
# LOOP CLOSURE FOUNDATION
# ============================================================

class LoopClosureDetector:

    def __init__(self):

        self.keyframes = []

        self.last_loop_frame = -999999

    def add_keyframe(
        self,
        frame,
        pose,
        scan
    ):

        self.keyframes.append(
            {
                "frame": frame,
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
                "scan": scan.copy()
            }
        )

        # Keep memory bounded.

        if len(self.keyframes) > 500:
            self.keyframes.pop(0)

    def search(
        self,
        frame,
        pose,
        scan,
        matcher
    ):

        if frame - self.last_loop_frame < 50:
            return None

        if len(self.keyframes) < 10:
            return None

        best = None

        for keyframe in self.keyframes:

            old_frame = keyframe["frame"]

            if frame - old_frame < 80:
                continue

            dx = pose.x - keyframe["x"]
            dy = pose.y - keyframe["y"]

            distance = math.hypot(
                dx,
                dy
            )

            if (
                distance < LOOP_MIN_DISTANCE_M
                or distance > LOOP_MAX_DISTANCE_M
            ):
                continue

            yaw_difference = abs(
                math.degrees(
                    angle_difference(
                        pose.yaw,
                        keyframe["yaw"]
                    )
                )
            )

            if yaw_difference > LOOP_MAX_YAW_DEG:
                continue

            result = matcher.match(
                keyframe["scan"],
                scan
            )

            if not result.valid:
                continue

            if result.error > LOOP_MATCH_ERROR_M:
                continue

            if (
                best is None
                or result.error
                < best["error"]
            ):

                best = {
                    "frame": old_frame,
                    "distance_m": distance,
                    "yaw_difference_deg":
                        yaw_difference,
                    "error": result.error
                }

        if best is not None:

            self.last_loop_frame = frame

        return best


# ============================================================
# FILE OUTPUT
# ============================================================

def save_json(
    pose,
    frame,
    occupied,
    matches,
    rejected,
    loops,
    fps,
    trajectory
):

    data = {

        "protocol": VERSION,

        "timestamp": time.time(),

        "frame": frame,

        "lidar_source":
            "STL-19P_SIMULATOR",

        "map": {
            "size_m":
                MAP_SIZE_M,

            "resolution_m":
                MAP_RESOLUTION_M,

            "occupied_cells":
                occupied
        },

        "pose": {
            "x_m":
                pose.x,

            "y_m":
                pose.y,

            "yaw_deg":
                math.degrees(
                    pose.yaw
                )
        },

        "statistics": {

            "frames":
                frame,

            "successful_matches":
                matches,

            "rejected_matches":
                rejected,

            "loop_closures":
                loops,

            "processing_fps":
                fps
        },

        "flight_control":
            "DISABLED",

        "display":
            "DISABLED",

        "webcam":
            "NOT_REQUIRED",

        "trajectory_points":
            len(trajectory)
    }

    with open(
        JSON_OUTPUT,
        "w"
    ) as f:

        json.dump(
            data,
            f,
            indent=2
        )


def save_csv(
    trajectory
):

    with open(
        CSV_OUTPUT,
        "w",
        newline=""
    ) as f:

        writer = csv.writer(f)

        writer.writerow(
            [
                "frame",
                "timestamp",
                "x_m",
                "y_m",
                "yaw_deg",
                "dx_m",
                "dy_m",
                "dyaw_deg",
                "match_error_m",
                "match_ok",
                "pose_confidence",
                "loop_detected"
            ]
        )

        for entry in trajectory:

            writer.writerow(
                [
                    entry.frame,
                    entry.timestamp,
                    entry.x,
                    entry.y,
                    entry.yaw_deg,
                    entry.dx,
                    entry.dy,
                    entry.dyaw_deg,
                    entry.match_error,
                    entry.match_ok,
                    entry.pose_confidence,
                    entry.loop_detected
                ]
            )


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 100)

    print(
        "NIDAR SLAM V5 - ROBUST HEADLESS 2D SLAM"
    )

    print("=" * 100)

    print(
        f"LiDAR source       : {LIDAR_BACKEND} ({lidar_source if "lidar_source" in locals() else "initializing"})"
    )

    print(
        f"Map size           : "
        f"{MAP_SIZE_M:.1f}m × {MAP_SIZE_M:.1f}m"
    )

    print(
        f"Map resolution     : "
        f"{MAP_RESOLUTION_M:.2f}m"
    )

    print(
        "Localization       : LiDAR SCAN MATCHING"
    )

    print(
        "Pose smoothing     : ENABLED"
    )

    print(
        "Motion validation  : ENABLED"
    )

    print(
        "Loop closure       : FOUNDATION ENABLED"
    )

    print(
        "Flight control     : DISABLED"
    )

    print(
        "Display            : DISABLED"
    )

    print("=" * 100)

    lidar = create_lidar_backend()
    lidar_source = lidar.name

    grid = OccupancyGrid(
        MAP_SIZE_M,
        MAP_RESOLUTION_M
    )

    matcher = ScanMatcher()

    estimator = PoseEstimator()

    loop_detector = LoopClosureDetector()

    trajectory = []

    previous_scan = None

    frame = 0

    successful_matches = 0

    rejected_matches = 0

    loop_closures = 0

    start_time = time.time()

    print(
        "STL-19P simulator  : READY"
    )

    print(
        "Occupancy grid      : READY"
    )

    print(
        "Scan matcher        : READY"
    )

    print(
        "Pose estimator      : READY"
    )

    print(
        "Motion validator    : READY"
    )

    print(
        "Loop closure        : READY"
    )

    print(
        "SLAM pipeline       : READY"
    )

    print(
        "Headless mode       : ENABLED"
    )

    print("=" * 100)

    try:

        while RUNNING:

            scan = lidar.next_scan()

            points = extract_xy(
                scan
            )

            points = downsample_points(
                points
            )

            if len(points) < 20:
                continue

            frame += 1

            timestamp = time.time()

            motion = MotionEstimate(
                0.0,
                0.0,
                0.0,
                0.0,
                True
            )

            match_ok = True

            if previous_scan is not None:

                motion = matcher.match(
                    previous_scan,
                    points
                )

                match_ok = motion.valid

                if match_ok:

                    successful_matches += 1

                else:

                    rejected_matches += 1

            pose = estimator.update(
                motion,
                match_ok
            )

            # Mapping is performed only with
            # accepted localization updates.

            if match_ok:

                grid.update(
                    points,
                    pose
                )

            # Loop closure foundation.

            loop = None

            if frame % 10 == 0:

                loop = loop_detector.search(
                    frame,
                    pose,
                    points,
                    matcher
                )

                if loop is not None:

                    loop_closures += 1

                    print(
                        f"[LOOP CLOSURE] "
                        f"current={frame} "
                        f"previous={loop['frame']} "
                        f"error={loop['error']:.4f}m"
                    )

            # Add periodic keyframes.

            if frame % 20 == 0:

                loop_detector.add_keyframe(
                    frame,
                    pose,
                    points
                )

            elapsed = (
                time.time()
                - start_time
            )

            fps = (
                frame / elapsed
                if elapsed > 0
                else 0.0
            )

            trajectory.append(
                TrajectoryEntry(
                    frame=frame,
                    timestamp=timestamp,
                    x=pose.x,
                    y=pose.y,
                    yaw_deg=math.degrees(
                        pose.yaw
                    ),
                    dx=motion.dx,
                    dy=motion.dy,
                    dyaw_deg=math.degrees(
                        motion.dyaw
                    ),
                    match_error=motion.error,
                    match_ok=match_ok,
                    pose_confidence=estimator.confidence,
                    loop_detected=loop is not None
                )
            )

            # ================================================================
            # LIVE SLAM STATE PUBLISHER
            # ================================================================
            #
            # The normal save_json() call happens during shutdown.
            # The integration monitor therefore cannot see a changing
            # SLAM frame while the process is running.
            #
            # Publish the current state every 10 frames.
            # ================================================================

            if frame % 10 == 0:

                live_state = {
                    "protocol": "NIDAR_SLAM_V5_1",
                    "timestamp": timestamp,
                    "frame": frame,

                    "lidar_source": "STL-19P_SIMULATOR",

                    "map": {
                        "size_m": MAP_SIZE_M,
                        "resolution_m": MAP_RESOLUTION_M,
                        "occupied_cells": grid.occupied_count()
                    },

                    "pose": {
                        "x_m": pose.x,
                        "y_m": pose.y,
                        "yaw_deg": math.degrees(
                            pose.yaw
                        )
                    },

                    "statistics": {
                        "frames": frame,
                        "successful_matches": successful_matches,
                        "rejected_matches": rejected_matches,
                        "loop_closures": loop_closures,
                        "processing_fps": fps
                    },

                    "flight_control": "DISABLED",
                    "display": "DISABLED",
                    "webcam": "NOT_REQUIRED",

                    "trajectory_points": len(trajectory),

                    "live": True
                }

                live_temp = Path(str(JSON_OUTPUT) + ".live.tmp")

                try:

                    with live_temp.open(
                        "w",
                        encoding="utf-8"
                    ) as f:

                        json.dump(
                            live_state,
                            f,
                            indent=2
                        )

                        f.flush()
                        os.fsync(f.fileno())

                    live_temp.replace(JSON_OUTPUT)

                except Exception as exc:

                    print(
                        f"[WARN] Live SLAM state write failed: {exc}"
                    )

            previous_scan = points.copy()

            if frame % PRINT_EVERY == 0:

                status = (
                    "OK"
                    if match_ok
                    else "REJECTED"
                )

                print("-" * 100)

                print(
                    f"Frame              : {frame}"
                )

                print(
                    f"Pose X             : "
                    f"{pose.x:+.3f} m"
                )

                print(
                    f"Pose Y             : "
                    f"{pose.y:+.3f} m"
                )

                print(
                    f"Yaw                : "
                    f"{math.degrees(pose.yaw):+.2f}°"
                )

                print(
                    f"LiDAR points       : "
                    f"{len(points)}"
                )

                print(
                    f"Scan match         : "
                    f"{status}"
                )

                print(
                    f"Relative motion    : "
                    f"dx={motion.dx:+.4f} m  "
                    f"dy={motion.dy:+.4f} m  "
                    f"dYaw={math.degrees(motion.dyaw):+.3f}°"
                )

                print(
                    f"Match error        : "
                    f"{motion.error:.4f} m"
                )

                print(
                    f"Pose confidence    : "
                    f"{estimator.confidence:.2f}"
                )

                print(
                    f"Successful matches : "
                    f"{successful_matches}"
                )

                print(
                    f"Rejected matches   : "
                    f"{rejected_matches}"
                )

                print(
                    f"Loop closures      : "
                    f"{loop_closures}"
                )

                print(
                    f"Occupied cells     : "
                    f"{grid.occupied_count()}"
                )

                print(
                    f"Processing FPS     : "
                    f"{fps:.2f}"
                )

                print(
                    f"Map                : "
                    f"{MAP_SIZE_M:.1f}m × "
                    f"{MAP_SIZE_M:.1f}m"
                )

                print(
                    f"Resolution         : "
                    f"{MAP_RESOLUTION_M:.2f}m"
                )

                print("-" * 100)

    finally:

        elapsed = (
            time.time()
            - start_time
        )

        fps = (
            frame / elapsed
            if elapsed > 0
            else 0.0
        )

        print()

        print("=" * 100)

        print(
            "NIDAR SLAM V5 SHUTDOWN"
        )

        print("=" * 100)

        print(
            f"Frames processed : "
            f"{frame}"
        )

        print(
            f"Successful match : "
            f"{successful_matches}"
        )

        print(
            f"Rejected matches : "
            f"{rejected_matches}"
        )

        print(
            f"Loop closures    : "
            f"{loop_closures}"
        )

        print(
            f"Occupied cells   : "
            f"{grid.occupied_count()}"
        )

        print(
            f"Final Pose X     : "
            f"{estimator.pose.x:+.3f} m"
        )

        print(
            f"Final Pose Y     : "
            f"{estimator.pose.y:+.3f} m"
        )

        print(
            f"Final Yaw        : "
            f"{math.degrees(estimator.pose.yaw):+.2f}°"
        )

        print(
            f"Average FPS      : "
            f"{fps:.2f}"
        )

        save_json(
            estimator.pose,
            frame,
            grid.occupied_count(),
            successful_matches,
            rejected_matches,
            loop_closures,
            fps,
            trajectory
        )

        save_csv(
            trajectory
        )

        grid.save_pgm(
            PGM_OUTPUT
        )

        print(
            f"JSON output       : "
            f"{JSON_OUTPUT}"
        )

        print(
            f"CSV output        : "
            f"{CSV_OUTPUT}"
        )

        print(
            f"Map output        : "
            f"{PGM_OUTPUT}"
        )

        print(
            "Flight control     : DISABLED"
        )

        print(
            "Display            : DISABLED"
        )

        print("=" * 100)


if __name__ == "__main__":
    main()
