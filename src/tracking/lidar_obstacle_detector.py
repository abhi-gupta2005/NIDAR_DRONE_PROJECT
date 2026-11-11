#!/usr/bin/env python3
"""
NIDAR V12.1 - LiDAR Obstacle Detector

Input:
    LidarScan from lidar_interface.py

Output:
    DetectedObstacle objects containing:
      - cluster ID
      - center X/Y
      - distance
      - bearing
      - width
      - point count
      - confidence

This module is intentionally independent of the camera, flight controller,
and physical LiDAR. It can therefore be tested with the STL-19P simulator
before real hardware is connected.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from lidar_interface import LidarInterfaceConfig, LidarScan, SimulatedSTL19PBackend


@dataclass
class ObstacleDetectorConfig:
    # Range filtering
    min_range_m: float = 0.05
    max_range_m: float = 12.0

    # Neighbor clustering threshold.
    # Two consecutive LiDAR points can belong to the same obstacle
    # when their Cartesian separation is below this value.
    cluster_distance_m: float = 0.18

    # Very small clusters are normally isolated noise.
    min_cluster_points: int = 3

    # Reject unrealistically large gaps inside a cluster.
    max_cluster_width_m: float = 5.0

    # Confidence parameters
    ideal_points: int = 20
    max_confidence_distance_m: float = 8.0


@dataclass
class DetectedObstacle:
    obstacle_id: int
    center_x_m: float
    center_y_m: float
    distance_m: float
    bearing_deg: float
    width_m: float
    depth_m: float
    point_count: int
    confidence: float
    start_angle_deg: float
    end_angle_deg: float

    def to_dict(self) -> dict:
        return {
            "obstacle_id": self.obstacle_id,
            "center_x_m": round(self.center_x_m, 4),
            "center_y_m": round(self.center_y_m, 4),
            "distance_m": round(self.distance_m, 4),
            "bearing_deg": round(self.bearing_deg, 3),
            "width_m": round(self.width_m, 4),
            "depth_m": round(self.depth_m, 4),
            "point_count": self.point_count,
            "confidence": round(self.confidence, 4),
            "start_angle_deg": round(self.start_angle_deg, 3),
            "end_angle_deg": round(self.end_angle_deg, 3),
        }


class LidarObstacleDetector:
    """Converts a normalized LiDAR scan into obstacle clusters."""

    def __init__(self, config: Optional[ObstacleDetectorConfig] = None):
        self.config = config or ObstacleDetectorConfig()
        self.next_obstacle_id = 1

    # ------------------------------------------------------------------
    # 1. Range filtering
    # ------------------------------------------------------------------

    def filter_scan(self, scan: LidarScan) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Return valid angle, distance and Cartesian XY arrays.

        Invalid samples are removed before clustering.
        """
        mask = (
            scan.valid
            & np.isfinite(scan.angle_deg)
            & np.isfinite(scan.distance_m)
            & (scan.distance_m >= self.config.min_range_m)
            & (scan.distance_m <= self.config.max_range_m)
        )

        angles = scan.angle_deg[mask]
        distances = scan.distance_m[mask]

        radians = np.deg2rad(angles)
        x = distances * np.cos(radians)
        y = distances * np.sin(radians)

        return angles, x, y

    # ------------------------------------------------------------------
    # 2. Consecutive-point clustering
    # ------------------------------------------------------------------

    def _cluster_points(
        self,
        angles: np.ndarray,
        x: np.ndarray,
        y: np.ndarray,
    ) -> List[np.ndarray]:
        """
        Cluster adjacent points in angular order.

        The simulator generates an ordered 360-degree scan, so adjacent
        samples are natural candidates for the same physical surface.
        """
        if len(x) == 0:
            return []

        order = np.argsort(angles)
        angles_sorted = angles[order]
        x_sorted = x[order]
        y_sorted = y[order]

        clusters: List[List[int]] = []
        current = [0]

        for i in range(1, len(x_sorted)):
            dx = x_sorted[i] - x_sorted[i - 1]
            dy = y_sorted[i] - y_sorted[i - 1]
            gap = math.hypot(dx, dy)

            if gap <= self.config.cluster_distance_m:
                current.append(i)
            else:
                clusters.append(current)
                current = [i]

        clusters.append(current)

        # Handle the 0°/360° wrap-around.
        if len(clusters) >= 2:
            first = clusters[0]
            last = clusters[-1]

            dx = x_sorted[first[0]] - x_sorted[last[-1]]
            dy = y_sorted[first[0]] - y_sorted[last[-1]]

            if math.hypot(dx, dy) <= self.config.cluster_distance_m:
                merged = last + first
                clusters = [merged] + clusters[1:-1]

        return [order[np.asarray(c, dtype=int)] for c in clusters]

    # ------------------------------------------------------------------
    # 3. Cluster geometry
    # ------------------------------------------------------------------

    def _cluster_to_obstacle(
        self,
        obstacle_id: int,
        indices: np.ndarray,
        angles: np.ndarray,
        x: np.ndarray,
        y: np.ndarray,
    ) -> Optional[DetectedObstacle]:

        if len(indices) < self.config.min_cluster_points:
            return None

        cx = float(np.mean(x[indices]))
        cy = float(np.mean(y[indices]))

        distances = np.hypot(x[indices], y[indices])
        distance = float(np.median(distances))

        bearing = math.degrees(math.atan2(cy, cx))
        if bearing < 0:
            bearing += 360.0

        # Bounding-box dimensions in local LiDAR coordinates.
        width = float(np.max(x[indices]) - np.min(x[indices]))
        depth = float(np.max(y[indices]) - np.min(y[indices]))

        start_angle = float(np.min(angles[indices]))
        end_angle = float(np.max(angles[indices]))

        # Reject an accidentally merged enormous cluster.
        cluster_span = max(width, depth)
        if cluster_span > self.config.max_cluster_width_m:
            return None

        point_factor = min(
            1.0,
            len(indices) / max(1, self.config.ideal_points),
        )

        distance_factor = max(
            0.0,
            1.0 - distance / self.config.max_confidence_distance_m,
        )

        # More points + closer object = stronger geometric confidence.
        confidence = 0.55 * point_factor + 0.45 * distance_factor

        return DetectedObstacle(
            obstacle_id=obstacle_id,
            center_x_m=cx,
            center_y_m=cy,
            distance_m=distance,
            bearing_deg=bearing,
            width_m=width,
            depth_m=depth,
            point_count=len(indices),
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            start_angle_deg=start_angle,
            end_angle_deg=end_angle,
        )

    # ------------------------------------------------------------------
    # 4. Complete detection pipeline
    # ------------------------------------------------------------------

    def detect(self, scan: LidarScan) -> List[DetectedObstacle]:
        angles, x, y = self.filter_scan(scan)

        clusters = self._cluster_points(angles, x, y)

        obstacles: List[DetectedObstacle] = []

        for cluster in clusters:
            obstacle = self._cluster_to_obstacle(
                self.next_obstacle_id,
                cluster,
                angles,
                x,
                y,
            )

            if obstacle is not None:
                obstacles.append(obstacle)
                self.next_obstacle_id += 1

        # Closest obstacles first.
        obstacles.sort(key=lambda o: o.distance_m)

        return obstacles


def print_obstacles(obstacles: List[DetectedObstacle]) -> None:
    print(f"Detected obstacles: {len(obstacles)}")

    if not obstacles:
        print("  No clusters passed the detector thresholds.")
        return

    for obj in obstacles:
        print(
            f"  ID={obj.obstacle_id:03d} "
            f"distance={obj.distance_m:5.2f} m "
            f"bearing={obj.bearing_deg:6.1f}° "
            f"center=({obj.center_x_m:6.2f}, {obj.center_y_m:6.2f}) m "
            f"size=({obj.width_m:5.2f} x {obj.depth_m:5.2f}) m "
            f"points={obj.point_count:3d} "
            f"confidence={obj.confidence:.2f}"
        )


def run_self_test(scans: int = 10) -> None:
    config = LidarInterfaceConfig(
        scan_rate_hz=10.0,
        measurements_per_second=5000,
        max_range_m=12.0,
        min_range_m=0.05,
        noise_scale=1.0,
    )

    backend = SimulatedSTL19PBackend(config)
    detector = LidarObstacleDetector(
        ObstacleDetectorConfig(
            min_range_m=0.05,
            max_range_m=12.0,
            cluster_distance_m=0.18,
            min_cluster_points=3,
        )
    )

    print("=" * 72)
    print("NIDAR V12.1 - LiDAR OBSTACLE DETECTOR SELF TEST")
    print("=" * 72)
    print("Backend             : STL-19P_SIMULATOR")
    print("Scan rate           : 10.0 Hz")
    print("Samples/scan        : 500")
    print("Cluster threshold   : 0.18 m")
    print("Minimum cluster     : 3 points")
    print()

    for _ in range(scans):
        scan = backend.next_scan()
        obstacles = detector.detect(scan)

        print(
            f"scan={scan.scan_id:04d} "
            f"valid={scan.valid_count:3d}/{scan.total_count:3d} "
            f"nearest={scan.nearest_distance_m:.3f} m"
        )

        print_obstacles(obstacles)
        print()

    print("=" * 72)
    print("Obstacle detector test complete.")
    print("=" * 72)


if __name__ == "__main__":
    run_self_test()
