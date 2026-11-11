"""
LiDAR chain: STL-19P simulator -> LidarScan -> obstacle detector -> tracker V3,
plus the co-simulated world's scan generator against the same detector.
"""

from __future__ import annotations

import math
import random

import numpy as np
import pytest

from src.core.lidar_interface import LidarInterfaceConfig, LidarScan, SimulatedSTL19PBackend, LidarProcessor
from src.simulation import nidar_lidar_simulator as sim
from src.simulation import nidar_world
from lidar_obstacle_detector import LidarObstacleDetector, ObstacleDetectorConfig
from lidar_obstacle_tracker_v3 import LidarObstacleTrackerV3, TrackerV3Config


def _expected_polar(obstacle):
    dx, dy = obstacle.x - sim.SENSOR_X, obstacle.y - sim.SENSOR_Y
    return math.hypot(dx, dy), math.degrees(math.atan2(dy, dx)) % 360.0


def test_simulator_scan_shape_and_determinism():
    random.seed(7)
    a1, d1, h1, p1 = sim.make_scan(0.0)
    random.seed(7)
    a2, d2, h2, p2 = sim.make_scan(0.0)
    assert len(a1) == sim.SAMPLES_PER_SCAN == 500
    assert np.array_equal(a1, a2) and np.array_equal(np.nan_to_num(d1), np.nan_to_num(d2))
    assert set(h1) <= {"Wall", "Chair", "Box", "Pillar", "Person", "Invalid"}


def test_backend_normalizes_invalid_samples():
    backend = SimulatedSTL19PBackend(LidarInterfaceConfig())
    scan = backend.next_scan()
    assert isinstance(scan, LidarScan) and scan.total_count == 500
    assert np.all(np.isnan(scan.distance_m[~scan.valid]))
    assert scan.valid_count > 450
    assert backend.next_scan().scan_id == 1


def test_detector_finds_static_obstacles_of_the_simulator():
    random.seed(1)
    backend = SimulatedSTL19PBackend(LidarInterfaceConfig())
    detector = LidarObstacleDetector(ObstacleDetectorConfig())
    obstacles = detector.detect(backend.next_scan())
    for expected in sim.STATIC_OBSTACLES:
        rng, bearing = _expected_polar(expected)
        match = min(obstacles, key=lambda o: abs((o.bearing_deg - bearing + 180) % 360 - 180))
        # tolerance = angular half-width of the object (visible-surface centroid can be off-centre)
        half_width = math.degrees(math.asin(min(1.0, expected.radius / rng)))
        assert abs((match.bearing_deg - bearing + 180) % 360 - 180) < half_width, expected.name
        # the cluster sits on the near surface: distance = range - radius (approximately)
        assert abs(match.distance_m - (rng - expected.radius)) < 0.25, expected.name


def test_detector_also_reports_wall_segments_as_obstacles():
    """Documented limitation: walls are clustered too (span < max_cluster_width_m)."""
    random.seed(1)
    backend = SimulatedSTL19PBackend(LidarInterfaceConfig())
    detector = LidarObstacleDetector(ObstacleDetectorConfig())
    obstacles = detector.detect(backend.next_scan())
    walls = [o for o in obstacles if max(o.width_m, o.depth_m) > 2.0]
    assert walls, "expected at least one wall-sized cluster -- V18 sees these as LiDAR tracks"


def test_tracker_keeps_ids_for_static_obstacles():
    random.seed(2)
    backend = SimulatedSTL19PBackend(LidarInterfaceConfig())
    detector = LidarObstacleDetector(ObstacleDetectorConfig())
    tracker = LidarObstacleTrackerV3(TrackerV3Config())
    histories = {}
    for i in range(12):
        scan = backend.next_scan()
        tracks = tracker.update(detector.detect(scan), timestamp=i * 0.1)
        for tr in tracks:
            if tr.state == "CONFIRMED" and tr.motion_state == "STATIONARY":
                histories.setdefault(tr.track_id, []).append(round(tr.bearing_deg))
    static_ids = [tid for tid, h in histories.items() if len(h) >= 8]
    assert len(static_ids) >= 3, f"static obstacles must keep their track ids: {histories}"


def test_tracker_velocity_is_measured_and_bounded():
    tracker = LidarObstacleTrackerV3(TrackerV3Config())
    from lidar_obstacle_detector import DetectedObstacle

    def d(x):
        return DetectedObstacle(1, x, 0.0, abs(x), 0.0, 0.3, 0.3, 20, 0.9, 0.0, 1.0)

    t = 0.0
    for step in range(15):
        tracks = tracker.update([d(2.0 + 0.1 * step)], timestamp=t)   # 1 m/s along x
        t += 0.1
    tr = tracks[0]
    assert tr.motion_state == "MOVING"
    assert 0.3 < tr.speed_mps <= TrackerV3Config().max_speed_mps
    assert tr.vx_mps > 0 and abs(tr.vy_mps) < 0.05


def test_processor_sectors_wrap_around_zero():
    backend = SimulatedSTL19PBackend(LidarInterfaceConfig())
    sectors = LidarProcessor().front_left_right(backend.next_scan())
    assert set(sectors) == {"forward", "left", "rear", "right"}
    assert sectors["forward"].valid_points > 20       # 345..15 deg wraps through 0


# ---------------------------------------------------------------- world ----

def test_world_scan_bearing_convention_matches_detector():
    """An object placed to the RIGHT of forward must get a positive (small) bearing."""
    w = nidar_world.World(room=None, seed=0, lidar_noise=nidar_world.LidarNoise(noise_scale=0.0, invalid_probability=0.0))
    w.pose = nidar_world.SensorPose(0.0, 0.0, 0.0)
    w.add(nidar_world.WorldObject("P", "person", 0.28, 1.7, nidar_world.static(4.0, -1.0)))   # forward 4, right 1
    truth = w.truth(0.0)[0]
    assert truth.bearing_deg == pytest.approx(math.degrees(math.atan2(1.0, 4.0)))
    detector = LidarObstacleDetector(ObstacleDetectorConfig())
    obstacles = [o for o in detector.detect(w.lidar_scan(0, 0.0)) if o.point_count > 5]
    assert len(obstacles) == 1
    assert obstacles[0].bearing_deg == pytest.approx(truth.bearing_deg, abs=1.0)
    assert obstacles[0].distance_m == pytest.approx(truth.range_m - 0.28, abs=0.1)


def test_world_camera_and_lidar_agree_on_bearing(world):
    """The whole point of the co-simulation: both sensors describe the same object."""
    detector = LidarObstacleDetector(ObstacleDetectorConfig())
    t = 1.0
    packet = world.camera_packet(1, t)
    person = [o for o in packet["objects"] if o["class"] == "person"]
    assert len(person) == 1
    cx = person[0]["center"][0]
    camera_bearing_true = math.degrees(math.atan((cx - 640.0) / nidar_world.focal_length_px()))
    truth = [o for o in world.camera_visible_observations(t) if o.obj.label == "person"][0]
    assert camera_bearing_true == pytest.approx(truth.bearing_deg, abs=0.6)
    obstacles = detector.detect(world.lidar_scan(0, t))
    nearest = min(obstacles, key=lambda o: abs((o.bearing_deg - truth.bearing_deg + 180) % 360 - 180))
    assert abs((nearest.bearing_deg - truth.bearing_deg + 180) % 360 - 180) < 2.0


def test_world_moving_sensor_truth_is_consistent():
    w = nidar_world.moving_sensor_scene(seed=0, speed_mps=0.5)
    p0, p1 = w.sensor_pose(0.0), w.sensor_pose(2.0)
    assert p1.x - p0.x == pytest.approx(1.0) and p1.y == p0.y
