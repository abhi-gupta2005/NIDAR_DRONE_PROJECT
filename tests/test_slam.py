"""
SLAM V5.1 (src/core/nidar_slam_v5_1.py): component behaviour and the
moving-sensor finding (SLAM-01) exposed by the co-simulated world.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from src.simulation import nidar_pipeline_sim as psim


def test_extract_xy_filters_range_and_nan(slam_module):
    slam = slam_module

    class Scan:
        angle_deg = np.array([0.0, 90.0, 180.0, 270.0, 45.0])
        distance_m = np.array([1.0, np.nan, 0.05, 9.0, 2.0])      # nan, <MIN, >MAX removed

    pts = slam.extract_xy(Scan())
    assert pts.shape == (2, 2)
    assert pts[0] == pytest.approx([1.0, 0.0])
    assert pts[1] == pytest.approx([2.0 * math.cos(math.radians(45)), 2.0 * math.sin(math.radians(45))])


def test_downsample_keeps_at_most_max_points(slam_module):
    pts = np.random.default_rng(0).normal(size=(1200, 2))
    out = slam_module.downsample_points(pts)
    assert len(out) == slam_module.MAX_SCAN_POINTS
    assert len(slam_module.downsample_points(pts[:100])) == 100


def test_occupancy_grid_marks_transformed_points(slam_module):
    slam = slam_module
    grid = slam.OccupancyGrid(slam.MAP_SIZE_M, slam.MAP_RESOLUTION_M)
    pose = slam.Pose(1.0, 0.0, math.pi / 2)                  # facing +y
    grid.update(np.array([[2.0, 0.0]]), pose)                 # 2 m ahead -> world (1, 2)
    gx, gy = grid.world_to_grid(1.0, 2.0)
    assert grid.grid[gy, gx] == 1 and grid.occupied_count() == 1


def test_pose_estimator_rejects_invalid_motion_and_decays_confidence(slam_module):
    slam = slam_module
    est = slam.PoseEstimator()
    before = est.confidence
    pose = est.update(slam.MotionEstimate(1.0, 1.0, 0.0, 9.0, False), accepted=False)
    assert pose.x == 0.0 and pose.y == 0.0 and est.confidence < before


def test_scan_matcher_requires_enough_points(slam_module):
    m = slam_module.ScanMatcher()
    assert m.match(None, np.zeros((30, 2))).valid is False
    assert m.match(np.zeros((5, 2)), np.zeros((30, 2))).valid is False


def test_stationary_sensor_pose_stays_near_origin():
    result = psim.run_slam_stationary(duration_s=6.0, seed=0)
    assert result["max_pose_drift_m"] <= 0.25, result


@pytest.mark.finding
@pytest.mark.xfail(strict=True, reason=(
    "SLAM-01: ScanMatcher estimates motion from the shift of the point-cloud centroid, not "
    "from point correspondences. With a moving sensor the per-frame estimate is noise "
    "(sign agreement ~50%, 5.95 m travelled -> ~0.3 m estimated). See docs/NIDAR_PROJECT_REGISTER.md. "
    "This test turns green (and must then be un-xfailed) once a real scan matcher is used."))
def test_moving_sensor_travel_is_recovered():
    result = psim.run_slam_scenario(duration_s=8.0, speed_mps=0.5, seed=0)
    assert result["checks"]["motion_sign_matches_truth"]["pass"], result["checks"]
    assert result["checks"]["final_position_error_below_0.5m"]["pass"], result["checks"]


def test_moving_sensor_matcher_reports_mostly_valid_matches():
    """Documents WHY SLAM-01 is dangerous: the bad estimates are reported as valid."""
    result = psim.run_slam_scenario(duration_s=6.0, speed_mps=0.5, seed=0)
    total = result["successful_matches"] + result["rejected_matches"]
    assert result["successful_matches"] / total > 0.8
