"""
Survivor mapper (src/mission/survivor_mapper.py, 2026-11-02e).

Measured with tools/../e2e harness (arena ray-cast LiDAR, pinhole camera boxes, REAL V18
fusion functions, SLAM V6 pose -- no ground truth pose), 'rooms' route with look-around:
5 of 6 survivors confirmed in each of arenas s1-s3, every one in the correct grid cell,
position error 0.01-0.15 m, no false survivor confirmed. The missed ones were low (lying)
survivors only ever seen at < 1.6 m: the level camera at 0.82 m cannot see the floor that
close (vertical half-FOV 27.7 deg) and the LiDAR plane passes over them (F22).
"""

from __future__ import annotations

import io
import contextlib
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from src.mission.survivor_mapper import CameraModel, Grid, SurvivorMapper, compose, MIN_OBS
from src.simulation import arena_raycast as ar
from src.simulation.arena_feed import project_bbox

ROOT = Path(__file__).resolve().parents[1]


def obj(box, oid=1, cls="person", conf=0.8, current=True):
    return {"id": oid, "class": cls, "confidence": conf, "bbox": list(box), "current_observation": current,
            "spatial_status": "OBSERVED" if current else "PREDICTED"}


def test_grid_labels_match_the_layout_convention():
    g = Grid(1.05, 0.05, 14)
    assert g.label(0.1, 0.1) == "A1"
    assert g.label(4.699, 2.685) == "E3"                     # arena s1 survivor S2
    assert g.label(14.7, 14.7) == "N14"
    assert g.label(7.9, -0.75) == "OUT"                      # launch pad


def test_camera_model_pinhole_bearing_and_ground_range():
    cam = CameraModel(height_m=0.82)
    assert cam.bearing_deg(640) == pytest.approx(35.0, abs=1e-6)
    for rng, brg in ((2.0, 0.0), (3.0, 15.0), (5.0, -20.0)):
        box, _ = project_bbox(rng, brg, "standing", 0.82)
        g = cam.ground_range(0.5 * (box[0] + box[2]), box[3])
        assert g is not None
        assert g[0] == pytest.approx(rng - 0.2, rel=0.03)    # floor point under the near body edge
        assert g[1] == pytest.approx(brg, abs=1.0)
    clipped, _ = project_bbox(1.0, 0.0, "standing", 0.82)  # feet below the image edge
    assert cam.ground_range(320, clipped[3]) is None


def test_lidar_range_used_for_standing_person():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))   # facing east
    box, _ = project_bbox(2.2, 10.0, "standing", 0.82)
    ms = m.measure([obj(box)], [{"camera_id": 1, "range_m": 2.0}], (0, 0, 0), 0.0)
    assert len(ms) == 1 and ms[0].method == "LIDAR"
    assert math.hypot(ms[0].x - 3.0, ms[0].y - 5.0) == pytest.approx(2.2, abs=0.05)
    assert ms[0].y < 5.0, "10 deg to the RIGHT is negative y (left-positive frame)"


def test_f22_wall_range_rejected_for_low_survivor():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))   # inside the arena
    box, _ = project_bbox(3.0, 0.0, "lying", 0.82)          # lying survivor 3 m away
    ms = m.measure([obj(box)], [{"camera_id": 1, "range_m": 4.6}], (0, 0, 0), 0.0)   # LiDAR hit the wall
    assert ms[0].method == "GROUND" and m.lidar_rejected == 1
    assert ms[0].x == pytest.approx(6.0, abs=0.2)


def test_low_object_below_lidar_plane_rejects_lidar_even_without_ground():
    m = SurvivorMapper(CameraModel(height_m=0.82))
    box, _ = project_bbox(1.2, 0.0, "lying", 0.82)          # close: near edge below the image
    assert box[3] >= 479
    ms = m.measure([obj(box)], [{"camera_id": 1, "range_m": 2.4}], (0, 0, 0), 0.0)
    assert ms == [] and m.lidar_rejected == 1, "no trustworthy range -> no measurement"


def test_predicted_boxes_and_other_classes_are_ignored():
    m = SurvivorMapper()
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    assert m.measure([obj(box, current=False), obj(box, oid=2, cls="chair")],
                     [{"camera_id": 1, "range_m": 2.0}, {"camera_id": 2, "range_m": 2.0}], (0, 0, 0), 0.0) == []


def test_measurements_cluster_and_confirm_in_arena_frame():
    start = (7.9, -0.75, math.pi / 2)                      # arena s1 launch pad, facing north
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start)
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    for k in range(MIN_OBS):
        m.update([obj(box)], [{"camera_id": 1, "range_m": 2.0}], (0.0, 0.0, 0.0), k * 0.25)
    rows = m.report()
    assert len(rows) == 1 and rows[0]["status"] == "CONFIRMED" and rows[0]["id"] == "S1"
    assert rows[0]["x_m"] == pytest.approx(7.9, abs=0.02) and rows[0]["y_m"] == pytest.approx(1.45, abs=0.05)
    assert rows[0]["grid"] == "H2"
    assert compose(start, (1.0, 0.0, 0.0))[:2] == pytest.approx((7.9, 0.25))


@pytest.mark.slow
def test_route_to_s2_maps_s2_in_the_right_cell_with_slam_pose():
    """Ray-cast LiDAR + projected camera -> REAL V18 fusion -> SLAM V6 pose -> mapper (no truth pose)."""
    from src.simulation.arena_trajectory import route_trajectory
    from src.simulation.arena_feed import visible_survivors, camera_packet
    from src.core.lidar_gazebo_backend import ros_scan_to_lidar_scan
    from src.core import nidar_slam_v6 as s6
    from lidar_obstacle_detector import LidarObstacleDetector, ObstacleDetectorConfig
    from lidar_obstacle_tracker_v3 import LidarObstacleTrackerV3, TrackerV3Config
    spec = importlib.util.spec_from_file_location("v18_map", ROOT / "src/perception/nidar_v18_headless_persistent_fusion.py")
    v18 = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(v18)
    arena = ar.load_arena(ar.default_layout(1))
    sp = arena.layout["spawn_pose"]
    mapper = SurvivorMapper(CameraModel(height_m=0.82), Grid.from_layout(arena.layout), (sp["x"], sp["y"], sp["yaw"]))
    slam, det, trk = s6.SlamV6(), LidarObstacleDetector(ObstacleDetectorConfig()), LidarObstacleTrackerV3(TrackerV3Config())
    rng = np.random.default_rng(0)
    for i, (t, x, y, yaw) in enumerate(route_trajectory(arena.layout, "survivor:S2")):
        r, _ = ar.cast(arena, x, y, yaw, 1.0)
        r = r.copy(); fin = np.isfinite(r); r[fin] += rng.normal(0, 0.01, fin.sum())
        msg = ar.laserscan_message(r, seq=i); msg["received"] = t
        scan = ros_scan_to_lidar_scan(msg, i, t)
        est = slam.process(s6.scan_to_points(scan), t)
        with contextlib.redirect_stdout(io.StringIO()):
            tracks = [n for n in (v18.normalize_track(tr) for tr in trk.update(det.detect(scan), t)) if n]
            pkt = camera_packet(i, visible_survivors(arena, x, y, yaw), camera_height_m=0.82)
            fused = v18.fuse_objects(pkt["objects"], tracks, 640)
        mapper.update(pkt["objects"], fused, (est["x"], est["y"], est["yaw"]), t)
    rows = {r["grid"]: r for r in mapper.report(include_candidates=False)}
    assert "E3" in rows, mapper.report()
    s2 = next(s for s in arena.layout["survivors"] if s["id"] == "S2")
    assert math.hypot(rows["E3"]["x_m"] - s2["x"], rows["E3"]["y_m"] - s2["y"]) < 0.25
    truth_cells = {s["grid"] for s in arena.layout["survivors"]}
    assert set(rows) <= truth_cells, f"false survivor cells: {set(rows) - truth_cells}"


# --------------------------------------------------- IDs (2026-11-02f) ---

def _feed(m, box, rng_m, pose, t0, n=6, cam_id=1, dt=0.25):
    for k in range(n):
        m.update([obj(box, oid=cam_id)], [{"camera_id": cam_id, "range_m": rng_m}], pose, t0 + k * dt)


def test_position_outside_the_arena_is_rejected_not_a_new_survivor():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(7.9, -0.75, math.pi / 2))
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    _feed(m, box, 2.0, (0.0, 0.0, math.pi), 0.0)            # facing south from the pad -> outside
    assert m.report() == [] and m.outside_rejected == 6


def test_same_camera_track_keeps_its_id_while_the_estimate_wanders():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    _feed(m, box, 2.0, (0.0, 0.0, 0.0), 0.0)
    assert [r["id"] for r in m.report()] == ["S1"]
    for k in range(6):                                        # pose drifts 1.2 m sideways, same camera track
        m.update([obj(box, oid=1)], [{"camera_id": 1, "range_m": 2.0}], (0.0, 0.2 * k, 0.0), 2.0 + 0.25 * k)
    rows = m.report()
    assert [r["id"] for r in rows] == ["S1"], rows


def test_duplicates_merge_into_the_older_id():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    box, _ = project_bbox(2.2, 0.0, "standing", 0.82)
    _feed(m, box, 2.0, (0.0, 0.0, 0.0), 0.0, cam_id=1)                    # S1 at x~5.2
    _feed(m, box, 2.0, (0.0, 0.85, 0.0), 10.0, cam_id=7)                  # new track, 0.85 m off -> S2
    assert [r["id"] for r in m.report()] == ["S1", "S2"]
    _feed(m, box, 2.0, (0.0, 0.42, 0.0), 20.0, n=8, cam_id=9)             # seen in between -> they meet
    rows = m.report()
    assert [r["id"] for r in rows] == ["S1"] and rows[0]["aliases"] == ["S2"], rows
    assert m.merges == [("S2", "S1")]


# ------------------------------------------- robust position (2026-11-02g) ---
# Gazebo run 2026-11-02 11:14: survivor S3 (H12) was confirmed 1.0 m off in H11 with a 0.88 m
# spread (plain mean of every measurement); a correct H12 candidate stayed unconfirmed.
# Offline fault injection (gazebo-like box jitter, half-body boxes, bursts of wrong V18
# ranges; scratchpad e2e/mapper_noise_eval.py), arenas s1-s3, faults level 2 and 3:
#   2026-11-02f mapper: 28/36 correct, 15 wrong/duplicate confirmations, mean error 0.21-0.31 m
#   2026-11-02g mapper: 27/36 correct,  3 wrong/duplicate confirmations, mean error 0.06-0.10 m
# Fault-free runs are unchanged (5/6 per arena, the lying survivors seen only up close missed).

TALL_FEET_CUT = (300.0, 100.0, 340.0, 480.0)    # standing body, centred, feet below the image: LiDAR only


def test_outliers_from_the_same_track_do_not_pull_the_position():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    for k in range(8):
        m.update([obj(TALL_FEET_CUT)], [{"camera_id": 1, "range_m": 2.0}], (0.0, 0.0, 0.0), k * 0.2)
    for k in range(4):                                       # V18 locked on the wall behind: +1.2 m
        m.update([obj(TALL_FEET_CUT)], [{"camera_id": 1, "range_m": 3.2}], (0.0, 0.0, 0.0), 1.6 + k * 0.2)
    rows = m.report()
    assert len(rows) == 1 and rows[0]["id"] == "S1", rows
    assert rows[0]["x_m"] == pytest.approx(5.2, abs=0.05), "plain mean would be 5.6 m"
    assert rows[0]["inliers"] == 8 and rows[0]["observations"] == 12 and rows[0]["std_m"] < 0.05


def test_a_wide_disagreeing_cluster_is_not_confirmed():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    for k in range(10):                                      # two ranges 1.2 m apart, same track
        r = 2.0 if k % 2 else 3.2
        m.update([obj(TALL_FEET_CUT)], [{"camera_id": 1, "range_m": r}], (0.0, 0.0, 0.0), k * 0.2)
    assert [r["status"] for r in m.report()] == ["CANDIDATE"], m.report()


def test_range_ghost_on_the_same_camera_ray_is_absorbed():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    for k in range(6):                                       # person at arena (5.2, 5.0), seen from the west...
        m.update([obj(TALL_FEET_CUT, oid=1)], [{"camera_id": 1, "range_m": 2.0}], (0.0, 0.0, 0.0), k * 0.2)
    for k in range(6):                                       # ...and from the south
        m.update([obj(TALL_FEET_CUT, oid=2)], [{"camera_id": 2, "range_m": 2.0}],
                 (2.2, -2.2, math.pi / 2), 2.0 + k * 0.2)
    for k in range(8):                                       # later, from the west, V18 returns +1.2 m
        m.update([obj(TALL_FEET_CUT, oid=3)], [{"camera_id": 3, "range_m": 3.2}], (0.0, 0.0, 0.0), 10.0 + k * 0.2)
    rows = m.report()
    assert [r["id"] for r in rows] == ["S1"], rows
    assert (rows[0]["x_m"], rows[0]["y_m"]) == pytest.approx((5.2, 5.0), abs=0.05)
    assert m.ghosts >= 1


def test_two_people_seen_from_different_sides_are_not_ghosts():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    for k in range(6):                                       # person A at (5.2, 5.0) from the west
        m.update([obj(TALL_FEET_CUT, oid=1)], [{"camera_id": 1, "range_m": 2.0}], (0.0, 0.0, 0.0), k * 0.2)
    for k in range(6):                                       # person B at (6.4, 6.2), seen from the north
        m.update([obj(TALL_FEET_CUT, oid=2)], [{"camera_id": 2, "range_m": 2.0}],
                 (3.4, 3.4, -math.pi / 2), 10.0 + k * 0.2)
    rows = m.report(include_candidates=False)
    assert sorted(r["id"] for r in rows) == ["S1", "S2"], rows


def test_point_inside_or_behind_an_outer_wall_is_rejected():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    # facing west from x=3: a 2.98 m range lands at x=0.02, inside the outer wall
    m.update([obj(TALL_FEET_CUT)], [{"camera_id": 1, "range_m": 2.78}], (0.0, 0.0, math.pi), 0.0)
    assert m.report() == [] and m.outside_rejected == 1


def test_edge_clipped_boxes():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    cut_tall = (0.0, 100.0, 60.0, 480.0)                     # left part of a standing body visible
    ms = m.measure([obj(cut_tall)], [{"camera_id": 1, "range_m": 2.0}], (0, 0, 0), 0.0)
    assert len(ms) == 1
    half = m.camera.f * 0.22 / 2.2
    assert ms[0].bearing_deg == pytest.approx(m.camera.bearing_deg(60.0 - half), abs=0.01)
    cut_wide = (0.0, 380.0, 120.0, 470.0)                    # lying body cut by the edge: bearing unknown
    assert m.measure([obj(cut_wide)], [], (0, 0, 0), 1.0) == [] and m.edge_clipped == 1


def test_journal_keeps_every_measurement_and_its_outcome():
    m = SurvivorMapper(CameraModel(height_m=0.82), Grid(), start_pose=(3.0, 5.0, 0.0))
    for k in range(6):
        m.update([obj(TALL_FEET_CUT)], [{"camera_id": 1, "range_m": 2.0}], (0.0, 0.0, 0.0), k * 0.2)
    m.update([obj(TALL_FEET_CUT)], [{"camera_id": 1, "range_m": 2.78}], (0.0, 0.0, math.pi), 2.0)
    rows = m.drain_journal()
    assert [r["outcome"] for r in rows] == ["INLIER"] * 6 + ["OUTSIDE_ARENA"]
    assert rows[0]["survivor"].startswith("cand@") and rows[5]["survivor"] == "S1"
    assert {"pose_x", "bbox", "lidar_m", "range_m", "bearing_deg", "x", "y"} <= set(rows[0])
    assert m.drain_journal() == []
