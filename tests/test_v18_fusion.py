"""
V18 camera + LiDAR fusion and persistent target manager, with an injected clock.
"""

from __future__ import annotations

import contextlib
import io

import pytest


def cam(obj_id, center_x, conf=0.8, label="person"):
    return {"id": obj_id, "class": label, "confidence": conf, "center": [center_x, 360.0]}


def lidar(track_id, bearing_deg, distance_m, speed=0.0, state="CONFIRMED"):
    return {"track_id": track_id, "bearing_deg": bearing_deg % 360.0, "distance_m": distance_m,
            "speed_mps": speed, "state": state}


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


# ---------------------------------------------------------------- fusion ---

def test_fuse_matches_within_gate_and_rejects_outside(v18):
    # camera at +10 deg  <->  pixel 640 + 640 * 10/35
    px = 640.0 + 640.0 * (10.0 / 35.0)
    fused = quiet(v18.fuse_objects, [cam(1, px)], [lidar(7, 12.0, 4.0)])
    assert len(fused) == 1
    f = fused[0]
    assert f["camera_id"] == 1 and f["lidar_id"] == 7 and f["range_m"] == 4.0
    assert f["bearing_error_deg"] == pytest.approx(2.0, abs=1e-6)
    assert f["fusion_status"] == "FUSED"
    assert quiet(v18.fuse_objects, [cam(1, px)], [lidar(7, 21.0, 4.0)]) == [], "11 deg is outside the ±10 gate"


def test_fuse_picks_nearest_bearing_and_uses_each_lidar_track_once(v18):
    px = 640.0
    tracks = [lidar(1, 8.0, 3.0), lidar(2, 2.0, 5.0), lidar(3, 355.0, 6.0)]
    fused = quiet(v18.fuse_objects, [cam(1, px), cam(2, px + 1)], tracks)
    assert [f["lidar_id"] for f in fused] == [2, 3], "nearest first, then the next free track (wrap-around aware)"


def test_fusion_confidence_formula(v18):
    px = 640.0
    f = quiet(v18.fuse_objects, [cam(1, px, conf=0.5)], [lidar(1, 5.0, 2.0)])[0]
    quality = 1.0 - 5.0 / v18.FUSION_GATE_DEG
    assert f["fusion_confidence"] == pytest.approx(round(0.6 * 0.5 + 0.4 * quality, 3))


def test_angular_difference_wraps(v18):
    assert v18.angular_difference(359.0, 1.0) == pytest.approx(2.0)
    assert v18.angular_difference(-10.0, 350.0) == pytest.approx(0.0)
    assert v18.angular_difference("x", 1.0) == 999.0


def test_polar_to_xy_frame_is_right_forward(v18):
    x, y = v18.PersistentTargetManager.polar_to_xy(2.0, 90.0)
    assert x == pytest.approx(2.0) and y == pytest.approx(0.0), "+90 deg bearing = right"
    x, y = v18.PersistentTargetManager.polar_to_xy(3.0, 0.0)
    assert x == pytest.approx(0.0) and y == pytest.approx(3.0), "0 deg bearing = forward"


# ---------------------------------------------------- persistent manager ---

def fused(camera_id, lidar_id, bearing, rng, conf=0.8):
    return {"camera_id": camera_id, "lidar_id": lidar_id, "class": "person",
            "camera_bearing_deg": bearing, "lidar_bearing_deg": bearing, "bearing_error_deg": 0.0,
            "range_m": rng, "speed_mps": 0.0, "confidence": conf, "fusion_confidence": conf}


def test_state_timeline_active_templost_predicted_expired(v18, sim_clock):
    mgr = v18.PersistentTargetManager()
    out = mgr.update([fused(1, 1, 5.0, 4.0)])
    assert [o["state"] for o in out] == ["ACTIVE"]
    sid = out[0]["stable_id"]

    sim_clock.advance(1.0)
    out = mgr.update([])
    assert out[0]["state"] == "TEMP_LOST" and out[0]["stable_id"] == sid

    sim_clock.advance(1.0)                      # age 2.0 s
    out = mgr.update([])
    assert out[0]["state"] == "PREDICTED"
    assert out[0]["x_m"] is not None, "PREDICTED keeps the last valid position"

    sim_clock.advance(1.5)                      # age 3.5 s > EXPIRE_TIMEOUT_SEC
    out = mgr.update([])
    assert out == []
    assert mgr.objects[sid]["state"] == "EXPIRED"


def test_identity_recovered_within_timeout_and_recreated_after(v18, sim_clock):
    mgr = v18.PersistentTargetManager()
    sid = mgr.update([fused(1, 1, 5.0, 4.0)])[0]["stable_id"]
    sim_clock.advance(2.0)
    mgr.update([])
    # same camera id, new lidar id -> exact camera match, same stable id
    out = mgr.update([fused(1, 9, 5.5, 4.1)])
    assert out[0]["stable_id"] == sid and out[0]["state"] == "ACTIVE"

    sim_clock.advance(3.5)                      # expire it
    mgr.update([])
    out = mgr.update([fused(1, 9, 5.5, 4.1)])
    assert out[0]["stable_id"] != sid, "after IDENTITY_RECOVERY_TIMEOUT a new identity is created"


def test_spatial_recovery_when_both_sensor_ids_change(v18, sim_clock):
    mgr = v18.PersistentTargetManager()
    sid = mgr.update([fused(1, 1, 5.0, 4.0)])[0]["stable_id"]
    sim_clock.advance(0.5)
    out = mgr.update([fused(2, 2, 6.0, 4.2)])      # new camera AND lidar ids, close in space
    assert out[0]["stable_id"] == sid, "spatial recovery must keep the identity"


def test_impossible_range_never_creates_object(v18):
    mgr = v18.PersistentTargetManager()
    assert mgr.update([fused(1, 1, 0.0, 0.05)]) == []        # < MIN_VALID_RANGE_M
    assert mgr.update([fused(1, 1, 0.0, 31.0)]) == []        # > MAX_VALID_RANGE_M
    assert mgr.objects == {}


def test_range_jump_is_rejected_and_previous_range_kept(v18, sim_clock):
    mgr = v18.PersistentTargetManager()
    mgr.update([fused(1, 1, 0.0, 4.0)])
    sim_clock.advance(0.1)
    out = mgr.update([fused(1, 1, 0.0, 0.3)])               # 4.0 -> 0.3 m in 0.1 s: outlier
    assert out[0]["outlier_count"] == 1
    assert out[0]["range_m"] == pytest.approx(4.0, abs=0.01)
    assert out[0]["state"] == "ACTIVE"


def test_smoothing_converges_to_measurement(v18, sim_clock):
    mgr = v18.PersistentTargetManager()
    mgr.update([fused(1, 1, 0.0, 4.0)])
    for _ in range(30):
        sim_clock.advance(0.1)
        out = mgr.update([fused(1, 1, 10.0, 4.5)])
    assert out[0]["range_m"] == pytest.approx(4.5, abs=0.02)
    assert out[0]["bearing_deg"] == pytest.approx(10.0, abs=0.1)
    x, y = v18.PersistentTargetManager.polar_to_xy(4.5, 10.0)
    assert out[0]["x_m"] == pytest.approx(x, abs=0.03) and out[0]["y_m"] == pytest.approx(y, abs=0.03)


def test_v18_1_has_camera_grace_period(v18):
    """V18.1: objects must not be aged between two normal camera frames."""
    assert hasattr(v18, "CAMERA_GRACE_SEC")
    assert 0.1 <= v18.CAMERA_GRACE_SEC <= 0.5
    assert v18.CAMERA_GRACE_SEC < v18.TEMP_LOST_TIMEOUT_SEC


def test_save_json_is_atomic_and_contains_sections(v18, tmp_path):
    v18.OUTPUT_JSON = str(tmp_path / "out.json")
    mgr = v18.PersistentTargetManager()
    persistent = mgr.update([fused(1, 1, 0.0, 4.0)])
    v18.save_json(12, [cam(1, 640.0)], [lidar(1, 0.0, 4.0)], [fused(1, 1, 0.0, 4.0)], persistent)
    import json
    data = json.loads((tmp_path / "out.json").read_text())
    assert data["protocol"] == "NIDAR_V18_HEADLESS_PERSISTENT_FUSION" and data["frame"] == 12
    for key in ("camera_objects", "lidar_tracks", "fused_objects", "persistent_objects", "configuration"):
        assert key in data
    assert data["persistent_objects"][0]["state"] == "ACTIVE"
    assert "fusion_confidence" in data["persistent_objects"][0]
    assert "confidence" not in data["persistent_objects"][0], \
        "documented: V18 emits fusion_confidence, the bridge reads confidence -> target server maps it"
    assert not (tmp_path / "out.json.tmp").exists()
