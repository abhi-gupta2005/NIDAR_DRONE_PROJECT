"""
NIDAR_PERCEPTION_V1 contract between the Windows sender, the Pi receiver and
V18 fusion.  Checked on three sources: the recorded 12 Oct 2026 stream, the
Windows object_record() function, and the simulated world.
"""

from __future__ import annotations

import json
import math

import pytest

from src.simulation import nidar_world

PACKET_KEYS = {"protocol", "frame", "timestamp", "camera", "objects"}
CAMERA_KEYS = {"width", "height", "source"}

# Keys consumed downstream (receiver CSV row, V18 fusion, integration monitor).
RECEIVER_OBJECT_KEYS = {"id", "class", "state", "spatial_status", "confidence", "motion_confidence",
                        "track_stability_score", "tracker_id", "center", "predicted_center",
                        "prediction_horizon_frames", "prediction_reliability"}
V18_OBJECT_KEYS = {"id", "class", "center", "confidence"}

# Complete record produced by nidar_v11_windows.object_record()
FULL_OBJECT_KEYS = {
    "frame", "timestamp", "id", "class", "state", "confidence", "tracker_id", "bbox", "center",
    "predicted_bbox", "predicted_center", "normalized_center", "bbox_size", "velocity_px_frame",
    "motion_confidence", "track_stability_score", "observation_age_frames",
    "prediction_horizon_frames", "prediction_budget_frames", "prediction_reliability",
    "spatial_status", "confirmed", "age_frames", "missed_frames", "max_missed_frames",
    "recoveries", "tracker_switches", "first_frame", "last_seen_frame", "state_since_frame",
    "current_observation",
}


def _check_packet(packet):
    assert PACKET_KEYS <= set(packet)
    assert packet["protocol"] == "NIDAR_PERCEPTION_V1"
    assert isinstance(packet["frame"], int) and packet["frame"] >= 1
    assert isinstance(packet["timestamp"], float)
    assert CAMERA_KEYS <= set(packet["camera"])
    w, h = packet["camera"]["width"], packet["camera"]["height"]
    assert w > 0 and h > 0
    for obj in packet["objects"]:
        assert RECEIVER_OBJECT_KEYS <= set(obj), set(RECEIVER_OBJECT_KEYS) - set(obj)
        assert V18_OBJECT_KEYS <= set(obj)
        x1, y1, x2, y2 = obj["bbox"]
        assert 0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h, obj["bbox"]
        cx, cy = obj["center"]
        assert math.isclose(cx, (x1 + x2) / 2, abs_tol=0.02)
        assert math.isclose(cy, (y1 + y2) / 2, abs_tol=0.02)
        assert 0.0 <= obj["confidence"] <= 1.0
        assert obj["state"] in {"CONFIRMED", "TEMP_LOST", "RECOVERED", "TENTATIVE"}
        assert obj["spatial_status"] in {"OBSERVED", "PREDICTED"}
        assert obj["prediction_reliability"] in {"MEASURED", "HIGH", "MEDIUM", "LOW", "STALE"}


def test_recorded_stream_matches_schema(recorded_packets):
    frames = [p["frame"] for p in recorded_packets]
    assert frames == sorted(frames) and len(set(frames)) == len(frames), "frames must be strictly increasing"
    for p in recorded_packets:
        _check_packet(p)
        for obj in p["objects"]:
            assert FULL_OBJECT_KEYS <= set(obj)


def test_recorded_stream_timestamps_increase(recorded_packets):
    ts = [p["timestamp"] for p in recorded_packets]
    assert all(b >= a for a, b in zip(ts, ts[1:]))


def test_windows_object_record_matches_schema(windows_module):
    w = windows_module
    det = w.Detection(class_id=0, prompt="person", label="person", confidence=0.71,
                      box=(300.0, 200.0, 500.0, 600.0), tracker_id=7)
    mgr = w.StableIdentityManager(1280, 720)
    for frame_no in range(1, 4):
        mgr.update([det], frame_no)
    obj = next(iter(mgr.objects.values()))
    rec = w.object_record(obj, 3, 1700000000.0, 1280, 720)
    assert set(rec) == FULL_OBJECT_KEYS
    packet = {"protocol": "NIDAR_PERCEPTION_V1", "frame": 3, "timestamp": 1700000000.0,
              "camera": {"width": 1280, "height": 720, "source": "test"}, "objects": [rec]}
    _check_packet(packet)
    assert rec["state"] == "CONFIRMED" and rec["confirmed"] is True
    assert rec["spatial_status"] == "OBSERVED" and rec["prediction_reliability"] == "MEASURED"


def test_world_packets_match_schema(world):
    for frame in range(1, 30):
        packet = world.camera_packet(frame, frame * 0.09)
        _check_packet(packet)
        for obj in packet["objects"]:
            assert set(obj) == FULL_OBJECT_KEYS


def test_wire_format_is_newline_delimited_compact_json():
    """Windows PiPerceptionPublisher and the replay tool must produce identical bytes."""
    from src.simulation.nidar_pipeline_sim import encode_packet_like_windows
    from src.simulation.nidar_replay import encode_packet
    packet = {"protocol": "NIDAR_PERCEPTION_V1", "frame": 1, "timestamp": 1.5,
              "camera": {"width": 1280, "height": 720, "source": "x"}, "objects": []}
    expected = (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")
    assert encode_packet_like_windows(packet) == expected
    assert encode_packet(packet) == expected
    assert expected.endswith(b"\n") and b"\n" not in expected[:-1]


def test_v18_camera_bearing_uses_center_x(v18):
    assert v18.camera_bearing_from_object({"center": [640.0, 360.0]}) == pytest.approx(0.0)
    assert v18.camera_bearing_from_object({"center": [1280.0, 0.0]}) == pytest.approx(35.0)
    assert v18.camera_bearing_from_object({"center": [0.0, 0.0]}) == pytest.approx(-35.0)
    assert v18.camera_bearing_from_object({}) is None
    assert v18.camera_bearing_from_object({"center": ["bad", 1]}) is None
