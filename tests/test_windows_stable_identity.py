"""
Windows perception layer (nidar_v11_windows.py) -- stable-identity logic
tested with synthetic detections; no camera, no YOLOE.

Detection(class_id, prompt, label, confidence, box, tracker_id)
"""

from __future__ import annotations

import pytest


def det(w, box, tracker_id, label="person", conf=0.7):
    return w.Detection(class_id=0, prompt=label, label=label, confidence=conf, box=box, tracker_id=tracker_id)


def shift(box, dx=0.0, dy=0.0):
    x1, y1, x2, y2 = box
    return (x1 + dx, y1 + dy, x2 + dx, y2 + dy)


BOX = (500.0, 200.0, 700.0, 600.0)


def test_geometry_helpers(windows_module):
    w = windows_module
    assert w.box_iou(BOX, BOX) == pytest.approx(1.0)
    assert w.box_iou(BOX, shift(BOX, 1000)) == 0.0
    assert w.center(BOX) == (600.0, 400.0)
    assert w.size_similarity(BOX, BOX) == pytest.approx(1.0)
    assert w.canonical_label("Cell Phone") == "phone"
    assert w.canonical_label("smartphone") == "phone"
    assert w.canonical_label("over ear headphones") == "headphones"
    assert w.canonical_label("unknown-thing") == "unknown-thing"


def test_filter_detection_rejects_trackerless_small_and_huge_boxes(windows_module):
    w = windows_module
    assert w.filter_detection(det(w, BOX, 1), 1280, 720) is True
    assert w.filter_detection(det(w, BOX, None), 1280, 720) is False, "tracker ID is mandatory"
    assert w.filter_detection(det(w, (0, 0, 10, 10), 1), 1280, 720) is False, "below MIN_BOX_WIDTH/HEIGHT"
    assert w.filter_detection(det(w, (0, 0, 1279, 719), 1), 1280, 720) is False, "covers > MAX_BOX_AREA_RATIO"


def test_duplicate_alias_detections_are_merged_but_two_people_are_kept(windows_module):
    w = windows_module
    phone_a = det(w, (100, 100, 180, 220), 1, "phone", 0.6)
    phone_b = det(w, (104, 103, 184, 224), 2, "phone", 0.5)      # alias/duplicate of the same phone
    kept = w.merge_duplicate_detections([phone_a, phone_b], 1280, 720)
    assert [d.tracker_id for d in kept] == [1], "highest-confidence duplicate wins"

    p1 = det(w, (300, 100, 420, 500), 3, "person", 0.8)
    p2 = det(w, (380, 110, 500, 510), 4, "person", 0.7)         # overlapping neighbour, IoU < 0.65
    kept = w.merge_duplicate_detections([p1, p2], 1280, 720)
    assert len(kept) == 2, "two nearby people must remain separate objects"


def test_new_identity_needs_consecutive_hits_and_trackerless_never_creates(windows_module):
    w = windows_module
    mgr = w.StableIdentityManager(1280, 720)
    mgr.update([det(w, BOX, None)], 1)
    assert mgr.objects == {}, "tracker_id=None must never create a stable ID"

    for frame in range(1, w.CONFIRM_HITS + 1):
        mgr.update([det(w, BOX, 5)], frame)
    obj = next(iter(mgr.objects.values()))
    assert obj.confirmed is True and obj.state == "CONFIRMED"
    assert obj.stable_id == 1 and obj.tracker_id == 5


def test_temp_lost_then_lost_after_prediction_budget(windows_module):
    w = windows_module
    mgr = w.StableIdentityManager(1280, 720)
    frame = 0
    for _ in range(10):
        frame += 1
        mgr.update([det(w, BOX, 5)], frame)
    sid = next(iter(mgr.objects))
    frame += 1
    mgr.update([], frame)
    obj = mgr.objects[sid]
    assert obj.state == "TEMP_LOST" and obj.missed == 1
    assert obj.prediction_budget in (w.PREDICTION_BUDGET_VERY_LOW, w.PREDICTION_BUDGET_LOW,
                                     w.PREDICTION_BUDGET_MEDIUM, w.PREDICTION_BUDGET_HIGH)
    budget = obj.prediction_budget
    for _ in range(budget + 1):
        frame += 1
        mgr.update([], frame)
    assert sid not in mgr.objects, "object must be LOST once missed > prediction budget"
    assert mgr.history[sid].state == "LOST"
    events = [e["event"] for e in mgr.lifecycle_events if e["id"] == sid]
    assert events[:2] == ["CREATED", "TENTATIVE->CONFIRMED"]
    assert "TEMP_LOST" in events and events[-1] == "LOST"


def test_tracker_id_switch_keeps_stable_id(windows_module):
    """BoT-SORT re-assigns IDs after occlusion; the stable ID must survive."""
    w = windows_module
    mgr = w.StableIdentityManager(1280, 720)
    frame = 0
    for _ in range(8):
        frame += 1
        mgr.update([det(w, shift(BOX, dx=2.0 * frame), 5)], frame)
    sid = next(iter(mgr.objects))
    for _ in range(3):                       # short occlusion
        frame += 1
        mgr.update([], frame)
    frame += 1
    visible = mgr.update([det(w, shift(BOX, dx=2.0 * frame), 9)], frame)   # new tracker ID 9
    assert [o.stable_id for o in visible] == [sid]
    assert mgr.objects[sid].tracker_id == 9
    assert mgr.objects[sid].recoveries == 1 and mgr.objects[sid].state == "RECOVERED"
    assert len(mgr.objects) == 1, "no second identity may be created"


def test_class_flicker_does_not_change_locked_label(windows_module):
    w = windows_module
    mgr = w.StableIdentityManager(1280, 720)
    frame = 0
    for _ in range(12):
        frame += 1
        mgr.update([det(w, BOX, 5, "bottle", 0.7)], frame)
    sid = next(iter(mgr.objects))
    assert mgr.objects[sid].label == "bottle"
    frame += 1
    mgr.update([det(w, BOX, 5, "book", 0.45)], frame)        # one noisy frame
    assert mgr.objects[sid].label == "bottle", "a single contradicting frame must not flip the class"


def test_predicted_box_is_clamped_and_reliability_decays(windows_module):
    w = windows_module
    mgr = w.StableIdentityManager(1280, 720)
    frame = 0
    for _ in range(10):
        frame += 1
        mgr.update([det(w, shift(BOX, dx=30.0 * frame), 5)], frame)     # fast mover
    sid = next(iter(mgr.objects))
    frame += 1
    mgr.update([], frame)
    obj = mgr.objects[sid]
    for gap in range(1, 6):
        box = w.predicted_box_for(obj, frame + gap, 1280, 720)
        assert 0 <= box[0] < box[2] <= 1279 and 0 <= box[1] < box[3] <= 719
    assert w.prediction_reliability_for(obj, frame) in {"HIGH", "MEDIUM", "LOW", "STALE"}
    assert w.prediction_reliability_for(obj, frame + 1000) == "STALE"
