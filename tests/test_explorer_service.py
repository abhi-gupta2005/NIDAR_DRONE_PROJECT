"""
Explorer service (src/mission/nidar_explorer_service.py, 2026-11-09): the Pi "brain" that turns
SLAM V6 output into NIDAR_NAV_V1 plans for the flight interface.

Pinned: no plan unless SLAM is fresh, TRACKING and accepted (a stale/lost SLAM cancels the plan
and holds); raw LiDAR hits only from the very scan SLAM used; DONE -> look recorded, next plan;
a new obstacle on the remaining path -> cancel + a new plan; a reconnecting flight interface gets
the same plan again; the TCP link carries newline JSON both ways.
"""

from __future__ import annotations

import json
import math
import socket
import time

import numpy as np
import pytest

from src.mission import nidar_explorer as ne
from src.mission import nidar_explorer_service as svc
from test_explorer import grid_with


def slam_state(t, x=0.0, y=0.0, yaw_deg=0.0, status="TRACKING", accepted=True, score=0.9, scan_t=None):
    return {"timestamp": t, "scan_timestamp": t if scan_t is None else scan_t, "status": status,
            "pose": {"x_m": x, "y_m": y, "yaw_deg": yaw_deg}, "match": {"score": score, "accepted": accepted},
            "map": {"resolution_m": 0.1, "origin_m": [-10.0, -10.0]}}


def room():
    """6 x 6 m walled room, all known free, drone in the middle; unknown outside."""
    walls = [(-3.0, -3.0, 3.0, -2.9), (-3.0, 2.9, 3.0, 3.0), (-3.0, -3.0, -2.9, 3.0), (2.9, -3.0, 3.0, 3.0)]
    return grid_with(walls, free_box=(-2.9, -2.9, 2.9, 2.9))


def brain(**kw):
    return svc.Brain(ne.Explorer(drone_radius=0.25, **kw))


def kinds(msgs):
    return [m["type"] for m in msgs]


def test_scan_points_follow_the_laserscan_convention():
    scan = {"angle_min": -math.pi, "angle_increment": math.pi / 2, "range_min": 0.15, "range_max": 12.0,
            "ranges": [2.0, None, 1.0, 3.0]}                  # -180, -90 (invalid), 0 (ahead), +90 (left)
    pts = svc.scan_points(scan)
    assert pts.shape == (3, 2)
    assert np.allclose(pts[0], [-2.0, 0.0], atol=1e-9)
    assert np.allclose(pts[1], [1.0, 0.0], atol=1e-9)
    assert np.allclose(pts[2], [0.0, 3.0], atol=1e-9)


def test_holds_without_fresh_tracking_slam():
    b = brain()
    assert kinds(b.step(100.0, None, None, [])) == ["hold"]
    assert b.step(100.1, slam_state(98.0), room(), []) == []      # 2.1 s old: same hold, not repeated
    assert kinds(b.step(100.3, slam_state(100.3, status="LOST"), None, [])) == ["hold"]
    assert b.step(100.4, slam_state(100.4, accepted=False), None, []) == []
    assert b.holding.startswith("SLAM lost")
    out = b.step(100.5, slam_state(100.5), None, [])
    assert kinds(out) == ["pose", "plan"]
    p = out[1]
    assert p["frame"] == "slam" and p["id"] == 1 and p["waypoints"] and p["end"] in ("look", "none")
    assert p["pose"] == [0.0, 0.0, 0.0]


def test_stale_slam_cancels_the_running_plan():
    b = brain()
    b.step(10.0, slam_state(10.0), room(), [])
    assert b.plan is not None
    out = b.step(12.0, slam_state(10.0), None, [])
    assert kinds(out) == ["cancel", "hold"] and out[0]["id"] == 1
    out = b.step(12.1, slam_state(12.1), None, [])                 # SLAM back: a new plan
    assert "plan" in kinds(out) and [m for m in out if m["type"] == "plan"][0]["id"] == 2


def test_done_records_the_look_and_sends_the_next_plan():
    b = brain()
    b.step(10.0, slam_state(10.0), room(), [])
    first = b.plan_msg
    gx, gy = first["goal"]
    out = b.step(10.1, slam_state(10.1, gx, gy), None, [{"type": "status", "id": 99, "state": "DONE"}])
    assert "plan" not in kinds(out) and b.ex.stats["looks"] == 0      # wrong id: ignored
    out = b.step(10.2, slam_state(10.2, gx, gy), None, [{"type": "status", "id": 1, "state": "DONE", "wp": 9}])
    assert b.ex.stats["looks"] == 1 and (gx, gy) == tuple(np.round(b.ex.visited[0], 3))
    nxt = [m for m in out if m["type"] == "plan"]
    assert nxt and nxt[0]["id"] == 2


def test_new_obstacle_on_the_path_cancels_and_replans():
    b = brain()
    b.step(10.0, slam_state(10.0), room(), [])
    b.plan = ne.Plan([(0.0, 0.0), (2.0, 0.0)], "none", (2.0, 0.0), "test leg")    # flying plan 1 east
    # the scan SLAM used at t=11.0 sees a new obstacle across the leg, 1 m ahead (|y| <= 0.55 m)
    angles = np.linspace(-0.5, 0.5, 41)
    scan = {"angle_min": float(angles[0]), "angle_increment": float(angles[1] - angles[0]), "range_min": 0.15,
            "range_max": 12.0, "ranges": [1.0 / math.cos(a) for a in angles], "received": 11.0}
    b.feed_scan(scan)
    out = b.step(11.0, slam_state(11.0, scan_t=11.0), None, [])
    assert kinds(out)[:2] == ["pose", "cancel"] and out[1]["id"] == 1
    new = [m for m in out if m["type"] == "plan"]
    assert new and new[0]["id"] == 2
    assert b.ex.plan_still_safe(0.0, 0.0, [tuple(w) for w in new[0]["waypoints"]])
    assert not b.ex.plan_still_safe(0.0, 0.0, [(2.0, 0.0)])


def test_hits_only_from_the_scan_slam_used():
    b = brain()
    b.step(10.0, slam_state(10.0), room(), [])
    scan = {"angle_min": 0.0, "angle_increment": 0.01, "range_min": 0.15, "range_max": 12.0,
            "ranges": [1.0] * 10, "received": 10.5}
    b.feed_scan(scan)
    b.step(10.6, slam_state(10.6, scan_t=10.6), None, [])          # SLAM pose is from another scan
    assert not b.ex.hits.any()
    b.step(10.7, slam_state(10.7, scan_t=10.5, score=0.3), None, [])   # poor match: not trusted
    assert not b.ex.hits.any()
    b.step(10.8, slam_state(10.8, scan_t=10.5), None, [])
    assert b.ex.hits.any()


def test_reconnect_gets_the_same_plan_and_land_completes():
    b = brain(mission_time_s=0.0)                       # time is up after the first plan: RETURN
    b.step(10.0, slam_state(10.0), room(), [])                 # take-off point (0, 0)
    b.step(10.1, slam_state(10.1, 1.0, 0.5), None, [{"type": "status", "id": 1, "state": "DONE"}])
    assert b.plan_msg["end"] == "land" and b.resend() == [b.plan_msg]
    assert b.plan_msg["waypoints"][-1] == [0.0, 0.0]          # lands on the take-off point itself
    pid = b.plan_id
    b.step(10.2, slam_state(10.2, 0.0, 0.0), None, [{"type": "status", "id": pid, "state": "DONE"}])
    assert b.complete and b.state_json(10.2)["state"] == "COMPLETE" and b.resend() == []
    assert b.state_json(10.2)["return_reason"].startswith("mission time")    # the GCS says WHY it came home


@pytest.mark.network
def test_link_loopback_newline_json():
    link = svc.Link("127.0.0.1", 0)
    try:
        c = socket.create_connection(("127.0.0.1", link.port), timeout=2.0)
        for _ in range(50):
            if link.connected:
                break
            time.sleep(0.02)
        assert link.connected and link.new_client.is_set()
        link.send({"type": "plan", "id": 3, "waypoints": [[1.0, 2.0]]})
        c.settimeout(2.0)
        buf = b""
        while b"\n" not in buf:
            buf += c.recv(4096)
        msg = json.loads(buf.split(b"\n")[0])
        assert msg["protocol"] == "NIDAR_NAV_V1" and msg["id"] == 3
        c.sendall(b'{"type":"status","id":3,"state":"FLY')
        c.sendall(b'ING","wp":1}\n')
        got = []
        for _ in range(50):
            got += link.drain()
            if got:
                break
            time.sleep(0.02)
        assert got == [{"type": "status", "id": 3, "state": "FLYING", "wp": 1}]
        c.close()
    finally:
        link.close()


def test_repeatedly_blocked_plans_end_in_a_hold_not_a_loop():
    """Offline SLAM loop 2026-11-09 (before the follower kept its alignment): 14 729 plans BLOCKED in a row.
    Five in a row without a DONE -> HOLD for good, no more plans."""
    b = brain()
    b.step(10.0, slam_state(10.0), room(), [])
    t = 10.0
    for k in range(4):
        t += 0.1
        out = b.step(t, slam_state(t), None, [{"type": "status", "id": b.plan_id, "state": "BLOCKED"}])
        assert "plan" in kinds(out)
    t += 0.1
    out = b.step(t, slam_state(t), None, [{"type": "status", "id": b.plan_id, "state": "BLOCKED"}])
    assert kinds(out)[-1] == "hold" and "stuck" in out[-1]["reason"] and "plan" not in kinds(out)
    t += 0.1
    assert "plan" not in kinds(b.step(t, slam_state(t), None, []))
    assert b.state_json(t)["state"] == "HOLD"


def test_pose_messages_carry_the_age_of_their_scan():
    b = brain()
    out = b.step(10.0, slam_state(10.0, scan_t=9.8), room(), [])
    pose = [m for m in out if m["type"] == "pose"][0]
    assert pose["age"] == pytest.approx(0.2) and pose["t"] == 9.8
