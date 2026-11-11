"""
Rig driver AUTONOMOUS mode (gazebo/ros2/nidar_rig_driver.py --brain, 2026-11-09): follows the
explorer service's NIDAR_NAV_V1 plans. Pure parts tested here (no ROS, no sockets):
SLAM->odometry alignment taken once while standing still; plan -> dense path with turn-in-place
look headings; DONE / BLOCKED / land handshake; LiDAR guard; hold / cancel / lost link -> stop.
The slow test runs Brain + BrainFollower in a closed loop on arena s2 with a rotated odometry
frame (tests/nav_loop_sim.py).
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("nidar_rig_driver_t", ROOT / "gazebo/ros2/nidar_rig_driver.py")
rig = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rig)


def plan(pid=1, wps=((1.0, 0.0),), end="none", looks=(), pose=None):
    m = {"type": "plan", "id": pid, "frame": "slam", "waypoints": [list(w) for w in wps], "end": end,
         "look_yaws": list(looks), "goal": list(wps[-1]) if wps else [0, 0], "reason": "test"}
    if pose is not None:
        m["pose"] = list(pose)
    return m


def settle(f, pose, t0=0.0, secs=1.2):
    t = t0
    while t < t0 + secs:
        f.update(pose, t)
        t += 0.05
    return t


def fly(f, pose, t, ahead=None, steps=4000, dt=0.05):
    """Integrate the commanded body velocities exactly; returns (pose, t, replies)."""
    x, y, yaw = pose
    replies = []
    for _ in range(steps):
        vx, vy, wz = f.update((x, y, yaw), t, ahead)
        replies += f.drain()
        if f.state != "FLYING":
            break
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = x + (c * vx - s * vy) * dt, y + (s * vx + c * vy) * dt
        yaw = rig.wrap(yaw + wz * dt)
        t += dt
    return (x, y, yaw), t, replies


def test_align_maps_slam_to_odometry():
    A = rig.Align((5.0, -2.0, math.radians(90)), (1.0, 0.0, 0.0))     # SLAM (1,0,0) is odom (5,-2,90deg)
    assert A.xy(1.0, 0.0) == pytest.approx((5.0, -2.0))
    assert A.xy(2.0, 0.0) == pytest.approx((5.0, -1.0))               # SLAM +x = odom +y
    assert A.heading(0.0) == pytest.approx(math.radians(90))


def test_plan_to_path_skips_tiny_legs_and_appends_the_look():
    path, wp_of = rig.plan_to_path((0.0, 0.0, 0.3), [(0.04, 0.03), (1.0, 0.0)], [1.0, 2.5])
    assert path[0] == (0.0, 0.0, 0.3)
    assert all(p[2] == pytest.approx(0.0) for p in path[1:5])         # the leg faces its direction
    assert path[4][:2] == pytest.approx((1.0, 0.0))
    assert path[-2:] == [(1.0, 0.0, 1.0), (1.0, 0.0, 2.5)]
    assert wp_of[1] == 1 and len(wp_of) == len(path)


def test_waits_for_alignment_while_moving_then_flies_and_reports_done():
    f = rig.BrainFollower(speed=0.4)
    f.on_message(plan(pose=(0.0, 0.0, 0.0)), (3.0, 4.0, 0.0), 0.0)
    assert f.align is None                                            # no history yet: not standing still
    # the rig is being moved: no alignment, no motion
    for k in range(30):
        assert f.update((3.0 + 0.02 * k, 4.0, 0.0), 0.05 * k) == (0.0, 0.0, 0.0)
    f.on_message({"type": "pose", "x": 0.0, "y": 0.0, "yaw": 0.0}, (3.6, 4.0, 0.0), 1.5)
    assert f.align is None
    t = settle(f, (3.6, 4.0, 0.0), 1.5)
    f.on_message({"type": "pose", "x": 0.0, "y": 0.0, "yaw": 0.0}, (3.6, 4.0, 0.0), t)
    assert f.align is not None
    f.update((3.6, 4.0, 0.0), t)
    assert f.state == "FLYING"
    end, t, replies = fly(f, (3.6, 4.0, 0.0), t)
    assert end[:2] == pytest.approx((4.6, 4.0), abs=0.06)              # SLAM (1, 0) in odometry
    assert replies[-1] == {"type": "status", "id": 1, "state": "DONE", "wp": 1}
    assert any(r["state"] == "FLYING" for r in replies[:-1]) and f.state == "HOLD"


def test_look_turns_in_place_and_land_completes():
    f = rig.BrainFollower()
    t = settle(f, (0.0, 0.0, 0.0))
    f.on_message(plan(1, [(0.0, 0.0)], "look", [math.radians(90), math.radians(180)], pose=(0, 0, 0)),
                 (0.0, 0.0, 0.0), t)
    end, t, replies = fly(f, (0.0, 0.0, 0.0), t)
    assert abs(rig.wrap(end[2] - math.pi)) < math.radians(4) and math.hypot(*end[:2]) < 0.05
    assert replies[-1]["state"] == "DONE"
    f.on_message(plan(2, [(0.5, 0.0), (0.0, 0.0)], "land"), end, t)
    end, t, replies = fly(f, end, t)
    assert f.state == "COMPLETE" and replies[-1]["state"] == "DONE" and math.hypot(*end[:2]) < 0.06


def test_lidar_guard_blocks_and_missing_lidar_only_waits():
    f = rig.BrainFollower()
    t = settle(f, (0.0, 0.0, 0.0))
    f.on_message(plan(1, [(2.0, 0.0)], pose=(0, 0, 0)), (0.0, 0.0, 0.0), t)
    assert f.update((0.0, 0.0, 0.0), t, lambda d: None) == (0.0, 0.0, 0.0)   # no scan: wait, still FLYING
    assert f.state == "FLYING" and not [r for r in f.drain() if r["state"] == "BLOCKED"]
    v = f.update((0.0, 0.0, 0.0), t + 0.05, lambda d: 3.0)
    assert v[0] > 0.0
    wall_at = 0.8

    def ahead(direction, pose_x=[0.0]):
        return wall_at - pose_x[0] if abs(direction) < 0.6 else 5.0
    x, tt = 0.0, t + 0.1
    replies = []
    for _ in range(400):
        vx, vy, wz = f.update((x, 0.0, 0.0), tt, lambda d: ahead(d, [x]))
        replies += f.drain()
        if f.state != "FLYING":
            break
        x += vx * 0.05
        tt += 0.05
    blocked = [r for r in replies if r["state"] == "BLOCKED"]
    assert blocked and blocked[0]["id"] == 1 and f.state == "HOLD"
    assert wall_at - x >= rig.STOP_AHEAD_M - 0.03


def test_hold_cancel_and_lost_link_stop_the_rig():
    f = rig.BrainFollower()
    t = settle(f, (0.0, 0.0, 0.0))
    f.on_message(plan(4, [(2.0, 0.0)], pose=(0, 0, 0)), (0.0, 0.0, 0.0), t)
    assert f.update((0.0, 0.0, 0.0), t)[0] > 0
    f.on_message({"type": "cancel", "id": 3}, (0.0, 0.0, 0.0), t)       # another plan's id: ignored
    assert f.state == "FLYING"
    f.on_message({"type": "cancel", "id": 4, "reason": "x"}, (0.0, 0.0, 0.0), t)
    assert f.state == "HOLD" and f.update((0.0, 0.0, 0.0), t) == (0.0, 0.0, 0.0)
    f.on_message(plan(5, [(2.0, 0.0)]), (0.0, 0.0, 0.0), t)
    assert f.update((0.0, 0.0, 0.0), t)[0] > 0
    f.on_message({"type": "hold", "reason": "SLAM lost"}, (0.0, 0.0, 0.0), t)
    assert f.update((0.0, 0.0, 0.0), t) == (0.0, 0.0, 0.0)
    f.on_message(plan(6, [(2.0, 0.0)]), (0.0, 0.0, 0.0), t)
    assert f.update((0.0, 0.0, 0.0), t)[0] > 0
    f.disconnected()
    assert f.state == "WAIT" and f.update((0.0, 0.0, 0.0), t) == (0.0, 0.0, 0.0)
    assert f.update(None, t) == (0.0, 0.0, 0.0)                         # stale odometry


@pytest.mark.slow
def test_brain_and_rig_explore_arena_s2_through_the_protocol():
    import nav_loop_sim as nl
    r = nl.run(ROOT / "gazebo/layouts/nidar_airmouse_arena_s2.json", mode="map", odom_offset=(1.3, -2.1, 0.7))
    assert r["aligned"] and r["complete"], r["rig_events"][-5:]
    assert r["coverage"] >= 0.95 and len(r["survivors_seen"]) == 6
    assert r["min_wall_clearance"] >= 0.25, r["min_clear_at"]
    assert r["end_dist_to_pad"] < 0.1 and r["blocked"] == 0


def test_alignment_follows_slam_and_ignores_poses_taken_while_turning():
    """2026-11-09: SLAM's first pose was 0.7 deg off; a fixed alignment put the rig 0.16 m off its plan 13 m away.
    Here the start alignment is 0.7 deg / 0.05 m wrong: later poses pull it in, so a 10 m leg ends on target."""
    truth_yaw, err = math.radians(90.0), math.radians(0.7)
    to_slam = rig.Align((7.9, -0.75, truth_yaw), (0.0, 0.0, 0.0))     # the true SLAM <- odometry relation
    f = rig.BrainFollower()
    t = settle(f, (7.9, -0.75, truth_yaw))
    f.on_message({"type": "pose", "x": 0.05, "y": 0.0, "yaw": -err, "age": 0.0}, (7.9, -0.75, truth_yaw), t)
    assert f.align is not None and abs(rig.wrap(f.align.yaw - truth_yaw - err)) < 1e-9
    f.on_message(plan(1, [(10.0, 0.0)], pose=(0, 0, 0)), (7.9, -0.75, truth_yaw), t)
    x, y, yaw = 7.9, -0.75, truth_yaw
    for k in range(5000):
        vx, vy, wz = f.update((x, y, yaw), t)
        f.drain()
        if f.state != "FLYING":
            break
        if k % 10 == 0:                                       # 2 Hz true SLAM poses, scan 0.1 s old
            sx, sy, syaw = to_slam.inv(x, y, yaw)
            f.on_message({"type": "pose", "x": sx, "y": sy, "yaw": syaw, "age": 0.1 - rig.LINK_LATENCY_S},
                         (x, y, yaw), t + 0.1)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, yaw = x + (c * vx - s * vy) * 0.05, y + (s * vx + c * vy) * 0.05, rig.wrap(yaw + wz * 0.05)
        t += 0.05
    gx, gy = to_slam.xy(10.0, 0.0)
    assert math.hypot(x - gx, y - gy) < 0.06, (x, y, gx, gy)          # fixed alignment: 0.16 m off
    assert f.realigned > 10
    # a pose whose scan was taken while the rig turned fast is not used
    n = f.realigned
    for k in range(20):
        f.update((x, y, rig.wrap(yaw + 0.05 * k)), t + 0.05 * k)
    f.on_message({"type": "pose", "x": 0.0, "y": 0.0, "yaw": 0.0, "age": 0.3}, (x, y, yaw), t + 0.5)
    assert f.realigned == n
