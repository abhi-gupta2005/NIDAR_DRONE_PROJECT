"""
PX4 route flyer (gazebo/ros2/nidar_px4_route_flyer.py, 2026-11-09) -- simulation-only offboard flight.

The ROS/PX4 part can only run in WSL. Tested here: the route geometry, the arena<->PX4 NED
frame, the fence, the LiDAR guard and the whole RouteExecutor flying every arena with a
simulated vehicle (first-order position/yaw lag) and the ray-cast LiDAR (walls + survivor
bodies at the LiDAR plane):
  * every straight leg keeps >= 0.49 m from every wall face (x500 prop tips reach 0.39 m);
  * the drone never gets closer than 0.60 m (centre) to a standing survivor, although two
    survivors stand 0.10 / 0.16 m from a door-cell centre (s2 R7 lying, s3 R7 standing):
    the probe into the room stops on the LiDAR;
  * yaw setpoint rate <= 30 deg/s, the route ends on the launch pad with LAND.
"""

from __future__ import annotations

import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("px4fly", ROOT / "gazebo" / "ros2" / "nidar_px4_route_flyer.py")
fly = importlib.util.module_from_spec(spec)
sys.modules["px4fly"] = fly
spec.loader.exec_module(fly)
from src.simulation import arena_raycast as ar  # noqa: E402


def layout(seed):
    return json.loads((ROOT / "gazebo" / "layouts" / f"nidar_airmouse_arena_s{seed}.json").read_text())


def seg_dist(p1, p2, q1, q2):
    """Shortest distance between segments p1p2 and q1q2 (2-D)."""
    def pt_seg(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        L2 = ax * ax + ay * ay
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L2))
        return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1, d2 = cross(q1, q2, p1), cross(q1, q2, p2)
    d3, d4 = cross(p1, p2, q1), cross(p1, p2, q2)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


ROUTES = [(s, "rooms") for s in (1, 2, 3)] + [(1, f"survivor:S{k}") for k in range(1, 7)]


@pytest.mark.parametrize("seed,route", ROUTES)
def test_every_leg_keeps_clear_of_the_walls_and_inside_the_fence(seed, route):
    L = layout(seed)
    tasks, info = fly.build_route(L, route)
    fence = fly.Fence(L)
    half = L["grid"]["wall_m"] / 2
    p = info["pad"]
    for t in tasks:
        if t.kind != "translate":
            continue
        q = t.target
        assert fence.inside(*q), q
        assert abs(q[0] - p[0]) < 1e-6 or abs(q[1] - p[1]) < 1e-6, "legs are axis-aligned (no diagonals)"
        for x1, y1, x2, y2 in L["walls"]:
            d = seg_dist(p, q, (x1, y1), (x2, y2)) - half              # wall = centre line +- half thickness
            assert d >= 0.49, f"leg {p}->{q} passes {d:.3f} m from wall face {x1, y1, x2, y2}"
        p = q
    assert tasks[-1].kind == "land" and p == info["pad"]


def test_arena_ned_frame_round_trip_and_yaw():
    f = fly.ArenaNed(7.9, -0.75, n0=0.3, e0=-0.2, d0=0.01)
    n, e, d = f.to_ned(7.9, 0.25, 0.64)                             # 1 m north of the pad, 0.64 m up
    assert (n, e, d) == pytest.approx((1.3, -0.2, 0.01 - 0.64))
    assert f.to_arena(n, e, d) == pytest.approx((7.9, 0.25, 0.64))
    assert f.to_ned(8.9, -0.75, 0)[1] == pytest.approx(0.8)         # east = +x arena
    assert fly.ArenaNed.yaw_to_ned(math.pi / 2) == pytest.approx(0.0)        # facing north
    assert fly.ArenaNed.yaw_to_ned(0.0) == pytest.approx(math.pi / 2)        # facing east
    for a in (-3.0, -1.0, 0.0, 0.7, 2.9):
        assert fly.ArenaNed.yaw_to_enu(fly.ArenaNed.yaw_to_ned(a)) == pytest.approx(a)


def test_scan_min_sector():
    r = [math.inf] * 360
    inc = 2 * math.pi / 360
    r[180] = 0.6                                                    # straight ahead (angle_min = -pi)
    r[270] = 0.5                                                    # 90 deg left
    assert fly.scan_min(r, -math.pi, inc, 0.16, 0.0) == pytest.approx(0.6)
    assert fly.scan_min(r, -math.pi, inc, 0.16, None) == pytest.approx(0.5)
    assert fly.scan_min(r, -math.pi, inc, 0.16, math.pi) == math.inf


def test_refuses_without_the_gazebo_world():
    assert fly.gazebo_world_running("nidar_airmouse_arena_s1", "/clock\n/world/nidar_airmouse_arena_s1/clock\n")
    assert not fly.gazebo_world_running("nidar_airmouse_arena_s1", "/fmu/out/vehicle_status\n")
    assert not fly.gazebo_world_running("nidar_airmouse_arena_s1", "/world/nidar_airmouse_arena_s2/clock\n")


def test_dry_run_needs_no_ros():
    r = subprocess.run([sys.executable, str(ROOT / "gazebo/ros2/nidar_px4_route_flyer.py"), "--seed", "1",
                        "--dry-run"], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0 and "route rooms" in r.stdout and "land" in r.stdout


def test_limits_are_enforced():
    L = layout(1)
    tasks, info = fly.build_route(L, "rooms")
    with pytest.raises(ValueError):
        fly.RouteExecutor(tasks, fly.Fence(L), info["pad"], alt=2.0)
    with pytest.raises(ValueError):
        fly.RouteExecutor(tasks, fly.Fence(L), info["pad"], speed=1.5)


# ------------------------------------------------------- simulated flight ---
class SimDrone:
    """PX4-like position loop: v = feed-forward + (setpoint - position) * P with P = 0.95/s (MPC_XY_P),
    <= vmax; yaw <= 60 deg/s, altitude tau 0.6 s. Without feed-forward this lags 0.32 m at 0.3 m/s --
    what made the first PX4 flight (2026-11-09) stop-and-go at 0.12 m/s."""

    def __init__(self, x, y, yaw, vmax=1.0, use_ff=True):
        self.x, self.y, self.yaw, self.alt = x, y, yaw, 0.0
        self.vmax, self.use_ff = vmax, use_ff

    def step(self, sp, dt):
        ff = (sp.get("vx", 0.0), sp.get("vy", 0.0)) if self.use_ff else (0.0, 0.0)
        vx, vy = ff[0] + (sp["x"] - self.x) * 0.95, ff[1] + (sp["y"] - self.y) * 0.95
        v = math.hypot(vx, vy)
        if v > self.vmax:
            vx, vy = vx / v * self.vmax, vy / v * self.vmax
        self.x += vx * dt
        self.y += vy * dt
        self.alt += (sp["alt"] - self.alt) / 0.6 * dt
        e = fly.wrap(sp["yaw"] - self.yaw)
        self.yaw = fly.wrap(self.yaw + max(-math.radians(60) * dt, min(math.radians(60) * dt, e)))


def simulate(seed, route="rooms", stuck=False, t_max=2400.0, vmax=1.0, use_ff=True):
    L = layout(seed)
    arena = ar.load_arena(ar.default_layout(seed))
    tasks, info = fly.build_route(L, route)
    ex = fly.RouteExecutor(tasks, fly.Fence(L), info["pad"], start_yaw=math.pi / 2)
    d = SimDrone(*info["pad"], math.pi / 2, vmax=vmax, use_ff=use_ff)
    dt, t = 0.05, 0.0
    scan_r = None
    log = []
    prev_yaw = ex.sp_yaw
    max_rate = 0.0
    while t < t_max:
        if int(round(t / dt)) % 2 == 0:                               # LiDAR at 10 Hz
            plane = 0.36 + d.alt
            r, _ = ar.cast(arena, d.x, d.y, d.yaw, plane)
            scan_r = r

        def scan(direction, _r=scan_r):
            return fly.scan_min(_r, -math.pi, 2 * math.pi / len(_r), 0.16, direction)

        out = ex.step(t, (d.x, d.y), d.alt, d.yaw, scan)
        if t > 0:
            max_rate = max(max_rate, abs(fly.wrap(out["yaw"] - prev_yaw)) / dt)
        prev_yaw = out["yaw"]
        if out["phase"] in ("LAND", "ABORT"):
            break
        if not stuck or ex.phase == "TAKEOFF":
            d.step(out, dt)
        log.append((t, d.x, d.y, d.alt, d.yaw))
        t += dt
    return ex, np.array(log), L, max_rate


def _standing_clearance(log, L):
    return min(min(math.hypot(x - s["x"], y - s["y"]) for _, x, y, _, _ in log)
               for s in L["survivors"] if s["posture"] == "standing")


def _wall_clearance(log, L):
    half = L["grid"]["wall_m"] / 2
    pts = log[::10]
    best = math.inf
    for x1, y1, x2, y2 in L["walls"]:
        for _, x, y, alt, _ in pts:
            if alt < 0.3:
                continue
            best = min(best, seg_dist((x, y), (x, y), (x1, y1), (x2, y2)) - half)
    return best


@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_flies_the_whole_rooms_route_safely(seed):
    ex, log, L, max_rate = simulate(seed)
    assert ex.phase == "LAND", ex.events[-5:]
    assert ex.abort_reason is None
    end = log[-1]
    assert math.hypot(end[1] - L["launch_pad"]["center"][0], end[2] - L["launch_pad"]["center"][1]) < 0.15
    assert max_rate <= math.radians(30) + 1e-6
    assert _wall_clearance(log, L) >= 0.42, "prop tips (0.39 m) must stay off the walls"
    assert _standing_clearance(log, L) >= 0.60, "drone centre >= 0.6 m from a standing body (0.2 m radius)"
    assert log[:, 3].max() <= 0.70


@pytest.mark.slow
def test_probe_stops_for_a_survivor_standing_in_the_door():
    ex, log, L, _ = simulate(3)                                      # s3 R7: S5 0.16 m from the door-cell centre
    assert any("probe stopped" in e for e in ex.events), ex.events
    s5 = next(s for s in L["survivors"] if s["id"] == "S5")
    assert min(math.hypot(x - s5["x"], y - s5["y"]) for _, x, y, _, _ in log) >= 0.60


def test_lands_when_the_vehicle_does_not_follow():
    ex, log, L, _ = simulate(1, route="survivor:S2", stuck=True, t_max=120)
    assert ex.phase == "ABORT" and ("no progress" in ex.abort_reason or "tracking" in ex.abort_reason), ex.abort_reason


def test_px4_topic_names_with_and_without_version_suffix():
    """2026-11-09: PX4 1.18 published /fmu/out/vehicle_status_v4; arming did nothing -> the inputs are
    found in the ROS graph the same way (exact name, else highest _vN)."""
    graph = [("/fmu/in/vehicle_command_v1", ["px4_msgs/msg/VehicleCommand"]),
             ("/fmu/in/vehicle_command_mode_executor", ["px4_msgs/msg/VehicleCommand"]),
             ("/fmu/in/trajectory_setpoint", ["px4_msgs/msg/TrajectorySetpoint"]),
             ("/fmu/out/vehicle_status_v4", ["px4_msgs/msg/VehicleStatus"]),
             ("/fmu/out/vehicle_status_v2", ["px4_msgs/msg/VehicleStatus"])]
    assert fly.pick_px4_topic(graph, "in", "vehicle_command") == ("/fmu/in/vehicle_command_v1",
                                                                  "px4_msgs/msg/VehicleCommand")
    assert fly.pick_px4_topic(graph, "in", "trajectory_setpoint")[0] == "/fmu/in/trajectory_setpoint"
    assert fly.pick_px4_topic(graph, "out", "vehicle_status")[0] == "/fmu/out/vehicle_status_v4"
    assert fly.pick_px4_topic(graph, "in", "offboard_control_mode") is None


def test_feed_forward_gives_the_commanded_speed():
    """Leg J2 -> J3 -> F3 of survivor:S2 took 32 s for 3.5 m on real PX4 (no feed-forward, carrot pausing)."""
    ex, log, L, _ = simulate(1, route="survivor:S2", t_max=400)
    assert ex.phase == "LAND", ex.events[-3:]
    moving = []
    for (t0, x0, y0, *_), (t1, x1, y1, *_) in zip(log[:-1], log[1:]):
        moving.append(math.hypot(x1 - x0, y1 - y0) / (t1 - t0))
    fast = [v for v in moving if v > 0.05]
    assert np.median(fast) >= 0.27, np.median(fast)


def test_slow_vehicle_is_not_mistaken_for_a_stuck_one():
    """A slow simulation (vehicle limited to 0.1 m/s) keeps making progress: no 'no progress' abort."""
    ex, log, L, _ = simulate(1, route="survivor:S2", t_max=900, vmax=0.1)
    assert ex.phase == "LAND", ex.events[-3:]


def test_position_gap_lands_only_when_px4_data_is_really_missing():
    """2026-11-09: Gazebo in WSL pauses ~2 s every ~35 s (LiDAR and PX4 stop together); a 2.01 s pause landed a
    flight. A pause (LiDAR silent too) is waited out up to 8 s; a gap with the LiDAR still coming lands at 2 s."""
    g = fly.gap_action
    assert g(0.3, 0.1) == "ok"
    assert g(1.5, 0.1) == "hold" and g(2.1, 0.1) == "land"            # PX4 data missing, Gazebo running
    assert g(2.1, 2.1) == "hold" and g(7.9, 7.9) == "hold"             # whole simulation paused
    assert g(8.1, 8.1) == "land"
    # 10:27 run: Gazebo resumed, the LiDAR 0.2 s ahead of PX4's position -> must not land
    assert g(2.2, 0.05, paused_in_gap=True, since_lidar_back=0.2) == "hold"
    assert g(2.2, 0.05, paused_in_gap=True, since_lidar_back=2.5) == "land"   # PX4 did not follow the LiDAR
    assert g(2.2, 0.05, paused_in_gap=False) == "land"                         # no pause: PX4 data lost


def _corner_executor():
    """In the air at (5.80, 3.70) facing north after a leg; next leg turns west to (4.75, 3.70)."""
    L = layout(1)
    ex = fly.RouteExecutor([fly.Task("translate", (5.80, 3.70)), fly.Task("translate", (4.75, 3.70)),
                            fly.Task("land")], fly.Fence(L), (5.80, 2.65), start_yaw=math.pi / 2)
    ex.phase, ex.sp_alt = "ROUTE", ex.alt
    return ex


def test_corner_turn_waits_until_back_on_the_corner_point():
    """09:55 crash: overshot 0.11 m at a corner, turned at once with the props 0.39 m from the wall."""
    ex = _corner_executor()
    t, far = 0.0, math.inf
    # fly leg 1 exactly
    while ex.i == 0 and t < 30:
        ex.step(t, tuple(ex.sp), ex.alt, math.pi / 2, lambda d: far)
        t += 0.05
    assert ex.i == 1
    for _ in range(40):                                       # overshot 0.11 m north of the corner: no turn
        out = ex.step(t, (5.80, 3.81), ex.alt, math.pi / 2, lambda d: far)
        t += 0.05
        assert out["yaw"] == pytest.approx(math.pi / 2) and out["phase"] == "ROUTE"
    for _ in range(15):                                       # back within 0.06 m and still: now it turns
        out = ex.step(t, (5.80, 3.73), ex.alt, out["yaw"], lambda d: far)
        t += 0.05
    assert out["yaw"] > math.pi / 2 + 1e-3
    y0 = out["yaw"]
    for _ in range(10):                                       # drifts off again mid-turn: the yaw holds
        out = ex.step(t, (5.80, 3.62), ex.alt, out["yaw"], lambda d: far)
        t += 0.05
    assert out["yaw"] == pytest.approx(y0, abs=0.06)


def test_leg_after_a_turn_ignores_the_lidar_until_the_heading_is_steady():
    """10:44 false LAND: a scan from mid-turn showed the side wall 0.43 m 'ahead'."""
    ex = _corner_executor()
    t, far = 0.0, math.inf
    while ex.i == 0 and t < 30:
        ex.step(t, tuple(ex.sp), ex.alt, math.pi / 2, lambda d: far)
        t += 0.05
    hdg = math.pi / 2
    t_steady = None
    while t < 60 and ex.i == 1 and ex.phase == "ROUTE":
        out = ex.step(t, (5.80, 3.70) if ex.sub != "move" else tuple(ex.sp), ex.alt, hdg,
                      (lambda d: 0.43) if (t_steady is None or t - t_steady < 1.0) else (lambda d: 0.60))
        hdg = out["yaw"]                                       # the drone follows the yaw setpoint
        if abs(fly.wrap(hdg - math.pi)) < 1e-6 and t_steady is None:
            t_steady = t
        t += 0.05
        if ex.sub == "move":
            break
    assert ex.phase == "ROUTE" and ex.abort_reason is None and ex.sub == "move"
    assert t - t_steady >= fly.SCAN_SETTLE_S - 0.1                 # waited for a steady heading first
