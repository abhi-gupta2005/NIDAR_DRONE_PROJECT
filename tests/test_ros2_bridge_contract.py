"""
Contract tests for the ROS 2 node LOGIC (nidar_bridge.py, px4_setpoint_sim.py)
run through src/simulation/ros2_shim.py -- no DDS, no PX4.  They pin down
the safety behaviour: when a command may be non-zero, how it is limited,
and how fast it decays to zero when the target or the link disappears.

Running them under WSL with ROS 2 sourced uses the same stub on purpose
(force_stub=True): these are logic tests, not integration tests.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from src.simulation.pipeline_clock import SimClock
from src.simulation.ros2_shim import load_node_module, Shim, ROS2_PACKAGE_DIR

pytestmark = pytest.mark.skipif(
    not Path(ROS2_PACKAGE_DIR, "nidar_bridge.py").exists(),
    reason="ROS 2 package source not present in ros2_ws_src/")


@pytest.fixture()
def bridge():
    shim = Shim()
    mod, _ = load_node_module("nidar_bridge", force_stub=True, shim=shim)
    clock = SimClock(1000.0)
    mod.time = clock
    shim.clock_seconds = clock.time
    node = mod.NIDARBridge()
    return mod, node, shim, clock


@pytest.fixture()
def setpoint():
    shim = Shim()
    mod, _ = load_node_module("px4_setpoint_sim", force_stub=True, shim=shim)
    clock = SimClock(1000.0)
    mod.time = clock
    shim.clock_seconds = clock.time
    node = mod.PX4AirMouseController()
    return mod, node, shim, clock


def last_cmd(shim):
    return [float(v) for v in shim.published["/nidar/airmouse_command"][-1].data]


def target(state="ACTIVE", conf=0.8, x=0.0, y=3.0, rng=3.0, sid=1):
    return {"state": state, "confidence": conf, "x_m": x, "y_m": y, "range_m": rng, "stable_id": sid}


# ------------------------------------------------------------- bridge -----

def test_bridge_zeroes_for_non_active_or_low_confidence(bridge):
    mod, node, shim, clock = bridge
    node.process_target(target(state="TEMP_LOST", y=6.0))
    assert last_cmd(shim) == [0.0, 0.0, 0.0, 0.0]
    node.process_target(target(conf=0.29, y=6.0))
    assert last_cmd(shim) == [0.0, 0.0, 0.0, 0.0]
    assert node.last_target_time == 0.0, "rejected targets must not refresh the timeout"


def test_bridge_p_control_and_limits(bridge):
    mod, node, shim, clock = bridge
    node.process_target(target(y=5.0, x=0.5))                 # 2 m too far, 0.5 m right
    f, l, v, valid = last_cmd(shim)
    assert f == pytest.approx(mod.KP_FORWARD * 2.0)
    assert l == pytest.approx(mod.KP_LATERAL * 0.5)
    assert v == 0.0 and valid == 1.0

    node.process_target(target(y=2.0, x=-0.05))                # 1 m too close, lateral in deadzone
    f, l, _, _ = last_cmd(shim)
    assert f == pytest.approx(-mod.KP_FORWARD * 1.0) and l == 0.0

    node.process_target(target(y=3.1, x=0.0))                  # inside range deadzone -> hold
    assert last_cmd(shim)[:3] == [0.0, 0.0, 0.0] and last_cmd(shim)[3] == 1.0

    node.process_target(target(y=30.0, x=-30.0))
    f, l, _, _ = last_cmd(shim)
    assert f == mod.MAX_FORWARD and l == -mod.MAX_LATERAL


def test_bridge_timeout_zeroes_after_one_second(bridge):
    mod, node, shim, clock = bridge
    node.process_target(target(y=5.0))
    assert last_cmd(shim)[3] == 1.0
    n = len(shim.published["/nidar/airmouse_command"])
    clock.advance(0.9)
    node.timeout_check()
    assert len(shim.published["/nidar/airmouse_command"]) == n, "no zero before TARGET_TIMEOUT"
    clock.advance(0.2)
    node.timeout_check()
    assert last_cmd(shim) == [0.0, 0.0, 0.0, 0.0]


def test_bridge_message_layout_is_forward_lateral_vertical_valid(bridge):
    mod, node, shim, clock = bridge
    node.publish_command(1.0, 2.0, 3.0, 1.0)
    msg = shim.published["/nidar/airmouse_command"][-1]
    assert list(msg.data) == [1.0, 2.0, 3.0, 1.0]


def test_bridge_pi_endpoint_is_configurable_or_documented(bridge):
    """PI_HOST is hard-coded today; record the value so a changed Pi IP is caught."""
    mod = bridge[0]
    assert mod.PI_PORT == 5010
    assert isinstance(mod.PI_HOST, str) and mod.PI_HOST


# ------------------------------------------------------------- setpoint ---

def test_setpoint_clamps_and_never_moves_vertically(setpoint):
    mod, node, shim, clock = setpoint
    node.command_callback(mod.Float32MultiArray(data=[1.5, -1.5, 0.7, 1.0]))
    assert node.forward == mod.MAX_FORWARD == 0.25
    assert node.lateral == -mod.MAX_LATERAL
    assert node.vertical == 0.0
    node.command_callback(mod.Float32MultiArray(data=[0.2, 0.1, 0.0, 0.0]))     # valid flag false
    assert node.forward == 0.0 and node.lateral == 0.0 and node.command_valid is False
    node.command_callback(mod.Float32MultiArray(data=[0.2]))                     # malformed
    assert node.forward == 0.0


def test_setpoint_body_to_ned_rotation_and_velocity_mode(setpoint):
    mod, node, shim, clock = setpoint
    # heading 90 deg (east): body-forward -> east, body-right -> south
    yaw = math.pi / 2
    node.attitude_callback(mod.VehicleAttitude(q=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]))
    assert node.heading == pytest.approx(yaw)
    node.command_callback(mod.Float32MultiArray(data=[0.2, 0.1, 0.0, 1.0]))
    node.publish_setpoint()
    sp = shim.published["/fmu/in/trajectory_setpoint"][-1]
    north, east, down = sp.velocity
    assert north == pytest.approx(-0.1, abs=1e-9) and east == pytest.approx(0.2, abs=1e-9) and down == 0.0
    assert all(math.isnan(v) for v in sp.position), "velocity control requires NaN position"
    assert sp.yaw == pytest.approx(yaw)
    node.publish_offboard_mode()
    mode = shim.published["/fmu/in/offboard_control_mode"][-1]
    assert mode.velocity is True and mode.position is False


def test_setpoint_times_out_stale_commands(setpoint):
    mod, node, shim, clock = setpoint
    node.command_callback(mod.Float32MultiArray(data=[0.2, 0.0, 0.0, 1.0]))
    clock.advance(mod.COMMAND_TIMEOUT + 0.05)
    node.publish_setpoint()
    assert list(shim.published["/fmu/in/trajectory_setpoint"][-1].velocity) == [0.0, 0.0, 0.0]


def test_setpoint_never_requests_offboard_or_arm_by_default(setpoint):
    mod, node, shim, clock = setpoint
    assert mod.ALLOW_OFFBOARD is False and mod.ALLOW_ARM is False
    clock.advance(10.0)
    node.control_loop()
    assert "/fmu/in/vehicle_command" not in shim.published or not shim.published["/fmu/in/vehicle_command"]
    assert node.offboard_requested is False and node.arm_requested is False


def test_bridge_sends_zero_on_ctrl_c_before_shutdown():
    """
    2026-11-02 (WSL): Ctrl+C gave "publisher's context is invalid" -- the final zero was never sent.
    Fixed bridge: rclpy.init(signal_handler_options=NO); zero published while the context is valid,
    and shutdown happens only afterwards.
    """
    shim = Shim()
    mod, _ = load_node_module("nidar_bridge", force_stub=True, shim=shim)
    import sys
    rclpy = sys.modules["rclpy"]
    calls = []
    rclpy.init = lambda *a, **k: calls.append(("init", k.get("signal_handler_options")))
    rclpy.shutdown = lambda: calls.append(("shutdown", None))
    mod.time = SimClock(0.0)                       # make the 3 x 50 ms sleeps instant

    def spin(node):
        node.publish_command(0.5, 0.2, 0.0, 1.0)   # a live command, then Ctrl+C
        calls.append(("spin", None))
        raise KeyboardInterrupt

    rclpy.spin = spin
    mod.main()
    sent = [list(m.data) for m in shim.published["/nidar/airmouse_command"]]
    assert sent[0] == [0.5, 0.2, 0.0, 1.0]
    assert sent[-1] == [0.0, 0.0, 0.0, 0.0] and len(sent) >= 2, "final zero must be published"
    assert calls[0] == ("init", rclpy.signals.SignalHandlerOptions.NO)
    assert calls[-1][0] == "shutdown", "shutdown must come after the zero command"
