#!/usr/bin/env python3
"""
Minimal ROS 2 stand-ins so the NIDAR ROS 2 node *logic* can be executed and
unit-tested on machines without ROS 2 (Windows .venv, Raspberry Pi, CI).

What this is NOT: a ROS 2 emulator.  No DDS, no QoS, no executor.  It only
provides the handful of attributes the NIDAR nodes touch at construction time
(create_publisher / create_subscription / create_timer / get_logger /
get_clock) and records every published message so tests can assert on them.

If the real `rclpy` is importable (e.g. in WSL with ROS 2 Jazzy sourced) the
real modules are used and the recorder is not installed -- then you must
call rclpy.init() yourself before constructing nodes.

Usage:
    from src.simulation.ros2_shim import load_node_module
    bridge_mod, shim = load_node_module("nidar_bridge")
    node = bridge_mod.NIDARBridge()
    node.process_target({...})
    shim.published["/nidar/airmouse_command"][-1].data  -> [f, l, v, valid]
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
import types
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ROS2_PACKAGE_DIR = os.path.join(_PROJECT_ROOT, "ros2_ws_src", "nidar_airmouse", "nidar_airmouse")


class _Logger:
    def __init__(self):
        self.records: List[Tuple[str, str]] = []

    def _log(self, level, msg):
        self.records.append((level, str(msg)))

    def info(self, msg):
        self._log("info", msg)

    def warn(self, msg):
        self._log("warn", msg)

    warning = warn

    def error(self, msg):
        self._log("error", msg)

    def debug(self, msg):
        self._log("debug", msg)


class _Publisher:
    def __init__(self, shim, topic):
        self.shim = shim
        self.topic = topic

    def publish(self, msg):
        self.shim.published[self.topic].append(msg)


class _Timer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback

    def cancel(self):
        pass


class _Subscription:
    def __init__(self, msg_type, topic, callback, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos


class _Time:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _Clock:
    def __init__(self, shim):
        self.shim = shim

    def now(self):
        return _Time(int(self.shim.clock_seconds() * 1e9))


class Shim:
    """Holds recorded publications and a controllable clock."""

    def __init__(self):
        self.published: Dict[str, List] = defaultdict(list)
        self.subscriptions: List[_Subscription] = []
        self.timers: List[_Timer] = []
        self.loggers: List[_Logger] = []
        self._clock_seconds = 0.0

    def clock_seconds(self) -> float:
        return self._clock_seconds

    def set_clock(self, seconds: float) -> None:
        self._clock_seconds = float(seconds)

    def make_node_class(self):
        shim = self

        class Node:
            def __init__(self, name: str):
                self._name = name
                self._logger = _Logger()
                shim.loggers.append(self._logger)

            def create_publisher(self, msg_type, topic, qos):
                return _Publisher(shim, topic)

            def create_subscription(self, msg_type, topic, callback, qos):
                sub = _Subscription(msg_type, topic, callback, qos)
                shim.subscriptions.append(sub)
                return sub

            def create_timer(self, period, callback):
                timer = _Timer(period, callback)
                shim.timers.append(timer)
                return timer

            def get_logger(self):
                return self._logger

            def get_clock(self):
                return _Clock(shim)

            def destroy_node(self):
                pass

        return Node


def _generic_msg_class(name: str, fields: Dict[str, object]):
    def __init__(self, **kwargs):
        for key, default in fields.items():
            value = kwargs.get(key, default)
            setattr(self, key, list(value) if isinstance(value, list) else value)

    def __repr__(self):
        return f"{name}({', '.join(f'{k}={getattr(self, k)!r}' for k in fields)})"

    return type(name, (), {"__init__": __init__, "__repr__": __repr__, "_fields": list(fields)})


def _install_stub_modules(shim: Shim) -> None:
    """Register fake rclpy / std_msgs / px4_msgs modules in sys.modules."""
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None
    rclpy.spin_once = lambda node, timeout_sec=0.0: None
    rclpy.ok = lambda: True

    rclpy_signals = types.ModuleType("rclpy.signals")

    class SignalHandlerOptions:
        NO = 0
        SIGINT = 1
        SIGTERM = 2
        ALL = 3

    rclpy_signals.SignalHandlerOptions = SignalHandlerOptions
    rclpy.signals = rclpy_signals

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = shim.make_node_class()

    rclpy_qos = types.ModuleType("rclpy.qos")

    class ReliabilityPolicy:
        BEST_EFFORT = "BEST_EFFORT"
        RELIABLE = "RELIABLE"

    class HistoryPolicy:
        KEEP_LAST = "KEEP_LAST"
        KEEP_ALL = "KEEP_ALL"

    class DurabilityPolicy:
        VOLATILE = "VOLATILE"
        TRANSIENT_LOCAL = "TRANSIENT_LOCAL"

    class QoSProfile:
        def __init__(self, **kwargs):
            self.reliability = kwargs.get("reliability", ReliabilityPolicy.RELIABLE)
            self.history = kwargs.get("history", HistoryPolicy.KEEP_LAST)
            self.depth = kwargs.get("depth", 10)
            self.durability = kwargs.get("durability", DurabilityPolicy.VOLATILE)

    rclpy_qos.ReliabilityPolicy = ReliabilityPolicy
    rclpy_qos.HistoryPolicy = HistoryPolicy
    rclpy_qos.DurabilityPolicy = DurabilityPolicy
    rclpy_qos.QoSProfile = QoSProfile

    rclpy.node = rclpy_node
    rclpy.qos = rclpy_qos

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32MultiArray = _generic_msg_class("Float32MultiArray", {"data": []})
    std_msgs.msg = std_msgs_msg

    nan3 = [float("nan")] * 3
    px4_msgs = types.ModuleType("px4_msgs")
    px4_msgs_msg = types.ModuleType("px4_msgs.msg")
    px4_msgs_msg.VehicleLocalPosition = _generic_msg_class(
        "VehicleLocalPosition",
        {"timestamp": 0, "x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "heading": 0.0})
    px4_msgs_msg.VehicleOdometry = _generic_msg_class("VehicleOdometry", {"timestamp": 0})
    px4_msgs_msg.VehicleAttitude = _generic_msg_class("VehicleAttitude", {"timestamp": 0, "q": [1.0, 0.0, 0.0, 0.0]})
    px4_msgs_msg.OffboardControlMode = _generic_msg_class(
        "OffboardControlMode",
        {"timestamp": 0, "position": False, "velocity": False, "acceleration": False, "attitude": False,
         "body_rate": False, "thrust_and_torque": False, "direct_actuator": False})
    px4_msgs_msg.TrajectorySetpoint = _generic_msg_class(
        "TrajectorySetpoint",
        {"timestamp": 0, "position": nan3, "velocity": nan3, "acceleration": nan3, "jerk": nan3,
         "yaw": 0.0, "yawspeed": 0.0})
    px4_msgs_msg.VehicleCommand = _generic_msg_class(
        "VehicleCommand",
        {"timestamp": 0, "command": 0, "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0,
         "param5": 0.0, "param6": 0.0, "param7": 0.0, "target_system": 0, "target_component": 0,
         "source_system": 0, "source_component": 0, "from_external": False, "confirmation": 0})
    px4_msgs.msg = px4_msgs_msg

    for name, module in (("rclpy", rclpy), ("rclpy.node", rclpy_node), ("rclpy.qos", rclpy_qos),
                         ("rclpy.signals", rclpy_signals),
                         ("std_msgs", std_msgs), ("std_msgs.msg", std_msgs_msg),
                         ("px4_msgs", px4_msgs), ("px4_msgs.msg", px4_msgs_msg)):
        sys.modules[name] = module


def real_ros2_available() -> bool:
    try:
        importlib.import_module("rclpy")
        importlib.import_module("px4_msgs.msg")
        return True
    except Exception:
        return False


def load_node_module(module_name: str, package_dir: Optional[str] = None,
                     force_stub: bool = False,
                     shim: Optional[Shim] = None) -> Tuple[types.ModuleType, Optional[Shim]]:
    """
    Load one node module (e.g. "nidar_bridge") from the ROS 2 package source
    copy under ros2_ws_src/.  Returns (module, shim); shim is None when the
    real ROS 2 stack was used.  Pass an existing `shim` to share one recorder
    (and one clock) between several node modules.
    """
    package_dir = package_dir or ROS2_PACKAGE_DIR
    path = os.path.join(package_dir, module_name + ".py")
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"{path} not found. Copy the ROS 2 package source into ros2_ws_src/ first "
            f"(cp -r ~/px4_ros2_ws/src/. /mnt/d/NIDAR_DRONE_PROJECT/ros2_ws_src/)."
        )

    if shim is None and (force_stub or not real_ros2_available()):
        shim = Shim()
    if shim is not None:
        _install_stub_modules(shim)

    unique = f"nidar_ros2_{module_name}_{'stub' if shim else 'real'}"
    spec = importlib.util.spec_from_file_location(unique, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[unique] = module
    spec.loader.exec_module(module)
    return module, shim
