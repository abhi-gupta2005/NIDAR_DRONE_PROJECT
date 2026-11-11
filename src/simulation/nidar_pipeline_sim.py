#!/usr/bin/env python3
"""
NIDAR pipeline simulation -- runs the REAL NIDAR code on a co-simulated world.

Chain exercised (real project code unless marked "sim"):

  World (sim) -> camera packet
      transport "direct": handed to the V18 stage in memory
      transport "file"  : written atomically like the receiver does, read by V18 load_camera_packet()
      transport "tcp"   : encoded with the Windows publisher wire format, sent over a loopback
                          socket to the real receiver ClientHandler, which writes the file
                          that V18 load_camera_packet() reads
  World (sim) -> LidarScan -> LidarObstacleDetector -> LidarObstacleTrackerV3 -> V18 normalize_track
  V18 fuse_objects -> V18 PersistentTargetManager (clock injected) -> V18 save_json
  nidar_airmouse_server.make_packet (the Pi's port-5010 server) -> nidar_bridge.NIDARBridge.process_target (ROS 2 stubbed)
  -> px4_setpoint_sim.PX4AirMouseController (ROS 2 stubbed) -> NED velocity setpoint

The world provides ground truth, so the report contains measured errors, not
impressions.  Nothing here changes the behaviour of the production modules;
they are imported as-is (paths and the `time` module reference are injected).

Run:
    python -m src.simulation.nidar_pipeline_sim --scenario all --out outputs/simulation
    python -m src.simulation.nidar_pipeline_sim --scenario stale_camera --transport tcp
"""

from __future__ import annotations

import argparse
import contextlib
import importlib
import importlib.util
import io
import json
import math
import os
import signal
import socket
import sys
import tempfile
import threading
import time as _real_time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

_THIS = Path(__file__).resolve()
PROJECT_ROOT = _THIS.parents[2]
SRC_ROOT = PROJECT_ROOT / "src"
for _p in (PROJECT_ROOT, SRC_ROOT, SRC_ROOT / "core", SRC_ROOT / "tracking",
           SRC_ROOT / "perception", SRC_ROOT / "simulation", SRC_ROOT / "communication"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from src.simulation import nidar_world as world_mod           # noqa: E402
from src.simulation.ros2_shim import load_node_module, Shim   # noqa: E402
from src.simulation.pipeline_clock import SimClock            # noqa: E402,F401  (re-exported)


# ---------------------------------------------------------------------------
# Loading production modules without running their main loops
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def _preserve_signal_handlers():
    saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, handler in saved.items():
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass


def load_v18(clock: Optional[SimClock] = None):
    """
    Import src/perception/nidar_v18_headless_persistent_fusion.py as a module.

    NIDAR_V18_PATH can point at another copy (e.g. archive/...) to A/B-test a
    change against the same scenarios.
    """
    path = Path(os.environ.get("NIDAR_V18_PATH", str(SRC_ROOT / "perception" / "nidar_v18_headless_persistent_fusion.py")))
    spec = importlib.util.spec_from_file_location("nidar_v18_sim_instance", str(path))
    module = importlib.util.module_from_spec(spec)
    with _preserve_signal_handlers():
        spec.loader.exec_module(module)
    if clock is not None:
        module.time = clock
    return module


def load_receiver():
    path = SRC_ROOT / "communication" / "nidar_pi_receiver.py"
    spec = importlib.util.spec_from_file_location("nidar_receiver_sim_instance", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_airmouse_server(clock: Optional[SimClock] = None):
    """The Pi's port-5010 server (project root), with an injected clock."""
    path = Path(os.environ.get("NIDAR_AIRMOUSE_SERVER_PATH", str(PROJECT_ROOT / "nidar_airmouse_server.py")))
    spec = importlib.util.spec_from_file_location("nidar_airmouse_server_sim_instance", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if clock is not None:
        module.time = clock
    return module


def load_slam():
    with _preserve_signal_handlers():
        return importlib.import_module("src.core.nidar_slam_v5_1")


def packet_status(packet: Optional[Dict]) -> Optional[str]:
    """Status of a port-5010 packet; derived for servers that send no "status" field."""
    if not packet:
        return None
    if packet.get("status"):
        return packet["status"]
    target = packet.get("target")
    if target is None:
        return "NO_TARGET"
    return "TRACKING" if target.get("state") == "ACTIVE" else "TARGET_NOT_ACTIVE"


def encode_packet_like_windows(packet: Dict) -> bytes:
    """Exactly PiPerceptionPublisher._worker's payload (nidar_v11_windows.py)."""
    return (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")


# ---------------------------------------------------------------------------
# Camera transports
# ---------------------------------------------------------------------------

class DirectTransport:
    name = "direct"

    def __init__(self):
        self.latest: Optional[Dict] = None

    def send(self, packet: Dict) -> None:
        self.latest = packet

    def read(self) -> Optional[Dict]:
        return self.latest

    def disconnect(self):
        pass

    def reconnect(self):
        pass

    def close(self):
        pass


class FileTransport:
    """Writes the packet the way nidar_pi_receiver.process_packet does."""

    name = "file"

    def __init__(self, v18_module, directory: Path):
        self.path = directory / "nidar_latest_perception.json"
        v18_module.CAMERA_PACKET = str(self.path)
        self.v18 = v18_module

    def send(self, packet: Dict) -> None:
        temp = self.path.with_suffix(".tmp")
        temp.write_text(json.dumps(packet, indent=2), encoding="utf-8")
        temp.replace(self.path)

    def read(self) -> Optional[Dict]:
        return self.v18.load_camera_packet()

    def disconnect(self):
        pass

    def reconnect(self):
        pass

    def close(self):
        pass


class TcpTransport:
    """
    Real receiver code (ClientHandler) behind a loopback socket.

    The sender side reproduces PiPerceptionPublisher's wire format byte for
    byte; the receiver side IS nidar_pi_receiver.ClientHandler.
    """

    name = "tcp"

    def __init__(self, v18_module, directory: Path):
        self.receiver = load_receiver()
        self.receiver.LATEST_FILE = directory / "nidar_latest_perception.json"
        self.receiver.OBJECT_LOG = directory / "nidar_perception_objects.csv"
        v18_module.CAMERA_PACKET = str(self.receiver.LATEST_FILE)
        self.v18 = v18_module

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(2)
        self.server.settimeout(0.5)
        self.port = self.server.getsockname()[1]
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._stop = threading.Event()
        self._accept_thread.start()
        self.sock: Optional[socket.socket] = None
        self.connections = 0
        self.send_failures = 0
        self.reconnect()

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.connections += 1
            self.receiver.ClientHandler(conn, addr).start()

    def reconnect(self):
        self.disconnect()
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=0.5)
        self.sock.settimeout(0.5)

    def disconnect(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def send(self, packet: Dict) -> None:
        if self.sock is None:
            self.send_failures += 1
            return
        try:
            self.sock.sendall(encode_packet_like_windows(packet))
        except OSError:
            self.send_failures += 1
            self.disconnect()
            return
        # Wait (briefly) until the real receiver thread has written the frame.
        deadline = _real_time.monotonic() + 1.0
        while _real_time.monotonic() < deadline:
            data = self.v18.load_camera_packet()   # returns None on any read error
            if data is not None and data.get("frame") == packet["frame"]:
                return
            _real_time.sleep(0.005)

    def read(self) -> Optional[Dict]:
        return self.v18.load_camera_packet()

    def close(self):
        self._stop.set()
        self.disconnect()
        try:
            self.server.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Scenario definition and per-tick records
# ---------------------------------------------------------------------------

@dataclass
class Scenario:
    name: str
    world_factory: Callable[[int], world_mod.World]
    duration_s: float = 20.0
    camera_hz: float = 11.0
    lidar_hz: float = 10.0
    v18_hz: float = 20.0                   # V18 LOOP_DELAY = 0.05 s
    target_hz: float = 10.0                # nidar_airmouse_server SEND_HZ
    setpoint_hz: float = 10.0              # px4_setpoint_sim PUBLISH_HZ
    camera_stop_at_s: Optional[float] = None     # camera packets stop after this time
    camera_resume_at_s: Optional[float] = None
    link_drop_at_s: Optional[float] = None       # tcp: close the sender socket
    link_restore_at_s: Optional[float] = None
    v18_stop_at_s: Optional[float] = None        # V18 process dies: its JSON stops updating
    description: str = ""


@dataclass
class TickRecord:
    t: float
    observable: bool                     # camera running AND link up (scenario script)
    camera_new: bool
    camera_objects: int
    lidar_tracks: int
    fused: int
    persistent_states: Dict[str, int]
    truth_visible: List[Dict]            # ground truth objects in camera FOV
    persistent: List[Dict]
    no_match_events: int
    target_status: Optional[str] = None
    target: Optional[Dict] = None
    command: Optional[List[float]] = None
    ned_velocity: Optional[List[float]] = None


# ---------------------------------------------------------------------------
# The simulation runner
# ---------------------------------------------------------------------------

class PipelineSimulation:

    def __init__(self, scenario: Scenario, transport: str = "direct", seed: int = 0,
                 workdir: Optional[Path] = None, keep_world_truth: bool = True):
        self.scenario = scenario
        self.seed = seed
        self.clock = SimClock()
        self.world = scenario.world_factory(seed)
        self.workdir = Path(workdir) if workdir else Path(tempfile.mkdtemp(prefix="nidar_sim_"))
        self.workdir.mkdir(parents=True, exist_ok=True)

        # --- real production modules -------------------------------------
        self.v18 = load_v18(self.clock)
        self.v18.OUTPUT_JSON = str(self.workdir / "nidar_v18_headless_persistent_fusion.json")
        self.v18.OUTPUT_CSV = str(self.workdir / "nidar_v18_headless_persistent_fusion.csv")

        from lidar_obstacle_detector import LidarObstacleDetector, ObstacleDetectorConfig
        from lidar_obstacle_tracker_v3 import LidarObstacleTrackerV3, TrackerV3Config
        self.detector = LidarObstacleDetector(ObstacleDetectorConfig())
        self.tracker = LidarObstacleTrackerV3(TrackerV3Config())
        self.manager = self.v18.PersistentTargetManager()

        self.airmouse = load_airmouse_server(self.clock)
        self.airmouse.JSON_FILE = Path(self.v18.OUTPUT_JSON)

        self.ros_shim = Shim()
        self.bridge_mod, _ = load_node_module("nidar_bridge", force_stub=True, shim=self.ros_shim)
        self.setpoint_mod, _ = load_node_module("px4_setpoint_sim", force_stub=True, shim=self.ros_shim)
        self.bridge_mod.time = self.clock
        self.setpoint_mod.time = self.clock
        self.ros_shim.clock_seconds = self.clock.time
        self.bridge = self.bridge_mod.NIDARBridge()
        self.setpoint = self.setpoint_mod.PX4AirMouseController()

        # --- transport ----------------------------------------------------
        if transport == "direct":
            self.transport = DirectTransport()
        elif transport == "file":
            self.transport = FileTransport(self.v18, self.workdir)
        elif transport == "tcp":
            self.transport = TcpTransport(self.v18, self.workdir)
        else:
            raise ValueError(f"unknown transport {transport}")

        self.records: List[TickRecord] = []
        self.events: List[Dict] = []
        self.keep_world_truth = keep_world_truth

    # -- helpers ------------------------------------------------------------

    def _log_event(self, t: float, kind: str, **data):
        self.events.append({"t": round(t, 3), "event": kind, **data})

    def _truth_visible(self, t: float) -> List[Dict]:
        out = []
        for ob in self.world.camera_visible_observations(t):
            if True:
                out.append({
                    "name": ob.obj.name, "label": ob.obj.label, "world_id": ob.obj.stable_id,
                    "range_m": round(ob.range_m, 3), "bearing_deg": round(ob.bearing_deg, 3),
                    "right_m": round(ob.right_m, 3), "forward_m": round(ob.forward_m, 3),
                })
        return out

    # -- main loop ----------------------------------------------------------

    def run(self) -> "SimulationReport":
        sc = self.scenario
        t0 = self.clock.time()
        v18_dt = 1.0 / sc.v18_hz
        next_camera = 0.0
        next_lidar = 0.0
        next_target = 0.0
        next_setpoint = 0.0
        camera_frame = 0
        lidar_id = 0
        camera_enabled = True
        link_up = True
        fired = {"camera_stop": False, "camera_resume": False, "link_drop": False, "link_restore": False,
                 "v18_stop": False}
        v18_alive = True

        last_camera_frame = None
        latest_camera_objects: List[Dict] = []
        latest_camera_packet_time: Optional[float] = None
        latest_camera_width: Optional[float] = None
        # V18.1 grace period; 0.0 reproduces the original V18 behaviour
        # (age persistent objects on every tick without a new frame).
        camera_grace = float(getattr(self.v18, "CAMERA_GRACE_SEC", 0.0))
        lidar_tracks: List[Dict] = []
        persistent: List[Dict] = []
        last_command: Optional[List[float]] = None
        last_target_packet: Optional[Dict] = None
        last_ned: Optional[List[float]] = None

        # heading for the setpoint node: level, yaw 0 -> identity quaternion
        self.setpoint.attitude_callback(self.setpoint_mod.VehicleAttitude(q=[1.0, 0.0, 0.0, 0.0]))

        steps = int(round(sc.duration_s / v18_dt))
        for step in range(steps):
            t = step * v18_dt
            self.clock.now = t0 + t

            # ---- scripted events (each fires once) --------------------------
            if sc.camera_stop_at_s is not None and not fired["camera_stop"] and t >= sc.camera_stop_at_s:
                fired["camera_stop"] = True
                camera_enabled = False
                self._log_event(t, "camera_stopped")
            if sc.camera_resume_at_s is not None and not fired["camera_resume"] and t >= sc.camera_resume_at_s:
                fired["camera_resume"] = True
                camera_enabled = True
                self._log_event(t, "camera_resumed")
            if sc.link_drop_at_s is not None and not fired["link_drop"] and t >= sc.link_drop_at_s:
                fired["link_drop"] = True
                link_up = False
                self.transport.disconnect()
                self._log_event(t, "link_dropped")
            if sc.link_restore_at_s is not None and not fired["link_restore"] and t >= sc.link_restore_at_s:
                fired["link_restore"] = True
                link_up = True
                self.transport.reconnect()
                self._log_event(t, "link_restored")
            if sc.v18_stop_at_s is not None and not fired["v18_stop"] and t >= sc.v18_stop_at_s:
                fired["v18_stop"] = True
                v18_alive = False
                self._log_event(t, "v18_stopped")
            observable = camera_enabled and link_up and v18_alive

            # ---- camera (Windows side) --------------------------------------
            if camera_enabled and t >= next_camera - 1e-9:
                camera_frame += 1
                packet = self.world.camera_packet(camera_frame, t, timestamp=self.clock.time())
                self.transport.send(packet)
                next_camera += 1.0 / sc.camera_hz

            # ---- LiDAR chain (real detector + tracker) ----------------------
            if t >= next_lidar - 1e-9:
                scan = self.world.lidar_scan(lidar_id, t, timestamp=self.clock.time())
                lidar_id += 1
                detections = self.detector.detect(scan)
                raw_tracks = self.tracker.update(detections, self.clock.time())
                lidar_tracks = [n for n in (self.v18.normalize_track(tr) for tr in raw_tracks) if n]
                next_lidar += 1.0 / sc.lidar_hz

            # ---- V18 stage: exactly the main-loop logic ---------------------
            packet = self.transport.read() if v18_alive else None
            camera_new = False
            frame = last_camera_frame
            if packet is not None:
                candidate = packet.get("frame")
                try:
                    candidate = int(candidate) if candidate is not None else None
                except (TypeError, ValueError):
                    candidate = None
                if candidate is not None and candidate != last_camera_frame:
                    frame = candidate
                    last_camera_frame = candidate
                    latest_camera_objects = packet.get("objects", [])
                    pcw = getattr(self.v18, "packet_camera_width", None)      # V18 >= 2026-11-02
                    latest_camera_width = pcw(packet) if pcw else None
                    latest_camera_packet_time = self.clock.time()
                    camera_new = True

            no_match = 0
            if camera_new:
                camera_objects = list(latest_camera_objects)
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    if latest_camera_width is None:                         # older V18 (A/B runs)
                        fused = self.v18.fuse_objects(camera_objects, lidar_tracks)
                    else:
                        fused = self.v18.fuse_objects(camera_objects, lidar_tracks, latest_camera_width)
                no_match = sum(1 for o in camera_objects
                               if self.v18.camera_bearing_from_object(o) is not None) - len(fused)
                persistent = self.manager.update(fused)
            elif not v18_alive:
                camera_objects, fused = [], []          # dead process: nothing changes, nothing written
            else:
                camera_objects = list(latest_camera_objects)
                fused = []
                camera_silent = (
                    latest_camera_packet_time is None
                    or (self.clock.time() - latest_camera_packet_time) >= camera_grace
                )
                if camera_silent:
                    persistent = self.manager.update([])

            if frame is not None and v18_alive:
                self.v18.save_json(frame, camera_objects, lidar_tracks, fused, persistent)

            # ---- target server (Pi) ----------------------------------------
            if t >= next_target - 1e-9:
                last_target_packet = self.airmouse.make_packet()
                # bridge receives the line (JSON round trip like the socket would do)
                line = (json.dumps(last_target_packet) + "\n").encode("utf-8")   # server's wire format
                received = json.loads(line.decode("utf-8"))
                # Same guard as nidar_bridge.receive_data(): a missing target is skipped.
                if received.get("target") is not None:
                    self.bridge.process_target(received["target"])
                next_target += 1.0 / sc.target_hz
            self.bridge.timeout_check()
            cmds = self.ros_shim.published["/nidar/airmouse_command"]
            if cmds:
                last_command = [float(v) for v in cmds[-1].data]
                self.setpoint.command_callback(cmds[-1])

            # ---- PX4 setpoint node (WSL) ------------------------------------
            if t >= next_setpoint - 1e-9:
                self.setpoint.publish_setpoint()
                sps = self.ros_shim.published["/fmu/in/trajectory_setpoint"]
                if sps:
                    last_ned = [float(v) for v in sps[-1].velocity]
                next_setpoint += 1.0 / sc.setpoint_hz

            # ---- record ------------------------------------------------------
            states: Dict[str, int] = {}
            for obj in persistent:
                states[obj.get("state", "?")] = states.get(obj.get("state", "?"), 0) + 1
            self.records.append(TickRecord(
                t=round(t, 4), observable=observable, camera_new=camera_new, camera_objects=len(camera_objects),
                lidar_tracks=len(lidar_tracks), fused=len(fused), persistent_states=states,
                truth_visible=self._truth_visible(t) if self.keep_world_truth else [],
                persistent=[{k: obj.get(k) for k in ("stable_id", "class", "state", "range_m",
                                                      "bearing_deg", "x_m", "y_m", "speed_mps",
                                                      "fusion_confidence", "camera_id", "lidar_id",
                                                      "hits", "lost_count", "outlier_count")}
                            for obj in persistent],
                no_match_events=no_match,
                target_status=packet_status(last_target_packet),
                target=last_target_packet.get("target") if last_target_packet else None,
                command=last_command, ned_velocity=last_ned,
            ))

        self.transport.close()
        return SimulationReport.from_run(self)


# ---------------------------------------------------------------------------
# Report / metrics
# ---------------------------------------------------------------------------

@dataclass
class SimulationReport:
    scenario: str
    transport: str
    seed: int
    duration_s: float
    ticks: int
    camera_frames_seen_by_v18: int
    lidar_scans: int
    metrics: Dict
    events: List[Dict]
    checks: Dict[str, Dict]
    workdir: str

    @staticmethod
    def from_run(sim: PipelineSimulation) -> "SimulationReport":
        recs = sim.records
        sc = sim.scenario
        m: Dict = {}

        # Fusion / estimation accuracy, measured only on OBSERVABLE ticks
        # (camera running, link up) when the truth is inside the camera FOV.
        # Truth objects and ACTIVE persistent objects are matched one-to-one
        # by bearing (greedy, gate 6 deg) so two nearby people are not both
        # credited to the same track.
        MATCH_GATE_DEG = 6.0
        range_err, bearing_err, pos_err = [], [], []
        truth_visible_ticks = 0
        active_ticks = 0
        ids_per_world: Dict[int, set] = {}
        last_id_per_world: Dict[int, Optional[int]] = {}
        id_switches = 0
        temp_lost_while_visible = 0
        unmatched_truth = 0
        for r in recs:
            if not r.observable:
                last_id_per_world = {}          # an outage legitimately breaks identity
                continue
            if not r.truth_visible:
                continue
            truth_visible_ticks += 1
            active = [p for p in r.persistent if p["state"] == "ACTIVE"]
            if any(p["state"] == "TEMP_LOST" for p in r.persistent):
                temp_lost_while_visible += 1
            if active:
                active_ticks += 1
            pairs = []
            for ti, tv in enumerate(r.truth_visible):
                for pi, p in enumerate(active):
                    if p.get("class") != tv["label"]:
                        continue
                    d = abs(sim.v18.angular_difference(p["bearing_deg"], tv["bearing_deg"]))
                    if d <= MATCH_GATE_DEG:
                        pairs.append((d, ti, pi))
            pairs.sort()
            used_t, used_p = set(), set()
            for d, ti, pi in pairs:
                if ti in used_t or pi in used_p:
                    continue
                used_t.add(ti)
                used_p.add(pi)
                tv, p = r.truth_visible[ti], active[pi]
                range_err.append(abs(p["range_m"] - tv["range_m"]))
                bearing_err.append(d)
                pos_err.append(math.hypot(p["x_m"] - tv["right_m"], p["y_m"] - tv["forward_m"]))
                ids_per_world.setdefault(tv["world_id"], set()).add(p["stable_id"])
                prev = last_id_per_world.get(tv["world_id"])
                if prev is not None and prev != p["stable_id"]:
                    id_switches += 1
                last_id_per_world[tv["world_id"]] = p["stable_id"]
            unmatched_truth += len(r.truth_visible) - len(used_t)

        # Recovery after a scripted outage: time from resume/restore to the
        # first ACTIVE object and to the first valid command.
        recovery = {}
        for ev in sim.events:
            if ev["event"] in ("camera_resumed", "link_restored"):
                after = [r for r in recs if r.t >= ev["t"]]
                first_active = next((r.t for r in after if r.persistent_states.get("ACTIVE", 0) > 0), None)
                first_cmd = next((r.t for r in after if r.command and r.command[3] > 0.5), None)
                recovery[ev["event"]] = {
                    "at_s": ev["t"],
                    "first_active_after_s": round(first_active - ev["t"], 3) if first_active is not None else None,
                    "first_valid_command_after_s": round(first_cmd - ev["t"], 3) if first_cmd is not None else None,
                }
        m["recovery"] = recovery

        # V18 crash: time from the crash to the first zero command at the bridge
        crash = [e for e in sim.events if e["event"] == "v18_stopped"]
        if crash:
            tc = crash[0]["t"]
            first_zero = next((r.t for r in recs if r.t >= tc and r.command is not None and r.command[3] < 0.5), None)
            m["v18_crash_first_zero_command_s"] = round(first_zero - tc, 3) if first_zero is not None else None
            m["v18_crash_valid_command_ticks_after_2s"] = sum(
                1 for r in recs if r.t >= tc + 2.0 and r.command and r.command[3] > 0.5)
        m["unmatched_truth_observations"] = unmatched_truth

        def stats(values):
            if not values:
                return {"n": 0}
            s = sorted(values)
            return {"n": len(s), "mean": round(sum(s) / len(s), 4), "median": round(s[len(s) // 2], 4),
                    "p95": round(s[int(0.95 * (len(s) - 1))], 4), "max": round(s[-1], 4)}

        m["truth_visible_ticks"] = truth_visible_ticks
        m["ticks_with_active_persistent_while_visible"] = active_ticks
        m["active_coverage_ratio"] = round(active_ticks / truth_visible_ticks, 4) if truth_visible_ticks else None
        m["temp_lost_ticks_while_visible"] = temp_lost_while_visible
        m["range_error_m"] = stats(range_err)
        m["bearing_error_deg"] = stats(bearing_err)
        m["position_error_m"] = stats(pos_err)
        m["stable_ids_per_world_object"] = {str(k): sorted(v) for k, v in ids_per_world.items()}
        m["identity_switches"] = id_switches
        m["fusion_no_match_events"] = sum(r.no_match_events for r in recs)
        m["camera_frames_seen"] = sum(1 for r in recs if r.camera_new)

        # Command behaviour
        cmd_valid = [r for r in recs if r.command is not None and r.command[3] > 0.5]
        m["command_valid_ticks"] = len(cmd_valid)
        m["command_valid_ratio_while_visible"] = round(
            sum(1 for r in recs if r.observable and r.truth_visible and r.command and r.command[3] > 0.5)
            / truth_visible_ticks, 4) if truth_visible_ticks else None
        # A valid command is only acceptable while the target is visible, or
        # within the V18.1 grace period right after it stopped being observable.
        grace = float(getattr(sim.v18, "CAMERA_GRACE_SEC", 0.0)) + 1.0 / sc.v18_hz + 1e-6
        # After a V18 crash, detection goes through the port-5010 server's staleness limit
        # (V18_STALE_SEC); that window is judged by the dedicated v18_crash check instead.
        crash_t = next((e["t"] for e in sim.events if e["event"] == "v18_stopped"), None)
        crash_window = float(getattr(sim.airmouse, "V18_STALE_SEC", 1.0)) + 1.0 / sc.target_hz + 1e-6
        last_ok_t = None
        bad = 0
        for r in recs:
            if r.observable and r.truth_visible:
                last_ok_t = r.t
                continue
            if r.command and r.command[3] > 0.5:
                if crash_t is not None and 0.0 <= r.t - crash_t <= crash_window:
                    continue
                if last_ok_t is None or (r.t - last_ok_t) > grace:
                    bad += 1
        m["command_valid_ticks_while_not_visible"] = bad
        m["max_abs_forward_cmd"] = round(max((abs(r.command[0]) for r in recs if r.command), default=0.0), 4)
        m["max_abs_lateral_cmd"] = round(max((abs(r.command[1]) for r in recs if r.command), default=0.0), 4)
        m["max_abs_vertical_cmd"] = round(max((abs(r.command[2]) for r in recs if r.command), default=0.0), 4)
        m["max_abs_ned_velocity"] = round(max((max(abs(v) for v in r.ned_velocity) for r in recs if r.ned_velocity), default=0.0), 4)
        m["target_status_counts"] = {}
        for r in recs:
            if r.target_status:
                m["target_status_counts"][r.target_status] = m["target_status_counts"].get(r.target_status, 0) + 1

        # Stale-camera timeline
        stop_events = [e for e in sim.events if e["event"] == "camera_stopped"]
        if stop_events:
            t_stop = stop_events[0]["t"]
            after = [r for r in recs if r.t >= t_stop]
            # the last camera frame V18 saw before the stop
            last_new = max((r.t for r in recs if r.camera_new and r.t <= t_stop), default=t_stop)

            def first_time(pred):
                for r in after:
                    if pred(r):
                        return round(r.t - last_new, 3)
                return None

            m["stale_timeline_s_after_last_frame"] = {
                "first_temp_lost": first_time(lambda r: r.persistent_states.get("TEMP_LOST", 0) > 0
                                              and r.persistent_states.get("ACTIVE", 0) == 0),
                "first_predicted": first_time(lambda r: r.persistent_states.get("PREDICTED", 0) > 0),
                "all_expired": first_time(lambda r: not r.persistent),
                "first_zero_command": first_time(lambda r: r.command is not None and r.command[3] < 0.5),
                "target_server_stale": first_time(lambda r: r.target_status in ("CAMERA_STALE", "V18_STALE")),
            }

        checks = evaluate_checks(sc, m, sim)

        return SimulationReport(
            scenario=sc.name, transport=sim.transport.name, seed=sim.seed, duration_s=sc.duration_s,
            ticks=len(recs), camera_frames_seen_by_v18=m["camera_frames_seen"],
            lidar_scans=sim.tracker.previous_timestamp is not None and int(round(sc.duration_s * sc.lidar_hz)) or 0,
            metrics=m, events=sim.events, checks=checks, workdir=str(sim.workdir),
        )

    def to_dict(self) -> Dict:
        return asdict(self)

    def passed(self) -> bool:
        return all(c["pass"] for c in self.checks.values())


def evaluate_checks(sc: Scenario, m: Dict, sim: PipelineSimulation) -> Dict[str, Dict]:
    """Pass/fail criteria -- thresholds are deliberately explicit and conservative."""
    checks: Dict[str, Dict] = {}

    def add(name, cond, detail, severity="FAIL"):
        checks[name] = {"pass": bool(cond), "detail": detail, "severity": severity}

    if m["truth_visible_ticks"]:
        add("fused_range_error_median_below_0.30m",
            m["range_error_m"].get("n", 0) > 0 and m["range_error_m"]["median"] <= 0.30,
            f"median range error {m['range_error_m'].get('median')} m over {m['range_error_m'].get('n')} samples")
        add("fused_bearing_error_median_below_3deg",
            m["bearing_error_deg"].get("n", 0) > 0 and m["bearing_error_deg"]["median"] <= 3.0,
            f"median bearing error {m['bearing_error_deg'].get('median')} deg")
        add("active_coverage_above_0.9",
            (m["active_coverage_ratio"] or 0) >= 0.9,
            f"ACTIVE persistent object present on {m['active_coverage_ratio']} of ticks while target visible")
        # One stable ID per real object while continuously observable.  After an
        # outage longer than EXPIRE_TIMEOUT_SEC a new ID is V18's designed behaviour,
        # so identity is judged by switches between consecutive observable ticks.
        outages = sum(1 for e in sim.events if e["event"] in ("camera_stopped", "link_dropped"))
        add("no_identity_switch_while_observable",
            m["identity_switches"] == 0 and m["stable_ids_per_world_object"]
            and all(len(v) <= 1 + outages for v in m["stable_ids_per_world_object"].values()),
            f"ids per object: {m['stable_ids_per_world_object']}, switches while observable={m['identity_switches']}, outages={outages}")
        if m.get("recovery"):
            for ev_name, rec in m["recovery"].items():
                add(f"recovery_after_{ev_name}_within_1.5s",
                    rec["first_valid_command_after_s"] is not None and rec["first_valid_command_after_s"] <= 1.5,
                    f"{ev_name} at {rec['at_s']} s: first ACTIVE after {rec['first_active_after_s']} s, "
                    f"first valid command after {rec['first_valid_command_after_s']} s")
        add("command_valid_ratio_above_0.9_while_visible",
            (m["command_valid_ratio_while_visible"] or 0) >= 0.9,
            f"valid command on {m['command_valid_ratio_while_visible']} of ticks while target visible")
    add("no_valid_command_without_visible_target",
        m["command_valid_ticks_while_not_visible"] == 0,
        f"{m['command_valid_ticks_while_not_visible']} ticks with valid command but no visible target")
    add("vertical_command_always_zero", m["max_abs_vertical_cmd"] == 0.0,
        f"max |vertical| = {m['max_abs_vertical_cmd']}")
    add("ned_velocity_within_setpoint_limit",
        m["max_abs_ned_velocity"] <= max(sim.setpoint_mod.MAX_FORWARD, sim.setpoint_mod.MAX_LATERAL) * math.sqrt(2) + 1e-6,
        f"max |NED v| = {m['max_abs_ned_velocity']} m/s (limit {sim.setpoint_mod.MAX_FORWARD} per axis)")

    if "v18_crash_first_zero_command_s" in m:
        z = m["v18_crash_first_zero_command_s"]
        add("v18_crash_zero_command_within_1.5s",
            z is not None and z <= 1.5 and m["v18_crash_valid_command_ticks_after_2s"] == 0,
            f"first zero command {z} s after V18 died; valid-command ticks >2 s after the crash: "
            f"{m['v18_crash_valid_command_ticks_after_2s']}")

    tl = m.get("stale_timeline_s_after_last_frame")
    if tl:
        outage = (sc.camera_resume_at_s - sc.camera_stop_at_s) if sc.camera_resume_at_s else float("inf")
        add("stale_first_zero_command_within_1.0s",
            tl["first_zero_command"] is not None and tl["first_zero_command"] <= 1.0,
            f"first zero command {tl['first_zero_command']} s after last camera frame")
        if outage > sim.v18.TEMP_LOST_TIMEOUT_SEC + 0.2:
            add("stale_predicted_between_1.5_and_3s",
                tl["first_predicted"] is not None and 1.5 <= tl["first_predicted"] <= 3.1,
                f"first PREDICTED {tl['first_predicted']} s after last frame (TEMP_LOST_TIMEOUT_SEC=1.5)")
        if outage > sim.v18.EXPIRE_TIMEOUT_SEC + 0.2:
            add("stale_all_expired_between_3_and_4s",
                tl["all_expired"] is not None and 3.0 <= tl["all_expired"] <= 4.0,
                f"all persistent objects expired {tl['all_expired']} s after last frame (V18 EXPIRE_TIMEOUT_SEC=3.0)")
        else:
            add("short_outage_keeps_identity",
                tl["all_expired"] is None,
                f"outage {outage:.1f} s < EXPIRE_TIMEOUT_SEC: objects must not expire (all_expired={tl['all_expired']})")
    return checks


# ---------------------------------------------------------------------------
# SLAM scenario (real SLAM classes, moving sensor, ground truth)
# ---------------------------------------------------------------------------

def run_slam_scenario(duration_s: float = 12.0, speed_mps: float = 0.5, seed: int = 0,
                      lidar_hz: float = 10.0) -> Dict:
    """
    Feed world scans from a MOVING sensor through the real SLAM V5.1 classes,
    reproducing main()'s loop, and compare the estimated pose with the truth.
    """
    slam = load_slam()
    world = world_mod.moving_sensor_scene(seed=seed, speed_mps=speed_mps)

    grid = slam.OccupancyGrid(slam.MAP_SIZE_M, slam.MAP_RESOLUTION_M)
    matcher = slam.ScanMatcher()
    estimator = slam.PoseEstimator()
    loop_detector = slam.LoopClosureDetector()

    previous = None
    frame = 0
    matches = rejected = loops = 0
    rows = []
    start_pose = world.sensor_pose(0.0)
    n = int(duration_s * lidar_hz)
    for i in range(n):
        t = i / lidar_hz
        scan = world.lidar_scan(i, t)
        points = slam.downsample_points(slam.extract_xy(scan))
        if len(points) < 20:
            continue
        frame += 1
        motion = slam.MotionEstimate(0.0, 0.0, 0.0, 0.0, True)
        ok = True
        if previous is not None:
            motion = matcher.match(previous, points)
            ok = motion.valid
            matches += int(ok)
            rejected += int(not ok)
        pose = estimator.update(motion, ok)
        if ok:
            grid.update(points, pose)
        loop = None
        if frame % 10 == 0:
            loop = loop_detector.search(frame, pose, points, matcher)
            if loop is not None:
                loops += 1
        if frame % 20 == 0:
            loop_detector.add_keyframe(frame, pose, points)
        previous = points.copy()

        truth = world.sensor_pose(t)
        rows.append({
            "t": round(t, 3), "frame": frame,
            "truth_dx_m": round(truth.x - start_pose.x, 4), "truth_dy_m": round(truth.y - start_pose.y, 4),
            "est_x_m": round(pose.x, 4), "est_y_m": round(pose.y, 4), "est_yaw_deg": round(math.degrees(pose.yaw), 3),
            "motion_dx": round(motion.dx, 4), "motion_dy": round(motion.dy, 4), "match_ok": ok,
            "match_error": round(motion.error, 4) if math.isfinite(motion.error) else None,
            "confidence": round(estimator.confidence, 3),
        })

    truth_total = rows[-1]["truth_dx_m"] if rows else 0.0
    est_total = rows[-1]["est_x_m"] if rows else 0.0
    # Sign agreement between per-frame estimated dx and true dx (+speed/lidar_hz)
    step_truth = speed_mps / lidar_hz
    dx_vals = [r["motion_dx"] for r in rows if r["match_ok"] and r["frame"] > 1]
    agree = sum(1 for v in dx_vals if v * step_truth > 0)
    final_err = math.hypot(est_total - truth_total, rows[-1]["est_y_m"] - rows[-1]["truth_dy_m"]) if rows else None
    result = {
        "scenario": "slam_moving_sensor",
        "speed_mps": speed_mps, "duration_s": duration_s, "frames": frame,
        "successful_matches": matches, "rejected_matches": rejected, "loop_closures": loops,
        "truth_travel_m": round(truth_total, 3), "estimated_travel_x_m": round(est_total, 3),
        "estimated_travel_y_m": round(rows[-1]["est_y_m"], 3) if rows else None,
        "final_position_error_m": round(final_err, 3) if final_err is not None else None,
        "per_frame_dx_sign_agreement_ratio": round(agree / len(dx_vals), 3) if dx_vals else None,
        "mean_estimated_dx_per_frame": round(sum(dx_vals) / len(dx_vals), 4) if dx_vals else None,
        "true_dx_per_frame": round(step_truth, 4),
        "occupied_cells": grid.occupied_count(),
        "rows": rows,
    }
    result["checks"] = {
        "motion_sign_matches_truth": {
            "pass": (result["per_frame_dx_sign_agreement_ratio"] or 0) > 0.8,
            "detail": f"estimated dx has the sign of true motion on {result['per_frame_dx_sign_agreement_ratio']} of frames "
                      f"(mean est dx {result['mean_estimated_dx_per_frame']} vs true {step_truth:.4f} m/frame)",
        },
        "final_position_error_below_0.5m": {
            "pass": final_err is not None and final_err <= 0.5,
            "detail": f"final error {result['final_position_error_m']} m after {truth_total:.2f} m of travel",
        },
    }
    return result


def run_slam_stationary(duration_s: float = 12.0, seed: int = 0, lidar_hz: float = 10.0) -> Dict:
    """Stationary sensor in the moving-person indoor scene: how much does the pose drift?"""
    slam = load_slam()
    world = world_mod.indoor_scene(seed=seed)
    matcher = slam.ScanMatcher()
    estimator = slam.PoseEstimator()
    previous = None
    frame = 0
    max_drift = 0.0
    for i in range(int(duration_s * lidar_hz)):
        t = i / lidar_hz
        points = slam.downsample_points(slam.extract_xy(world.lidar_scan(i, t)))
        if len(points) < 20:
            continue
        frame += 1
        motion = slam.MotionEstimate(0.0, 0.0, 0.0, 0.0, True)
        ok = True
        if previous is not None:
            motion = matcher.match(previous, points)
            ok = motion.valid
        pose = estimator.update(motion, ok)
        max_drift = max(max_drift, math.hypot(pose.x, pose.y))
        previous = points.copy()
    return {"scenario": "slam_stationary_sensor", "frames": frame, "max_pose_drift_m": round(max_drift, 4),
            "final_pose": {"x_m": round(estimator.pose.x, 4), "y_m": round(estimator.pose.y, 4),
                           "yaw_deg": round(math.degrees(estimator.pose.yaw), 3)},
            "checks": {"stationary_drift_below_0.25m": {
                "pass": max_drift <= 0.25,
                "detail": f"max |pose| while the sensor never moved = {max_drift:.3f} m "
                          f"(a moving person in view shifts the point-cloud centroid)"}}}


# ---------------------------------------------------------------------------
# Scenario catalogue
# ---------------------------------------------------------------------------

SCENARIOS: Dict[str, Scenario] = {
    "nominal": Scenario(
        "nominal", lambda s: world_mod.indoor_scene(seed=s), duration_s=20.0,
        description="Indoor room, one person walking across the camera FOV at ~4 m, camera 11 Hz."),
    "stale_camera": Scenario(
        "stale_camera", lambda s: world_mod.indoor_scene(seed=s), duration_s=16.0, camera_stop_at_s=8.0,
        description="Camera packets stop at t=8 s. Verifies TEMP_LOST/PREDICTED/EXPIRED timing and zero commands."),
    "camera_dropout_recovery": Scenario(
        "camera_dropout_recovery", lambda s: world_mod.indoor_scene(seed=s), duration_s=20.0,
        camera_stop_at_s=8.0, camera_resume_at_s=10.0,
        description="Camera lost for 2 s then back. Checks identity recovery and command resumption."),
    "link_drop_tcp": Scenario(
        "link_drop_tcp", lambda s: world_mod.indoor_scene(seed=s), duration_s=20.0,
        link_drop_at_s=8.0, link_restore_at_s=11.0,
        description="(tcp transport) Windows->Pi socket closed for 3 s then reconnected; receiver must recover."),
    "two_person": Scenario(
        "two_person", lambda s: world_mod.two_person_scene(seed=s), duration_s=20.0,
        description="Two people crossing at different ranges; identity stability under association stress."),
    "bearing_sweep": Scenario(
        "bearing_sweep", lambda s: world_mod.indoor_scene(seed=s, person_speed_mps=0.8), duration_s=14.0,
        description="Faster sweep across the FOV to expose the camera bearing model error vs true bearing."),
    "v18_crash": Scenario(
        "v18_crash", lambda s: world_mod.indoor_scene(seed=s), duration_s=14.0, v18_stop_at_s=6.0,
        description="V18 process dies at t=6 s with the person ACTIVE; its JSON freezes. The port-5010 "
                    "server must stop sending an ACTIVE target and the bridge must publish zero."),
    "empty_scene": Scenario(
        "empty_scene", lambda s: world_mod.World(room=world_mod.Room(8.0, 6.0), seed=s), duration_s=6.0,
        description="Room with no objects: no fused objects, no targets, no valid commands."),
}


def run_scenario(name: str, transport: str = "direct", seed: int = 0, out_dir: Optional[Path] = None) -> SimulationReport:
    sc = SCENARIOS[name]
    if name == "link_drop_tcp" and transport != "tcp":
        transport = "tcp"
    workdir = (out_dir / f"work_{name}") if out_dir else None
    sim = PipelineSimulation(sc, transport=transport, seed=seed, workdir=workdir)
    report = sim.run()
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / f"{name}_report.json").write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
        (out_dir / f"{name}_timeline.json").write_text(
            json.dumps([asdict(r) for r in sim.records], indent=1), encoding="utf-8")
    return report


def format_report_markdown(reports: List[SimulationReport], slam_results: List[Dict]) -> str:
    lines = ["# NIDAR pipeline simulation report", ""]
    lines.append("All stages below ran the project's real code (receiver, V18, target server, bridge, setpoint node); "
                 "only the world, the clock and ROS 2 transport were simulated.")
    lines.append("")
    for rep in reports:
        lines.append(f"## Scenario `{rep.scenario}` (transport={rep.transport}, seed={rep.seed})")
        lines.append("")
        lines.append(SCENARIOS[rep.scenario].description)
        lines.append("")
        lines.append(f"Ticks: {rep.ticks} | camera frames seen by V18: {rep.camera_frames_seen_by_v18} | "
                     f"overall: **{'PASS' if rep.passed() else 'FAIL'}**")
        lines.append("")
        lines.append("| check | result | detail |")
        lines.append("|---|---|---|")
        for name, c in rep.checks.items():
            lines.append(f"| {name} | {'PASS' if c['pass'] else 'FAIL'} | {c['detail']} |")
        lines.append("")
        m = rep.metrics
        lines.append(f"Range error (m): {m['range_error_m']} | bearing error (deg): {m['bearing_error_deg']} | "
                     f"position error (m): {m['position_error_m']}")
        lines.append("")
        lines.append(f"Target status counts: {m['target_status_counts']} | fusion no-match events: {m['fusion_no_match_events']} | "
                     f"TEMP_LOST ticks while visible: {m['temp_lost_ticks_while_visible']} / {m['truth_visible_ticks']}")
        if m.get("stale_timeline_s_after_last_frame"):
            lines.append("")
            lines.append(f"Stale timeline (s after last camera frame): {m['stale_timeline_s_after_last_frame']}")
        if rep.events:
            lines.append("")
            lines.append(f"Events: {rep.events}")
        lines.append("")
    for res in slam_results:
        lines.append(f"## SLAM `{res['scenario']}`")
        lines.append("")
        lines.append("| check | result | detail |")
        lines.append("|---|---|---|")
        for name, c in res["checks"].items():
            lines.append(f"| {name} | {'PASS' if c['pass'] else 'FAIL'} | {c['detail']} |")
        lines.append("")
        summary = {k: v for k, v in res.items() if k not in ("rows", "checks")}
        lines.append(f"Summary: {summary}")
        lines.append("")
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="NIDAR pipeline simulation (real code, simulated world)")
    parser.add_argument("--scenario", default="all", help=f"one of {sorted(SCENARIOS)} or 'all'")
    parser.add_argument("--transport", default="direct", choices=["direct", "file", "tcp"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default=str(PROJECT_ROOT / "outputs" / "simulation"))
    parser.add_argument("--no-slam", action="store_true", help="skip the SLAM scenarios")
    args = parser.parse_args(argv)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    names = sorted(SCENARIOS) if args.scenario == "all" else [args.scenario]

    reports: List[SimulationReport] = []
    for name in names:
        print(f"[SIM] running {name} (transport={args.transport if name != 'link_drop_tcp' else 'tcp'}) ...", flush=True)
        rep = run_scenario(name, transport=args.transport, seed=args.seed, out_dir=out_dir)
        reports.append(rep)
        status = "PASS" if rep.passed() else "FAIL"
        print(f"[SIM] {name}: {status}", flush=True)
        for cname, c in rep.checks.items():
            print(f"       {'PASS' if c['pass'] else 'FAIL'}  {cname}: {c['detail']}", flush=True)

    slam_results: List[Dict] = []
    if not args.no_slam:
        print("[SIM] running SLAM scenarios ...", flush=True)
        for res in (run_slam_stationary(seed=args.seed), run_slam_scenario(seed=args.seed)):
            slam_results.append(res)
            (out_dir / f"{res['scenario']}_report.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
            for cname, c in res["checks"].items():
                print(f"       {'PASS' if c['pass'] else 'FAIL'}  {res['scenario']}.{cname}: {c['detail']}", flush=True)

    md = format_report_markdown(reports, slam_results)
    (out_dir / "SIMULATION_REPORT.md").write_text(md, encoding="utf-8")
    print(f"[SIM] report written to {out_dir / 'SIMULATION_REPORT.md'}", flush=True)
    all_pass = all(r.passed() for r in reports) and all(all(c["pass"] for c in r["checks"].values()) for r in slam_results)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
