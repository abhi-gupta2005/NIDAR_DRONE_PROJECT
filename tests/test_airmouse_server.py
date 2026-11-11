"""
The Pi's port-5010 server (nidar_airmouse_server.py, project root) -- the
process that feeds nidar_bridge in WSL.

Pins down the two defects measured on 2026-11-02 on the original file
(archive/pi_originals_2026-11-02/nidar_airmouse_server.py):
  1. a V18 JSON frozen for an hour was still sent as an ACTIVE target and
     the bridge published fwd 0.99 m/s, valid=1;
  2. accept() with a 1 s timeout limited the loop to ~0.9 Hz (gaps 1.1 s,
     longer than the bridge's 1.0 s TARGET_TIMEOUT).
"""

from __future__ import annotations

import json
import socket
import threading
import time
from pathlib import Path

import pytest

from conftest import PROJECT_ROOT, load_module_from_path
from src.simulation.pipeline_clock import SimClock
from src.simulation.ros2_shim import load_node_module, Shim, ROS2_PACKAGE_DIR

SERVER = PROJECT_ROOT / "nidar_airmouse_server.py"
pytestmark = pytest.mark.skipif(not SERVER.exists(), reason="nidar_airmouse_server.py not in project root")

BRIDGE_KEYS = ("state", "confidence", "x_m", "y_m", "range_m", "stable_id")


@pytest.fixture()
def server(tmp_path):
    m = load_module_from_path(f"airmouse_{id(tmp_path)}", SERVER)
    m.JSON_FILE = tmp_path / "v18.json"
    m.time = SimClock(5000.0)
    return m


def write(m, frame, age=0.0, objects=None):
    if objects is None:
        objects = [person()]
    m.JSON_FILE.write_text(json.dumps({
        "protocol": "NIDAR_V18_HEADLESS_PERSISTENT_FUSION",
        "timestamp": m.time.time() - age, "frame": frame, "persistent_objects": objects}), encoding="utf-8")


def person(sid=1, state="ACTIVE", conf=0.7, x=0.44, y=4.98):
    return {"stable_id": sid, "class": "person", "state": state, "range_m": 5.0, "bearing_deg": 5.0,
            "x_m": x, "y_m": y, "speed_mps": 0.1, "fusion_confidence": conf, "camera_confidence": 0.8}


def test_default_json_path_is_next_to_the_server():
    m = load_module_from_path("airmouse_paths", SERVER)
    assert m.JSON_FILE == SERVER.resolve().parent / "nidar_v18_headless_persistent_fusion.json"
    v18 = load_module_from_path("v18_paths", PROJECT_ROOT / "src" / "perception" / "nidar_v18_headless_persistent_fusion.py")
    assert Path(v18.OUTPUT_JSON).resolve() == m.JSON_FILE.resolve(), "server must read the file V18 writes"
    assert m.PORT == 5010


def test_fresh_active_target_matches_bridge_contract(server):
    write(server, 10)
    p = server.make_packet()
    assert p["protocol"] == "NIDAR_AIRMOUSE_V1" and p["status"] == "TRACKING"
    t = p["target"]
    for key in BRIDGE_KEYS:
        assert key in t
    assert t["state"] == "ACTIVE" and t["confidence"] == 0.7, "confidence must come from fusion_confidence"
    assert (t["x_m"], t["y_m"]) == (0.44, 4.98)


def test_strongest_active_target_is_selected(server):
    write(server, 10, objects=[person(1, conf=0.5), person(2, conf=0.9), person(3, state="TEMP_LOST", conf=0.99)])
    assert server.make_packet()["target"]["stable_id"] == 2


@pytest.mark.parametrize("case, expected", [
    ("stale_v18", "V18_STALE"),
    ("missing", "V18_MISSING"),
    ("no_objects", "NO_TARGET"),
])
def test_stop_cases_send_explicit_non_active_target(server, case, expected):
    if case == "stale_v18":
        write(server, 1773, age=3600.0)          # the exact defect measured on 2026-11-02
    elif case == "no_objects":
        write(server, 11, objects=[])
    # "missing": no file written
    p = server.make_packet()
    assert p["status"] == expected
    assert p["target"]["state"] == expected != "ACTIVE"
    assert p["target"]["confidence"] == 0.0


def test_frozen_camera_frame_becomes_camera_stale(server):
    write(server, 50)
    assert server.make_packet()["status"] == "TRACKING"
    server.time.advance(0.6)
    write(server, 50)                             # V18 alive (fresh timestamp) but same frame
    assert server.make_packet()["status"] == "TRACKING"
    server.time.advance(0.6)
    write(server, 50)
    assert server.make_packet()["status"] == "CAMERA_STALE"
    write(server, 51)
    assert server.make_packet()["status"] == "TRACKING", "a new frame restores tracking"


def test_only_non_active_target_is_reported_but_not_tracked(server):
    """Original fallback behaviour preserved: the best non-ACTIVE object is sent; the bridge zeroes it."""
    write(server, 12, objects=[person(4, state="TEMP_LOST")])
    p = server.make_packet()
    assert p["status"] == "TARGET_NOT_ACTIVE" and p["target"]["state"] == "TEMP_LOST"


@pytest.mark.skipif(not Path(ROS2_PACKAGE_DIR, "nidar_bridge.py").exists(), reason="ROS 2 sources absent")
def test_bridge_publishes_zero_for_stale_data_and_moves_for_fresh(server):
    shim = Shim()
    bm, _ = load_node_module("nidar_bridge", force_stub=True, shim=shim)
    bridge = bm.NIDARBridge()
    write(server, 1773, age=3600.0)
    bridge.process_target(json.loads(json.dumps(server.make_packet()))["target"])
    assert list(shim.published["/nidar/airmouse_command"][-1].data) == [0.0, 0.0, 0.0, 0.0]
    write(server, 1774)
    bridge.process_target(json.loads(json.dumps(server.make_packet()))["target"])
    f, l, v, valid = shim.published["/nidar/airmouse_command"][-1].data
    assert valid == 1.0 and f > 0 and l > 0 and v == 0.0


@pytest.mark.network
def test_send_rate_reaches_send_hz_with_a_connected_client(tmp_path):
    m = load_module_from_path("airmouse_rate", SERVER)
    m.JSON_FILE = tmp_path / "v18.json"
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    m.PORT = s.getsockname()[1]
    s.close()
    m.HOST = "127.0.0.1"
    m.print = lambda *a, **k: None
    frame = [1]

    def keep_v18_alive():
        while True:
            frame[0] += 1
            m.JSON_FILE.write_text(json.dumps({"timestamp": time.time(), "frame": frame[0],
                                               "persistent_objects": [person()]}), encoding="utf-8")
            time.sleep(0.05)

    threading.Thread(target=keep_v18_alive, daemon=True).start()
    threading.Thread(target=m.main, daemon=True).start()
    deadline = time.time() + 3
    while True:
        try:
            c = socket.create_connection(("127.0.0.1", m.PORT), timeout=1)
            break
        except OSError:
            if time.time() > deadline:
                raise
            time.sleep(0.05)
    c.settimeout(0.3)
    t0, n = time.time(), 0
    while time.time() - t0 < 2.0:
        try:
            n += c.recv(65536).count(b"\n")
        except socket.timeout:
            pass
    c.close()
    rate = n / 2.0
    assert rate >= 0.8 * m.SEND_HZ, f"measured {rate:.1f} Hz (original file: ~0.9 Hz)"
