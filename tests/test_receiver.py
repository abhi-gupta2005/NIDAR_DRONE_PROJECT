"""
Pi perception receiver (src/communication/nidar_pi_receiver.py) -- the REAL
ClientHandler behind a loopback socket, fed with the Windows wire format.
"""

from __future__ import annotations

import csv
import json
import socket
import time

import pytest

from src.simulation.nidar_replay import encode_packet, replay

pytestmark = pytest.mark.network


def _packet(frame, objects=None, protocol="NIDAR_PERCEPTION_V1", ts=None):
    return {
        "protocol": protocol, "frame": frame, "timestamp": ts if ts is not None else 1700000000.0 + frame * 0.09,
        "camera": {"width": 1280, "height": 720, "source": "test"},
        "objects": objects if objects is not None else [{
            "id": 1, "class": "person", "state": "CONFIRMED", "spatial_status": "OBSERVED",
            "confidence": 0.8, "motion_confidence": 1.0, "track_stability_score": 90.0,
            "tracker_id": 1, "center": [640.0, 360.0], "predicted_center": [640.0, 360.0],
            "prediction_horizon_frames": 0, "prediction_reliability": "MEASURED",
        }],
    }


class _Server:
    """Accept one connection and hand it to the production ClientHandler."""

    def __init__(self, receiver_module):
        self.rx = receiver_module
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(2)
        self.sock.settimeout(5.0)
        self.port = self.sock.getsockname()[1]
        self.handlers = []

    def accept(self):
        conn, addr = self.sock.accept()
        handler = self.rx.ClientHandler(conn, addr)
        handler.start()
        self.handlers.append(handler)
        return handler

    def close(self):
        self.sock.close()


def _wait_for_frame(rx, frame, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            data = json.loads(rx.LATEST_FILE.read_text(encoding="utf-8"))
            if data.get("frame") == frame:
                return data
        except (OSError, json.JSONDecodeError):
            pass  # OSError covers Windows PermissionError during the receiver's atomic replace
        time.sleep(0.01)
    raise AssertionError(f"receiver never stored frame {frame}")


def test_valid_packet_is_stored_and_logged(receiver_module):
    rx = receiver_module
    server = _Server(rx)
    client = socket.create_connection(("127.0.0.1", server.port), timeout=2)
    server.accept()
    try:
        client.sendall(encode_packet(_packet(1)))
        data = _wait_for_frame(rx, 1)
        assert data["protocol"] == "NIDAR_PERCEPTION_V1"
        assert data["objects"][0]["class"] == "person"
        # CSV row appended with the documented header
        rows = list(csv.reader(rx.OBJECT_LOG.open("r", encoding="utf-8")))
        assert rows[0][:4] == ["timestamp", "frame", "stable_id", "class"]
        assert rows[1][1] == "1" and rows[1][3] == "person"
    finally:
        client.close()
        server.close()


def test_latest_file_tracks_the_newest_frame_and_no_tmp_remains(receiver_module):
    rx = receiver_module
    server = _Server(rx)
    client = socket.create_connection(("127.0.0.1", server.port), timeout=2)
    server.accept()
    try:
        for frame in range(1, 51):
            client.sendall(encode_packet(_packet(frame)))
        data = _wait_for_frame(rx, 50)
        assert data["frame"] == 50
        assert not rx.LATEST_FILE.with_suffix(".tmp").exists(), "atomic replace must not leave a .tmp"
    finally:
        client.close()
        server.close()


def test_split_and_coalesced_packets_are_reassembled(receiver_module):
    """TCP is a byte stream: half a packet, then one and a half packets."""
    rx = receiver_module
    server = _Server(rx)
    client = socket.create_connection(("127.0.0.1", server.port), timeout=2)
    server.accept()
    try:
        p1, p2, p3 = (encode_packet(_packet(i)) for i in (1, 2, 3))
        client.sendall(p1[:20])
        time.sleep(0.05)
        client.sendall(p1[20:] + p2 + p3[:-5])
        time.sleep(0.05)
        client.sendall(p3[-5:])
        data = _wait_for_frame(rx, 3)
        assert data["frame"] == 3
        rows = list(csv.reader(rx.OBJECT_LOG.open("r", encoding="utf-8")))
        assert [r[1] for r in rows[1:]] == ["1", "2", "3"], "every packet must be logged exactly once"
    finally:
        client.close()
        server.close()


def test_invalid_json_and_foreign_protocol_are_ignored(receiver_module):
    rx = receiver_module
    server = _Server(rx)
    client = socket.create_connection(("127.0.0.1", server.port), timeout=2)
    server.accept()
    try:
        client.sendall(encode_packet(_packet(1)))
        _wait_for_frame(rx, 1)
        client.sendall(b"{this is not json}\n")
        client.sendall(encode_packet(_packet(99, protocol="SOMETHING_ELSE")))
        client.sendall(b"\n\n")
        client.sendall(encode_packet(_packet(2)))
        data = _wait_for_frame(rx, 2)
        assert data["frame"] == 2
        rows = list(csv.reader(rx.OBJECT_LOG.open("r", encoding="utf-8")))
        assert [r[1] for r in rows[1:]] == ["1", "2"]
    finally:
        client.close()
        server.close()


def test_client_disconnect_ends_handler_and_server_accepts_again(receiver_module):
    rx = receiver_module
    server = _Server(rx)
    client = socket.create_connection(("127.0.0.1", server.port), timeout=2)
    handler = server.accept()
    client.sendall(encode_packet(_packet(1)))
    _wait_for_frame(rx, 1)
    client.close()
    handler.join(timeout=3.0)
    assert not handler.is_alive(), "handler thread must exit when the Windows side disconnects"

    client2 = socket.create_connection(("127.0.0.1", server.port), timeout=2)
    server.accept()
    try:
        client2.sendall(encode_packet(_packet(2)))
        assert _wait_for_frame(rx, 2)["frame"] == 2
    finally:
        client2.close()
        server.close()


def test_replay_tool_streams_the_recording_into_the_real_receiver(receiver_module, recorded_packets, tmp_path):
    """End-to-end: recording -> replay sender -> receiver -> latest file (first 120 packets, max speed)."""
    rx = receiver_module
    server = _Server(rx)
    import threading
    threading.Thread(target=server.accept, daemon=True).start()
    recording = tmp_path / "rec.jsonl"
    with recording.open("w", encoding="utf-8") as f:
        for p in recorded_packets[:120]:
            f.write(json.dumps(p) + "\n")
    stats = replay(recording, "127.0.0.1", server.port, speed=0, rewrite_time=True, quiet=True)
    assert stats["sent"] == 120 and stats["errors"] == 0
    data = _wait_for_frame(rx, recorded_packets[119]["frame"])
    assert data["camera"]["source"] == "windows_laptop_webcam"
    assert abs(data["timestamp"] - time.time()) < 5.0, "--rewrite-time must stamp packets with the current clock"
    server.close()
