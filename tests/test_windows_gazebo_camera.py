"""
Windows opt-in Gazebo camera source (nidar_v11_windows.GazeboCameraCapture),
tested against a loopback server speaking the relay's wire format
(4-byte big-endian length + JPEG). Proves: frame size reported from the
stream, each frame returned once, read() fails (not a frozen frame) when the
stream stops, isOpened() turns False when the relay disconnects.
Not proved here: nidar_camera_relay.py itself (needs ROS 2 + Gazebo).
"""

from __future__ import annotations

import socket
import struct
import threading
import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
if not hasattr(cv2, "imencode"):
    pytest.skip("real OpenCV needed", allow_module_level=True)


def jpeg(w=640, h=480, value=0):
    img = np.full((h, w, 3), value, dtype=np.uint8)
    ok, enc = cv2.imencode(".jpg", img)
    assert ok
    return enc.tobytes()


class Relay:
    def __init__(self):
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.conn = None
        self.accepted = threading.Event()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        self.conn, _ = self.srv.accept()
        self.accepted.set()

    def send(self, payload):
        self.accepted.wait(2.0)
        self.conn.sendall(struct.pack("!I", len(payload)) + payload)

    def close(self):
        if self.conn:
            self.conn.close()
        self.srv.close()


@pytest.mark.network
def test_gazebo_capture_reads_relay_frames_once(windows_module):
    relay = Relay()
    threading.Timer(0.2, lambda: relay.send(jpeg(value=10))).start()
    cap = windows_module.GazeboCameraCapture(f"127.0.0.1:{relay.port}", connect_timeout=3.0, read_timeout=0.3)
    try:
        assert cap.isOpened()
        assert cap.get(cv2.CAP_PROP_FRAME_WIDTH) == 640 and cap.get(cv2.CAP_PROP_FRAME_HEIGHT) == 480
        ok, frame = cap.read()
        assert ok and frame.shape == (480, 640, 3)
        t0 = time.monotonic()
        ok, _ = cap.read()
        assert not ok, "the same frame must not be returned twice"
        assert time.monotonic() - t0 >= 0.25
        relay.send(jpeg(value=200))
        ok, frame = cap.read()
        assert ok and int(frame.mean()) > 150
        relay.close()
        deadline = time.monotonic() + 3.0
        while cap.isOpened() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert not cap.isOpened(), "relay disconnect must be visible"
    finally:
        cap.release()


def test_gazebo_capture_unreachable_relay_is_not_opened(windows_module):
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()                                            # nothing listens there now
    cap = windows_module.GazeboCameraCapture(f"127.0.0.1:{port}", connect_timeout=0.5)
    assert not cap.isOpened() and cap.error is not None
    assert cap.read() == (False, None)


def test_webcam_remains_the_default_source(windows_module, monkeypatch):
    import os
    if "NIDAR_CAMERA_SOURCE" in os.environ:
        pytest.skip("NIDAR_CAMERA_SOURCE set in this shell")
    assert windows_module.CAMERA_SOURCE == "webcam"
