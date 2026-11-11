#!/usr/bin/env python3
"""
NIDAR camera relay -- WSL side (2026-11-02).

Parameterised version of ~/gazebo_camera_relay.py (that file is left as it
is). Same wire format, so gazebo_camera_receiver.py, gazebo_yolo_test.py and
nidar_v11_windows.py (NIDAR_CAMERA_SOURCE=gazebo) all read it:
    4-byte big-endian length + JPEG, one frame after another, TCP :5012

  Gazebo camera --ros_gz_bridge--> /nidar/camera/image --this node--> TCP --> Windows

Run in WSL (ROS 2 sourced):
    python3 gazebo/ros2/nidar_camera_relay.py
    python3 gazebo/ros2/nidar_camera_relay.py --topic <other image topic> --port 5012

Differences from the original relay: topic/port/quality are arguments, the
JPEG is sent from a separate thread (a slow Windows client no longer blocks
the ROS callback), and only the newest frame is sent.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading
import time


def encode_frame(bgr, quality: int = 80) -> bytes | None:
    import cv2
    ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    payload = enc.tobytes()
    return struct.pack("!I", len(payload)) + payload


class LatestFrameServer(threading.Thread):
    """One Windows client at a time (a new client replaces the old one); newest frame only."""

    def __init__(self, host: str, port: int):
        super().__init__(daemon=True, name="camera-server")
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(1)
        self.srv.settimeout(0.5)
        self.port = self.srv.getsockname()[1]
        self._cond = threading.Condition()
        self._packet: bytes | None = None
        self._halt = False
        self.sent = 0
        self.dropped = 0
        self.client = None

    def submit(self, packet: bytes) -> None:
        with self._cond:
            if self._packet is not None:
                self.dropped += 1
            self._packet = packet
            self._cond.notify()

    def stop(self) -> None:
        with self._cond:
            self._halt = True
            self._cond.notify()
        try:
            self.srv.close()
        except OSError:
            pass

    def run(self) -> None:
        while not self._halt:
            try:
                conn, addr = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(2.0)
            self.client = addr
            print(f"[CAMERA] client connected: {addr}", flush=True)
            with self._cond:
                self._packet = None                      # never start a client on an old frame
            try:
                while not self._halt:
                    with self._cond:
                        self._cond.wait_for(lambda: self._packet is not None or self._halt, timeout=1.0)
                        pkt, self._packet = self._packet, None
                    if pkt is None:
                        continue
                    conn.sendall(pkt)
                    self.sent += 1
            except OSError as exc:
                print(f"[CAMERA] client {addr} gone: {exc}", flush=True)
            finally:
                self.client = None
                try:
                    conn.close()
                except OSError:
                    pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Relay a Gazebo camera topic to Windows (length-prefixed JPEG over TCP).")
    ap.add_argument("--topic", default="/nidar/camera/image")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5012)
    ap.add_argument("--quality", type=int, default=80)
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    server = LatestFrameServer(args.host, args.port)
    server.start()
    print(f"[CAMERA] listening on {args.host}:{server.port}", flush=True)

    class CameraRelay(Node):
        def __init__(self):
            super().__init__("nidar_camera_relay")
            self.bridge = CvBridge()
            self.frames = 0
            self.last_report = time.time()
            self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)
            self.get_logger().info(f"subscribed to {args.topic}")

        def on_image(self, msg):
            self.frames += 1
            now = time.time()
            if now - self.last_report >= 5.0:
                self.last_report = now
                self.get_logger().info(
                    f"frames={self.frames} sent={server.sent} dropped={server.dropped} "
                    f"size={msg.width}x{msg.height} client={server.client}")
            if server.client is None:
                return                                   # nobody connected: skip the JPEG work
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                pkt = encode_frame(frame, args.quality)
                if pkt:
                    server.submit(pkt)
            except Exception as exc:
                self.get_logger().error(f"frame relay error: {exc}")

    rclpy.init()
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
