#!/usr/bin/env python3
"""
NIDAR LiDAR relay -- WSL side (2026-11-02).

  Gazebo gpu_lidar --ros_gz_bridge--> /nidar/scan (sensor_msgs/LaserScan)
  --this node--> NIDAR_LIDAR_V1 newline JSON over TCP --> Pi :5011
  (src/communication/nidar_lidar_receiver.py, started by start_nidar_pi.sh
   when NIDAR_LIDAR_BACKEND=GAZEBO)

Run in WSL (ROS 2 sourced):
    python3 gazebo/ros2/nidar_lidar_relay.py --pi-host 10.218.70.147

Only the NEWEST scan is ever sent: if the link is slow, older scans are
dropped instead of queued, so the Pi never works on old geometry. The Pi
side decides freshness with its own clock ("received"), so the WSL and Pi
clocks do not need to agree.

The message keeps LaserScan's angle convention (counter-clockwise from the
sensor's forward axis); the Pi converts to NIDAR's clockwise convention in
src/core/lidar_gazebo_backend.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time

PROTOCOL = "NIDAR_LIDAR_V1"


def scan_to_wire(msg, seq: int, now: float | None = None) -> dict:
    """sensor_msgs/LaserScan (or any object with the same fields) -> NIDAR_LIDAR_V1 dict."""
    stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    scan_time = float(getattr(msg, "scan_time", 0.0) or 0.0)
    ranges = []
    for r in msg.ranges:
        r = float(r)
        ranges.append(round(r, 4) if math.isfinite(r) else None)
    return {
        "protocol": PROTOCOL,
        "seq": int(seq),
        "stamp": stamp,                                   # Gazebo sim time
        "sent": float(now if now is not None else time.time()),   # WSL wall clock (info only)
        "frame_id": str(msg.header.frame_id),
        "angle_min": float(msg.angle_min),
        "angle_max": float(msg.angle_max),
        "angle_increment": float(msg.angle_increment),
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
        "scan_rate_hz": (1.0 / scan_time) if scan_time > 1e-6 else 10.0,
        "ranges": ranges,
    }


class LatestLineSender(threading.Thread):
    """Keeps one TCP connection to the Pi; sends only the newest line; reconnects."""

    def __init__(self, host: str, port: int, name: str = "LIDAR", connect_timeout: float = 3.0):
        super().__init__(daemon=True, name=f"{name.lower()}-sender")
        self.host, self.port, self.label = host, int(port), name
        self.connect_timeout = connect_timeout
        self._cond = threading.Condition()
        self._line: bytes | None = None
        self._halt = False
        self.sent = 0
        self.dropped = 0
        self.connected = False

    def submit(self, line: bytes) -> None:
        with self._cond:
            if self._line is not None:
                self.dropped += 1                         # replaced before it could be sent
            self._line = line
            self._cond.notify()

    def stop(self) -> None:
        with self._cond:
            self._halt = True
            self._cond.notify()

    def run(self) -> None:
        backoff = 0.5
        while not self._halt:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(1.0)
            except OSError as exc:
                print(f"[{self.label}] cannot reach {self.host}:{self.port} ({exc}); retrying", flush=True)
                time.sleep(backoff)
                backoff = min(backoff * 2, 5.0)
                continue
            backoff = 0.5
            self.connected = True
            print(f"[{self.label}] connected to {self.host}:{self.port}", flush=True)
            try:
                while not self._halt:
                    with self._cond:
                        self._cond.wait_for(lambda: self._line is not None or self._halt, timeout=1.0)
                        line, self._line = self._line, None
                    if line is None:
                        continue
                    sock.sendall(line)
                    self.sent += 1
            except OSError as exc:
                print(f"[{self.label}] link lost: {exc}; reconnecting", flush=True)
            finally:
                self.connected = False
                try:
                    sock.close()
                except OSError:
                    pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Relay Gazebo LaserScan to the NIDAR Pi (NIDAR_LIDAR_V1, TCP).")
    ap.add_argument("--pi-host", default=os.environ.get("NIDAR_PI_HOST", ""), help="Pi IP (or env NIDAR_PI_HOST)")
    ap.add_argument("--port", type=int, default=int(os.environ.get("NIDAR_LIDAR_PORT", "5011")))
    ap.add_argument("--topic", default="/nidar/scan")
    args = ap.parse_args(argv)
    if not args.pi_host:
        print("Give the Pi address: --pi-host <PI_IP>  (or export NIDAR_PI_HOST)", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    sender = LatestLineSender(args.pi_host, args.port)
    sender.start()

    class LidarRelay(Node):
        def __init__(self):
            super().__init__("nidar_lidar_relay")
            self.seq = 0
            self.last_report = time.time()
            self.create_subscription(LaserScan, args.topic, self.on_scan, qos_profile_sensor_data)
            self.get_logger().info(f"{args.topic} -> {args.pi_host}:{args.port} ({PROTOCOL})")

        def on_scan(self, msg):
            self.seq += 1
            line = (json.dumps(scan_to_wire(msg, self.seq), separators=(",", ":")) + "\n").encode()
            sender.submit(line)
            now = time.time()
            if now - self.last_report >= 5.0:
                self.last_report = now
                valid = sum(1 for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max)
                self.get_logger().info(
                    f"scans={self.seq} sent={sender.sent} dropped={sender.dropped} "
                    f"link={'UP' if sender.connected else 'DOWN'} points={len(msg.ranges)} valid={valid}")

    rclpy.init()
    node = LidarRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("[LIDAR] relay stopped (the Pi reports LiDAR STALE after 1 s and stops fusion).", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
