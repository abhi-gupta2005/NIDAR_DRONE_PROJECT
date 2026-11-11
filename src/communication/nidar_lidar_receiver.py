#!/usr/bin/env python3
"""
NIDAR LiDAR receiver -- Raspberry Pi side (2026-11-02).

Receives NIDAR_LIDAR_V1 scans (newline-delimited JSON over TCP) from
gazebo/ros2/nidar_lidar_relay.py running in WSL, and keeps the newest scan in
src/communication/nidar_latest_scan.json (atomic replace), the same pattern
nidar_pi_receiver.py uses for camera packets.

Only needed when the LiDAR comes from Gazebo (NIDAR_LIDAR_BACKEND=GAZEBO);
start_nidar_pi.sh starts it automatically in that case.

    NIDAR_LIDAR_PORT=5011 python src/communication/nidar_lidar_receiver.py
"""

from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time
from pathlib import Path

PROTOCOL = "NIDAR_LIDAR_V1"
HOST = "0.0.0.0"
PORT = int(os.environ.get("NIDAR_LIDAR_PORT", "5011"))
BASE_DIR = Path(__file__).resolve().parent
LATEST_FILE = Path(os.environ.get("NIDAR_SCAN_FILE", BASE_DIR / "nidar_latest_scan.json"))
MAX_LINE_BYTES = 2_000_000


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.received = 0
        self.rejected = 0
        self.last_seq = None


_WRITE_LOCK = threading.Lock()


def store_scan(msg: dict, path: Path = None) -> None:
    """Validate a scan dict and write it atomically. Raises ValueError if invalid."""
    path = Path(path or LATEST_FILE)
    if not isinstance(msg, dict) or msg.get("protocol") != PROTOCOL:
        raise ValueError("unknown protocol")
    ranges = msg.get("ranges")
    if not isinstance(ranges, list) or len(ranges) < 10:
        raise ValueError("ranges missing or too short")
    for key in ("angle_min", "angle_increment", "seq"):
        if key not in msg:
            raise ValueError(f"missing {key}")
    try:
        inc = float(msg["angle_increment"])
        float(msg["angle_min"])
        int(msg["seq"])
    except (TypeError, ValueError):
        raise ValueError("angle_min / angle_increment / seq not numeric")
    if not (0.0 < abs(inc) <= 0.1):
        raise ValueError(f"angle_increment {inc} out of range")
    for r in ranges:
        if r is not None and not isinstance(r, (int, float)):
            raise ValueError("ranges must be numbers or null")
    msg = dict(msg)
    msg["received"] = time.time()
    with _WRITE_LOCK:                                   # two clients must not share the .tmp
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(msg, separators=(",", ":")), encoding="utf-8")
        tmp.replace(path)


class ClientHandler(threading.Thread):
    def __init__(self, conn, addr, stats: Stats, path: Path = None):
        super().__init__(daemon=True)
        self.conn, self.addr, self.stats = conn, addr, stats
        self.path = Path(path or LATEST_FILE)

    def run(self):
        print(f"[LIDAR] client connected: {self.addr}", flush=True)
        buf = b""
        try:
            self.conn.settimeout(10.0)
            while True:
                chunk = self.conn.recv(262144)
                if not chunk:
                    break
                buf += chunk
                if len(buf) > MAX_LINE_BYTES and b"\n" not in buf:
                    buf = b""                       # garbage without newline: drop
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    if not raw.strip():
                        continue
                    try:
                        msg = json.loads(raw.decode("utf-8"))
                        store_scan(msg, self.path)
                        with self.stats.lock:
                            self.stats.received += 1
                            self.stats.last_seq = msg.get("seq")
                            n = self.stats.received
                        if n % 50 == 0:
                            print(f"[LIDAR] scans={n} seq={msg.get('seq')} points={len(msg['ranges'])}", flush=True)
                    except (ValueError, KeyError, UnicodeDecodeError) as exc:
                        with self.stats.lock:
                            self.stats.rejected += 1
                        print(f"[LIDAR] rejected packet: {exc}", flush=True)
        except Exception as exc:
            print(f"[LIDAR] {self.addr} disconnected: {exc}", flush=True)
        finally:
            try:
                self.conn.close()
            except OSError:
                pass
            print(f"[LIDAR] client closed: {self.addr}", flush=True)


def serve(host: str = HOST, port: int = PORT, path: Path = None, stop: threading.Event = None,
          stats: Stats = None, ready: threading.Event = None, bound: list = None) -> Stats:
    """Accept relay clients until `stop` is set (or Ctrl+C). port=0 -> any free port."""
    stats = stats or Stats()
    stop = stop or threading.Event()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(2)
    server.settimeout(0.2)
    if bound is not None:
        bound.append(server.getsockname()[1])
    if ready is not None:
        ready.set()
    try:
        while not stop.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            ClientHandler(conn, addr, stats, path).start()
    finally:
        server.close()
    return stats


def main() -> int:
    print("=" * 70, flush=True)
    print("NIDAR LIDAR RECEIVER (Gazebo scans -> Pi)", flush=True)
    print(f"Listening on : {HOST}:{PORT}", flush=True)
    print(f"Latest scan  : {LATEST_FILE}", flush=True)
    print("=" * 70, flush=True)
    try:
        serve()
    except KeyboardInterrupt:
        print("\nStopping LiDAR receiver...", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
