#!/usr/bin/env python3
"""
NIDAR V11 - Raspberry Pi Perception Receiver

Receives NIDAR_PERCEPTION_V1 JSON packets from the Mac over TCP,
keeps the newest perception state on disk, and appends a compact
object log for later LiDAR/SLAM fusion.

Run on Raspberry Pi:
    python3 nidar_pi_receiver.py

Optional:
    NIDAR_PI_PORT=5000 python3 nidar_pi_receiver.py
"""

import csv
import json
import os
import socket
import threading
import time
from pathlib import Path

HOST = "0.0.0.0"
PORT = int(os.environ.get("NIDAR_PI_PORT", "5000"))

BASE_DIR = Path(__file__).resolve().parent
LATEST_FILE = BASE_DIR / "nidar_latest_perception.json"
OBJECT_LOG = BASE_DIR / "nidar_perception_objects.csv"


class ClientHandler(threading.Thread):
    def __init__(self, conn, addr):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr

    def run(self):
        print(f"[CLIENT] Connected: {self.addr}")
        buffer = b""

        try:
            self.conn.settimeout(10.0)

            while True:
                chunk = self.conn.recv(65536)
                if not chunk:
                    break

                buffer += chunk

                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    if not raw.strip():
                        continue

                    try:
                        packet = json.loads(raw.decode("utf-8"))
                    except Exception as exc:
                        print(f"[WARN] Invalid JSON from {self.addr}: {exc}")
                        continue

                    if packet.get("protocol") != "NIDAR_PERCEPTION_V1":
                        print("[WARN] Unknown protocol packet ignored.")
                        continue

                    self.process_packet(packet)

        except Exception as exc:
            print(f"[CLIENT] {self.addr} disconnected: {exc}")
        finally:
            try:
                self.conn.close()
            except Exception:
                pass
            print(f"[CLIENT] Closed: {self.addr}")

    def process_packet(self, packet):
        # Atomic-ish latest-state update: write a temporary file then replace.
        temp = LATEST_FILE.with_suffix(".tmp")
        temp.write_text(json.dumps(packet, indent=2), encoding="utf-8")
        temp.replace(LATEST_FILE)

        timestamp = packet.get("timestamp", time.time())
        frame = packet.get("frame", -1)

        rows = []
        for obj in packet.get("objects", []):
            rows.append([
                timestamp,
                frame,
                obj.get("id"),
                obj.get("class"),
                obj.get("state"),
                obj.get("spatial_status"),
                obj.get("confidence"),
                obj.get("motion_confidence"),
                obj.get("track_stability_score"),
                obj.get("tracker_id"),
                obj.get("center", [None, None])[0],
                obj.get("center", [None, None])[1],
                obj.get("predicted_center", [None, None])[0],
                obj.get("predicted_center", [None, None])[1],
                obj.get("prediction_horizon_frames"),
                obj.get("prediction_reliability"),
            ])

        file_exists = OBJECT_LOG.exists()
        with OBJECT_LOG.open("a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "timestamp", "frame", "stable_id", "class", "state",
                    "spatial_status", "confidence", "motion_confidence",
                    "track_stability_score", "tracker_id",
                    "center_x", "center_y", "pred_center_x", "pred_center_y",
                    "prediction_horizon_frames", "prediction_reliability",
                ])
            writer.writerows(rows)

        if frame % 30 == 0:
            confirmed = sum(
                1 for o in packet.get("objects", [])
                if o.get("state") in ("CONFIRMED", "RECOVERED")
            )
            predicted = sum(
                1 for o in packet.get("objects", [])
                if o.get("spatial_status") == "PREDICTED"
            )
            print(
                f"[NIDAR] frame={frame} objects={len(packet.get('objects', []))} "
                f"confirmed={confirmed} predicted={predicted}"
            )


def main():
    print("=" * 70)
    print("NIDAR V11 - RASPBERRY PI PERCEPTION RECEIVER")
    print("=" * 70)
    print(f"Listening on         : {HOST}:{PORT}")
    print(f"Latest state         : {LATEST_FILE}")
    print(f"Object log           : {OBJECT_LOG}")
    print()
    print("This Pi receiver does NOT control the flight controller.")
    print("It only receives NIDAR perception data.")
    print("Press Ctrl+C to stop.")
    print("=" * 70)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(2)
    server.settimeout(1.0)

    try:
        while True:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue

            ClientHandler(conn, addr).start()

    except KeyboardInterrupt:
        print("\nStopping receiver...")
    finally:
        server.close()


if __name__ == "__main__":
    main()
