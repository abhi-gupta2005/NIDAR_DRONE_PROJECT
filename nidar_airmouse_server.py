#!/usr/bin/env python3

import json
import socket
import time
from pathlib import Path

HOST = "0.0.0.0"
PORT = 5010

# 2026-11-02: was the absolute path "/home/abhishek/drone_project/...".
# Same file on the Pi; now also correct when the project lives elsewhere
# (e.g. /mnt/d/NIDAR_DRONE_PROJECT in WSL).
JSON_FILE = (
    Path(__file__).resolve().parent
    / "nidar_v18_headless_persistent_fusion.json"
)

SEND_HZ = 10.0

# ------------------------------------------------------------
# 2026-11-02 SAFETY: freshness checks.
#
# Before this change, a V18 JSON frozen for any length of time
# (V18 crashed or stopped, camera disconnected) was still sent
# as an ACTIVE target, and nidar_bridge kept publishing a valid
# non-zero command (measured: 1-hour-old file -> fwd 0.99 m/s).
#
# A target is only forwarded when:
#   - the V18 JSON timestamp is newer than V18_STALE_SEC, and
#   - the V18 camera frame number changed within CAMERA_STALE_SEC.
# Otherwise an explicit non-ACTIVE target is sent, which makes
# nidar_bridge publish a zero command immediately.
# ------------------------------------------------------------

V18_STALE_SEC = 1.0
CAMERA_STALE_SEC = 1.0

_last_frame = None
_last_frame_change = None


def stop_target(reason):
    """A target the bridge always rejects (state != ACTIVE, confidence 0)."""
    return {
        "stable_id": None,
        "class": "none",
        "state": reason,
        "range_m": 0.0,
        "bearing_deg": 0.0,
        "x_m": 0.0,
        "y_m": 0.0,
        "speed_mps": 0.0,
        "confidence": 0.0,
        "camera_confidence": 0.0,
        "frame": None,
        "timestamp": None,
    }


def freshness_status(data, now):
    """Return TRACKING, V18_STALE or CAMERA_STALE for a loaded V18 JSON."""
    global _last_frame, _last_frame_change

    try:
        age = now - float(data.get("timestamp"))
    except (TypeError, ValueError):
        return "V18_STALE"

    if age > V18_STALE_SEC:
        return "V18_STALE"

    frame = data.get("frame")

    if frame is None:
        return "CAMERA_STALE"

    if frame != _last_frame or _last_frame_change is None:
        _last_frame = frame
        _last_frame_change = now

    if now - _last_frame_change > CAMERA_STALE_SEC:
        return "CAMERA_STALE"

    return "TRACKING"


def load_target():
    try:
        with JSON_FILE.open("r") as f:
            data = json.load(f)
    except Exception:
        return stop_target("V18_MISSING")

    status = freshness_status(data, time.time())

    if status != "TRACKING":
        return stop_target(status)

    targets = data.get("persistent_objects", [])

    if not targets:
        return stop_target("NO_TARGET")

    # Prefer ACTIVE targets.
    active = [
        t for t in targets
        if t.get("state") == "ACTIVE"
    ]

    if active:
        targets = active

    # Select the strongest target.
    target = max(
        targets,
        key=lambda t: float(
            t.get("fusion_confidence", 0.0)
        )
    )

    return {
        "stable_id": target.get("stable_id"),
        "class": target.get("class", "unknown"),
        "state": target.get("state", "UNKNOWN"),

        "range_m": float(
            target.get("range_m", 0.0)
        ),

        "bearing_deg": float(
            target.get("bearing_deg", 0.0)
        ),

        "x_m": float(
            target.get("x_m", 0.0)
        ),

        "y_m": float(
            target.get("y_m", 0.0)
        ),

        "speed_mps": float(
            target.get("speed_mps", 0.0)
        ),

        "confidence": float(
            target.get("fusion_confidence", 0.0)
        ),

        "camera_confidence": float(
            target.get("camera_confidence", 0.0)
        ),

        "frame": data.get("frame"),
        "timestamp": data.get("timestamp"),
    }


def make_packet():
    target = load_target()

    packet = {
        "protocol": "NIDAR_AIRMOUSE_V1",
        "timestamp": time.time(),
        # 2026-11-02: explicit status (TRACKING / NO_TARGET / V18_STALE /
        # CAMERA_STALE / V18_MISSING).  Extra field; the bridge ignores it.
        "status": (
            target.get("state")                     # stop reason
            if target.get("stable_id") is None
            else "TRACKING"
            if target.get("state") == "ACTIVE"
            else "TARGET_NOT_ACTIVE"                # bridge zeroes these
        ),
        "target": target,
    }

    return packet


def main():
    print("=" * 60)
    print("       NIDAR → AIRMOUSE NETWORK SERVER")
    print("=" * 60)
    print(f"Listening on {HOST}:{PORT}")
    print(f"Source: {JSON_FILE}")
    print("Flight control: DISABLED")
    print("=" * 60)

    server = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )

    server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1
    )

    server.bind(
        (HOST, PORT)
    )

    server.listen(5)

    # 2026-11-02: was settimeout(1.0).  accept() then blocked for a full
    # second on every loop without a new client, so packets went out at
    # ~0.9 Hz instead of SEND_HZ (measured gaps 1.1 s) -- longer than the
    # bridge's 1.0 s TARGET_TIMEOUT, so commands pulsed on/off.
    # Non-blocking accept keeps the loop at SEND_HZ.
    server.setblocking(False)

    print("Waiting for WSL2 AirMouse client...")

    clients = []

    last_status = None
    last_print = 0.0

    try:
        while True:

            try:
                conn, addr = server.accept()

                conn.settimeout(2.0)

                clients.append(conn)

                print(
                    f"[CLIENT CONNECTED] "
                    f"{addr[0]}:{addr[1]}"
                )

            except (socket.timeout, BlockingIOError, InterruptedError):
                pass

            packet = make_packet()

            message = (
                json.dumps(packet)
                + "\n"
            ).encode("utf-8")

            alive_clients = []

            for conn in clients:

                try:
                    conn.sendall(message)
                    alive_clients.append(conn)

                except Exception:
                    try:
                        peer = conn.getpeername()
                    except Exception:
                        peer = "unknown"

                    print(
                        f"[CLIENT DISCONNECTED] "
                        f"{peer}"
                    )

                    try:
                        conn.close()
                    except Exception:
                        pass

            clients = alive_clients

            target = packet["target"]

            # 2026-11-02: the loop now really runs at SEND_HZ; print once per
            # second, or immediately when the status changes, so the log
            # grows no faster than before.
            now = time.time()
            if (
                packet["status"] == last_status
                and now - last_print < 1.0
            ):
                time.sleep(1.0 / SEND_HZ)
                continue

            last_status = packet["status"]
            last_print = now

            if target.get("stable_id") is not None:

                print(
                    f"[NIDAR] "
                    f"{target['class']} "
                    f"{target['state']} | "
                    f"R={target['range_m']:.2f}m | "
                    f"X={target['x_m']:+.2f}m | "
                    f"Y={target['y_m']:+.2f}m | "
                    f"conf={target['confidence']:.2f}"
                )

            else:

                print(
                    f"[NIDAR] {packet['status']} -> STOP"
                )

            time.sleep(
                1.0 / SEND_HZ
            )

    except KeyboardInterrupt:
        print("\nStopping server...")

    finally:

        for conn in clients:
            try:
                conn.close()
            except Exception:
                pass

        server.close()

        print("Server stopped.")


if __name__ == "__main__":
    main()
