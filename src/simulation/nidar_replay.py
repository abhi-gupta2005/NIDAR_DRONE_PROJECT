#!/usr/bin/env python3
"""
NIDAR perception replay -- send a recorded NIDAR_PERCEPTION_V1 stream to the
Pi receiver exactly like nidar_v11_windows.py would, without a camera or GPU.

Why: the Windows -> Pi link has never been verified (the Pi's last stored
packet came from a Mac on 1 Oct 2026).  This tool lets you test the network,
the receiver and the whole Pi pipeline from the Windows laptop, the Pi
itself (localhost) or WSL, using the real 37-minute recording.

Examples
--------
Windows PowerShell (.venv), to the Pi:
    python -m src.simulation.nidar_replay --host 10.89.74.147 --port 5000

Loopback test on the Pi (receiver already running from start_nidar_pi.sh):
    python3 -m src.simulation.nidar_replay --host 127.0.0.1 --port 5000 --limit 500

Options:
    --speed 2.0        replay twice as fast as recorded (0 = as fast as possible)
    --rewrite-time     stamp packets with the current wall clock so the
                       integration monitor shows LIVE (recommended)
    --keep-time        keep the original timestamps (tests stale detection)
    --loop             restart from the beginning when the file ends

Wire format: compact JSON + "\\n" per packet -- identical bytes to
PiPerceptionPublisher in nidar_v11_windows.py (checked by the test suite).
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path
from typing import Iterator, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RECORDING = PROJECT_ROOT / "multi_object_tracking_v10.jsonl"


def encode_packet(packet: dict) -> bytes:
    """Same bytes as nidar_v11_windows.PiPerceptionPublisher._worker()."""
    return (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")


def iter_packets(path: Path, limit: Optional[int] = None) -> Iterator[dict]:
    count = 0
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            packet = json.loads(line)
            if packet.get("protocol") != "NIDAR_PERCEPTION_V1":
                continue
            yield packet
            count += 1
            if limit is not None and count >= limit:
                return


def connect(host: str, port: int, timeout: float = 2.0) -> socket.socket:
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.settimeout(timeout)
    return sock


def replay(path: Path, host: str, port: int, speed: float = 1.0, rewrite_time: bool = True,
           limit: Optional[int] = None, loop: bool = False, quiet: bool = False) -> dict:
    stats = {"sent": 0, "reconnects": 0, "errors": 0, "started": time.time()}
    sock: Optional[socket.socket] = None
    frame_offset = 0
    last_frame = 0

    def log(msg):
        if not quiet:
            print(msg, flush=True)

    while True:
        prev_ts = None
        for packet in iter_packets(path, limit):
            # keep frame numbers strictly increasing across loops
            packet = dict(packet)
            packet["frame"] = int(packet["frame"]) + frame_offset
            last_frame = packet["frame"]
            if rewrite_time:
                now = time.time()
                packet["timestamp"] = now
                for obj in packet.get("objects", []):
                    obj["timestamp"] = now
                    obj["frame"] = packet["frame"]

            # pacing from the recorded inter-packet interval
            if speed > 0 and prev_ts is not None:
                dt = max(0.0, (packet_ts_original(packet, prev_ts) - prev_ts)) / speed
                if dt > 0:
                    time.sleep(min(dt, 2.0))
            prev_ts = packet_ts_original(packet, prev_ts)

            payload = encode_packet(packet)
            while True:
                if sock is None:
                    try:
                        sock = connect(host, port)
                        stats["reconnects"] += 1
                        log(f"[REPLAY] connected to {host}:{port}")
                    except OSError as exc:
                        stats["errors"] += 1
                        log(f"[REPLAY] connect failed ({exc}); retrying in 1 s")
                        time.sleep(1.0)
                        continue
                try:
                    sock.sendall(payload)
                    stats["sent"] += 1
                    break
                except OSError as exc:
                    stats["errors"] += 1
                    log(f"[REPLAY] send failed ({exc}); reconnecting")
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = None

            if stats["sent"] % 100 == 0:
                elapsed = time.time() - stats["started"]
                log(f"[REPLAY] sent={stats['sent']} frame={packet['frame']} "
                    f"objects={len(packet.get('objects', []))} rate={stats['sent'] / max(elapsed, 1e-6):.1f} pkt/s")

        if not loop:
            break
        frame_offset = last_frame
        log("[REPLAY] end of recording, looping")

    if sock is not None:
        sock.close()
    stats["elapsed_s"] = round(time.time() - stats["started"], 3)
    return stats


_ORIGINAL_TS_KEY = "_original_timestamp"


def packet_ts_original(packet: dict, fallback: Optional[float]) -> float:
    """Recorded timestamp used for pacing (survives --rewrite-time)."""
    ts = packet.get(_ORIGINAL_TS_KEY)
    if ts is None:
        ts = packet.get("timestamp", fallback if fallback is not None else 0.0)
        packet[_ORIGINAL_TS_KEY] = ts
    return float(ts)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Replay a NIDAR_PERCEPTION_V1 recording to the Pi receiver")
    parser.add_argument("--file", default=str(DEFAULT_RECORDING))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--speed", type=float, default=1.0, help="1.0 = real time, 0 = max speed")
    parser.add_argument("--limit", type=int, default=None, help="stop after N packets")
    parser.add_argument("--loop", action="store_true")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--rewrite-time", dest="rewrite", action="store_true", default=True)
    group.add_argument("--keep-time", dest="rewrite", action="store_false")
    args = parser.parse_args(argv)

    path = Path(args.file)
    if not path.exists():
        print(f"recording not found: {path}", file=sys.stderr)
        return 2

    print("=" * 70)
    print("NIDAR PERCEPTION REPLAY")
    print("=" * 70)
    print(f"Recording : {path}")
    print(f"Target    : {args.host}:{args.port}")
    print(f"Speed     : {'max' if args.speed <= 0 else f'{args.speed:.2f}x'}")
    print(f"Timestamps: {'rewritten to now' if args.rewrite else 'original (stale)'}")
    print("Press Ctrl+C to stop.")
    print("=" * 70, flush=True)
    try:
        stats = replay(path, args.host, args.port, args.speed, args.rewrite, args.limit, args.loop)
    except KeyboardInterrupt:
        print("\nstopped by user")
        return 0
    print(f"[REPLAY] done: {stats}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
