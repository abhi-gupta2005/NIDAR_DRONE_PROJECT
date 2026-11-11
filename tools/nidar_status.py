#!/usr/bin/env python3
"""
NIDAR run status (2026-11-09) -- used by tools/nidar_run.sh, runs in WSL. Read-only: sends nothing.

Reads the Pi mission server (NIDAR_MISSION_V1, TCP 5020) like the ground station does and turns it into
one status line + plain event lines ("LOST: ...", "BACK: ...") so a dropped link is named, not guessed.

    python3 tools/nidar_status.py --pi 10.218.70.147 --wait-tracking 240   # exit 0 once SLAM is TRACKING
    python3 tools/nidar_status.py --pi 10.218.70.147 --watch logs/run_x     # status.line / events.log, every 1 s
"""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

PORT = int(os.environ.get("NIDAR_MISSION_PORT", "5020"))
STALE_S = 3.0          # a sensor older than this is reported LOST
LINK_LOST_S = 4.0      # no message from the mission server for this long -> link LOST
SLOW_RTF = 0.3         # Gazebo slower than this: warn (PX4 SITL runs in lockstep with it)


def _age(v) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def summarize(snap: Optional[dict], rx_age: Optional[float], no_brain: str = "off") -> Dict[str, object]:
    """Mission-server state message -> flat summary (pure; tested)."""
    s: Dict[str, object] = {"link": "UP" if rx_age is not None and rx_age < LINK_LOST_S else "DOWN",
                            "rx_age": rx_age}
    snap = snap or {}
    h = snap.get("health") or {}
    sl = h.get("slam") or {}
    s["slam"] = sl.get("status") or "NO DATA"
    s["slam_age"] = _age(sl.get("age_s"))
    s["slam_hz"] = _age(sl.get("fps"))
    s["lidar_age"] = _age((h.get("lidar") or {}).get("age_s"))
    s["camera_age"] = _age((h.get("perception") or {}).get("age_s"))
    s["fusion_age"] = _age((h.get("v18") or {}).get("age_s"))
    ex = snap.get("explorer")
    s["brain"] = (ex or {}).get("state") or no_brain       # "PX4 route" in PX4 runs (the brain is not used there)
    s["plan"] = (ex or {}).get("plan_id")
    s["hold"] = (ex or {}).get("hold_reason")
    s["return_reason"] = (ex or {}).get("return_reason")
    s["survivors"] = sum(1 for v in snap.get("survivors") or [] if v.get("status") == "CONFIRMED")
    p = snap.get("pose") or {}
    s["cell"] = p.get("grid")
    s["tracking"] = s["link"] == "UP" and s["slam"] == "TRACKING" and (s["slam_age"] or 99) < 1.5
    return s


def fmt_age(a) -> str:
    return "—" if a is None else ("LOST" if a > STALE_S else f"{a:.1f}s")


def status_line(s: Dict[str, object], rtf: Optional[float] = None) -> str:
    if s["link"] != "UP":
        return f"Pi mission server: NO DATA ({fmt_age(s['rx_age'])}) — Pi stack not up yet or network down"
    hz = f" {s['slam_hz']:.1f} Hz" if isinstance(s["slam_hz"], float) else ""
    brain = f"{s['brain']}" + (f" plan {s['plan']}" if s["plan"] else "") + \
        (f" ({s['hold']})" if s["brain"] == "HOLD" and s["hold"] else "")
    out = (f"SLAM {s['slam']}{hz} | LiDAR {fmt_age(s['lidar_age'])} | camera {fmt_age(s['camera_age'])} | "
           f"fusion {fmt_age(s['fusion_age'])} | BRAIN {brain} | survivors {s['survivors']}")
    if rtf is not None:
        out += f" | Gazebo speed {rtf:.2f}x"
    return out


def events(prev: Optional[Dict[str, object]], cur: Dict[str, object]) -> List[str]:
    """Plain-language transitions between two summaries (pure; tested)."""
    out: List[str] = []
    p = prev or {}
    if cur["link"] != p.get("link"):
        out.append("BACK: Pi mission server link" if cur["link"] == "UP" and p else
                   ("Pi mission server link UP" if cur["link"] == "UP" else
                    "LOST: Pi mission server link (Wi-Fi/network or the Pi stack stopped)"))
    if cur["link"] != "UP":
        return out
    if cur["slam"] != p.get("slam") and p.get("link") == "UP":
        out.append(f"SLAM {p.get('slam')} -> {cur['slam']}" + (" (position not trusted: brain holds)"
                                                               if cur["slam"] == "LOST" else ""))
    names = {"lidar_age": "LiDAR scans (WSL relay -> Pi)", "camera_age": "camera detections (Windows YOLO -> Pi)",
             "fusion_age": "V18 fusion", "slam_age": "SLAM updates"}
    for k, name in names.items():
        a, b = p.get(k), cur[k]
        was = a is not None and a <= STALE_S
        now = b is not None and b <= STALE_S
        if p.get("link") == "UP" and was and not now:
            out.append(f"LOST: {name}")
        elif p.get("link") == "UP" and not was and now and a is not None:
            out.append(f"BACK: {name}")
    if cur["brain"] != p.get("brain") and p.get("link") == "UP":
        why = f" — {cur['hold']}" if cur["brain"] == "HOLD" and cur["hold"] else ""
        out.append(f"BRAIN {p.get('brain')} -> {cur['brain']}{why}")
        if cur["brain"] == "COMPLETE":
            out.append("MISSION COMPLETE" + (f" ({cur['return_reason']})" if cur["return_reason"] else ""))
    if (cur["survivors"] or 0) > (p.get("survivors") or 0) and p.get("link") == "UP":
        out.append(f"survivors confirmed: {cur['survivors']}")
    return out


class Reader(threading.Thread):
    """Keeps a connection to the mission server; holds the newest state message."""

    def __init__(self, host: str, port: int = PORT):
        super().__init__(daemon=True)
        self.addr = (host, port)
        self.state: Optional[dict] = None
        self.last_rx: Optional[float] = None
        self.lock = threading.Lock()
        self.halt = threading.Event()

    def run(self):
        while not self.halt.is_set():
            try:
                s = socket.create_connection(self.addr, timeout=3.0)
                s.settimeout(5.0)
                buf = b""
                while not self.halt.is_set():
                    chunk = s.recv(1 << 20)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if b'"type": "state"' in line[:200] or b'"type":"state"' in line[:200]:
                            try:
                                msg = json.loads(line)
                            except ValueError:
                                continue
                            with self.lock:
                                self.state, self.last_rx = msg, time.time()
                        else:
                            with self.lock:
                                self.last_rx = time.time()
                s.close()
            except OSError:
                pass
            self.halt.wait(1.0)

    def snapshot(self):
        with self.lock:
            age = None if self.last_rx is None else time.time() - self.last_rx
            return self.state, age


def gazebo_sim_time(timeout: float = 4.0) -> Optional[float]:
    """Gazebo's simulated time (s) from `gz topic -e -n 1 -t /stats`, or None."""
    try:
        r = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", "/stats"], capture_output=True, text=True,
                           timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return parse_sim_time(r.stdout)


def parse_sim_time(text: str) -> Optional[float]:
    m = re.search(r"sim_time\s*\{([^}]*)\}", text)
    if not m:
        return None
    sec = re.search(r"\bsec:\s*(\d+)", m.group(1))
    nsec = re.search(r"\bnsec:\s*(\d+)", m.group(1))
    return (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9


class SpeedMeter:
    """Gazebo speed = simulated seconds per real second between two readings. (The real_time_factor field in
    /stats said 0.03-0.12x on 2026-11-09 while the LiDAR's 10 Hz arrived at 4.2-4.5 Hz, i.e. ~0.43x.)"""

    def __init__(self):
        self.prev = None

    def update(self, wall: float, sim: Optional[float]) -> Optional[float]:
        if sim is None:
            return None
        out = None
        if self.prev is not None and wall - self.prev[0] > 1.0 and sim >= self.prev[1]:
            out = (sim - self.prev[1]) / (wall - self.prev[0])
        self.prev = (wall, sim)
        return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi", required=True, help="Pi address")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--wait-tracking", type=float, default=None, metavar="SECONDS")
    ap.add_argument("--watch", default=None, metavar="DIR", help="write DIR/status.line + DIR/events.log every 1 s")
    ap.add_argument("--no-gz", action="store_true", help="do not ask Gazebo for its speed")
    ap.add_argument("--px4-route", action="store_true", help="PX4 run: show BRAIN as 'PX4 route', not 'off'")
    args = ap.parse_args(argv)
    rd = Reader(args.pi, args.port)
    rd.start()
    t0 = time.time()
    if args.wait_tracking is not None:
        last = 0.0
        while time.time() - t0 < args.wait_tracking:
            snap, age = rd.snapshot()
            s = summarize(snap, age)
            if s["tracking"]:
                print(f"[status] SLAM TRACKING after {time.time() - t0:.0f} s", flush=True)
                return 0
            if time.time() - last >= 10:
                last = time.time()
                print(f"[status] waiting for SLAM TRACKING ({time.time() - t0:.0f} s): {status_line(s)}", flush=True)
            time.sleep(0.5)
        print("[status] SLAM did not reach TRACKING in time", flush=True)
        return 1
    if not args.watch:
        time.sleep(3.0)
        snap, age = rd.snapshot()
        print(status_line(summarize(snap, age)))
        return 0
    d = Path(args.watch)
    d.mkdir(parents=True, exist_ok=True)
    prev, rtf, last_rtf, last_slow = None, None, 0.0, 0.0
    meter = SpeedMeter()
    try:
        while True:
            snap, age = rd.snapshot()
            cur = summarize(snap, age, "PX4 route" if args.px4_route else "off")
            if not args.no_gz and time.time() - last_rtf > 15:
                last_rtf = time.time()
                rtf = meter.update(time.time(), gazebo_sim_time())
                if rtf is not None and rtf < SLOW_RTF and time.time() - last_slow > 120:
                    last_slow = time.time()           # 2026-11-09 PX4 run: 0.05x -> 2 s position gap -> flyer landed
                    with open(d / "events.log", "a", encoding="utf-8") as f:
                        f.write(f"[{time.strftime('%H:%M:%S')}] SLOW: Gazebo at {rtf:.2f}x real time - the laptop is "
                                "overloaded (PX4 may lose its position and land, QGC may say 'communication lost'). "
                                "Close QGroundControl / other heavy programs, or run with --no-qgc.\n")
            if time.time() - t0 > 5 or cur["link"] == "UP":          # give the first connection a moment
                for e in events(prev, cur):
                    stamp = time.strftime("%H:%M:%S")
                    with open(d / "events.log", "a", encoding="utf-8") as f:
                        f.write(f"[{stamp}] {e}\n")
                    print(f"[{stamp}] {e}", flush=True)
                prev = cur
            tmp = d / "status.line.tmp"
            tmp.write_text(status_line(cur, rtf) + "\n", encoding="utf-8")
            tmp.replace(d / "status.line")
            (d / "status.json").write_text(json.dumps(cur, default=str), encoding="utf-8")
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
