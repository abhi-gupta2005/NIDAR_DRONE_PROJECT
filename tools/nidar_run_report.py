#!/usr/bin/env python3
"""
NIDAR run report -- READ ONLY analysis of the most recent live run on the Pi.

Answers three questions from the files a run leaves behind:
  1. In what ORDER did the AirMouse server send targets and stops?
     (logs/airmouse_server.log, no timestamps -> line order)
  2. How fast did camera frames really arrive at the Pi?
     (src/communication/nidar_perception_objects.csv)
  3. When V18 marked the person TEMP_LOST, was it because
       a) no new camera frame arrived for > CAMERA_GRACE_SEC   -> camera slow (or closed)
       b) a new frame arrived but did not fuse with LiDAR      -> fusion miss
     (nidar_v18_headless_persistent_fusion.csv)

Run on the Pi from the project folder:
    python tools/nidar_run_report.py
Nothing is modified.
"""

from __future__ import annotations

import csv
import os
import statistics
import sys
from collections import Counter, deque
from pathlib import Path

ROOT = Path(os.environ.get("NIDAR_PROJECT_DIR", Path(__file__).resolve().parents[1]))
AIRMOUSE_LOG = ROOT / "logs" / "airmouse_server.log"
PERCEPTION_CSV = ROOT / "src" / "communication" / "nidar_perception_objects.csv"
V18_CSV = ROOT / "nidar_v18_headless_persistent_fusion.csv"
TAIL_ROWS = 400_000          # rows read from the end of each CSV (bounded memory on a Pi 3)


def tail_csv(path: Path, n: int):
    """Header + last n data rows of a CSV, without loading the whole file at once."""
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        rows = deque(reader, maxlen=n)
    return header, list(rows)


def last_session(rows, frame_idx):
    """Rows after the last time the frame number went DOWN (= camera program restarted)."""
    start = 0
    prev = None
    for i, r in enumerate(rows):
        try:
            fr = int(float(r[frame_idx]))
        except (ValueError, IndexError):
            continue
        if prev is not None and fr < prev:
            start = i
        prev = fr
    return rows[start:]


def section(title):
    print()
    print("=" * 70)
    print(title)
    print("=" * 70)


def airmouse_order():
    section("1. AIRMOUSE SERVER: order of events (latest run)")
    if not AIRMOUSE_LOG.exists():
        print("log not found:", AIRMOUSE_LOG)
        return
    lines = [l.rstrip("\n") for l in AIRMOUSE_LOG.open(encoding="utf-8", errors="replace")]
    events = []
    for no, line in enumerate(lines, 1):
        if not line.startswith("[NIDAR]"):
            if "CLIENT" in line:
                events.append((no, line.strip()))
            continue
        body = line[len("[NIDAR] "):]
        if "-> STOP" in body:
            kind = body.split()[0] + " -> STOP"
        else:
            parts = body.split()
            kind = f"{parts[0]} {parts[1]}" if len(parts) > 1 else body   # e.g. "person ACTIVE"
        events.append((no, kind))
    # compress consecutive identical kinds; ACTIVE/TEMP_LOST alternation is one block
    blocks = []
    for no, kind in events:
        group = "person ACTIVE/TEMP_LOST" if kind.startswith("person ") else kind
        if blocks and blocks[-1][2] == group:
            blocks[-1][1] = no
            blocks[-1][3][kind] += 1
        else:
            blocks.append([no, no, group, Counter({kind: 1})])
    print(f"{len(lines)} lines in {AIRMOUSE_LOG.name}")
    for first, last, group, counts in blocks:
        detail = ", ".join(f"{k}: {v}" for k, v in counts.items())
        print(f"  lines {first:5d}-{last:<5d}  {group:28s} ({detail})")
    if blocks:
        last_group = blocks[-1][2]
        print()
        if "STOP" in last_group:
            print("  -> The run ENDED with a stop. If the camera was closed before the Pi was")
            print("     stopped, this is the stale-camera stop working on the real Pi.")
        else:
            print("  -> The run ended while a person was still being sent: the camera was still")
            print("     running when the Pi was stopped, so the stop-after-close test did not happen.")


def camera_rate():
    section("2. CAMERA FRAMES ARRIVING AT THE PI (latest camera session)")
    if not PERCEPTION_CSV.exists():
        print("not found:", PERCEPTION_CSV)
        return None
    header, rows = tail_csv(PERCEPTION_CSV, TAIL_ROWS)
    fi, ti = header.index("frame"), header.index("timestamp")
    rows = last_session(rows, fi)
    frames = {}
    for r in rows:
        try:
            frames[int(float(r[fi]))] = float(r[ti])
        except (ValueError, IndexError):
            pass
    if len(frames) < 3:
        print("not enough frames with objects in the latest session")
        return None
    nums = sorted(frames)
    ts = [frames[n] for n in nums]
    span = ts[-1] - ts[0]
    total = nums[-1] - nums[0] + 1
    dts = [b - a for a, b in zip(ts, ts[1:])]
    print(f"frames {nums[0]} .. {nums[-1]} over {span:.1f} s (Windows clock)")
    print(f"camera rate          : {(total - 1) / span:.2f} frames/s  (frame numbers / time)")
    print(f"frames with objects  : {len(nums)} of {total} ({100 * len(nums) / total:.0f} %) -- others had nobody in view")
    print(f"gap between frames   : median {statistics.median(dts):.3f} s, 95th pct "
          f"{sorted(dts)[int(0.95 * (len(dts) - 1))]:.3f} s, max {max(dts):.2f} s")
    slow = sum(1 for d in dts if d > 0.25)
    print(f"gaps > 0.25 s        : {slow} of {len(dts)} ({100 * slow / len(dts):.0f} %)  "
          f"<- V18.1 CAMERA_GRACE_SEC; longer gaps age the person to TEMP_LOST")
    return (total - 1) / span


def v18_causes():
    section("3. WHY V18 MARKED THE PERSON TEMP_LOST (latest camera session)")
    if not V18_CSV.exists():
        print("not found:", V18_CSV)
        return
    header, rows = tail_csv(V18_CSV, TAIL_ROWS)
    fi, ti = header.index("frame"), header.index("timestamp")
    si, sti = header.index("stable_id"), header.index("state")
    rows = last_session(rows, fi)
    state_counts = Counter()
    gap_aged = fusion_miss = 0
    last_active_frame = {}
    last_frame_change_t = {}
    prev_frame = None
    frame_change_t = None
    for r in rows:
        try:
            fr = int(float(r[fi])); t = float(r[ti]); sid = r[si]; st = r[sti]
        except (ValueError, IndexError):
            continue
        if fr != prev_frame:
            prev_frame, frame_change_t = fr, t
        state_counts[st] += 1
        if st == "ACTIVE":
            last_active_frame[sid] = fr
        elif st == "TEMP_LOST" and sid in last_active_frame:
            if fr == last_active_frame[sid]:
                gap_aged += 1       # same camera frame as when it was ACTIVE -> no new frame arrived
            else:
                fusion_miss += 1    # a newer frame arrived, object not refreshed -> no LiDAR match
    total = sum(state_counts.values())
    print(f"V18 rows (one per object per 50 ms loop): {total}")
    for st, n in state_counts.most_common():
        print(f"  {st:10s} {n:7d}  ({100 * n / max(total, 1):.0f} %)")
    tl = gap_aged + fusion_miss
    if tl:
        print()
        print(f"TEMP_LOST rows with NO new camera frame since ACTIVE : {gap_aged:6d} ({100 * gap_aged / tl:.0f} %)  -> camera slow or stopped")
        print(f"TEMP_LOST rows AFTER a new camera frame              : {fusion_miss:6d} ({100 * fusion_miss / tl:.0f} %)  -> frame did not fuse with LiDAR")


def main():
    print(f"NIDAR run report -- project {ROOT}")
    airmouse_order()
    camera_rate()
    v18_causes()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
