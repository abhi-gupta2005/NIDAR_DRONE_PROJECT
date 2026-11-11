"""Flight records: the ground station saves the mission-server stream, the Logs screen replays it (pure; tested).

File: <folder>/flight.jsonl.gz, one JSON object per line:
  {"kind": "meta", "t": ..., "drone": ..., "source": "live" | "demo", "endpoint": ..., "note": ...}
  {"kind": "state", "t": ..., "msg": <NIDAR_MISSION_V1 state, path_tail cut to 30 points>}
  {"kind": "map", "t": ..., "msg": <NIDAR_MISSION_V1 map message, as received>}
<folder>/summary.json is written when the recording is closed (duration, survivors); a record cut off by a
crash still replays up to its last complete line.
"""

from __future__ import annotations

import base64
import bisect
import gzip
import json
import time
import zlib
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

RECORD_NAME = "flight.jsonl.gz"
SUMMARY_NAME = "summary.json"
STATE_HZ = 2.0
TAIL_KEEP = 30


def decode_map(msg: dict) -> Optional[dict]:
    try:
        raw = zlib.decompress(base64.b64decode(msg["data"]))
        grid = np.frombuffer(raw, dtype=np.int8).reshape(msg["shape"]).copy()
    except (KeyError, ValueError, TypeError, zlib.error):
        return None
    return dict(msg, grid=grid)


class Recorder:
    def __init__(self, folder: Path, meta: dict, state_hz: float = STATE_HZ):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / RECORD_NAME
        self.f = gzip.open(self.path, "at", encoding="utf-8")
        self.min_dt = 1.0 / state_hz
        self.last_state_t = None
        self.last_flush = time.time()
        self.t0 = None
        self.t1 = None
        self.n_states = 0
        self.survivors = set()
        self.closed = False
        self._write({"kind": "meta", "t": time.time(), **meta})

    def _write(self, obj: dict) -> None:
        self.f.write(json.dumps(obj, separators=(",", ":"), default=str) + "\n")
        if time.time() - self.last_flush > 2.0:
            self.f.flush()                      # Z_SYNC_FLUSH: everything so far is readable after a crash
            self.last_flush = time.time()

    def add_state(self, t: float, state: dict) -> bool:
        if self.closed or (self.last_state_t is not None and t - self.last_state_t < self.min_dt):
            return False
        self.last_state_t = t
        msg = dict(state)
        if isinstance(msg.get("path_tail"), list):
            msg["path_tail"] = msg["path_tail"][-TAIL_KEEP:]
        self._write({"kind": "state", "t": t, "msg": msg})
        self.t0 = t if self.t0 is None else self.t0
        self.t1 = t
        self.n_states += 1
        for s in msg.get("survivors") or []:
            if s.get("status") == "CONFIRMED":
                self.survivors.add(s.get("id"))
        return True

    def add_map(self, t: float, msg: dict) -> None:
        if not self.closed:
            self._write({"kind": "map", "t": t, "msg": {k: v for k, v in msg.items() if k != "grid"}})

    def close(self, extra: Optional[dict] = None) -> None:
        if self.closed:
            return
        self.closed = True
        self.f.close()
        summary = {"duration_s": (self.t1 - self.t0) if self.t0 is not None else 0.0, "states": self.n_states,
                   "survivors": sorted(s for s in self.survivors if s), "closed": time.time()}
        summary.update(extra or {})
        (self.folder / SUMMARY_NAME).write_text(json.dumps(summary, indent=1))


class FlightRecord:
    def __init__(self, path: Path, meta: dict, states: List[Tuple[float, dict]], maps: List[Tuple[float, dict]],
                 complete: bool):
        self.path, self.meta, self.states, self.maps, self.complete = Path(path), meta, states, maps, complete
        self.state_t = [t for t, _ in states]
        self.map_t = [t for t, _ in maps]
        self._decoded = {}

    @classmethod
    def load(cls, path) -> "FlightRecord":
        path = Path(path)
        if path.is_dir():
            path = path / RECORD_NAME
        meta, states, maps, complete = {}, [], [], True
        try:
            with gzip.open(path, "rt", encoding="utf-8") as f:
                for line in f:
                    try:
                        obj = json.loads(line)
                    except ValueError:
                        complete = False                  # half-written last line
                        break
                    kind = obj.get("kind")
                    if kind == "meta" and not meta:
                        meta = obj
                    elif kind == "state":
                        states.append((float(obj["t"]), obj["msg"]))
                    elif kind == "map":
                        maps.append((float(obj["t"]), obj["msg"]))
        except (EOFError, OSError, zlib.error):
            complete = False                              # cut off by a crash: keep what was read
        states.sort(key=lambda s: s[0])
        maps.sort(key=lambda m: m[0])
        return cls(path, meta, states, maps, complete)

    @property
    def t0(self) -> float:
        return self.states[0][0] if self.states else 0.0

    @property
    def t1(self) -> float:
        return self.states[-1][0] if self.states else 0.0

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    def state_at(self, t: float) -> Tuple[Optional[dict], int]:
        k = bisect.bisect_right(self.state_t, t) - 1
        return (self.states[k][1], k) if k >= 0 else (None, -1)

    def map_at(self, t: float) -> Tuple[Optional[dict], int]:
        k = bisect.bisect_right(self.map_t, t) - 1
        if k < 0:
            return None, -1
        if k not in self._decoded:
            if len(self._decoded) > 8:
                self._decoded.clear()
            self._decoded[k] = decode_map(self.maps[k][1])
        return self._decoded[k], k


def list_records(roots) -> List[dict]:
    """Every flight.jsonl.gz under the given folders (one level of run folders deep), newest first."""
    seen, out = set(), []
    for root in roots:
        root = Path(root)
        if not root.is_dir():
            continue
        for p in list(root.glob(f"*/{RECORD_NAME}")) + list(root.glob(f"*/*/{RECORD_NAME}")):
            rp = p.resolve()
            if rp in seen:
                continue
            seen.add(rp)
            info = {"path": p, "folder": p.parent, "mtime": p.stat().st_mtime, "size": p.stat().st_size,
                    "meta": {}, "summary": {}}
            try:
                with gzip.open(p, "rt", encoding="utf-8") as f:
                    info["meta"] = json.loads(f.readline() or "{}")
            except (OSError, ValueError, EOFError, zlib.error):
                pass
            try:
                info["summary"] = json.loads((p.parent / SUMMARY_NAME).read_text())
            except (OSError, ValueError):
                pass
            out.append(info)
    out.sort(key=lambda r: r["meta"].get("t", r["mtime"]), reverse=True)
    return out


class ReplaySource:
    """Plays a FlightRecord as if it were the live mission server (demo mode). Same interface the app
    uses for MissionClient: snapshot(), connected, last_rx, host, port, error."""

    def __init__(self, record: FlightRecord, speed: float = 1.0, start_frac: float = 0.0, loop: bool = False):
        self.rec, self.speed, self.loop = record, speed, loop
        self.host, self.port, self.error = "demo", 0, ""
        self.offset = record.t0 + start_frac * record.duration
        self.wall0 = time.time()
        self.connected = True
        self.last_rx = time.time()
        self.paused = False

    def sim_time(self) -> float:
        t = self.offset + (time.time() - self.wall0) * self.speed
        if t > self.rec.t1:
            if self.loop:
                self.offset, self.wall0 = self.rec.t0, time.time()
                return self.rec.t0
            return self.rec.t1
        return t

    def snapshot(self):
        t = self.sim_time()
        self.last_rx = time.time()
        st, _ = self.rec.state_at(t)              # the recorded object itself: a new one = a new message
        mp, k = self.rec.map_at(t)
        return st, mp, k

    def start(self):
        pass

    def stop(self):
        pass
