"""What the ground station works out from the mission-server stream (pure Python, no Qt; tested).

The same MissionModel runs on the live stream and on a recorded flight, so the event list, mission clock,
survivor first-seen times and flown distance agree between the Fly screen and the replay.
"""

from __future__ import annotations

import importlib.util
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("nidar_status_for_gcs", REPO / "tools/nidar_status.py")
status = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(status)

LINK_LOST_S = status.LINK_LOST_S
STALE_S = status.STALE_S
PATH_STEP_M = 0.05
PAD_RADIUS_M = 0.6


@dataclass
class Event:
    t: float
    kind: str            # survivor | alert | warn | ok | brain | info
    text: str


def where(s: dict) -> str:
    cell = s.get("grid")
    if cell and cell not in ("--", "OUT"):
        return cell
    try:
        return f"({float(s['x_m']):.1f}, {float(s['y_m']):.1f})"
    except (KeyError, TypeError, ValueError):
        return "?"


def confirmed(state: Optional[dict]) -> List[dict]:
    return [s for s in (state or {}).get("survivors") or [] if s.get("status") == "CONFIRMED"]


def _kind(line: str) -> str:
    if line.startswith("LOST"):
        return "alert" if "mission server" in line else "warn"
    if line.startswith("BACK") or "link UP" in line:
        return "ok"
    if line.startswith("SLAM"):
        return "alert" if line.split("->")[-1].strip().startswith("LOST") else "ok"
    if line.startswith("BRAIN") or line.startswith("MISSION"):
        return "ok" if "COMPLETE" in line else "brain"
    return "info"


def plan_text(explorer: Optional[dict]) -> str:
    plan = (explorer or {}).get("plan") or {}
    reason = str(plan.get("reason") or "").split("->")[0].strip()
    pid = (explorer or {}).get("plan_id")
    if not plan:
        return ""
    end = {"look": "then look around", "land": "then land", "none": ""}.get(plan.get("end"), "")
    return " · ".join(x for x in (f"plan {pid}" if pid is not None else "", reason, end) if x)


def alerts(state: Optional[dict], link_up: bool) -> List[Tuple[str, str]]:
    """Conditions that hold right now, worst first: (kind, text)."""
    if not link_up:
        return [("alert", "NO LINK TO THE PI — mission server not answering")]
    st = state or {}
    out: List[Tuple[str, str]] = []
    h = st.get("health") or {}
    sl = h.get("slam") or {}
    pose = st.get("pose") or {}
    if sl.get("status") == "LOST" or pose.get("trusted") is False:
        out.append(("alert", "SLAM LOST — position not trusted, the brain holds"))
    elif sl.get("age_s") is None or sl.get("age_s") > STALE_S:
        out.append(("alert", "No fresh SLAM pose"))

    def stale(key, name):
        a = (h.get(key) or {}).get("age_s")
        if a is None or a > STALE_S:
            out.append(("warn", f"{name} {'stale' if a is not None else 'no data'}"))
    stale("lidar", "LiDAR scans")
    stale("perception", "Camera detections")
    stale("v18", "V18 fusion")
    ex = st.get("explorer") or {}
    if ex.get("state") == "HOLD":
        out.append(("warn", f"Brain HOLD — {ex.get('hold_reason') or 'waiting'}"))
    elif ex.get("state") == "COMPLETE":
        why = ex.get("return_reason") or ""
        out.append(("warn" if "time" in why else "ok",
                    "Mission complete — " + ("TIME LIMIT, not everything explored" if "time" in why
                                             else "back on the take-off point")))
    order = {"alert": 0, "warn": 1, "ok": 2}
    return sorted(out, key=lambda a: order.get(a[0], 3))


class MissionModel:
    def __init__(self, no_brain: str = "off"):
        self.no_brain = no_brain
        self.events: List[Event] = []
        self.first_seen: Dict[str, float] = {}
        self.where_seen: Dict[str, str] = {}
        self.brain_seen = set()
        self.prev = None
        self.poses = deque(maxlen=400)            # (t, x, y) for the speed
        self.path: List[Tuple[float, float, float]] = []
        self.distance = 0.0
        self.t_first = self.t_start = self.t_end = None
        self.state: Optional[dict] = None
        self.t: Optional[float] = None
        self.link_up = False

    # ------------------------------------------------------------------ update --
    def update(self, t: float, state: Optional[dict], link_age: Optional[float]) -> List[Event]:
        """state: a NEW state message, or None when nothing new arrived since the last call."""
        new: List[Event] = []
        self.t = t
        self.link_up = link_age is not None and link_age < LINK_LOST_S
        fresh = state is not None
        if fresh:
            self.state = state
        cur = status.summarize(self.state if self.link_up else None, link_age if self.link_up else None,
                               self.no_brain)
        if self.prev is not None or cur["link"] == "UP":
            for line in status.events(self.prev, cur):
                if not line.startswith("survivors confirmed"):
                    new.append(Event(t, _kind(line), line))
            self.prev = cur
        st = self.state if (self.link_up and fresh) else None
        if st:
            if self.t_first is None:
                self.t_first = t
            for s in confirmed(st):
                sid = s.get("id")
                if sid and sid not in self.first_seen:
                    self.first_seen[sid] = t
                    self.where_seen[sid] = where(s)
                    new.append(Event(t, "survivor", f"Survivor {sid} found at {where(s)}"))
            for e in (st.get("explorer") or {}).get("events") or []:
                if e not in self.brain_seen:
                    self.brain_seen.add(e)
                    if not e.startswith("plan "):          # plan-by-plan lines stay in the brain's own log
                        new.append(Event(t, "brain", e))
            pose = st.get("pose")
            if pose:
                x, y = float(pose["x_m"]), float(pose["y_m"])
                self.poses.append((t, x, y))
                if not self.path or math.hypot(x - self.path[-1][1], y - self.path[-1][2]) >= PATH_STEP_M:
                    if self.path:
                        self.distance += math.hypot(x - self.path[-1][1], y - self.path[-1][2])
                    self.path.append((t, x, y))
            ex = st.get("explorer") or {}
            if self.t_start is None and (ex.get("state") in ("EXPLORE", "RETURN") or
                                         (pose and (self.home_distance() or 0.0) > PAD_RADIUS_M)):
                self.t_start = t
                new.append(Event(t, "ok", "Mission started"))
            if self.t_start is not None and self.t_end is None and ex.get("state") == "COMPLETE":
                self.t_end = t
                if not any("MISSION COMPLETE" in e.text for e in new):     # also across a link drop
                    why = ex.get("return_reason")
                    new.append(Event(t, "ok", "MISSION COMPLETE" + (f" ({why})" if why else "")))
        self.events.extend(new)
        return new

    # --------------------------------------------------------------- readouts --
    def mission_time(self, t: Optional[float] = None) -> float:
        if self.t_start is None:
            return 0.0
        return max(0.0, (self.t_end if self.t_end is not None else (t if t is not None else self.t)) - self.t_start)

    def speed(self, window_s: float = 2.0) -> Optional[float]:
        if len(self.poses) < 2:
            return None
        t1 = self.poses[-1][0]
        pts = [p for p in self.poses if p[0] >= t1 - window_s]
        dt = pts[-1][0] - pts[0][0]
        if dt < 0.5:
            return None
        d = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(pts, pts[1:]))
        return d / dt

    def home_distance(self) -> Optional[float]:
        st = self.state or {}
        pose, sp = st.get("pose"), st.get("start_pose")
        if not pose or not sp:
            return None
        return math.hypot(float(pose["x_m"]) - float(sp[0]), float(pose["y_m"]) - float(sp[1]))

    def home_bearing_deg(self) -> Optional[float]:
        """Direction to the take-off point, arena frame (deg, CCW from +x)."""
        st = self.state or {}
        pose, sp = st.get("pose"), st.get("start_pose")
        if not pose or not sp:
            return None
        return math.degrees(math.atan2(float(sp[1]) - float(pose["y_m"]), float(sp[0]) - float(pose["x_m"])))


def fmt_clock(seconds: Optional[float]) -> str:
    if seconds is None:
        return "--:--"
    s = int(max(0, seconds))
    return f"{s // 3600}:{s % 3600 // 60:02d}:{s % 60:02d}" if s >= 3600 else f"{s // 60:02d}:{s % 60:02d}"
