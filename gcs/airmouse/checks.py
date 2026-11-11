"""Pre-flight checklist: every link and sensor a mission needs, judged from live data (pure; tested).

A check is REQUIRED (a failure blocks "READY") or OPTIONAL (a failure is shown, not blocking).
Thresholds follow the rest of the project: data older than 1 s is stale (mission server STALE_S),
a SLAM match score under 0.30 is not trusted (mission server MIN_POSE_SCORE).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

MIN_POSE_SCORE = 0.30
GROUPS = ("Links", "Sensors", "Navigation", "Autonomy", "Flight controller", "Safety")


@dataclass
class Check:
    group: str
    name: str
    status: str           # pass | warn | fail | na
    detail: str
    required: bool
    hint: str = ""


def _age(v) -> Optional[float]:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


def _grade(age: Optional[float], ok: float, warn: float) -> str:
    if age is None:
        return "fail"
    return "pass" if age <= ok else ("warn" if age <= warn else "fail")


def _s(age: Optional[float]) -> str:
    return "no data" if age is None else f"age {age:.2f} s"


def detect_mode(px4: Optional[dict]) -> str:
    if os.environ.get("NIDAR_BRAIN_MODE", "").strip().lower() == "route" or px4:
        return "px4"
    return "rig"


def run_checks(state: Optional[dict], link_up: bool, link_age: Optional[float], endpoint: str = "",
               ros_on: bool = False, image_age: Optional[float] = None, px4: Optional[Dict[str, object]] = None,
               mode: str = "auto") -> List[Check]:
    px4 = px4 or {}
    mode = detect_mode(px4) if mode == "auto" else mode
    st = (state or {}) if link_up else {}
    h = st.get("health") or {}
    out: List[Check] = []

    # Links
    la = link_age if link_up else None
    out.append(Check("Links", "Pi mission server", _grade(la, 1.5, 4.0),
                     f"{endpoint or 'not configured'} · " + ("last message " + f"{la:.1f} s" if la is not None
                                                              else "not connected"),
                     True, "Start the Pi stack (tools/nidar_run.sh) and check the Pi address."))
    out.append(Check("Links", "ROS 2 on this laptop", "pass" if ros_on else "warn",
                     "connected" if ros_on else "off — no camera video, no PX4 telemetry here", False,
                     "Run the app from a WSL terminal with ROS 2 sourced."))

    # Sensors
    li = h.get("lidar") or {}
    a = _age(li.get("age_s"))
    out.append(Check("Sensors", "LiDAR scans reaching the Pi", _grade(a, 0.5, 2.0),
                     f"{_s(a)} · {li.get('points') or 0} points", True,
                     "Check the LiDAR relay (WSL) or the LiDAR on the drone."))
    pc = h.get("perception") or {}
    a = _age(pc.get("age_s"))
    out.append(Check("Sensors", "Camera detections reaching the Pi", _grade(a, 1.0, 3.0),
                     _s(a) + (f" · frame {pc.get('frame')}" if pc.get("frame") is not None else ""), True,
                     "Start YOLO on Windows (tools/nidar_yolo.ps1) and check NIDAR_PI_HOST."))
    out.append(Check("Sensors", "Camera video on this laptop", _grade(image_age, 1.0, 3.0) if ros_on else "na",
                     _s(image_age) if ros_on else "needs ROS 2", False,
                     "Start the camera relay or the Gazebo camera bridge."))
    v = h.get("v18") or {}
    a = _age(v.get("age_s"))
    out.append(Check("Sensors", "V18 camera + LiDAR fusion", _grade(a, 1.0, 3.0),
                     f"{_s(a)} · {v.get('fused') or 0} fused", True, "Check logs/v18.log on the Pi."))

    # Navigation
    sl = h.get("slam") or {}
    a = _age(sl.get("age_s"))
    s = sl.get("status") or "NO DATA"
    trk = "pass" if s == "TRACKING" and a is not None and a <= 1.0 else ("warn" if s == "TRACKING" and a is not None
                                                                         and a <= 3.0 else "fail")
    out.append(Check("Navigation", "SLAM tracking", trk, f"{s} · {_s(a)}", True,
                     "SLAM needs fresh LiDAR scans; restart the run if it stays LOST (no recovery yet)."))
    fps = _age(sl.get("fps"))
    out.append(Check("Navigation", "SLAM update rate", "fail" if fps is None else
                     ("pass" if fps >= 4.0 else ("warn" if fps >= 2.0 else "fail")),
                     "no data" if fps is None else f"{fps:.1f} Hz (needs 4 Hz)", True,
                     "A slow rate usually means the Pi CPU is overloaded."))
    sc = _age(sl.get("score"))
    out.append(Check("Navigation", "SLAM match quality", "fail" if sc is None else
                     ("pass" if sc >= 0.5 else ("warn" if sc >= MIN_POSE_SCORE else "fail")),
                     "no data" if sc is None else f"score {sc:.2f} (trusted from {MIN_POSE_SCORE:.2f})", True,
                     "A low score means the scan does not match the map."))
    pose = st.get("pose") or {}
    cell = pose.get("grid")
    out.append(Check("Navigation", "Drone on the take-off pad", "na" if not pose else
                     ("pass" if cell == "PAD" else "warn"),
                     "no position yet" if not pose else ("on the pad" if cell == "PAD" else f"at {cell}"), False,
                     "Start every mission from the pad so the map lines up with the arena grid."))
    grid = st.get("grid") or {}
    out.append(Check("Navigation", "Arena layout loaded", "na" if not grid else
                     ("pass" if grid.get("bounded", True) else "warn"),
                     "no data" if not grid else (f"{grid.get('cells')} × {grid.get('cells')} grid"
                                                  if grid.get("bounded", True) else "unknown arena: no grid cells"),
                     False, "Set NIDAR_ARENA_LAYOUT on the Pi (the launcher does this)."))

    # Autonomy
    ex = st.get("explorer")
    if mode == "px4":
        out.append(Check("Autonomy", "Explorer brain", "na", "PX4 route mode: PX4 flies a fixed route", False))
    else:
        out.append(Check("Autonomy", "Explorer brain running", "pass" if ex else "fail",
                         f"state {ex.get('state')}" if ex else "no brain state from the Pi", True,
                         "Start nidar_explorer_service.py on the Pi (the launcher does this)."))
        if ex:
            es = ex.get("state")
            out.append(Check("Autonomy", "Brain ready", "pass" if es in ("EXPLORE", "RETURN") else "warn",
                             {"HOLD": f"HOLD — {ex.get('hold_reason') or 'waiting'}",
                              "COMPLETE": "mission already complete — restart the brain for a new one"}.get(
                                 es, str(es)), False))

    # Flight controller
    if not px4:
        out.append(Check("Flight controller", "PX4 telemetry", "fail" if mode == "px4" else "na",
                         "not received" if mode == "px4" else "rig mode: no PX4", mode == "px4",
                         "Start PX4 SITL and the MicroXRCE agent (UDP 8888)."))
    else:
        out.append(Check("Flight controller", "PX4 telemetry", "pass", ", ".join(sorted(px4)), mode == "px4"))
        vs = px4.get("vehicle_status")
        if vs is not None:
            armed = getattr(vs, "arming_state", None) == 2
            out.append(Check("Flight controller", "PX4 disarmed before start", "warn" if armed else "pass",
                             "ARMED" if armed else "disarmed", False))
        bs = px4.get("battery_status")
        rem = getattr(bs, "remaining", None) if bs is not None else None
        if rem is not None and rem >= 0:
            out.append(Check("Flight controller", "Battery", "pass" if rem > 0.5 else ("warn" if rem > 0.3 else "fail"),
                             f"{rem * 100:.0f} %", True))

    # Safety
    out.append(Check("Safety", "Ground station is watch-only", "pass",
                     "this app sends no commands · stop = Ctrl+C in the launcher terminal", False))
    return out


def verdict(checks: List[Check]) -> Tuple[str, str]:
    """(level, text): level ready | warn | blocked."""
    fails = [c for c in checks if c.required and c.status == "fail"]
    warns = [c for c in checks if c.status in ("warn", "fail") and c not in fails]
    if fails:
        return "blocked", f"NOT READY — {len(fails)} required check{'s' if len(fails) > 1 else ''} failing"
    if warns:
        return "warn", f"READY WITH {len(warns)} WARNING{'S' if len(warns) > 1 else ''}"
    return "ready", "READY FOR MISSION"
