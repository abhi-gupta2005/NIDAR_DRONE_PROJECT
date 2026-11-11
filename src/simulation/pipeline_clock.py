#!/usr/bin/env python3
"""
Controllable clock injected into production modules in place of `time`.

Modules such as V18, nidar_bridge and px4_setpoint_sim call time.time() /
time.monotonic() through their module-level `time` reference.  Replacing
that reference with a SimClock makes every timeout deterministic and lets a
20 s scenario run in well under a second.
"""

from __future__ import annotations


class SimClock:
    def __init__(self, start: float = 1_700_000_000.0):
        self.now = float(start)

    def time(self) -> float:
        return self.now

    def monotonic(self) -> float:
        return self.now

    def perf_counter(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, float(seconds))

    def advance(self, dt: float) -> None:
        self.now += float(dt)
