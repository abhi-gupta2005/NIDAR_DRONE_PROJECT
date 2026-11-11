#!/usr/bin/env python3
"""
NIDAR V12.2 - Controlled LiDAR Tracker Validation GUI

This is a deterministic tracker-validation GUI. It deliberately creates:
STATIC, APPROACHING, SIDEWAYS and TEMP_LOSS objects so we can visually
validate V3 before camera/LiDAR fusion.

No flight-controller commands are sent.
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass

from lidar_obstacle_detector import DetectedObstacle
from lidar_obstacle_tracker_v3 import (
    LidarObstacleTrackerV3,
    TrackerV3Config,
)


@dataclass
class TestObject:
    name: str
    x: float
    y: float
    vx: float
    vy: float
    active: bool = True
    color: str = ""


class ValidationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NIDAR V12.2 — Controlled LiDAR Tracker Validation")
        self.root.geometry("1400x850")
        self.root.minsize(1150, 720)

        self.running = False
        self.scan_id = 0
        self.t = 0.0
        self.dt = 0.1

        self.objects = []
        self.tracker = None
        self.temp_loss_id = None
        self.recovery_seen = False
        self.max_speed_seen = 0.0

        self.build_ui()
        self.reset()

    def build_ui(self):
        header = ttk.Frame(self.root, padding=12)
        header.pack(fill="x")

        ttk.Label(
            header,
            text="NIDAR V12.2",
            font=("Arial", 21, "bold")
        ).pack(side="left")

        ttk.Label(
            header,
            text="  Controlled LiDAR Tracker V3 Validation",
            font=("Arial", 12)
        ).pack(side="left")

        self.status = ttk.Label(
            header,
            text="● READY",
            font=("Arial", 11, "bold")
        )
        self.status.pack(side="right")

        controls = ttk.Frame(self.root, padding=(12, 0, 12, 8))
        controls.pack(fill="x")

        ttk.Button(controls, text="Start", command=self.start).pack(side="left", padx=4)
        ttk.Button(controls, text="Pause", command=self.pause).pack(side="left", padx=4)
        ttk.Button(controls, text="Reset", command=self.reset).pack(side="left", padx=4)
        ttk.Button(controls, text="1 Scan", command=self.step).pack(side="left", padx=4)

        self.event_label = ttk.Label(
            controls,
            text="TEST READY",
            font=("Arial", 10, "bold")
        )
        self.event_label.pack(side="right", padx=10)

        body = ttk.Frame(self.root, padding=12)
        body.pack(fill="both", expand=True)

        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)

        self.canvas = tk.Canvas(
            left,
            bg="#08111f",
            highlightthickness=0
        )
        self.canvas.pack(fill="both", expand=True)

        right = ttk.Frame(body, width=455)
        right.pack(side="right", fill="y", padx=(12, 0))
        right.pack_propagate(False)

        ttk.Label(
            right,
            text="VALIDATION SCENARIO",
            font=("Arial", 12, "bold")
        ).pack(anchor="w")

        scenario = (
            "T01  STATIC       0.00 m/s\n"
            "T02  APPROACHING -0.45 m/s X\n"
            "T03  SIDEWAYS    -0.40 m/s Y\n"
            "T04  TEMP_LOSS   hidden scans 15–18\n"
            "\n"
            "Expected recovery:\n"
            "same ID + RECOVERED at scan 18"
        )
        ttk.Label(
            right,
            text=scenario,
            font=("Courier New", 9),
            justify="left"
        ).pack(anchor="w", pady=(5, 12))

        self.stats = tk.Text(
            right,
            height=13,
            font=("Courier New", 9),
            state="disabled"
        )
        self.stats.pack(fill="x")

        ttk.Label(
            right,
            text="TRACK OUTPUT",
            font=("Arial", 12, "bold")
        ).pack(anchor="w", pady=(12, 5))

        cols = ("id", "state", "obs", "motion", "x", "y", "speed", "miss")
        self.tree = ttk.Treeview(
            right,
            columns=cols,
            show="headings",
            height=17
        )

        heads = {
            "id": "ID", "state": "State", "obs": "Obs",
            "motion": "Motion", "x": "X", "y": "Y",
            "speed": "Speed", "miss": "Miss"
        }
        widths = {
            "id": 42, "state": 82, "obs": 78, "motion": 85,
            "x": 48, "y": 48, "speed": 60, "miss": 45
        }

        for c in cols:
            self.tree.heading(c, text=heads[c])
            self.tree.column(c, width=widths[c], anchor="center")

        self.tree.pack(fill="both", expand=True)

        ttk.Label(
            right,
            text="Tracker only — FC disconnected / no flight commands",
            font=("Arial", 9)
        ).pack(anchor="w", pady=8)

    def reset(self):
        self.running = False
        self.scan_id = 0
        self.t = 0.0
        self.temp_loss_id = None
        self.recovery_seen = False
        self.max_speed_seen = 0.0

        self.objects = [
            TestObject("STATIC", 2.5, 0.8, 0.0, 0.0),
            TestObject("APPROACHING", 4.0, -0.8, -0.45, 0.0),
            TestObject("SIDEWAYS", 2.0, 3.0, 0.0, -0.40),
            TestObject("TEMP_LOSS", -2.5, 1.0, 0.0, 0.0),
        ]

        self.tracker = LidarObstacleTrackerV3(
            TrackerV3Config(
                association_distance_m=0.55,
                recovery_distance_m=0.75,
                max_missed_scans=3,
                confirmation_hits=2,
                position_alpha=0.55,
                velocity_alpha=0.30,
                velocity_deadband_mps=0.08,
                max_speed_mps=5.0,
                max_acceleration_mps2=3.0,
            )
        )

        self.status.config(text="● READY")
        self.event_label.config(text="TEST READY")
        self.draw([])
        self.update_panel([])

    def start(self):
        if not self.running:
            self.running = True
            self.status.config(text="● RUNNING")
            self.tick()

    def pause(self):
        self.running = False
        self.status.config(text="● PAUSED")

    def step(self):
        self.process()

    def tick(self):
        if not self.running:
            return
        self.process()
        self.root.after(100, self.tick)

    def process(self):
        # TEMP_LOSS disappears on scans 15,16,17,18.
        self.objects[3].active = not (15 <= self.scan_id <= 18)

        if self.scan_id == 15:
            self.event_label.config(text="EVENT: TEMP_LOSS HIDDEN")
        elif self.scan_id == 18:
            self.event_label.config(text="EVENT: TEMP_LOSS RESTORED")
        elif self.scan_id < 15:
            self.event_label.config(text="EVENT: NORMAL TRACKING")
        elif self.scan_id > 18:
            self.event_label.config(
                text="EVENT: RECOVERY VALIDATION"
            )

        detections = []
        for obj in self.objects:
            if not obj.active:
                continue

            obj.x += obj.vx * self.dt
            obj.y += obj.vy * self.dt

            d = math.hypot(obj.x, obj.y)
            b = math.degrees(math.atan2(obj.y, obj.x)) % 360.0

            detections.append(
                DetectedObstacle(
                    obstacle_id=self.scan_id * 100 + len(detections),
                    center_x_m=obj.x,
                    center_y_m=obj.y,
                    distance_m=d,
                    bearing_deg=b,
                    width_m=0.30,
                    depth_m=0.30,
                    point_count=20,
                    confidence=0.90,
                    start_angle_deg=(b - 2) % 360,
                    end_angle_deg=(b + 2) % 360,
                )
            )

        tracks = self.tracker.update(
            detections,
            timestamp=self.t
        )

        # Capture original TEMP_LOSS ID.
        if self.scan_id < 15:
            for tr in tracks:
                if math.hypot(tr.x_m + 2.5, tr.y_m - 1.0) < 0.8:
                    self.temp_loss_id = tr.track_id

        if self.scan_id == 18 and self.temp_loss_id is not None:
            for tr in tracks:
                if (
                    tr.track_id == self.temp_loss_id
                    and tr.observation_state == "RECOVERED"
                ):
                    self.recovery_seen = True

        self.max_speed_seen = max(
            [self.max_speed_seen] +
            [tr.speed_mps for tr in tracks]
        )

        self.draw(tracks)
        self.update_panel(tracks)

        self.scan_id += 1
        self.t += self.dt

    def xy(self, x, y, cx, cy, scale):
        return cx + x * scale, cy - y * scale

    def draw(self, tracks):
        self.canvas.delete("all")

        w = max(self.canvas.winfo_width(), 650)
        h = max(self.canvas.winfo_height(), 600)
        cx, cy = w * 0.50, h * 0.52
        scale = min(w, h) * 0.038

        for r_m in (1, 2, 4, 6, 8, 10, 12):
            r = r_m * scale
            self.canvas.create_oval(
                cx-r, cy-r, cx+r, cy+r,
                outline="#253858"
            )
            self.canvas.create_text(
                cx+r+3, cy,
                text=f"{r_m}m",
                fill="#7182a8",
                anchor="w",
                font=("Arial", 8)
            )

        self.canvas.create_line(
            cx-12*scale, cy, cx+12*scale, cy,
            fill="#304363"
        )
        self.canvas.create_line(
            cx, cy-12*scale, cx, cy+12*scale,
            fill="#304363"
        )

        # Drone.
        self.canvas.create_oval(
            cx-8, cy-8, cx+8, cy+8,
            outline="white", width=2
        )
        self.canvas.create_text(
            cx, cy+20,
            text="DRONE / LiDAR",
            fill="white",
            font=("Arial", 9, "bold")
        )

        # Track visualization.
        for tr in tracks:
            x, y = self.xy(tr.x_m, tr.y_m, cx, cy, scale)

            if tr.state == "TEMP_LOST":
                outline = "#f59e0b"
                dash = (6, 4)
            elif tr.motion_state == "MOVING":
                outline = "#ff4d4d"
                dash = None
            else:
                outline = "#35d07f"
                dash = None

            radius = 9 + 10 * tr.track_quality

            self.canvas.create_oval(
                x-radius, y-radius,
                x+radius, y+radius,
                outline=outline,
                width=3,
                dash=dash
            )

            label = f"T{tr.track_id:02d}"
            self.canvas.create_text(
                x, y-radius-12,
                text=label,
                fill="white",
                font=("Arial", 10, "bold")
            )

            self.canvas.create_text(
                x, y+radius+11,
                text=f"{tr.speed_mps:.2f} m/s",
                fill="#b7c4dc",
                font=("Arial", 8)
            )

            if tr.motion_state == "MOVING":
                self.canvas.create_line(
                    x, y,
                    x + tr.vx_mps*scale*0.7,
                    y - tr.vy_mps*scale*0.7,
                    fill="#ff4d4d",
                    width=3,
                    arrow=tk.LAST
                )

        self.canvas.create_text(
            18, 18,
            text=f"SCAN {self.scan_id:02d}   TIME {self.t:4.1f}s",
            fill="white",
            anchor="nw",
            font=("Courier New", 12, "bold")
        )

        if 15 <= self.scan_id <= 18:
            self.canvas.create_text(
                18, 48,
                text="⚠ TEMP_LOSS OBJECT INTENTIONALLY HIDDEN",
                fill="#f59e0b",
                anchor="nw",
                font=("Arial", 10, "bold")
            )

        if self.scan_id >= 18 and self.recovery_seen:
            self.canvas.create_text(
                18, 48,
                text=f"✓ RECOVERED WITH SAME ID T{self.temp_loss_id:02d}",
                fill="#35d07f",
                anchor="nw",
                font=("Arial", 10, "bold")
            )

    def update_panel(self, tracks):
        confirmed = sum(t.state == "CONFIRMED" for t in tracks)
        moving = sum(t.motion_state == "MOVING" for t in tracks)
        stationary = sum(t.motion_state == "STATIONARY" for t in tracks)
        predicted = sum(t.observation_state == "PREDICTED" for t in tracks)
        recovered = sum(t.observation_state == "RECOVERED" for t in tracks)

        self.stats.config(state="normal")
        self.stats.delete("1.0", "end")
        self.stats.insert(
            "end",
            f"SCAN                 : {self.scan_id:02d}\n"
            f"TRACKS               : {len(tracks)}\n"
            f"CONFIRMED            : {confirmed}\n"
            f"MOVING               : {moving}\n"
            f"STATIONARY           : {stationary}\n"
            f"PREDICTED            : {predicted}\n"
            f"RECOVERED            : {recovered}\n"
            f"ORIGINAL TEMP ID     : "
            f"{('T'+str(self.temp_loss_id).zfill(2)) if self.temp_loss_id else '—'}\n"
            f"RECOVERY VALIDATED    : "
            f"{'YES' if self.recovery_seen else 'NO'}\n"
            f"MAX SPEED OBSERVED   : {self.max_speed_seen:.2f} m/s\n"
            f"\n"
            f"Association gate      : 0.55 m\n"
            f"Recovery gate        : 0.75 m\n"
            f"Velocity deadband    : 0.08 m/s\n"
            f"Max missed scans     : 3\n"
            f"Scan rate            : 10 Hz\n"
        )
        self.stats.config(state="disabled")

        for item in self.tree.get_children():
            self.tree.delete(item)

        for tr in tracks:
            self.tree.insert(
                "",
                "end",
                values=(
                    f"T{tr.track_id:02d}",
                    tr.state,
                    tr.observation_state,
                    tr.motion_state,
                    f"{tr.x_m:+.2f}",
                    f"{tr.y_m:+.2f}",
                    f"{tr.speed_mps:.2f}",
                    tr.missed_scans
                )
            )


if __name__ == "__main__":
    root = tk.Tk()
    app = ValidationGUI(root)
    root.mainloop()
