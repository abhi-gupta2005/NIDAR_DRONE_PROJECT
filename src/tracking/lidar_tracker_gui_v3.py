#!/usr/bin/env python3
"""
NIDAR V12.2 - LiDAR Tracking GUI V3

Visual demonstration of the current LiDAR pipeline:
STL-19P simulator -> obstacle detection -> persistent tracking V3.

The GUI is a development/validation dashboard. It does not control
the flight controller.
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass

from lidar_interface import LidarInterfaceConfig, SimulatedSTL19PBackend
from lidar_obstacle_detector import LidarObstacleDetector, ObstacleDetectorConfig
from lidar_obstacle_tracker_v3 import LidarObstacleTrackerV3, TrackerV3Config


@dataclass
class DemoObject:
    name: str
    x: float
    y: float
    vx: float
    vy: float
    active: bool = True


class LidarTrackingGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("NIDAR V12.2 — LiDAR Tracking & Tracking Quality")
        self.root.geometry("1280x820")
        self.root.minsize(1050, 700)

        self.running = False
        self.scan_id = 0
        self.sim_time = 0.0
        self.dt = 0.1

        self.backend = SimulatedSTL19PBackend(
            LidarInterfaceConfig(
                scan_rate_hz=10.0,
                measurements_per_second=5000,
                max_range_m=12.0,
                min_range_m=0.05,
                noise_scale=1.0,
            )
        )

        self.detector = LidarObstacleDetector(
            ObstacleDetectorConfig(
                min_range_m=0.05,
                max_range_m=12.0,
                cluster_distance_m=0.18,
                min_cluster_points=3,
            )
        )

        self.tracker = LidarObstacleTrackerV3(
            TrackerV3Config()
        )

        self.build_ui()
        self.draw_scene()
        self.update_stats()

    def build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(
            top,
            text="NIDAR V12.2",
            font=("Arial", 20, "bold"),
        ).pack(side="left")

        ttk.Label(
            top,
            text="  STL-19P LiDAR → Detection → Persistent Tracker V3",
            font=("Arial", 12),
        ).pack(side="left")

        self.status = ttk.Label(
            top,
            text="● STOPPED",
            font=("Arial", 11, "bold"),
        )
        self.status.pack(side="right")

        controls = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        controls.pack(fill="x")

        ttk.Button(
            controls,
            text="Start",
            command=self.start,
        ).pack(side="left", padx=4)

        ttk.Button(
            controls,
            text="Pause",
            command=self.pause,
        ).pack(side="left", padx=4)

        ttk.Button(
            controls,
            text="Reset",
            command=self.reset,
        ).pack(side="left", padx=4)

        ttk.Button(
            controls,
            text="Run 1 Scan",
            command=self.one_scan,
        ).pack(side="left", padx=4)

        self.mode_var = tk.StringVar(value="LIVE SIMULATOR")
        ttk.Label(
            controls,
            textvariable=self.mode_var,
            font=("Arial", 10, "bold"),
        ).pack(side="right", padx=8)

        body = ttk.Frame(self.root, padding=10)
        body.pack(fill="both", expand=True)

        # Left: polar/radar-style scene.
        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)

        self.canvas = tk.Canvas(
            left,
            bg="#0b1020",
            highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True)

        # Right: stats + architecture.
        right = ttk.Frame(body, width=410)
        right.pack(side="right", fill="y", padx=(10, 0))
        right.pack_propagate(False)

        ttk.Label(
            right,
            text="PIPELINE",
            font=("Arial", 12, "bold"),
        ).pack(anchor="w", pady=(0, 5))

        pipeline = (
            "STL-19P SIMULATOR\n"
            "      ↓\n"
            "360° / 10 Hz / 5000 measurements/s\n"
            "      ↓\n"
            "Obstacle Detector\n"
            "      ↓\n"
            "Persistent Tracker V3\n"
            "      ↓\n"
            "ID + Position + Velocity\n"
            "      ↓\n"
            "MOVING / STATIONARY\n"
            "      ↓\n"
            "MEASURED / PREDICTED\n"
            "      ↓\n"
            "Track Quality"
        )

        ttk.Label(
            right,
            text=pipeline,
            font=("Courier New", 9),
            justify="left",
        ).pack(anchor="w", pady=(0, 12))

        self.stats_text = tk.Text(
            right,
            height=10,
            width=48,
            font=("Courier New", 9),
            state="disabled",
        )
        self.stats_text.pack(fill="x", pady=5)

        ttk.Label(
            right,
            text="TRACKS",
            font=("Arial", 12, "bold"),
        ).pack(anchor="w", pady=(10, 5))

        columns = (
            "id", "state", "obs", "motion",
            "dist", "speed", "quality"
        )

        self.tree = ttk.Treeview(
            right,
            columns=columns,
            show="headings",
            height=15,
        )

        headings = {
            "id": "ID",
            "state": "State",
            "obs": "Obs",
            "motion": "Motion",
            "dist": "Dist",
            "speed": "Speed",
            "quality": "Quality",
        }

        widths = {
            "id": 40,
            "state": 85,
            "obs": 85,
            "motion": 85,
            "dist": 55,
            "speed": 60,
            "quality": 65,
        }

        for col in columns:
            self.tree.heading(
                col,
                text=headings[col],
            )
            self.tree.column(
                col,
                width=widths[col],
                anchor="center",
            )

        self.tree.pack(fill="both", expand=True)

        ttk.Label(
            right,
            text=(
                "Development dashboard only\n"
                "No flight-controller commands are sent."
            ),
            font=("Arial", 9),
        ).pack(anchor="w", pady=8)

    def start(self):
        if not self.running:
            self.running = True
            self.status.config(text="● RUNNING")
            self.tick()

    def pause(self):
        self.running = False
        self.status.config(text="● PAUSED")

    def reset(self):
        self.running = False
        self.status.config(text="● STOPPED")
        self.scan_id = 0
        self.sim_time = 0.0
        self.tracker = LidarObstacleTrackerV3(
            TrackerV3Config()
        )
        self.draw_scene([])
        self.update_stats([])

    def one_scan(self):
        self.process_scan()

    def tick(self):
        if not self.running:
            return

        self.process_scan()
        self.root.after(100, self.tick)

    def process_scan(self):
        scan = self.backend.next_scan()

        detections = self.detector.detect(scan)

        tracks = self.tracker.update(
            detections,
            timestamp=scan.timestamp,
        )

        self.scan_id = scan.scan_id
        self.sim_time = scan.timestamp

        self.draw_scene(tracks)
        self.update_stats(tracks)

    def world_to_canvas(
        self,
        x,
        y,
        cx,
        cy,
        scale,
    ):
        return (
            cx + x * scale,
            cy - y * scale,
        )

    def draw_scene(self, tracks=None):
        if tracks is None:
            tracks = []

        self.canvas.delete("all")

        w = max(
            self.canvas.winfo_width(),
            600,
        )
        h = max(
            self.canvas.winfo_height(),
            500,
        )

        cx = w * 0.50
        cy = h * 0.53
        scale = min(w, h) * 0.037

        # Range rings.
        for radius in (2, 4, 6, 8, 10, 12):
            r = radius * scale
            self.canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                outline="#263453",
            )

            self.canvas.create_text(
                cx + r + 4,
                cy,
                text=f"{radius}m",
                fill="#7182a8",
                anchor="w",
                font=("Arial", 8),
            )

        # Axes.
        self.canvas.create_line(
            cx - 12 * scale,
            cy,
            cx + 12 * scale,
            cy,
            fill="#314263",
        )

        self.canvas.create_line(
            cx,
            cy - 12 * scale,
            cx,
            cy + 12 * scale,
            fill="#314263",
        )

        # Heading indicator.
        self.canvas.create_line(
            cx,
            cy,
            cx + 1.2 * scale,
            cy,
            fill="#d8dee9",
            width=3,
        )

        self.canvas.create_text(
            cx + 1.4 * scale,
            cy,
            text="FRONT",
            fill="#d8dee9",
            anchor="w",
            font=("Arial", 9, "bold"),
        )

        # Drone marker.
        self.canvas.create_oval(
            cx - 7,
            cy - 7,
            cx + 7,
            cy + 7,
            outline="#ffffff",
            width=2,
        )

        self.canvas.create_text(
            cx,
            cy + 18,
            text="DRONE / LiDAR",
            fill="#ffffff",
            font=("Arial", 9, "bold"),
        )

        # Tracks.
        for track in tracks:
            x, y = self.world_to_canvas(
                track.x_m,
                track.y_m,
                cx,
                cy,
                scale,
            )

            # Track marker size by quality.
            size = 7 + 8 * track.track_quality

            if track.state == "TEMP_LOST":
                outline = "#f59e0b"
                dash = (5, 3)
            elif track.motion_state == "MOVING":
                outline = "#ef4444"
                dash = None
            else:
                outline = "#22c55e"
                dash = None

            self.canvas.create_oval(
                x - size,
                y - size,
                x + size,
                y + size,
                outline=outline,
                width=2,
                dash=dash,
            )

            self.canvas.create_text(
                x,
                y - size - 10,
                text=f"T{track.track_id:02d}",
                fill="#ffffff",
                font=("Arial", 9, "bold"),
            )

            label = (
                f"{track.distance_m:.2f}m  "
                f"{track.speed_mps:.2f}m/s"
            )

            self.canvas.create_text(
                x,
                y + size + 9,
                text=label,
                fill="#aebbd4",
                font=("Arial", 8),
            )

            # Velocity vector.
            if track.motion_state == "MOVING":
                vx = track.vx_mps * scale * 0.7
                vy = -track.vy_mps * scale * 0.7

                self.canvas.create_line(
                    x,
                    y,
                    x + vx,
                    y + vy,
                    fill="#ef4444",
                    width=2,
                    arrow=tk.LAST,
                )

        self.canvas.create_text(
            18,
            18,
            text=f"SCAN {self.scan_id:04d}   |   "
                 f"TIME {self.sim_time:5.1f}s",
            fill="#d8dee9",
            anchor="nw",
            font=("Courier New", 11, "bold"),
        )

    def update_stats(self, tracks=None):
        if tracks is None:
            tracks = []

        confirmed = sum(
            t.state == "CONFIRMED"
            for t in tracks
        )

        predicted = sum(
            t.observation_state == "PREDICTED"
            for t in tracks
        )

        moving = sum(
            t.motion_state == "MOVING"
            for t in tracks
        )

        stationary = sum(
            t.motion_state == "STATIONARY"
            for t in tracks
        )

        self.stats_text.config(state="normal")
        self.stats_text.delete("1.0", "end")

        self.stats_text.insert(
            "end",
            f"SCAN              : {self.scan_id:04d}\n"
            f"TRACKS            : {len(tracks)}\n"
            f"CONFIRMED         : {confirmed}\n"
            f"MOVING            : {moving}\n"
            f"STATIONARY        : {stationary}\n"
            f"PREDICTED         : {predicted}\n"
            f"ASSOCIATION GATE  : 0.55 m\n"
            f"RECOVERY GATE     : 0.75 m\n"
            f"VEL. DEADBAND     : 0.08 m/s\n"
            f"SCAN RATE         : 10 Hz\n"
        )

        self.stats_text.config(
            state="disabled"
        )

        for item in self.tree.get_children():
            self.tree.delete(item)

        for track in tracks:
            self.tree.insert(
                "",
                "end",
                values=(
                    f"T{track.track_id:02d}",
                    track.state,
                    track.observation_state,
                    track.motion_state,
                    f"{track.distance_m:.2f}",
                    f"{track.speed_mps:.2f}",
                    f"{track.track_quality:.2f}",
                ),
            )


if __name__ == "__main__":
    root = tk.Tk()
    app = LidarTrackingGUI(root)
    root.mainloop()
