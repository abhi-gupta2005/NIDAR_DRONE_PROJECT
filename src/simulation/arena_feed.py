#!/usr/bin/env python3
"""
Feed the Pi pipeline from the generated arena without Gazebo (2026-11-02).

Writes, at fixed rates, exactly the two files the Pi processes read:
  * src/communication/nidar_latest_perception.json  (camera packet, as written
    by nidar_pi_receiver.py) -- survivors projected with a PINHOLE camera
    (what Gazebo renders), 640 px wide, 70 deg HFOV;
  * src/communication/nidar_latest_scan.json  (LiDAR, as written by
    nidar_lidar_receiver.py) -- ray-cast from gazebo/layouts/<world>.json.

Used by tests/test_gazebo_pi_pipeline.py to run the REAL V18 process with
NIDAR_LIDAR_BACKEND=GAZEBO. It is a stand-in for Gazebo + relays, not a
replacement: no rendering, no detector -- every visible survivor is
"detected" with confidence 0.8.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from src.simulation import arena_raycast as ar
from src.communication.nidar_lidar_receiver import store_scan

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_HFOV_DEG = 70.0
CAMERA_BELOW_LIDAR_M = 0.18          # x500_nidar_airmouse / nidar_rig geometry (2026-11-02d)


def visible_survivors(arena: ar.Arena, x: float, y: float, yaw: float,
                      hfov_deg: float = CAMERA_HFOV_DEG, max_range: float = 10.0) -> List[Tuple[dict, float, float]]:
    """(survivor, range, bearing_deg right-positive) for survivors in the FOV with line of sight."""
    out = []
    walls_only = ar.Arena(arena.segments, arena.wall_height_m, [], arena.layout)
    for s in arena.survivors:
        rng, brg = ar.bearing_to(x, y, yaw, s["x"], s["y"])
        if rng > max_range or abs(brg) > hfov_deg / 2.0 - 1.0:
            continue
        # line of sight against walls only (camera sees over the LiDAR plane issue)
        body_ccw = math.radians(-brg)
        r, _ = ar.cast(walls_only, x, y, yaw + body_ccw, height_m=1.0, samples=1, angle_min=0.0)
        if r[0] < rng - 0.3:
            continue
        out.append((s, rng, brg))
    return out


def project_bbox(rng: float, brg_deg: float, posture: str, camera_height_m: float,
                 width: int = CAMERA_WIDTH, height: int = CAMERA_HEIGHT, hfov_deg: float = CAMERA_HFOV_DEG):
    """Pinhole box of a survivor body (cylinder from arena_raycast.SURVIVOR_SHAPES) seen by a level camera."""
    f_px = (width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
    shape = ar.SURVIVOR_SHAPES.get(posture, ar.SURVIVOR_SHAPES["standing"])
    rad, top = shape["radius_m"], shape["height_m"]
    b = math.radians(brg_deg)
    fwd, lat = rng * math.cos(b), rng * math.sin(b)
    near = max(fwd - rad, 0.15)
    u = width / 2.0 + f_px * lat / fwd
    half_w = f_px * rad / near
    v_top = height / 2.0 - f_px * (top - camera_height_m) / (near if top > camera_height_m else fwd + rad)
    v_bot = height / 2.0 + f_px * camera_height_m / near            # feet / near edge on the floor
    box = [max(0.0, u - half_w), max(0.0, v_top), min(width - 1.0, u + half_w), min(height - 1.0, v_bot)]
    return [round(v, 2) for v in box], [round(u, 2), round((box[1] + box[3]) / 2.0, 2)]


def camera_packet(frame: int, detections: List[Tuple[dict, float, float]],
                  width: int = CAMERA_WIDTH, height: int = CAMERA_HEIGHT,
                  hfov_deg: float = CAMERA_HFOV_DEG, source: str = "gazebo_sim",
                  camera_height_m: float = 1.0 - CAMERA_BELOW_LIDAR_M) -> Dict:
    objects = []
    now = time.time()
    for k, (s, rng, brg) in enumerate(detections):
        bbox, center = project_bbox(rng, brg, s.get("posture", "standing"), camera_height_m,
                                    width, height, hfov_deg)
        objects.append({                                   # same fields as nidar_v11_windows.object_record()
            "frame": frame, "timestamp": now, "id": k + 1, "class": "person",
            "state": "CONFIRMED", "confidence": 0.8, "tracker_id": k + 1,
            "bbox": bbox, "center": center, "predicted_bbox": bbox, "predicted_center": center,
            "normalized_center": [round(center[0] / width, 4), round(center[1] / height, 4)],
            "bbox_size": [round(bbox[2] - bbox[0], 2), round(bbox[3] - bbox[1], 2)],
            "velocity_px_frame": [0.0, 0.0], "motion_confidence": 1.0, "track_stability_score": 90.0,
            "observation_age_frames": 0, "prediction_horizon_frames": 0, "prediction_budget_frames": 0,
            "prediction_reliability": "MEASURED", "spatial_status": "OBSERVED", "confirmed": True,
            "age_frames": frame, "missed_frames": 0, "max_missed_frames": 0, "recoveries": 0,
            "tracker_switches": 0, "first_frame": 1, "last_seen_frame": frame, "state_since_frame": 1,
            "current_observation": True,
            "survivor": s["id"],                           # ground truth tag (ignored by the Pi)
        })
    return {"protocol": "NIDAR_PERCEPTION_V1", "frame": frame, "timestamp": now,
            "camera": {"width": width, "height": height, "source": source}, "objects": objects}


def write_json_atomic(path: Path, data: Dict) -> None:
    tmp = path.with_suffix(".tmp_feed")
    tmp.write_text(json.dumps(data), encoding="utf-8")
    tmp.replace(path)


class ArenaFeeder:
    def __init__(self, arena: ar.Arena, project_dir: Path, lidar_height_m: float = 1.0):
        self.arena = arena
        self.camera_file = Path(project_dir) / "src" / "communication" / "nidar_latest_perception.json"
        self.scan_file = Path(project_dir) / "src" / "communication" / "nidar_latest_scan.json"
        self.camera_file.parent.mkdir(parents=True, exist_ok=True)
        self.lidar_height_m = lidar_height_m
        self.frame = 0
        self.seq = 0

    def step(self, pose: Tuple[float, float, float], camera: bool = True, lidar: bool = True) -> List:
        x, y, yaw = pose
        dets = []
        if camera:
            self.frame += 1
            dets = visible_survivors(self.arena, x, y, yaw)
            write_json_atomic(self.camera_file, camera_packet(
                self.frame, dets, camera_height_m=self.lidar_height_m - CAMERA_BELOW_LIDAR_M))
        if lidar:
            self.seq += 1
            r, _ = ar.cast(self.arena, x, y, yaw, height_m=self.lidar_height_m)
            store_scan(ar.laserscan_message(r, seq=self.seq, sent=time.time()), self.scan_file)
        return dets
