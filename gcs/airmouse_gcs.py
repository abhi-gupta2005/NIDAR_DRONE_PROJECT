#!/usr/bin/env python3
"""
NIDAR AirMouse ground station (2026-11-10) -- the professional app: Fly · Checklist · Survivors · Logs.

Watch-only: it never sends a command to the drone. It reads the Pi mission server (NIDAR_MISSION_V1,
TCP 5020) and, when ROS 2 is sourced, the camera image and PX4 telemetry. Every connected flight is
recorded (gcs_records/<time>/flight.jsonl.gz) and can be replayed on the Logs screen.

Run in WSL (ROS 2 sourced; PyQt5: sudo apt install python3-pyqt5):
    python3 gcs/airmouse_gcs.py --pi-host 10.218.70.147 --layout gazebo/layouts/nidar_airmouse_arena_s1.json
Try it without the Pi (plays a recorded flight as if live):
    python3 gcs/airmouse_gcs.py --demo gcs_records/demo_s1_offline/flight.jsonl.gz --no-ros
The previous window (gcs/nidar_gcs.py) still works unchanged.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gcs.airmouse.app import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
