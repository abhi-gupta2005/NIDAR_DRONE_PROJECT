# NIDAR AirMouse

A drone system that **explores a building and searches for people without GPS**: LiDAR SLAM and an
autonomous explorer run on a Raspberry Pi, YOLO finds people in the camera image, and everything is tested in a
Gazebo + PX4 simulation on a Windows laptop (WSL 2, ROS 2 Jazzy). Simulation only so far — nothing here arms a
real drone.

```
 Laptop · WSL (Ubuntu 24.04)                 Raspberry Pi                      Laptop · Windows
 Gazebo arena, PX4 SITL, QGroundControl  ─▶  SLAM V6 · explorer brain     ◀─  YOLO (camera AI)
 rig driver, AirMouse ground station     ◀─  V18 fusion · mission server
```

## Install on a new laptop + Pi

Step-by-step guide: **[docs/SETUP.md](docs/SETUP.md)**. In short:

1. **Windows** — PowerShell **as administrator**:
   ```powershell
   irm https://raw.githubusercontent.com/abhi-gupta2005/NIDAR_DRONE_PROJECT/main/tools/install_windows.ps1 | iex
   ```
   then restart if asked, open **Ubuntu 24.04** from the Start menu and pick a user name + password.
2. **Ubuntu (WSL)** — ROS 2 Jazzy, Gazebo, PX4, the agent, px4_msgs, QGroundControl, … (about 1 hour):
   ```bash
   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/install_wsl.sh
   ```
3. **Raspberry Pi** — Raspberry Pi OS Lite 64-bit with SSH + Wi-Fi set in Raspberry Pi Imager, then in Ubuntu:
   ```bash
   nidar setup --pi <PI_ADDRESS> --user <PI_USER>
   nidar install-pi
   ```
4. **Run** — `nidar` (autonomous mission) · `nidar px4` (PX4 + QGroundControl) · `nidar stop`.

Try the ground station without a Pi:
`python3 gcs/airmouse_gcs.py --demo gcs_records/demo_s1_offline/flight.jsonl.gz --no-ros`

## Where things are

| Path | What |
|---|---|
| `tools/nidar_run.sh` | one-command launcher (`nidar`), setup, Pi install/update, stop |
| `tools/install_windows.ps1`, `tools/install_wsl.sh`, `tools/install_pi_fresh.sh` | installers |
| `src/core/nidar_slam_v6.py` | SLAM (LiDAR → map + position), runs on the Pi |
| `src/mission/` | explorer brain, mission server, survivor mapping (Pi) |
| `src/perception/` | V18 camera + LiDAR fusion (Pi) |
| `nidar_v11_windows.py` | YOLO + tracking on Windows |
| `gazebo/` | arena generator, worlds, models, ROS 2 relays, rig driver, PX4 route flyer |
| `gcs/airmouse_gcs.py` | AirMouse ground station app (watch-only) |
| `ros2_ws_src/nidar_airmouse/` | ROS 2 package (bridge, telemetry) |
| `tests/` | `python3 -m pytest -q -m "not slow"` |
| `docs/NIDAR_PROJECT_REGISTER.md` | status, findings, test results |
