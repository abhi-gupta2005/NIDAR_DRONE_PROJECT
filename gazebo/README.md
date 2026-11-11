# NIDAR AirMouse – Gazebo simulation runbook (2026-11-02)

This folder builds the competition arena in Gazebo and connects it to the real NIDAR
pipeline (Windows YOLO → Pi receiver/V18/SLAM/AirMouse server → WSL ROS 2 bridge).

**Safety:** nothing here arms, takes off or enables offboard. `px4_setpoint_sim` keeps
`ALLOW_ARM = ALLOW_OFFBOARD = False`. The rig (Stage B) has no flight controller at all.

## ONE COMMAND (2026-11-09) — instead of the terminals below

`tools/nidar_run.sh` starts and supervises everything from one WSL terminal (Pi stack + brain over SSH,
Gazebo, relays, AirMouse bridge, YOLO in its own Windows window, ground station, QGroundControl for PX4),
waits for SLAM TRACKING, starts the flight program, prints one status line every 5 s and names any lost
link (`LOST: LiDAR scans (WSL relay -> Pi)`). Relays / bridge / rig driver are restarted if they die
(never the PX4 flyer). Ctrl+C stops it in a safe order and copies the Pi's files of the run into
`logs/run_<time>_<mode>/pi/`. Gazebo runs without its window unless `--gui` (less load).
```bash
bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh setup       # once (asks the Pi password one last time)
bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh update-pi   # after a new Pi bundle
bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh             # autonomous: brain + rig
bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh px4         # PX4 + QGroundControl + route flyer
bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh stop        # from any terminal
```
Options: `--seed N --pi HOST --minutes M --radius R --route rooms|survivor:ID --gui --no-yolo --no-gcs --no-qgc --dry-run`.

## What is where

| Path | Runs on | Purpose |
|---|---|---|
| `nidar_arena_generator.py` | any | Generates `worlds/<world>.sdf` + `layouts/<world>.json` (ground truth) + `.png` |
| `models/x500_nidar_airmouse/` | WSL (Gazebo) | PX4 x500 + 70° camera (640×480, 15 Hz) + 360° LiDAR (500 pts, 10 Hz, 12 m) |
| `models/nidar_rig/` | WSL (Gazebo) | Same sensors on a floating rig without PX4, moved by velocity commands |
| `config/nidar_gz_bridge_*.yaml` | WSL | ros_gz_bridge: `/nidar/scan`, `/nidar/camera/image` (+ rig `cmd_vel`/`odom`) |
| `ros2/nidar_lidar_relay.py` | WSL | `/nidar/scan` → NIDAR_LIDAR_V1 JSON → **Pi :5011** |
| `ros2/nidar_camera_relay.py` | WSL | `/nidar/camera/image` → JPEG → **Windows :5012** (same format as your old relay) |
| `ros2/nidar_rig_driver.py` | WSL | Drives the rig along a route from the layout, or (`--brain`) flies the Pi explorer's plans; logs ground truth CSV |
| `tools/send_synthetic_scans.py` | WSL or Windows | Pi dry run **without Gazebo**: ray-cast LiDAR + perfect camera packets |
| `tools/check_scan_vs_truth.py` | WSL | Gazebo LiDAR vs the layout: geometry, angle convention, self-hits |
| `../src/core/nidar_slam_v6.py` | Pi | SLAM V6 (scan-to-map), started with `NIDAR_SLAM=V6` |
| `../src/mission/nidar_mission_server.py` | Pi | survivors (camera + LiDAR + SLAM pose) + data for the ground station, TCP 5020 |
| `../gcs/nidar_gcs.py` | WSL | **Ground station window**: camera, SLAM map, survivors (red dots + grid cell), telemetry |
| `install_into_px4.sh` | WSL | Copies the 2 models + worlds into `~/PX4-Autopilot/Tools/simulation/gz/` (new names only) |
| `run_rig.sh`, `run_px4.sh`, `nidar_gz_env.sh` | WSL | Launch helpers |

Pi side: `src/communication/nidar_lidar_receiver.py` (TCP 5011) and
`src/core/lidar_gazebo_backend.py`; V18 and SLAM use them when started with
`NIDAR_LIDAR_BACKEND=GAZEBO ./start_nidar_pi.sh`. Default (`SIMULATOR`) is unchanged.

Your existing `x500_nidar_indoor`, `mono_cam_nidar_test`, `nidar_indoor_rescue.sdf` and
`~/gazebo_camera_relay.py` are not modified.

```
 Gazebo (WSL)                                    Raspberry Pi                       WSL ROS 2
 gpu_lidar ─/nidar/scan─► nidar_lidar_relay ─TCP 5011─► lidar_receiver ─► V18 ◄─ receiver ◄─TCP 5000─┐
 camera ─/nidar/camera/image─► nidar_camera_relay ─TCP 5012─► Windows nidar_v11 (YOLO) ─────────────┘
                                                              V18 ─► airmouse_server ─TCP 5010─► nidar_bridge ─► /nidar/airmouse_command
```

Replace `10.218.70.147` below with the Pi's current address (`hostname -I` on the Pi).

### 2026-11-02g – colourful arena, "ABHISHEK DRONE", new ground station
* **Worlds:** every room has its own colour (walls + tinted floor), corridor walls warm
  white, outer walls navy, concrete floor with golden grid lines, grass outside, an H
  launch pad, sky. Only colours changed: the collision boxes are identical and the
  coloured wall pieces tile the old walls exactly (`tests/test_gazebo_worlds.py`), so
  the LiDAR sees the same arena. Shadows are off (rendering speed: the camera and
  gpu_lidar rates matter for SLAM); `nidar_arena_generator.py --seed N --shadows`
  makes a world with sun shadows if your GPU keeps up.
* **Rig:** spawned as entity `abhishek_drone`: blue frame, red motor bells, orange front
  props, red/green navigation lights, red name plate. Topics are unchanged
  (`/model/nidar_rig/*` pinned in the model), so bridge and rig driver need no change.
* **Ground station:** `gcs/nidar_gcs.py` shows "ABHISHEK DRONE" (or `--drone-name "..."`).
* After copying the update: run **Stage 0 again** (`install_into_px4.sh`) so PX4's
  Gazebo folders get the new worlds/model (old ones are kept as `.bak_<time>`), then
  `tools/check_scan_vs_truth.py` once to confirm the LiDAR still matches the layout.

---

## Stage 0 – install the assets (WSL, once)

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/d/NIDAR_DRONE_PROJECT
bash gazebo/install_into_px4.sh
```
Expected: `installed : Tools/simulation/gz/models/x500_nidar_airmouse` (and `nidar_rig`, 3 worlds),
then `OK  models/x500_nidar_airmouse`, `OK  models/nidar_rig`, and `OK` or `OK*` for each world.
Send me the whole output if any line says `FAIL`.

## Stage A – Pi path without Gazebo (proves Pi side first)

**Pi terminal** (after installing the Pi bundle):
```bash
cd ~/drone_project
NIDAR_LIDAR_BACKEND=GAZEBO ./start_nidar_pi.sh
```
Expected: `[OK] LiDAR backend: GAZEBO` and `[2b/6] ... LiDAR receiver started`.
Until scans arrive, `logs/v18.log` shows `[LiDAR STALE] ... fusion paused` once per second (correct).

**WSL terminal 1** – feed a virtual drone route (S2 = a standing survivor in arena seed 1).
Stop `nidar_v11_windows.py` first: this script sends the camera packets itself.
```bash
cd /mnt/d/NIDAR_DRONE_PROJECT
python3 gazebo/tools/send_synthetic_scans.py --pi-host 10.218.70.147 --route survivor:S2 --camera --speed 0.8
```
**WSL terminal 2** – the bridge you already use:
```bash
source /opt/ros/jazzy/setup.bash && source ~/px4_ros2_ws/install/setup.bash
export NIDAR_PI_HOST=10.218.70.147
ros2 run nidar_airmouse nidar_bridge
```
Success = when the sender prints `truth in view: S2 1.09 m -0.0 deg`, the Pi monitor shows a
person ACTIVE with R ≈ 0.9 m (surface of the body) and bearing ≈ 0°, and the bridge prints
TRACKING. After Ctrl+C on the sender: Pi shows LiDAR STALE, the server reports `V18_STALE`,
the bridge publishes zero. (Verified here on a copy of the Pi stack: R 0.917 m, bearing 0.001°,
then `V18_STALE`.)

## Stage B – Gazebo rig, no PX4

**WSL terminal 1** (Gazebo + rig + bridge for the rig):
```bash
bash /mnt/d/NIDAR_DRONE_PROJECT/gazebo/run_rig.sh 1 1.0
```
First launch downloads the three Fuel survivor models (needs internet in WSL).

**WSL terminal 2** – check the sensors really publish:
```bash
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
ros2 topic hz /nidar/scan            # expect ~10 Hz   (Ctrl+C after a few lines)
ros2 topic hz /nidar/camera/image    # expect ~15 Hz
ros2 topic echo /nidar/rig/odom --once
```
**WSL terminal 3** – LiDAR to the Pi (Pi started with `NIDAR_LIDAR_BACKEND=GAZEBO`):
```bash
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
python3 $NIDAR_REPO/gazebo/ros2/nidar_lidar_relay.py --pi-host 10.218.70.147
```
**WSL terminal 4** – camera to Windows:
```bash
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
python3 $NIDAR_REPO/gazebo/ros2/nidar_camera_relay.py
```
**Windows PowerShell** – YOLO on the Gazebo camera, packets to the Pi:
```powershell
cd D:\NIDAR_DRONE_PROJECT
$env:NIDAR_CAMERA_SOURCE = 'gazebo'
$env:NIDAR_PI_HOST = '10.218.70.147'
py -3 nidar_v11_windows.py
```
Expected: `Camera source : GAZEBO relay 127.0.0.1:5012`, `Camera resolution : 640 x 480`.

**WSL terminal 5** – move the rig to look at survivor S2:
```bash
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
python3 $NIDAR_REPO/gazebo/ros2/nidar_rig_driver.py --seed 1 --route survivor:S2
```
Plus the bridge (Stage A, terminal 2). What to send me: the last 20 lines of the Pi
monitor, `tail -30 ~/drone_project/logs/v18.log`, the YOLO window/console (is the survivor
detected as `person`?), and the rig's `[RIG] route finished ...` line.

## Stage C – PX4 + x500_nidar_airmouse in the arena (no arming)

```bash
# WSL terminal 1 – the uXRCE-DDS agent, started the way you normally do, e.g.
MicroXRCEAgent udp4 -p 8888
# WSL terminal 2
bash /mnt/d/NIDAR_DRONE_PROJECT/gazebo/run_px4.sh 1
# WSL terminal 3
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$NIDAR_REPO/gazebo/config/nidar_gz_bridge_sensors.yaml
```
Then the relays and Windows exactly as in Stage B (no rig driver). Telemetry check:
```bash
ros2 topic hz --help | grep -i qos     # see which QoS options this CLI accepts first
ros2 topic list | grep fmu/out
```

### Stage C2 – fly the PX4 drone through the arena (simulation only, 2026-11-09)

With everything from Stage C running (+ relays, Pi stack, YOLO, ground station), last terminal:
```bash
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
python3 $NIDAR_REPO/gazebo/ros2/nidar_px4_route_flyer.py --seed 1 --route rooms     # ~15 min
python3 $NIDAR_REPO/gazebo/ros2/nidar_px4_route_flyer.py --seed 1 --route survivor:S2   # ~3 min
```
It arms PX4 in OFFBOARD, takes off to 0.64 m (LiDAR plane 1.0 m, camera 0.82 m, like the rig),
flies corridor centrelines with stop-and-turn (x500 prop tips reach 0.39 m, walls are 0.5 m
from the centreline), enters each room only as far as the LiDAR shows 0.45 m free, looks around,
backs out, returns to the pad and lands. Refuses to run unless the arena world is running on
this machine. Ctrl+C = LAND. Typing a mode at `pxh>` takes over (the script stops commanding).
Log: `logs/px4_flight_<time>.csv`. Not connected to NIDAR's AirMouse commands (those stay off).

---

### Stage C3 – QGroundControl (PX4's ground station) on the simulated PX4

PX4 uses **QGroundControl**; Mission Planner is ArduPilot's (a different autopilot). Everything in this project
(uXRCE-DDS, `/fmu/*` topics, OFFBOARD, the flyer) is PX4. QGC talks MAVLink to PX4 (UDP), separately from ROS 2.
Easiest: QGC inside WSL — it connects to PX4 SITL by itself (PX4 docs, "Windows Development Environment (WSL2)").
Once (WSL):
```bash
sudo apt install -y libfuse2t64 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
cd ~ && wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage && chmod +x QGroundControl-x86_64.AppImage
```
Each time, after `run_px4.sh` (Stage C): `~/QGroundControl-x86_64.AppImage` (if it complains about FUSE:
`~/QGroundControl-x86_64.AppImage --appimage-extract-and-run`).
QGC on Windows instead: install `https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-installer-AMD64.exe`,
then Application Settings → Comm Links → Add → UDP, server = WSL's address (`hostname -I` in WSL, first number)
port 18570 → Connect (the address changes when WSL restarts).
QGC can arm and fly the simulated drone: don't press anything in it while the route flyer is flying.
The SITL-only arming parameters (§1h) must never be set on a real flight controller.

## Stage D – survivors on the ground station (SLAM V6 + mapping + GCS)

Needs the Pi bundle `nidar_pi_install_2026-11-02e.tgz` (or newer) installed (see the main register).
Start the Pi stack and wait until the ground station shows SLAM **TRACKING** with the rig still
on the launch pad; start the rig driver last (F28: SLAM's origin is wherever it starts).

**Pi terminal:**
```bash
cd ~/drone_project
NIDAR_LIDAR_BACKEND=GAZEBO NIDAR_SLAM=V6 NIDAR_ARENA_LAYOUT=gazebo/layouts/nidar_airmouse_arena_s1.json ./start_nidar_pi.sh
```
`NIDAR_ARENA_LAYOUT` gives the mission server the start pose (the launch pad) and the grid.
In the real competition use `NIDAR_START_POSE="x,y,yaw_deg"` (pad position in the arena) instead.
`NIDAR_CAMERA_HEIGHT_M` (default 0.82 = rig at 1.0 m) is used to range low survivors from the image.

**WSL:** rig world (Stage B terminal 1), LiDAR relay (Stage B terminal 3), camera relay (terminal 4),
then the ground station (PyQt5 once: `sudo apt install python3-pyqt5`):
```bash
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
python3 $NIDAR_REPO/gcs/nidar_gcs.py --pi-host 10.218.70.147 --layout gazebo/layouts/nidar_airmouse_arena_s1.json
```
**Windows:** YOLO on the Gazebo camera (Stage B PowerShell). **WSL:** drive the rig through the rooms:
```bash
python3 $NIDAR_REPO/gazebo/ros2/nidar_rig_driver.py --seed 1 --route rooms
```
Without Gazebo/Windows the same Pi chain can be fed by
`python3 gazebo/tools/send_synthetic_scans.py --pi-host <PI> --route rooms --camera --speed 0.6`.

What the window shows: camera (top left) with the detection boxes; SLAM map in the arena
frame with the grid A–N × 1–14, the drone (yellow), its path (blue), survivors (red dots
labelled e.g. `S2 E3`), ground truth (green, simulation only; `--layout`); survivor table;
pipeline health (green < 1 s old, orange < 3 s, red older/missing); PX4 telemetry when PX4
SITL + uXRCE-DDS run; AirMouse command. "Export report" saves survivors (JSON/CSV) and
screenshots under `gcs_reports/`. The window never sends commands.

Measured before Gazebo (ray-cast arena, REAL V18 fusion, SLAM V6 pose, no truth): `rooms`
route in arenas s1–s3 → 5 of 6 survivors confirmed in each, every one in the correct cell,
0.01–0.15 m error, no false survivor. Full Pi chain with the real start script on a copy of
the Pi stack (route `survivor:S2`): S1 K5 (0.25 m) and S2 E3 (0.03 m), SLAM end error 0.04 m.

## Stage E – AUTONOMOUS: the drone explores an unknown arena by itself (rig, 2026-11-09)

Same as Stage D, except: no route. The **explorer service** on the Pi (the "brain") knows nothing
about the arena; it reads only SLAM V6 (pose, map, the scan) and sends flight plans to the rig driver
(`--brain`), which flies them. It explores until nothing is left (or 20 min), then returns to the
take-off point. Needs the Pi bundle `nidar_pi_install_2026-11-09.tgz`.

**Pi SSH window 1** (stack; the layout here only labels cells / scores on the GCS — the brain never reads it):
```bash
cd ~/drone_project
NIDAR_LIDAR_BACKEND=GAZEBO NIDAR_SLAM=V6 NIDAR_ARENA_LAYOUT=gazebo/layouts/nidar_airmouse_arena_s1.json ./start_nidar_pi.sh
```
(Truly unknown arena: leave out `NIDAR_ARENA_LAYOUT` — the GCS then shows coordinates instead of cells.)

**Pi SSH window 2** (the brain):
```bash
cd ~/drone_project && source .venv/bin/activate
python src/mission/nidar_explorer_service.py --radius 0.30 --mission-min 45
```
`--radius 0.30` for the 0.25 m rig = 5 cm extra for SLAM error (offline with full SLAM V6: 0.25 → closest wall
0.21 m; 0.30 → 0.28 m, still 6/6 and coverage 0.98 — register §1k, F35).
`--mission-min` counts REAL minutes. Gazebo in WSL runs at about half speed with the cameras on (2026-11-09: rig
0.21–0.25 m/s of 0.4 commanded, odometry 14.4 of 30 Hz), so 20 min left two rooms unvisited; use 45 in Gazebo.
A real drone keeps the real battery time (20).
**WSL:** rig world, LiDAR relay, camera relay, GCS (Stage D); **Windows:** YOLO (Stage B). Rig still on the pad,
GCS shows SLAM TRACKING, then **WSL last terminal**:
```bash
source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
python3 $NIDAR_REPO/gazebo/ros2/nidar_rig_driver.py --brain 10.218.70.147 --seed 1
```
Expected: `[RIG] brain connected`, `aligned: SLAM origin = odom (...)`, then `plan 1 (look): ...`; the GCS shows a
green **BRAIN EXPLORE** pill, the plan as a dashed magenta line with a LOOK/GOAL marker, and `BRAIN plan N` lines in
the event log. At the end: `MISSION COMPLETE` (rig stops on the pad; a real drone would land).
Safety: the rig stands still when the brain, the odometry or the LiDAR is missing, when the brain says HOLD (SLAM
stale/LOST) and when the LiDAR sees anything < 0.30 m in the direction of motion (reported BLOCKED; the brain
replans). Ctrl+C on either side stops it.

First Gazebo run (2026-11-09, s1, 20 min): explored by itself, back on the pad 0.05 m, 3 of 6 survivors right
(K5 lying found for the first time), H12 reported twice in wrong cells (F31), E3 and K8 never seen (time ran out:
see above), closest wall 0.22 m (fixed alignment error, now kept up to date — register §1k).

Measured offline (ray-cast arena, SLAM V6 grid code, the real Brain + BrainFollower talking NIDAR_NAV_V1 JSON,
rotated odometry frame; `tests/nav_loop_sim.py`): s1 and s2 explored in 19–20 min, camera coverage 0.98, 6/6
survivors in view, back on the pad 0.04 m, closest approach to a wall 0.30 m (rig radius 0.25 m), never BLOCKED.

## Measured on the arena before Gazebo (ray-cast, perfect detections)

`python3 tools/arena_fusion_sweep.py --seed 1 --height 1.0`

| Survivor | Views | V18 fused | Range error | Notes |
|---|---|---|---|---|
| standing (1.7 m) | 140 | 140 (100 %) | median 0.03 m | LiDAR plane hits the body |
| low/lying (0.45 m) | 85 | 13 (15 %) | all 13 > 0.5 m wrong | LiDAR passes over it; V18 fused the wall behind (F22) |

With the LiDAR plane at 0.3 m the low survivors fuse 87 % with 0.04 m error. Gazebo will
show whether the real meshes behave the same.

## Known limits (see docs/NIDAR_PROJECT_REGISTER.md)

* **F22** low survivors + 1.0 m LiDAR plane → no fusion or wrong (wall) range.
* **F24** V18 freezes a target's bearing after one jump > 25° (e.g. a fast yaw) while
  keeping it ACTIVE. Blocks closed-loop use until fixed.
* **SLAM-01** SLAM **V5.1** pose does not follow motion (arena: 6.8 m travelled → 0.44 m estimated).
  SLAM **V6** fixes it (arena routes 17–157 m: final error ≤ 0.04 m in the ray-cast tests).
* Low survivors closer than ~1.6 m: the level camera at 0.82 m cannot see the floor that close
  and the LiDAR passes over them, so they are only mapped from further away (a camera tilted
  down ~20° would fix this — not done).
* V18 converts pixels to bearing linearly; a pinhole camera gives up to ~2° error at 25°
  (inside the 10° fusion gate).
* Gazebo/PX4/ROS parts of this folder were written against PX4 v1.18.0-beta1 and
  Gazebo Harmonic 8.15 but have **not been run yet** – Stages 0, B, C are their first test.
