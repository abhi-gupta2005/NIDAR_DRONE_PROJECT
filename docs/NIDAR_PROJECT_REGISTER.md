# NIDAR AirMouse — Project Register

Last updated: 2026-11-09m (PX4 launcher runs: crash at D10 = F33, Gazebo pauses F36, §1m). Before that 2026-11-09l (one-command launcher tools/nidar_run.sh, link-loss reporting, §1l). Before that 2026-11-09k (first autonomous Gazebo run: 3/6, time limit, F35 alignment fix, §1k). Before that 2026-11-09j (autonomous explorer — the Pi "brain" + rig follow mode + GCS plan view, §1j). Before that 2026-11-09i (PX4 SITL flight 05:49 UTC: uncommanded spin at F3 → SLAM LOST for good; F33, F34). Before that 2026-11-02f (first full Gazebo run on the Pi: SLAM V6 diverged → fixed; survivor IDs kept; GCS grid). Before that 2026-11-02e (SLAM V6, survivor mapping, Pi mission server, desktop ground station; F25 verified fixed in Gazebo). Before that 2026-11-02b (Gazebo phase: competition arena, Gazebo sensor models, Gazebo-LiDAR path to the Pi, findings F22–F24). Before that: Pi snapshot; airmouse server fixed; bundle installed on the Pi; live webcam → Pi → V18 → ROS 2 bridge verified incl. stop on camera close. Evidence levels: **VERIFIED-SIM** (proven by the automated simulation/tests on real project code), **VERIFIED-HIST** (historical log evidence only), **UNVERIFIED** (no evidence yet), **SIMULATED** (works, but on simulated sensor data), **NOT IMPLEMENTED**.

Nothing in this document claims the system works on hardware. Every claim below cites the test or file that supports it.

---

## 1. What changed in this session (2026-11-01)

| File | Change | Why (evidence) |
|---|---|---|
| `src/communication/nidar_target_server.py` | **SUPERSEDED 2026-11-02**, moved to `archive/superseded_2026-11-02/` with its tests | Written because the Windows copy had no port-5010 server. The Pi already had `nidar_airmouse_server.py`; that one was kept and fixed instead (below), to preserve the existing architecture. |
| `nidar_airmouse_server.py` (Pi, project root) | **Fixed 2026-11-02** (original in `archive/pi_originals_2026-11-02/`): (1) freshness: V18 JSON older than 1.0 s → `V18_STALE`, camera frame not advancing for 1.0 s → `CAMERA_STALE`, file missing → `V18_MISSING`, no objects → `NO_TARGET`; all sent as non-ACTIVE targets so the bridge zeroes immediately. (2) non-blocking `accept()` so the loop really runs at 10 Hz. (3) JSON path relative to the file. (4) `status` field added; log printing throttled to 1/s or on change | Measured on the original file: 1-hour-old V18 JSON with an ACTIVE person → bridge published fwd 0.99 m/s, valid=1 (F13); 6 packets in 6 s, gaps 1.1 s > bridge timeout 1.0 s → PX4 velocity setpoint carried the command only **69.4 %** of the time (F14). After: stale → zero; 10.2 Hz; `v18_crash` scenario zero command 1.0 s after V18 dies (original: never). |
| `src/perception/nidar_v18_headless_persistent_fusion.py` | **Re-based 2026-11-02 on the Pi's own file** (no Windows `[FUSION DEBUG]` block) with the same V18.1 changes | The Windows copy differed from the Pi only by that debug block (full diff checked). The debug block printed every unmatched camera object every frame. |
| `src/perception/nidar_v18_headless_persistent_fusion.py` | V18.1: `CAMERA_GRACE_SEC = 0.25`; objects are aged only when the camera has been silent longer than that | The 20 Hz loop aged every object on each tick without a new frame (camera ≈ 11 Hz), so objects flickered ACTIVE↔TEMP_LOST and commands were zero ~40 % of the time *with the person in plain view*. Simulation: ACTIVE coverage 0.55 → **1.00**, valid-command ratio 0.60 → **1.00**. Stale-camera timing unchanged (zero command 0.25 s, PREDICTED 1.55 s, EXPIRED 3.05 s). |
| same file | `PROJECT_DIR` derived from the file location (override: `NIDAR_PROJECT_DIR`) instead of hard-coded `~/drone_project` | In WSL the project is at `/mnt/d/NIDAR_DRONE_PROJECT`, so V18 read a file nobody writes and reported `CAMERA STALE` forever. Identical behaviour on the Pi. Test: `test_v18_reads_the_file_the_receiver_writes`. |
| `src/core/nidar_slam_v5_1.py` | One f-string: inner `"` → `'` | Nested same-type quotes are Python ≥ 3.12 only; a `SyntaxError` on Python 3.11. Output unchanged. **2026-11-02:** the Pi runs Debian 13 / Python 3.13.5, so this never affected the Pi. |
| `start_nidar_pi.sh`, `start_nidar_wsl.sh` | `export PYTHONUNBUFFERED=1`; start **`nidar_airmouse_server.py`** with the pipeline (log `logs/airmouse_server.log`), kill stale copies at start, stop it on exit | `logs/receiver.log` was 0 bytes: stdout of background Python is block-buffered and lost when killed. |
| `archive/pre_session_2026-11-01/` | Exact copies of the four files above before modification | Roll back with a copy if needed. |
| `tests/` (93 tests), `src/simulation/*`, `training/*`, `pytest.ini` | **NEW.** Test harness, co-simulation, replay tool, training pipeline | See sections 5–7. |
| `tools/nidar_pi_status.sh`, `tools/install_on_pi.sh`, `nidar_pi_install_2026-11-02.tgz` | **NEW.** Read-only Pi status report; guarded Pi installer (refuses if a file changed since audit or NIDAR is running; backs up everything it replaces) | Rehearsed on a mock Pi: refusal cases, install, rerun, tests 90 passed / 2 skipped / 1 xfailed on Python 3.11 and 3.13, live start-script run with 10.0 Hz on port 5010 |
| `models/nidar_aerial_yolo26n_cpu15_640.pt` | **NEW.** First fine-tuned aerial detector | See section 7. |

Not modified: `nidar_v11_windows.py`, `nidar_pi_receiver.py`, LiDAR interface/detector/tracker, `integration_monitor.py`, every ROS 2 node.

---

## 1c. Gazebo phase (2026-11-02b) — what changed

Goal (Mission Brief): the competition arena in Gazebo, connected to the existing pipeline/Pi.
Default behaviour of every Pi process is unchanged; the Gazebo path is opt-in
(`NIDAR_LIDAR_BACKEND=GAZEBO`, `NIDAR_CAMERA_SOURCE=gazebo`).

| File | Change | Evidence |
|---|---|---|
| `gazebo/nidar_arena_generator.py` + `worlds/`, `layouts/` (seeds 1–3) | **NEW.** 14.75 m arena, 14×14 cells of 1.05 m, 0.05 m walls 2.44 m high, 8 rooms with one door, 6 survivors (Fuel: Rescue Randy, Standing person, Casual female), ceiling net, launch pad, grid labels; ground truth JSON | All brief checks PASS for seeds 1–3; 196/196 cells reachable; XML well-formed. **Not yet loaded in Gazebo** |
| `gazebo/models/x500_nidar_airmouse`, `gazebo/models/nidar_rig` | **NEW.** x500 + camera (70° HFOV = V18's value, 640×480, 15 Hz) + 360° gpu_lidar (500 pts, 10 Hz, 0.15–12 m); rig = same sensors without PX4 (VelocityControl + OdometryPublisher) | XML well-formed; `gz sdf -k` must run in WSL (`install_into_px4.sh`) |
| `gazebo/config/*.yaml`, `gazebo/ros2/nidar_lidar_relay.py`, `nidar_camera_relay.py`, `nidar_rig_driver.py`, `gazebo/*.sh` | **NEW.** ros_gz_bridge configs, LaserScan → Pi :5011, image → Windows :5012 (same format as the old relay), route driver, install/launch scripts | Non-ROS parts tested: `tests/test_gazebo_wsl_nodes.py` (relay → real Pi receiver → backend; camera server → real Windows capture; routes never closer than 0.25 m to a wall). ROS parts **UNVERIFIED** |
| `src/communication/nidar_lidar_receiver.py`, `src/core/lidar_gazebo_backend.py` | **NEW.** Pi receiver for NIDAR_LIDAR_V1 (TCP 5011, atomic latest-scan file); backend converts ROS CCW angles → NIDAR clockwise, returns each scan once, rejects scans older than 1 s (also a file left from an earlier run), raises `LidarStaleError` | `tests/test_gazebo_lidar_path.py` (11) |
| `src/perception/nidar_v18_headless_persistent_fusion.py` | Backend chosen by `NIDAR_LIDAR_BACKEND` (SIMULATOR default / GAZEBO); camera width from each packet (`packet_camera_width`, fallback 1280) — fixes F10's width part; `NIDAR_CAMERA_HFOV_DEG` (default 70); labels `camera_source`/`lidar_source` now from the real sources; `LidarStaleError` → fusion paused, JSON not refreshed → server `V18_STALE` → bridge zero | Real V18 process test with arena feed: no JSON while LiDAR silent; ACTIVE range 1.33 m vs body surface 1.30 m, bearing 23.3° vs 25.0°; JSON stops after LiDAR stops. Default-mode tests unchanged |
| `src/core/nidar_slam_v5_1.py` | `LIDAR_BACKEND` from env (default SIMULATOR), GAZEBO option, waits on `LidarStaleError`, label from the backend | Real SLAM process test: reads Gazebo-format scans, survives an outage |
| `start_nidar_pi.sh` | Validates `NIDAR_LIDAR_BACKEND`; starts `nidar_lidar_receiver.py` before V18/SLAM only for GAZEBO; stops it on exit; Windows hint now `py -3`; stop labels 1/5–5/5 | Dry run of the whole script on a copy of the Pi stack (below) |
| `integration_monitor.py` | `fmt_num()` for SLAM pose values that are `None` (F23) | Test fails on the old file, passes on the new |
| `nidar_v11_windows.py` | Opt-in `NIDAR_CAMERA_SOURCE=gazebo`: `GazeboCameraCapture` (newest frame only, each frame once, read fails instead of repeating a frozen frame, disconnect detected); packet `camera.source` = `gazebo_sim` | `tests/test_windows_gazebo_camera.py`; webcam remains default |
| `src/simulation/arena_raycast.py`, `arena_feed.py`, `tools/arena_fusion_sweep.py`, `gazebo/tools/send_synthetic_scans.py` | **NEW.** 2-D ray-caster over the arena, perfect-detection camera packets (pinhole), fusion sweep, Pi dry-run sender | Used by the tests above |
| `tools/install_on_pi.sh` | Accepts the audited original **or** the 2026-11-02 installed version of each changed file; also guards `integration_monitor.py`; detects a running LiDAR receiver | Rehearsed on a mock Pi holding the 2026-11-02 install: 5 files "known version", 11 replaced (backed up), 18 new; tests there 121 passed / 2 skipped / 3 xfailed |

**First WSL run (2026-11-02, Stage 0 of `gazebo/README.md`) — results and fixes (2026-11-02c):**
- `install_into_px4.sh`: models `x500_nidar_airmouse`, `nidar_rig` and worlds s1–s3 **installed** into `~/PX4-Autopilot/Tools/simulation/gz/`. `gz sdf -k`: `nidar_rig` **Valid**; worlds OK* (Fuel URIs only). `x500_nidar_airmouse` reported `Unable to find uri[x500]` (+ the `base_link` errors that follow from it): `gz sdf -k` has no Gazebo resource-path callback, so it cannot resolve PX4's bare `<uri>x500</uri>` include — the same include PX4's own `x500_lidar_2d` uses. Script now runs the checker with `SDF_PATH` from the models folder and calibrates against `x500_lidar_2d`: identical failure → **INCONCLUSIVE** (real test = loading it in Stage C), any other error → FAIL. `gz_frame_id` "not defined in SDF" warnings are harmless (PX4's models use the same element).
- `run_rig.sh` **aborted before Gazebo started**: `/opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable` — the script's `set -u` applied to ROS's setup file. Hence `ros2 topic hz /nidar/scan`: "not published yet". Fixed in `nidar_gz_env.sh` (turns `-u` off while sourcing ROS, restores it; also affects `run_px4.sh`). `run_rig.sh` also waits up to 300 s for the world (first launch downloads the Fuel survivors) and no longer uses `cmd | grep -q` under `pipefail`.
- Regression tests `tests/test_gazebo_scripts.py` (fake `gz`/ROS): both failures reproduced with the old scripts, pass with the new. Backups of the replaced scripts on D: in `archive/pre_gazebo_fix_2026-11-02c/`.
- Status after the fix: see Stage B below.

**Stage B, first rig run (2026-11-02, WSL, `run_rig.sh 1 1.0`) — VERIFIED LIVE:**
- World `nidar_airmouse_arena_s1` loads; rig spawned (`create` → `data: true`); gz topics present: `/nidar/scan`, `/nidar/scan/points`, `/nidar/camera/image`, `/nidar/camera/camera_info`, `/model/nidar_rig/{cmd_vel,odometry,pose}`.
- ros_gz_bridge created all 4 bridges. Measured in ROS 2 (`ros2 topic hz`): **/nidar/scan 7.7 Hz** (configured 10 Hz, std 0.025 s, max gap 0.167 s); **/nidar/camera/image 8.4 Hz** (configured 15 Hz, one 1.35 s gap). Rates below the sensor settings → Gazebo probably runs slower than real time (real-time factor not yet read). Consequences: V18 runs at the LiDAR rate (~7.7 Hz); a camera gap > 1.0 s makes the AirMouse server report CAMERA_STALE (a stop — correct, but frequent gaps would cause stop-and-go).
- Harmless messages: PX4's `server.config` lists PX4-built plugins (`libOpticalFlowSystem.so`, `libGstCameraSystem.so`) that are only on PX4's own plugin path (set when PX4 launches Gazebo); `Thermal` and `gz_frame_id` SDF warnings.
- **Rig driver VERIFIED LIVE:** route `survivor:S2` (18.1 m) driven through the maze; finished at x=4.71 y=1.60 yaw 90° (route end 4.75, 1.60). Odometry is in the **arena/world frame** (not relative to spawn). A few odometry gaps > 0.5 s (driver holds still during them).
- **LiDAR vs ground truth VERIFIED LIVE** (`check_scan_vs_truth.py` at the S2 view pose): 500/500 beams, **469 of 471 wall beams within 5 cm, median error 0.007 m, p95 0.022 m**; mirrored hypothesis only 151 → the ROS CCW angle convention, the arena geometry in Gazebo and the layout JSON agree; no rotation offset. So `lidar_gazebo_backend.py`'s conversion assumption holds for real Gazebo scans.
- **F25 – rig LiDAR saw its own body (FOUND by that check, FIXED 2026-11-02d, re-test pending).** 29 beams straight ahead (±10°) read 0.15–0.17 m where survivor S2 stood ~0.9 m away: the rig's orange "nose" box (0.15–0.19 m forward) and body box were in the scan plane, and gpu_lidar renders visuals. Through V18 this would have put the person at ~0.16 m (bridge: full reverse). Fix: the rig now carries the PX4 x500 meshes (it looks like the drone; still no PX4/physics) with every visual ≥ 6.9 cm below the scan plane (frame mesh top measured from NXP-HGD-CF.dae); camera moved to 0.15 m forward / 0.18 m below the LiDAR on both rig and `x500_nidar_airmouse` (no frame vertex in its field of view, props out of view); LiDAR puck visual below the plane. `check_scan_vs_truth.py` now FAILs on returns < 0.35 m that the arena does not explain; `tests/test_gazebo_models.py` keeps primitives out of the plane (the old rig fails it). Lesson for the real drone: the STL-19P needs a clear 360° plane; consider a self-mask in software.
- **F25 fix VERIFIED LIVE (2026-11-02, Gazebo):** at the S2 view pose: forward 0.889 m (predicted 0.885), **near returns 0**, 470/470 wall beams within 5 cm, median 0.007 m. The rig now shows the x500 meshes.
- Not yet verified: real-time factor, relays to Pi/Windows, YOLO on rendered survivors.

---

## 1d. Survivor localisation + ground station (2026-11-02e)

Request: localise survivors with SLAM + camera + LiDAR, show them as red dots with their grid cell on a ground-station GUI with camera view, SLAM map, position and telemetry. Decisions (user): **desktop window**; SLAM + mapping **on the Pi**.

| File | What | Evidence |
|---|---|---|
| `src/core/nidar_slam_v6.py` | **NEW SLAM V6** (V5.1 unchanged): occupancy grids 0.20/0.10/0.05 m (log-odds, hits + free space), each scan matched to the **map** by Gauss-Newton on the bilinear occupancy, coarse→fine, constant-velocity prediction; match accepted only if score ≥ 0.2 and correction ≤ 0.35 m / 20°. Right-handed body frame (x fwd, y left) — V5.1's map was mirrored. State every scan (monitor-compatible), 0.10 m grid `.npy` every 1 s | Arena ray-cast scans through the real NIDAR_LIDAR_V1 → LidarScan path, vehicle-like motion at 7.7 Hz: S2 route 16.9 m → **final 0.041 m**, max 0.044; rooms routes 126–157 m (arenas s1–s3) → **final ≤ 0.031 m, max ≤ 0.142 m**; 3× LiDAR noise → 0.12 m. ~7 ms/scan on the laptop (Pi rate to be measured). `tests/test_slam_v6.py` (incl. real process, GAZEBO backend, outage) |
| `src/mission/survivor_mapper.py` | **NEW.** Per camera frame: pinhole bearing from the box; range from LiDAR (V18 fused) **or** from the box bottom on a flat floor (camera height); LiDAR rejected when it disagrees with the floor range or when the box top lies below the LiDAR plane (F22 fix for mapping, V18 untouched); SLAM pose at that time → arena frame via start pose; clustering, CONFIRMED after ≥ 5 obs over ≥ 0.8 s; grid label (A–N, 1–14) | REAL V18 fusion functions + SLAM V6 pose (no truth): rooms route (with 360° look in each room) arenas s1–s3 → **5/6 survivors confirmed in each, all in the correct cell, 0.01–0.15 m error, no false survivor**. Missed: low survivors only seen < 1.6 m (floor not visible to a level camera that close, LiDAR passes over). `tests/test_survivor_mapper.py` |
| `src/mission/nidar_mission_server.py` | **NEW Pi process** (started with `NIDAR_SLAM=V6`): reads SLAM V6/V18/perception/scan files, runs the mapper, serves `NIDAR_MISSION_V1` on TCP 5020 (state 5 Hz, map ≤ 1 Hz, cropped+zlib), writes `outputs/mission/survivors.json`. Config: `NIDAR_ARENA_LAYOUT` or `NIDAR_START_POSE`, `NIDAR_CAMERA_HEIGHT_M` | `tests/test_mission_server.py` |
| `gcs/nidar_gcs.py` | **NEW desktop ground station (PyQt5, WSL)**: camera (ROS 2 `/nidar/camera/image`) + detection boxes; SLAM map in arena frame + grid + drone/heading/FOV + path + **red survivor dots "S2 E3"** + optional ground truth; survivor table; telemetry: link, pipeline health (age-coloured), PX4 `/fmu/out/*` (auto-discovered, best effort), AirMouse command; event log; export (JSON/CSV/PNG). Read-only | Offscreen screenshots against a real mission server; `tests/test_gcs.py` (map transform vs `compose()`, live server → table/log/export). ROS 2 side not run yet |
| `start_nidar_pi.sh` | `NIDAR_SLAM=V5` (default, unchanged) / `V6` → SLAM V6 + mission server | **Dry run of the real script** (copy of the Pi stack, GAZEBO + V6, synthetic camera+LiDAR along `survivor:S2`): all services up; SLAM V6 422 scans accepted / 0 rejected, end pose error 0.04 m; **S1 K5 (0.25 m), S2 E3 (0.03 m)**; GCS window showed both, map aligned with truth walls |
| `integration_monitor.py` | With `NIDAR_SLAM=V6` shows SLAM V6's state file | — |
| `src/simulation/arena_trajectory.py`, `arena_feed.py`, `arena_raycast.py`, `gazebo/ros2/nidar_rig_driver.py`, `gazebo/tools/send_synthetic_scans.py` | Vehicle-like trajectories; pinhole bounding boxes from body size and camera height; `rooms` route now enters each room and turns 360°; follower waits for heading on turn-in-place steps; synthetic sender uses the same motion | `tests/test_gazebo_wsl_nodes.py` |
| `gazebo/ros2/nidar_*_relay.py`, `gcs` | Internal fix: `threading.Thread` subclasses used `_stop`/`_handle` attribute names that Python's threading uses itself (crashed the GCS client on Python 3.13; `_stop` risky on 3.12) → renamed | GCS test |
| `nidar_pi_install_2026-11-02c.tgz`, `tools/install_on_pi.sh` | Bundle c; installer accepts the a- and b-installed versions | Rehearsed over both: OK; mock Pi tests **155 passed, 2 skipped, 3 xfailed** |

---

**Full Pi dry run in GAZEBO mode (copy of the Pi stack in the analysis workspace, real `start_nidar_pi.sh`):**
synthetic camera + LiDAR along route `survivor:S2` → receivers → V18 → AirMouse server → TCP client:
`TRACKING`, person ACTIVE, range **0.917 m** (truth: body surface 0.89 m), bearing **0.001°** (truth 0.0°);
sender stopped → `V18_STALE`. SLAM in the same run: rig moved 3.9 m net → SLAM pose (−0.14, −0.08) m (SLAM-01).

---

## 1e. First full Gazebo run with the Pi (2026-11-02) — findings and fixes (2026-11-02f)

Evidence from the user (GCS screenshot, `survivors.json/.csv`): rooms route in Gazebo, Pi running V18 + SLAM V6 + mission server.
- **F26 – SLAM V6 diverged on the Pi (FIXED, verified offline).** Health: SLAM 5.0 Hz, **accepted 3400 / rejected 2272**, score 0.00; at the end the SLAM pose was 21.2 m from the truth; the map was smeared outside the arena. Reproduced offline (arena scans at 7.7 Hz, Wi-Fi bursts: every 2nd scan held ~120 ms, SLAM busy 0.2 s/scan and taking the newest scan): final error **13.8 m** (receive times) / **18.6 m** (sensor stamps); at 7.2 Hz with stamps it was fine (0.04 m) → two causes: (1) the motion model used the Pi **receive time**, which arrives in bursts (velocity estimates from 15 ms gaps); (2) at ~5 Hz the 90° turn-in-place steps in square rooms let the matcher lock onto a wrong rotation. Fix: SLAM uses the Gazebo **sensor stamp** (relay already sends it; backend now passes `scan.sensor_time`); matching tries the prediction, then "did not move", then a wide yaw/xy search, each starting with a coarse yaw search; motion gate against the last pose (≤ 1.5 m/s, 1.5 rad/s); velocity clamped and reset on rejection; the last good pose is held when nothing matches; status **LOST** after 10 rejections in a row; only clearly good matches (score ≥ 0.35) write into the map; cheaper map updates (free space ≤ 6 m, no sort), 240 points. Same reproduction after the fix: final **0.02–0.03 m**, max ≤ 0.14 m, 0 rejected, arenas s1–s3, with stamps AND with receive times, at 4.8 Hz and 2.9 Hz. Tests: `test_slam_v6.py::test_slow_pi_and_wifi_bursts_with_room_spins`, `test_lost_after_bad_scans_and_recovers`.
- **F27 – Same person got several IDs; survivors placed outside the arena (FIXED).** 12 "confirmed" survivors (S1–S12), all grid "OUT", clusters within ~1 m of each other — caused mainly by F26 (wrong poses), plus association only by a 0.8 m distance gate. Fix: survivors are placed only with a trusted SLAM pose (TRACKING, accepted, score ≥ 0.3); positions outside the arena (+0.25 m) are rejected; a Windows camera track that fed S2 in the last 3 s keeps feeding S2 (2 m gate); entries closer than 0.75 m are merged and the older ID survives (the other is listed as "also seen as"). Tests in `test_survivor_mapper.py`, `test_mission_server.py`.
- **GCS:** competition grid with the survivor's cell filled red and the drone's cell outlined; map/path clipped to the arena (+1.5 m, switchable); red banner "SLAM LOST" / orange "outside the arena"; candidates hidden by default; ground-truth overlay off by default (its walls looked like a second grid); events for merged IDs; health shows SLAM status and mapper rejections.
- Full fake-Pi dry run after the fixes (real start script, V6 + GAZEBO, synthetic camera+LiDAR, whole rooms route at 1 m/s): SLAM **2271 accepted / 0 rejected**, end at H1; **5 survivors, one ID each, all in the right cell** (K5, E3, K8, L13, H12).
- Not yet re-verified in Gazebo on the real Pi. Also seen in that run: YOLO detected the rendered Gazebo people (survivors confirmed from LIDAR + camera); camera panel showed the Gazebo image.
- **Gazebo re-run with update f on the real Pi (2026-11-02, 10:53–11:07 UTC, arena s1, rooms route).** SLAM V6 on the Pi: TRACKING, 5.8 Hz, score 0.94, 3782 accepted, reject streak 0; map crisp and self-consistent (F26 fix holds on the Pi). Survivor IDs: one ID per person, no duplicates (F27 fix holds). Found 2 of 6.
- **F28 – Pi stack started 133 s after the rig began moving → whole map rotated 90° and shifted in the arena frame (OPEN, procedure).** Evidence: GCS events "mission server NOT CONNECTED" 10:53:27 → "CONNECTED" 10:55:56; rig truth starts 10:53:37 on the pad; best fit of the GCS end pose (x 2.49, y −2.13, yaw −178.9°) to the truth end pose gives SLAM start at 10:55:50 with the rig at (6.57, 5.51) facing west (fit error 0.09). The mission server assumes SLAM started on the pad (7.9, −0.75, 90°). Consequences: S1 (room R6) was visited before SLAM started; S3, S5, S6 were measured but their (rotated) positions fell outside the arena and were rejected ("outside arena 39"); the two that stayed inside were mapped correctly once the offset is undone: GCS "S1 M5" = true S4 A11 (predicted 13.39, 5.49; shown 13.37, 5.03) and GCS "S2 E1" = true S2 E3 (predicted 5.04, 1.06; shown 5.09, 0.86). Fix now: start the Pi stack and wait for SLAM TRACKING with the rig still on the pad, start the rig driver last. Proposed safeguards (not built): pre-flight READY check in the GCS (SLAM TRACKING, pose within 0.2 m of start, all inputs fresh) and/or rig driver waits for it; later, register the SLAM map to the known 14.75 m arena boundary to recover a wrong start pose. Note: `logs/pi_run_f` files were copied at 10:54, before the stack started — they belong to an earlier session; the SLAM trajectory of this run was not copied.
- PX4 panel "not connected" in this run is expected: the rig world (`run_rig.sh`) has no PX4 SITL and no uXRCE-DDS agent; PX4 telemetry needs Stage C (`run_px4.sh` + `MicroXRCEAgent udp4 -p 8888`). AirMouse "no messages": `nidar_bridge` was not running (not part of this test).

---

## 1f. Second Gazebo run on the Pi (2026-11-02 11:14–11:27 UTC) and update 2026-11-02g

Evidence: GCS screenshot + `survivors.json/.csv` (export 11:27), rig truth `logs/rig_truth_20261102_111432.csv`.
(`logs/pi_run_g` was copied at 11:14, before the run: not this run's Pi files.)

| Truth | Result | Error |
|---|---|---|
| S1 K5 lying | S1 CONFIRMED K5 | 0.08 m |
| S2 E3 standing | S2 CONFIRMED E3 | 0.22 m |
| S5 K8 standing | S3 CONFIRMED K8 | 0.14 m |
| S6 L13 standing | S4 CONFIRMED L13 | 0.38 m |
| S3 H12 standing | **S5 CONFIRMED H11 (wrong cell, 1.0 m off, spread 0.88 m, 8 obs)**; correct H12 candidate (5 obs) unconfirmed | 1.0 m |
| S4 A11 lying | candidate only (3 obs, 0.24 m off) — not confirmed | — |

SLAM V6 on the Pi: TRACKING, 5.5 Hz, score 0.96, 4439 accepted, reject streak 0, end error vs truth **0.04 m**;
no duplicate IDs, no "outside arena" rejections (F26, F27, F28 procedure all held). PX4 panel "not connected" is
expected in the rig world (no PX4).

- **F29 – one wrong-cell confirmation from a mixed cluster (FIXED offline, not yet re-run in Gazebo).** The survivor
  position was the plain mean of all its measurements; a few wrong ranges / half-body boxes (close-range view from
  the room centre, 0.57 m from S3) pulled it 1 m into the next cell, and it confirmed with 0.88 m spread.
  2026-11-02g mapper: robust centre (median → mean of measurements within 0.5 m), confirmation needs ≥ 5 inliers
  over ≥ 0.8 s, inlier spread ≤ 0.45 m and ≥ 60 % of the entry's measurements agreeing; range ghosts (an entry seen
  only along camera rays through another entry = wrong range on the same ray) are absorbed without moving the
  person; boxes cut by the left/right image edge use the visible edge + LiDAR range (standing) or are skipped;
  points inside/behind an outer wall are rejected. Offline fault injection (box jitter, half-body boxes, bursts of
  wrong V18 ranges), arenas s1–s3, faults 2+3: **wrong/duplicate confirmations 15 → 3**, correct 28 → 27 of 36,
  mean error 0.21–0.31 → 0.06–0.10 m; fault-free runs unchanged (5/6 each). Every measurement is now written to
  `outputs/mission/measurements_<time>.csv` (inputs + outcome) for replay of the next Gazebo run.
- **F30 – lying survivors seen only up close are missed (OPEN, design decision).** The level camera at 0.82 m cannot
  see the floor closer than ~1.6 m (vertical half-FOV 27.7°) and the LiDAR plane passes over a lying body; the rig's
  room look-around stands 0.5–1 m from the survivor. Options: tilt the camera down ~15–20° (needs the tilt in the
  ground-range/bearing maths and the Gazebo models), or look around from the doorway. Not changed yet.
- **Visual update (no effect on sensing geometry):** colourful worlds (room colours, golden grid, navy outer walls,
  grass, H pad, sky; shadows off by default, `--shadows` option) — collision boxes identical, wall visual pieces tile
  the old walls exactly (`tests/test_gazebo_worlds.py`); rig spawned as `abhishek_drone` with livery + name plate +
  nav lights (topics pinned to `/model/nidar_rig/*`); GCS redesigned ("ABHISHEK DRONE" header, status pills, camera
  HUD, quadcopter marker, survivor halos, compass/scale bar, larger text; `--drone-name`). Not yet seen in Gazebo.
- Tests: 179 passed, 3 xfailed (SLAM-01, F22, F24). Pi bundle `nidar_pi_install_2026-11-02e.tgz`.

---

## 1g. Third Gazebo run, update g (2026-11-02 12:20–12:36 UTC) — results and plan

Evidence: GCS export 12:35 (`survivors.json/.csv`, screenshot), rig truth `logs/rig_truth_20261102_122010.csv`.
Pi files of this run were NOT copied yet (the measurement journal is still on the Pi in `outputs/mission/`).
New coloured world and ABHISHEK DRONE GCS ran (sky/grass visible in the camera).

| Truth | Result | Error |
|---|---|---|
| S1 K5 lying | S1 K5 | 0.08 m |
| S2 E3 | S4 E3 | 0.21 m |
| S4 A11 lying | S3 A11 (**found this time**) | 0.38 m |
| S5 K8 | S5 K8 | 0.07 m |
| S6 L13 | S6 L13 | 0.28 m |
| S3 H12 | S8 H12 | 0.12 m |
| — | **S2 I4 false**: 5 obs in 0.9 s, YOLO conf ≤ 0.54, 2.0 m from S1; rig in room R6 (K4) during the look-around, point at the right image edge (−35…−40°), S1 lying at −90° (outside the view) | — |
| — | **S7 H10 false**: 1.7 m from the H12 person; now only 3 of 17 measurements agree (confirmed early, evidence fell apart); rig in I13 looking south through the door, point straight ahead at 2.7–3.4 m, true person at −17…−45° and 1.3–1.9 m | — |

6/6 real survivors in the correct cell (mean 0.19 m), 2 false confirmations. SLAM V6: TRACKING, score 0.97,
4711 accepted / 0 rejected, end error 0.06 m; 4.7 Hz (5.5 Hz in the previous run).

- **F31 – two false confirmations (OPEN, needs the journal).** Cause not yet known: bearing error (half-visible body
  at the image edge), wrong LiDAR range, or camera/pose timing. Next: replay `measurements_<time>.csv` against the rig
  truth. Candidate fixes after that: confirmation needs ≥ 1.5 s or two separate looks; a confirmed entry whose inlier
  share drops below ~50 % is shown as UNCERTAIN; camera capture time carried through the chain if timing is the cause.
- **F32 – SLAM rate 5.5 → 4.7 Hz (CHECK).** Possibly lower Gazebo LiDAR/camera rate with the new visuals (sky, more
  wall pieces) or more mission-server CPU on the Pi. Measure `ros2 topic hz /nidar/scan` and `/nidar/camera/image`
  with the rig on the pad, and `top` on the Pi.

**Plan (next session):** 1) copy this run's Pi files (journal); 2) sensor rates + Pi CPU; 3) targeted fix for F31 +
offline replay; 4) repeat in arenas s2 and s3 (only s1 tested in Gazebo so far); 5) camera tilt decision (F30);
6) PX4 Stage C (SITL + uXRCE-DDS telemetry in the GCS, no arming).

---

## 1h. 2026-11-09 — PX4 route flyer (simulation only)

- Run 2026-11-09 04:36 (rig, s1, update g): 4 correct (A11, E3, K8, L13), H12 person again in the wrong cell (G10, 1.8 m),
  K5 lying missed; SLAM 5.1 Hz, end error 0.01 m. Pi files of that run not copied yet; `logs/pi_run_h` = the 2 Nov run.
- PX4 "not connected" in rig runs explained: `run_rig.sh` has no PX4/agent. Stage C commands given; not yet confirmed working.
- New `gazebo/ros2/nidar_px4_route_flyer.py`: flies the PX4 x500_nidar_airmouse through the arena in OFFBOARD position
  mode (SITL only — refuses without the arena's Gazebo world on this machine). Corridor centrelines, stop-and-turn,
  0.64 m altitude, 0.3 m/s, 30°/s yaw; room probes stop on the LiDAR (0.45 m ahead) because survivors can stand in a
  door (s2 R7 0.10 m, s3 R7 0.16 m from the door-cell centre); guards: obstacle < 0.30 m, cross-track > 0.30 m,
  tracking > 0.8 m, no progress → LAND; Ctrl+C → LAND; leaves PX4 alone if it exits OFFBOARD. Separate from
  nidar_bridge/px4_setpoint_sim (ALLOW_ARM/ALLOW_OFFBOARD stay False there).
- Verified offline only (no ROS/PX4 in the analysis workspace): simulated vehicle + ray-cast LiDAR, whole rooms route
  s1/s2/s3: lands on the pad, 14–17 min, wall clearance 0.50 m (prop tips 0.39 m), standing-survivor clearance
  ≥ 0.64 m, one probe stop (s3 R7). `tests/test_px4_route_flyer.py` (19 tests). **Not yet run against real PX4 SITL.**
  Unverified on the user's machine: topic names (`/fmu/in/*`, versioned `/fmu/out/*` found by prefix), arming in
  OFFBOARD without RC, how closely PX4 tracks in the narrow corridors.
- **First PX4 SITL flights (2026-11-09 05:28–05:36, user's WSL):** PX4 telemetry reaches ROS 2 and the GCS (**Stage C
  telemetry verified**: disarmed/OFFBOARD/local position/attitude/battery shown). PX4 1.18 topics: `/fmu/out/vehicle_status_v4`,
  `vehicle_local_position_v1`, `vehicle_command_ack_v1`, `failsafe_flags`; inputs unversioned `/fmu/in/{offboard_control_mode,
  trajectory_setpoint,vehicle_command}`. Arming was TEMPORARILY_REJECTED (flags: manual_control_signal_lost,
  gcs_connection_lost, gnss_lost) until the user set, in the SIMULATED PX4 only, `COM_RC_IN_MODE 4, NAV_RCL_ACT 0,
  NAV_DLL_ACT 0, COM_RCL_EXCEPT 4` (must never go to a real flight controller; undo with `param reset_all`). Then:
  ARMED in OFFBOARD, take-off, legs pad→H1→G1→G2→J2→J3 flown, each corner reached within 0.07–0.1 m.
  Problems found and fixed in the flyer: rclpy one-severity-per-call-site crash; 0.5 s position-gap limit too strict
  (periodic 1.7–1.8 s gaps every ~34 s under WSL load → now hold ≤ 2 s, land beyond); stop-and-go at 0.12 m/s because
  PX4 lags a moving position setpoint by ~0.32 m (MPC_XY_P) → velocity feed-forward + pause at 0.45 m; fixed per-leg
  time limit landed a slow but moving drone → progress-based watchdog (15 s without movement/turning).
  SLAM map from the PX4 flight shows radial streaks through walls (not seen with the rig): open, needs the Pi files of
  that flight + the flight log; candidate causes: the 2-D LiDAR tilting with the drone, scans during take-off/landing
  at 0.36 m, sim stalls. `gnss_lost` with a valid local position also still unexplained.

---

## 1i. PX4 SITL flight 2026-11-09 05:49–05:51 UTC (arena s1, `--route survivor:S2`) — uncommanded spin at F3, SLAM LOST

Evidence: GCS screenshot + `survivors.json/.csv` (export 05:51:58 UTC); flyer log `logs/px4_flight_20261109_054926.csv`
(PX4 position in the arena frame, 20 Hz). Route identified by matching the logged task sequence against `--dry-run`:
s1 `survivor:S2` matches tasks 0–6 exactly (s1 `rooms`, s2, s3 do not). **Not yet available:** PX4 ulog, flyer console
output, Pi files of this flight (`logs/pi_px4_1109` was copied at 05:42, before it).

| UTC | t (s) | Event |
|---|---|---|
| 05:49:28.7 | 0 | flyer log starts on the pad |
| 05:49:53 | 24.5 | airborne 0.58 m → ROUTE |
| — | 29.8–75.1 | H1, G1, G2, J2, J3 reached; GCS SLAM path follows the same legs |
| 05:50:39.9–45.7 | 71.2–77.0 | S1 measured 8× → CONFIRMED K5 (11.106, 4.664); truth (11.048, 4.743): **0.10 m** |
| 05:51:06.8 | 98.2 | F3 reached; task 6 = turn on the spot 180° → −90° (CCW) then F3 → F1 |
| 05:51:08.4 | 99.75 | log pose (5.717, 2.664, −167.8°) = SLAM's held pose on the GCS (5.75, 2.58, −167.7°) within 0.09 m / 0.1° → **SLAM tracked correctly up to here** |
| 05:51:08.9 | 100.25 | **uncommanded clockwise spin** (commanded turn was CCW at 30°/s): −467° in 1.5 s, several hundred °/s, 0.80 m drift north, altitude 0.55 → 0.37 m |
| 05:51:10.4 | 101.75 | flyer guard: tracking error 0.81 m > 0.80 → LAND (log stops when LANDING) |
| ≤ 05:51:58 | — | GCS: disarmed on the ground at arena (5.48, 3.59), heading −143° ENU (PX4 NED 4.34, −2.42, yaw −126.7°); SLAM LOST, held at F3 → 1.05 m / 24° from PX4. Mode still shown OFFBOARD (NAV_LAND expected — unexplained) |

- **F33 – Uncommanded yaw spin during a turn on the spot at F3 (OPEN, SAFETY, cause not proven).** Leading hypothesis:
  prop strike on room R5's east wall (x = 5.275, 0.05 m thick). The drone overshot the westward leg and turned
  0.092 m west of the corridor centreline: centre-to-wall-face 0.408 m vs prop-tip reach 0.39 m (flyer constant) →
  ≤ 2 cm margin, the closest approach to any wall in the whole flight. Signature (spin + height loss + sideways drift,
  starting while an arm swept toward the wall) fits losing one rotor. Not excluded: sim/physics glitch under WSL load
  (no position gap in the log there), estimator yaw jump (unlikely: SLAM lost lock at the same moment on an
  independent sensor). Decisive data: PX4 ulog at 05:51:08.9 (yaw-rate setpoint vs measured, motor outputs).
  Flyer design gaps found in the code (not changed yet): (a) corner "align" turns have no clearance check
  (`TURN_CLEAR_M` is only applied to look turns) and do not wait for the drone to settle back on the cell centre;
  (b) `EMERGENCY_M` 0.30 < prop reach 0.39 → the LiDAR guard can only fire after the props reach a wall;
  (c) `TURN_CLEAR_M` 0.40 leaves 1 cm over the prop reach; (d) no heading-error / yaw-rate guard — the spin ran
  1.5 s until the 0.8 m tracking guard fired.
- **F34 – SLAM V6 cannot re-acquire after losing lock (CONFIRMED from code).** With no accepted hypothesis it holds
  the last good pose; recovery searches ±45° / ±0.30 m around it (`WIDE_SEARCH`), and `_plausible` limits any result
  to V_MAX·dt + 0.15 m and W_MAX·dt + 0.10 rad from the held pose, with dt = time since the previous scan (not since
  the last accepted one) → ≈ 0.45 m / 23° at 5 Hz. Here the vehicle ended 1.05 m / 24° away → LOST for good. No
  5 Hz scan matcher could follow this spin; the defect is the missing recovery. Fix direction: gate against time
  since the last accepted scan + global relocalisation on the existing map, accepted only with a clearly good score
  confirmed over several scans.
- Worked as designed: SLAM declared LOST; GCS red banner; mission server stopped placing survivors ("bad pose 119"
  counts V18 frames skipped, not people); no ghost / outside-arena survivors.
- GCS cosmetic: "PERSONS IN VIEW" = `len(self.objects)` (every class), so a 0.23 "bed" on the floor showed as
  1 person. The mapper uses class "person" only (`SURVIVOR_CLASSES`), so no mapping risk. Fix: count persons only.
- Camera panel 4.3 FPS (rig runs 8.4 Hz): lower rate with PX4 SITL running too — part of F32, not measured.
- Still open from 1h: radial streaks in the SLAM map (visible again); needs this flight's Pi SLAM trajectory.
- Minor: S1 (lying) was LiDAR-ranged in all 8 measurements with none rejected, yet 0.10 m from truth — check in the
  journal what the beam hit (F22 expects it to pass over a lying person).

**Next:** 1) copy the PX4 ulog + flyer console of this flight → decide F33; 2) copy the Pi files (SLAM trajectory,
`measurements_*.csv`); 3) only then: flyer turn-clearance fix (F33 a–d) and SLAM recovery (F34), each with a test.

---

## 1j. 2026-11-09 — Autonomous exploration of an unknown arena (the "brain")

Request: the drone maps any arena by itself, without collisions, with no prior knowledge (not even its size), brain on
the Pi; GPS-denied navigation with optical flow + rangefinder + SLAM; sensor not chosen. Staged: **(1) brain on the Pi
[done, offline-verified] → (2) the flight side follows it: Gazebo rig [done, offline-verified], PX4 [next] → (3) PX4
without GPS: flow + rangefinder in SITL, SLAM pose into EKF2 [not started].** Note: optical flow keeps the drone from
*drifting* (velocity over the floor) when there is no GPS; it does not see walls — the LiDAR + planner do that.

- `src/mission/nidar_explorer.py` (**NEW**): frontier exploration from the SLAM V6 grid + pose only. Targets = LiDAR
  frontiers (free next to unknown) + camera frontiers (free cells the camera has not looked at), only inside the walls
  seen so far; viewpoints ≤ 0.8 m from a target through free space; Dijkstra over cells ≥ radius + 0.05 m from walls,
  pulled to corridor middles; straight legs only with ≥ 0.10 m extra clearance; turn while moving; at a viewpoint
  sweep only the arc with unseen space; sticky walls + raw LiDAR hits (thin walls eroded by grazing beams were planned
  through — fixed); visited viewpoints (a loop — fixed); nearest safe cell searched through free space only (escaped
  through a wall — fixed). After 20 min (or nothing left): back to the take-off point, land.
- `src/mission/nidar_explorer_service.py` (**NEW**, Pi): protocol `NIDAR_NAV_V1`, TCP 5030. Plans in the SLAM frame
  (plan / cancel / hold / pose ↔ status FLYING/DONE/BLOCKED). HOLD (and cancel) whenever SLAM is > 1 s old, LOST or
  not accepted; raw hits only from the scan SLAM used for that pose (score ≥ 0.5); re-checks the remaining plan at 1 Hz
  against new obstacles → cancel + new plan; a reconnecting flight side gets the same plan again. Writes
  `outputs/mission/explorer_state.json`. Started by hand (start_nidar_pi.sh unchanged).
- `gazebo/ros2/nidar_rig_driver.py --brain HOST` (WSL): flies the plans with the rig. Aligns SLAM → odometry ONCE from
  the brain's pose while standing still ≥ 1 s; legs face the direction of travel; looks = turn in place. Stands still
  with no brain / no odometry / no fresh LiDAR / HOLD; LiDAR < 0.30 m in the direction of motion → stop, BLOCKED.
- Mission server + GCS: snapshot field `explorer` (state, plan in the arena frame, events); GCS pill **BRAIN**, dashed
  magenta plan + LOOK/GOAL/LAND marker, HOLD / MISSION COMPLETE banner, `BRAIN …` events. Without `NIDAR_ARENA_LAYOUT`
  the arena is treated as unknown: no cell labels (coordinates instead), no "outside arena" rejection of survivors
  (before: anything at negative SLAM x/y was rejected), the map view fits the mapped area.
- **Evidence (VERIFIED-SIM, offline only):** `tests/explorer_sim.py` (radius 0.25, 20 min): s1–s3 and new random arenas
  (generator seeds 4–9): camera coverage 0.95–0.98, 6/6 survivors in view, landed 0.07–0.09 m from the pad; full SLAM
  V6 estimate (s1): 0.985, 6/6; closest wall approach 0.30–0.33 m. `tests/nav_loop_sim.py` (real Brain + BrainFollower
  over JSON, rotated odometry frame): s1/s2 complete in 19–20 min, coverage 0.98, 6/6, back on the pad 0.04 m, min
  clearance 0.30 m, 0 BLOCKED. Planning cost 0.06 s mean / 0.13 s max on the analysis PC; **Pi 3B+ not measured**.
  Tests: 225 passed, 3 xfailed (fast 209 + slow 16).
- **UNVERIFIED:** a Gazebo run with the Pi (Stage E in `gazebo/README.md`); brain CPU load next to SLAM on the Pi.
  **NOT IMPLEMENTED:** PX4 follow mode; flow + rangefinder (GPS-denied) SITL; EKF2 fusion of the SLAM pose.
- The x500 (prop tips 0.39 m) cannot keep clear in 1 m corridors (explorer sim with radius 0.40: 0.35–0.37 m to the wall
  face; F33's spin happened at 0.408 m): the real drone must be ≤ ~0.25 m radius, or the corridors wider.
- With SLAM LOST the brain holds (and with F34, for good): safe, but the mission stops — F34 must be fixed before PX4
  follow mode.
- GCS: "PERSONS IN VIEW" now counts class person only (§1i cosmetic note).
- Bundle `nidar_pi_install_2026-11-09.tgz` (installer accepts the 2026-11-02e files).


## 1k. First autonomous Gazebo run, 2026-11-09 06:55–07:19 UTC (rig, s1, brain on the Pi) — findings and fixes

Evidence: GCS screenshot + `gcs_reports/20261109_071905` (survivors), rig ground truth
`logs/rig_truth_20261109_065546.csv`. Not available: brain/rig consoles, Pi files (SLAM trajectory, journal).

- **Autonomy worked (VERIFIED-SIM, Gazebo):** explored with no route and no arena knowledge, 125 plans, MISSION COMPLETE,
  rig back on the pad 0.05 m; SLAM TRACKING 5.1 Hz, end error 0.08 m. The Pi brain kept up: idle 0.8 s per plan on
  average (162 s in 23.6 min). Time: translating 711 s, turning in place 437 s, still 228 s; path 150 m.
- **Survivors 3 of 6 right:** K5 lying 0.27 m (first time found), A11 lying 0.15 m, L13 0.16 m. H12 (standing) reported
  twice, both wrong cells: H13 (0.71 m) and H10 (1.80 m, 22/43 inliers) — F31 again, needs the journal of this run.
  **E3 and K8 were never in camera view** (truth trajectory + ray-cast): rooms R5 and R2 were not visited.
- **Why:** Gazebo ran at ≈ 0.5× real time (odometry 14.4 of 30 Hz, rig 0.21–0.25 m/s of 0.4 commanded, yaw
  17–32 °/s, camera 4.1 FPS) while `--mission-min 20` counts real minutes → ~10 simulated minutes; it returned at
  the time limit. The GCS banner said "explored" → fixed: shows TIME LIMIT and "not everything explored".
  Gazebo runs: `--mission-min 45`.
- **F35 – closest approach below the drone radius (OPEN, SAFETY).** 0.217 m from the end of wall x = 13.675
  (y 6.325–8.425) at t = 1267 s (planned ≥ 0.30 m). Reproduced offline with the full SLAM V6 estimate in the protocol
  loop (`tests/nav_loop_sim.py`, mode "slam"): the rig's alignment was fixed at start-up from SLAM's first pose
  (0.7° off) → up to 0.16 m off the plan 13 m from the pad → 0.224 m, and a BLOCKED livelock (14 729 BLOCKED, coverage
  0.46, never finished). Fixed: the rig flies in SLAM's frame and blends its alignment from every brain pose (2 Hz,
  scan age compensated, only while turning < 6 °/s, jumps > 0.5 m / 5° ignored); the brain HOLDs for good after 5
  BLOCKED in a row. After the fix (same test): complete, coverage 0.985, 6/6, 7 BLOCKED, pad 0.04 m — but closest
  wall still 0.213 m: SLAM's own error (up to 0.11 m) exceeds the 0.05 m map margin. With `--radius 0.30`
  (planning radius 0.35 m for the 0.25 m rig): SLAM mode complete, 0.98, 6/6, 1 BLOCKED, **closest 0.279 m**; map
  mode s1–s3 0.97–0.985, 6/6, 0.34 m, 17–19 min. Interim rule: radius + 0.05 m. Real fix (before PX4): margin from
  measured SLAM error, LiDAR guard ≥ prop reach (also F33 b).
- **PX4 / ground stations:** the rig run has no PX4 by design (`run_rig.sh`); the brain cannot fly PX4 yet (follow mode
  NOT IMPLEMENTED). QGroundControl is PX4's ground station (Mission Planner is ArduPilot's — not used): `gazebo/README.md`
  Stage C3 (QGC in WSL auto-connects; QGC on Windows → UDP to WSL address port 18570, per PX4's WSL2 docs). Unverified here.
- **`/nidar/airmouse_command` "no messages":** published only by `nidar_bridge` (WSL) from the Pi AirMouse server
  (port 5010): the older follow-a-person command [forward, lateral, vertical, valid]. Not started in Stage E, not used
  by the explorer, not sent to PX4 (ALLOW_ARM/ALLOW_OFFBOARD False). F24 still blocks it for control.
- Tests: 228 passed (fast 212 + slow 16), 3 xfailed. SLAM-mode check (25 min, manual): `python3 tests/nav_loop_sim.py gazebo/layouts/nidar_airmouse_arena_s1.json slam`. Bundle `nidar_pi_install_2026-11-09b.tgz`.


## 1l. 2026-11-09 — one-command launcher, link-loss reporting, small fixes

User request: no more nine terminals; a "communication lost" error during a test; BRAIN "off" in the PX4 run.
- `tools/nidar_run.sh` (**NEW**, WSL): `setup` (password-free SSH to the Pi, QGroundControl, copy of PX4's x500 /
  x500_flow / optical_flow / LW20 files + airframes + dds_topics to `gazebo_existing/files/px4_models` for the next
  step), `update-pi`, run modes `rig` (brain) and `px4` (route flyer + QGC), `stop`. Starts the Pi stack + brain over
  SSH, Gazebo headless by default, relays, AirMouse bridge, YOLO (`tools/nidar_yolo.ps1`, own window), GCS, QGC;
  waits for SLAM TRACKING; supervises (restarts relays/bridge/rig driver ≤ 5×; never the PX4 flyer); Ctrl+C =
  flight program first (flyer: LAND, 15 s), Pi stop + copy of its run files, then the rest, then YOLO. PX4 mode sets
  the SITL-only RC/data-link parameters through `px4-param` (simulated PX4 only).
- Bug found while testing it: bash starts background jobs with SIGINT ignored, so Ctrl+C would never have reached
  the rig driver (zero velocity) or the flyer (LAND) — components are started through `env --default-signal=INT,QUIT`
  (test checks `/proc/<pid>/status` SigIgn).
- `tools/nidar_status.py` (**NEW**): reads the mission server like the GCS; one status line + plain events
  (`LOST:/BACK:` Pi link, LiDAR scans, camera detections, fusion, SLAM; SLAM/BRAIN state changes, survivors, MISSION
  COMPLETE) + Gazebo speed (real-time factor). **"Communication lost": the exact message was not captured** — the
  likely causes are covered (WSL overload: Gazebo GUI off by default, the PX4 data-link failsafe parameter set, QGC
  connected), and the next occurrence is named in `logs/run_*/events.log`.
- GCS: BRAIN pill shows "PX4 route" in PX4 runs (the brain cannot fly PX4 yet) instead of "off"; pose on the launch
  pad labelled PAD (was "OUT", mission server). `gazebo/run_px4.sh`: `PX4_DAEMON=1` (no pxh shell).
- Tests: `tests/test_nidar_run.py` runs the launcher for real with stand-in components, fake ssh/scp/powershell and a
  fake mission server (start order, restart, MISSION COMPLETE, stop order, Pi stopped + files fetched, no process
  left, refusal without SSH key). **UNVERIFIED on the user's machine** (WSL interop to PowerShell, SSH key, headless
  Gazebo/PX4, px4-param client). Bundle `nidar_pi_install_2026-11-09c.tgz` (mission server PAD label).
- **First real launcher runs (09:08–09:13) stopped at step 2:** `a && b &` over SSH put the whole list (holding SSH's
  output) in the background, so `ssh` never returned; and the remote stop command had a line break inside a `for`
  list (syntax error: the Pi processes were not all stopped). Fixed (`(… &)`, continuation lines); the test's fake
  `ssh` now runs the command like a real SSH channel (output piped), which catches both. `stop` also ends a stuck
  launcher (and its ssh child) and recognises only a real launcher in the PID file.


## 1m. 2026-11-09 PX4 runs with the launcher (09:49–10:16) — findings

- 09:55 run: **crash** at the D10 corner (2.65, 10.0): overshot 0.11 m south, the flyer turned on the spot at once,
  0.39 m from the wall = the x500's prop reach → uncommanded spin (yaw −113° → +62° → +160° in ~0.75 s), 0.7 m
  drift, flipped (roll −179°, "Imbalanced propeller", "Accel clipping"); SLAM LOST afterwards. Same as F33; F33 (a)–(d)
  still open. Needs the smaller sim drone (PX4 model files now copied to `gazebo_existing/files/px4_models` by
  `setup`) and the flyer corner fix.
- 10:11 run (`--no-qgc`): landed after "no PX4 position for 2.0 s" (gap 2.01 s); S1 K5 found 0.17 m from truth;
  a floor "bed" (0.60) was boxed but not counted as a person (GCS fix works); SLAM = PX4 within 0.07 m.
- **F36 – Gazebo in WSL pauses 1.7–2.8 s every ~35 s (OPEN, cause unknown).** In every run with and without PX4,
  QGC or the Gazebo window (rig 06:55: 41 pauses, spacing 34.6–35.1 s; the rig moved 0.01 m in a 2.5 s pause while
  driving at 0.2 m/s → the whole simulation stops, not just a receiver). Flyer: a gap with the LiDAR silent too is
  now waited out up to 8 s (`gap_action`); a gap with the LiDAR still arriving still lands at 2 s.
- The launcher's "Gazebo speed" came from /stats `real_time_factor` and said 0.03–0.12x; the LiDAR (10 Hz simulated)
  arrived at 4.2–4.5 Hz, i.e. ~0.43x in rig and PX4 runs alike. My "QGC / Gazebo window cause the slowness" advice was
  wrong. Now measured from simulated time between two readings (`SpeedMeter`).
- BRAIN flickered off/on in the 09:24 rig run (brain state > 2 s old at times): mission server waits 5 s now.
- Bundle `nidar_pi_install_2026-11-09d.tgz` (mission server).
- 10:25 run: landed again after "no PX4 position for 2.2 s (LiDAR still arriving)" — the 5th periodic pause
  (gaps at 11.0, 45.5, 79.9, 114.3, ~148.8 s: every ~34.4 s). After a pause the LiDAR resumes a moment before PX4's
  position, so the new rule saw "LiDAR alive, PX4 silent". Now: if the LiDAR was silent during the gap, PX4 gets
  2 s after the LiDAR is back (`RESUME_GRACE_S`). "Disarmed" = PX4's normal auto-disarm after landing; QGC's
  "communication lost" at the end = PX4 shut down by the launcher's Ctrl+C ("PX4 Exiting"). S1 K5 0.04 m from truth.
- 10:39 run: the pause fix held (9 pauses survived); landed on "unexpected obstacle 0.43 m ahead in a corridor" at
  (5.73, 6.87) right after the corner turn west→north. Ray-cast truth there: 0.57 m ahead, the side wall 0.43 m at
  +77° → the LiDAR scan used was from mid-turn (the scan arrives up to ~1 s behind PX4's heading). QGC "No offboard
  signal" = the flyer had stopped after landing; no QGC setting involved. Fixes (flyer, F33 a): yaw on the spot only
  while within 0.06 m of the turn point and not drifting (the yaw holds otherwise, also mid-turn); after a turn the
  leg starts only after 1.5 s of steady heading. Closed-loop s1–s3: land on the pad, 16.5–19.2 min (was 14–17),
  wall clearance 0.485 m. F33 (b)–(d) and the x500 width still open.


## 1n. 2026-11-10 — AirMouse ground station app (`gcs/airmouse_gcs.py`), watch-only

- New desktop app (PyQt5, WSL) with four screens; the old window `gcs/nidar_gcs.py` is unchanged and still works
  (launcher: `--classic-gcs`). **Watch-only: it sends no command to the drone** (same as before).
  - **Fly**: full-size camera (or map while there is no video) + picture-in-picture (click / Space swaps); HUD with
    autonomy state + current plan, live alerts (SLAM lost, stale LiDAR/camera/fusion, brain HOLD), survivors found
    with mission time, events, telemetry bar (cell, x, y, compass heading, speed from SLAM poses, distance home,
    PX4 altitude when PX4 runs, SLAM score), heading-up compass with the take-off point, "survivor found" toasts.
  - **Checklist**: 15–16 pre-flight checks from live data (`gcs/airmouse/checks.py`), REQUIRED vs OPTIONAL,
    one verdict READY / READY WITH WARNINGS / NOT READY. Required: Pi link, LiDAR, camera detections, V18,
    SLAM tracking / ≥ 4 Hz / score ≥ 0.30, brain (rig mode) or PX4 telemetry (PX4 mode), battery when reported.
  - **Survivors**: tiles (confirmed, candidates, mission time, distance, mapped area), table, map with the
    selected survivor, camera snapshot saved when a survivor is confirmed (only when video is live),
    **Export CSV / PDF report** (`gcs/airmouse/report.py`, Qt PDF writer, no extra packages).
  - **Logs**: every connected flight is recorded (`flight.jsonl.gz`, states at 2 Hz + every map; launcher runs:
    `logs/run_*/gcs/`, otherwise `gcs_records/<time>/`); replay with timeline (survivor marks), 1–64× speed,
    events list (click to jump), report export for a past flight. A record cut off by a crash still replays.
- Same event logic for live and replay (`gcs/airmouse/model.py`, reuses `tools/nidar_status.py`).
- **Demo record** `gcs_records/demo_s1_offline/` (made by `tools/make_demo_record.py` from the offline closed loop:
  SLAM V6 + brain + follower, arena s1, 19.4 min, 98.5 % searched, 6/6 seen). Limits written into the record: no
  camera pipeline in that run (camera/V18 show "no data"), survivors placed at their simulated positions when first in
  camera view. Try: `python3 gcs/airmouse_gcs.py --demo gcs_records/demo_s1_offline/flight.jsonl.gz --no-ros`.
- Tests: `tests/test_airmouse_gcs.py` (12 fast + 1 slow). Fast suite 234 passed, 2 xfailed.
- NOT verified yet: the app on the real WSL desktop with a live Pi run (only offscreen screenshots here); camera
  snapshots (need the Gazebo camera over ROS 2); PX4 battery/altitude fields with PX4 running.


## 1p. 2026-11-11 — the project is on GitHub

- **https://github.com/abhi-gupta2005/NIDAR_DRONE_PROJECT** — public, branch `main`, first commit `251032c`
  (193 files, ~21 MB: code, tests, Gazebo worlds/models, ROS 2 package source, installers, `yoloe-26n-seg.pt`,
  `models/nidar_aerial_yolo26n_cpu15_640.pt`, the demo flight record). The git repository is
  `D:\NIDAR_DRONE_PROJECT\.git` (remote `origin`). Not uploaded (`.gitignore`): logs, run outputs, videos, `.tgz`
  bundles, `.venv`, `mobileclip2_b.ts`, `archive/`, `gazebo_existing/`, `pi_files/`, `Claude outputs/`.
- Before uploading: no passwords, tokens or private keys found in the uploaded files (only the Pi's private LAN
  address and user name as launcher defaults).
- Placeholders filled in `tools/install_windows.ps1`, `README.md`, `docs/SETUP.md`.
- **Verified:** a fresh clone from GitHub has the 193 files, LF endings in every `.sh`, and `tests/test_nidar_run.py`
  passes on it (6 passed); the friend's raw file `tools/install_windows.ps1` downloads with the real repo address.
- **Friend's first command** (PowerShell as administrator):
  `irm https://raw.githubusercontent.com/abhi-gupta2005/NIDAR_DRONE_PROJECT/main/tools/install_windows.ps1 | iex`
- **Uploading later changes:** WSL `bash /mnt/d/NIDAR_DRONE_PROJECT/tools/publish_github.sh` (logs in to GitHub once
  in WSL, then commits and pushes). The login used for this first upload was in a temporary workspace and was removed.
- Still **not verified:** the install on the friend's laptop and Pi.

## 1o. 2026-11-11 — installing everything on a new laptop + Pi (GitHub, 4 commands)

- **Owner, once:** `tools/publish_github.sh` (git + GitHub CLI, browser login, creates the repo, writes its address into
  the installer/README/SETUP, pushes; later runs push changes). `.gitignore` keeps logs, outputs, videos, bundles,
  `.venv`, `mobileclip2_b.ts` (254 MB > GitHub limit) and `ros2_ws_src/px4_msgs` out; `.gitattributes` pins LF for
  `.sh/.py` (a Windows clone would otherwise break bash with `$'\r'`).
- **Windows** (`tools/install_windows.ps1`, `irm …/install_windows.ps1 | iex` as admin): Git, clone to
  `D:\NIDAR_DRONE_PROJECT` (C: if no D:), Python 3.12 + `requirements-windows.txt` (ultralytics, OpenCV, numpy),
  WSL 2 + Ubuntu 24.04, prints the WSL command.
- **WSL** (`tools/install_wsl.sh`, ~1 h, re-runnable): ROS 2 Jazzy desktop + ros_gz + cv_bridge + colcon; PyQt5 etc.;
  **PX4 pinned to 9fda8a51a4952dd6964fb7e2c86e0a4f748f740b** (the user's v1.18.0-beta1-956) + `Tools/setup/ubuntu.sh
  --no-nuttx` + `make px4_sitl`; Micro XRCE-DDS Agent **v2.4.3**; `~/px4_ros2_ws` with **px4_msgs
  0be57f29b494be6f8a81cf428a1c50fad203ca6a** (release/1.18, the user's) + nidar_airmouse (resource marker added);
  `gazebo/install_into_px4.sh`; QGroundControl; `~/.bashrc` block (environment + alias `nidar`); checks + fast tests.
- **Pi** (new): `nidar setup --pi ADDR --user NAME` (key + remembers the Pi in `~/.nidar_pi`, used by every later
  command) then `nidar install-pi` (copies the project to `~/drone_project`, `tools/install_pi_fresh.sh`: OS packages
  numpy/OpenCV/pyserial/PyYAML/pytest, `.venv --system-site-packages`, folders, import check). Refuses a Pi that
  already has the project (`update-pi` for that; `--force` to copy again).
- Guide: `README.md`, `docs/SETUP.md`; video `docs/NIDAR_AirMouse_Installation.mp4` (3 min, voice).
- **Verified here:** shellcheck clean; dry run of install_wsl.sh on Ubuntu 24.04; both pinned commits exist on GitHub;
  px4_msgs checkout + nidar_airmouse copy; Micro XRCE-DDS Agent v2.4.3 builds on Ubuntu 24.04 (gcc 13);
  `install_pi_fresh.sh` in a temp folder (all Pi modules import); launcher `setup`/`install-pi` with a fake Pi
  (`tests/test_nidar_run.py`); PowerShell script parses and its flow runs with mocked commands; publish dry run
  (182 files, ignore rules). **NOT verified:** a real end-to-end install (ROS/Gazebo apt servers are blocked here),
  PX4's ubuntu.sh + build, Windows/WSL install, a real Pi.

---

## 1b. Raspberry Pi snapshot (2026-11-02, `tools/nidar_pi_status.sh`, report in `pi_status.txt`)

| Item | Observed |
|---|---|
| Board / OS | Raspberry Pi **3 Model B+**, 905 MB RAM, Debian 13 (trixie), kernel 6.18.39, Python **3.13.5** (venv) |
| Power | `throttled=0x50000`: **under-voltage has occurred** and **throttling has occurred** since boot (not active at the moment of the snapshot). Weak power supply — can cause slowdowns, Wi-Fi drops, SD-card corruption |
| Network | Wi-Fi "CMF by Nothing Phone 2 Pro_9054" (phone hotspot), 10.218.70.147; old address 10.89.74.147 is gone |
| Disk | 29 GB, 7.3 GB free (74 %). `nidar_v18_headless_persistent_fusion.csv` = **64 MB and growing** (V18 appends every 50 ms loop, no rotation) |
| Packages (venv) | numpy 2.5.3, opencv-python 5.0.0.93, pyserial 3.5, pytest 9.2.0, PyYAML 6.0.2, pyqtgraph 0.14.0; no ultralytics/torch (expected) |
| Code vs Windows copy | SLAM, start script, receiver, LiDAR interface/detector/tracker, simulator, monitor: **identical**. V18: **different** — the Windows copy has an extra `[FUSION DEBUG]` print block; the Pi version (57,913 B, 5 Oct 16:51) does not. V18.1 must be applied to the Pi's own file, not copied from Windows |
| Pi-only files | `nidar_airmouse_server.py` (port 5010), `integration_monitor_WORKING_2026-09-30.py`, `~/nidar_v17_persistent_tracking.py`, stray file `Dockerfiledocker compose build` |
| Running | Nothing (ports listening: 22, 111). Last run today 03:52–03:53: V18 processed **1** frame (the 20-day-old frame 24118) → no fresh perception arrived; `receiver.log` 0 bytes again (F5 confirmed on the Pi) |
| Hardware | `/dev/serial0 → ttyAMA0` present, user in `dialout` → UART ready for the STL-19P; no USB camera; LiDAR not tested |
| Docker | container `nidar` exited (255) 5 weeks ago — unused |
| Label bug | V18 JSON says `camera_source: mac_webcam` although the data came from the Windows webcam (hard-coded string) — **fixed 2026-11-02b** (label taken from the packet) |

## 2. Architecture (actual, as found in the code)

```
 WINDOWS LAPTOP                         RASPBERRY PI (or WSL via start_nidar_wsl.sh)              WSL (ROS 2 Jazzy)                       PX4 SITL
 ───────────────                        ─────────────────────────────────────────                 ─────────────────                       ────────
 webcam ─► nidar_v11_windows.py         nidar_pi_receiver.py  :5000                                nidar_bridge  (client of PI:5010)
           YOLOE-26n-seg + BoT-SORT  ──TCP 5000──►  writes src/communication/                      │ /nidar/airmouse_command
           + StableIdentityManager   NIDAR_PERCEPTION_V1   nidar_latest_perception.json            │ Float32MultiArray
           (UNVERIFIED link)                                │ (file poll, 20 Hz)                   ▼ [fwd, lat, vert, valid]
                                        STL-19P SIMULATOR ─►│                                      px4_setpoint_sim ──/fmu/in/*──► PX4
                                        detector+tracker V3 ▼                                      (ALLOW_OFFBOARD=False,
                                        nidar_v18 (V18.1) ── writes nidar_v18_..._fusion.json       ALLOW_ARM=False)
                                                                 │                                 telemetry ◄──/fmu/out/*── PX4
                                        nidar_airmouse_server.py ◄┘ (fixed) ─TCP 5010─► bridge    (BEST_EFFORT QoS in code)
                                        nidar_slam_v5_1 (SIMULATOR only, output consumed by NOTHING except the monitor)
                                        integration_monitor.py (reads the three JSON files)
```

Coordinate frames (documented for the first time — previously implicit):
- **Camera bearing** (V18): `(center_x − 640)/640 × 35°`, positive = right of image centre. Assumes 1280 px width and 70° HFOV (both hard-coded, **not calibrated** for the webcam).
- **V18 object position**: `x_m = range·sin(bearing)` = **right**, `y_m = range·cos(bearing)` = **forward**.
- **Bridge**: "X = lateral, Y = forward" → `forward_cmd = 0.5·(y − 3.0)`, `lateral_cmd = 0.8·x`, clamped ±1.5, deadzones 0.25 m / 0.10 m. Consistent with V18.
- **Setpoint node**: body (forward, right) → NED via vehicle heading; clamps to ±0.25 m/s; vertical forced to 0.
- **LiDAR angle** must equal the camera bearing convention (0° = forward, clockwise positive) or fusion silently fails. The real STL-19P mounting/rotation is **unverified**.

---

## 3. Status table

| Component | Status | Evidence |
|---|---|---|
| Windows camera | VERIFIED-HIST | `multi_object_tracking_v10.jsonl`, 12 Oct 2026: 24,118 frames over 36.9 min, 10.9 FPS mean, 2 gaps > 0.5 s |
| Detection (YOLOE-26n-seg) | VERIFIED-HIST | Same recording: person 44 k, glasses 13.7 k detections; 316 stable IDs |
| Tracking (BoT-SORT + stable ID layer) | VERIFIED-SIM (logic) | `tests/test_windows_stable_identity.py` (8 tests: confirmation, TEMP_LOST/LOST, ID switch recovery, class-flicker lock, prediction clamp) |
| Windows → Pi network link | **VERIFIED LIVE 2026-11-02** | Live webcam via `py -3 nidar_v11_windows.py` (NIDAR_PI_HOST=10.218.70.147) → Pi monitor: perception `LIVE`, frame 974 advancing, age 0.27 s, "FULL NIDAR INTEGRATION LIVE". Earlier evidence: frame 24118 on 12 Oct. Pi address changes with the phone hotspot |
| Pi receiver | VERIFIED-SIM | `tests/test_receiver.py`: real `ClientHandler` over loopback; split/coalesced packets, bad JSON, reconnect, atomic file, replay of the real recording |
| V18 fusion | VERIFIED-SIM; **runs live on the Pi (2026-11-02)** | Simulation: median range error 0.22 m, bearing 1.0°. Pi live: V18 frame = perception frame 974, age 0.03 s; person ACTIVE with TempLost 0 in both monitor screens (V18.1 installed). Range shown (4.02 m) is the SIMULATED wall, not the person (F6) |
| Distance sensor | **SIMULATED** | `LIDAR_BACKEND="SIMULATOR"`; `SerialSTL19PBackend.next_scan` raises `NotImplementedError` (no packet decoder) |
| SLAM V5.1 | **FAILING for motion** (SLAM-01) | Simulation: moving sensor 5.95 m → estimated 0.34 m, direction correct 51 %. **Pi live 2026-11-02 (stationary STL-19P simulator):** pose wandered to X=+0.56 m, Y=+0.69 m, yaw +14° by frame 2460 (19.5 FPS) — drift with a sensor that never moves. `tests/test_slam.py` xfail |
| AirMouse server (Pi → ROS 2, port 5010) | Fixed version **installed and running on the Pi (2026-11-02)**; stop-after-camera-close ordering still to confirm | Pi live run: 117 `person ACTIVE` lines sent; `CAMERA_STALE -> STOP` and `NO_TARGET -> STOP` observed (order unknown — log has no timestamps; `tools/nidar_run_report.py` reports it). Original measured FAILING (stale data as ACTIVE; 1 Hz). Live log shows ACTIVE↔TEMP_LOST alternation — cause being measured |
| ROS 2 bridge | **VERIFIED LIVE 2026-11-02** (WSL, real DDS) | Pi :5010 → `nidar_bridge` → `/nidar/airmouse_command`: `[0.5055, 0.1728, 0.0, 1.0]` while ACTIVE, zeros otherwise. **Stop after closing the camera proven end-to-end:** last valid command 405.267 s, first zero 405.370 s (+0.10 s), server `CAMERA_STALE` from 406.200 s (+0.93 s), only zeros for the next ~8 s (Pi perception OFFLINE, frame frozen at 1886). Forward 0.5 m/s comes from the FAKE 4 m range (F6). Packets sometimes arrive in pairs ~10–20 ms apart then ~200 ms gap (Wi-Fi batching; well within the 1.0 s bridge timeout). Shutdown defect F20 |
| ROS 2 telemetry | UNVERIFIED at runtime | Code already uses BEST_EFFORT (QoS fix appears applied); never observed receiving |
| PX4 telemetry | UNVERIFIED | `ros2 topic hz` attempt failed on CLI syntax |
| Command delivery to PX4 | UNVERIFIED | `px4_setpoint_sim` subscribes and publishes setpoints; arming/offboard disabled by design |
| Simulated vehicle response | NOT TESTED | |
| Gazebo arena world | GENERATED, **not yet loaded** | Generator checks pass (seeds 1–3); first load = Stage 0/B of `gazebo/README.md` |
| Gazebo sensors / ros_gz_bridge / WSL relays | **UNVERIFIED** | Written for PX4 v1.18.0-beta1 + Gazebo 8.15; non-ROS logic tested |
| Pi Gazebo-LiDAR path (receiver → backend → V18/SLAM → server) | **VERIFIED-SIM** | Tests + full dry run of `start_nidar_pi.sh` in GAZEBO mode (section 1c) |
| SLAM V6 | **VERIFIED-SIM** (ray-cast arena, real Pi conversion path, real process) | Section 1d; not yet run with Gazebo scans or on the Pi CPU |
| Survivor mapping + mission server | **VERIFIED-SIM** | Section 1d; full Pi chain dry run with the real start script |
| Ground station (desktop) | **VERIFIED-SIM** (offscreen) | Section 1d; WSL/ROS 2 side unverified |
| Physical drone readiness | **NOT READY** | Blockers: F24, no real range sensor, uncalibrated camera, SLAM V6 untested on real data, low survivors < 1.6 m |

---

## 4. Findings (numbered for reference)

**F1 – Missing target server — RETRACTED 2026-11-02.** Based on the Windows copy only. The Pi has `nidar_airmouse_server.py` on port 5010; it maps `fusion_confidence` → `confidence` correctly. Its real defects are F13 and F14.

**F13 – AirMouse server forwarded stale data as ACTIVE (SAFETY BLOCKER, fixed, not yet installed).** No freshness check: if V18 stops or crashes with a person ACTIVE, the frozen file is sent forever and the bridge keeps commanding motion (measured: fwd 0.99 m/s, valid=1, from a 1-hour-old file; simulation `v18_crash` on the original files: valid commands until the end of the run). The server was also started by hand, outside `start_nidar_pi.sh`, so stopping the pipeline left it running on the frozen file.

**F14 – AirMouse server ran at ~1 Hz instead of 10 Hz (fixed, not yet installed).** `server.settimeout(1.0)` made `accept()` wait 1 s each loop. Gaps of 1.1 s exceed the bridge's 1.0 s timeout and the setpoint node's 0.75 s timeout → stop-and-go: command present 69.4 % of the time.

**F15 – Pi power supply (OPEN, hardware).** `vcgencmd get_throttled` = 0x50000 (under-voltage and throttling occurred since boot). Replace with a 5.1 V / 2.5 A supply before any flight-related testing.

**F19 – Live TEMP_LOST flicker is caused by the simulated LiDAR (MEASURED 2026-11-02, `tools/nidar_run_report.py`).** Live webcam run on the Pi: camera 9.06 FPS at the Pi, median gap 0.057 s, 2 % of gaps > 0.25 s. V18 rows: ACTIVE 79 %, TEMP_LOST 21 %; of the TEMP_LOST rows 74 % came right after a new camera frame that did not fuse with a (simulated) LiDAR track, 26 % after camera gaps. Not fixable by tuning: needs a real range sensor or a deliberate camera-only mode (F6). Same run: for ~17 s before the camera connected the server sent `CAMERA_STALE -> STOP` instead of the stale V18 object — the frozen-frame stop working on the real Pi. Stop-after-camera-close not yet observed (run ended with the person in view).

**F20 – Bridge's final zero on Ctrl+C was never sent (fix copied into WSL — `nidar_bridge.py.bak_shutdown_2026-11-02` exists there — Ctrl+C check still pending).** Observed: `RCLError: Failed to publish: publisher's context is invalid` from `zero_command()` in `main()`'s `finally`. rclpy's default signal handling shuts the context down on SIGINT before `finally` runs. Fix: `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`, publish zero ×3 while `rclpy.ok()`, then shut down; log `Shutdown: ZERO command sent`. Stub test `test_bridge_sends_zero_on_ctrl_c_before_shutdown` checks the ordering; the real-rclpy check must be done in WSL (ROS packages are not reachable from the analysis workspace). Pre-fix WSL file sha256 starts `adf3956e6bfa`.

**F21 – px4_setpoint_sim has the same shutdown pattern (OPEN, fix when that node is tested).** `finally: node.shutdown()` publishes zero setpoints and calls `spin_once` after Ctrl+C — expected to fail the same way, so "Sending zero velocity before shutdown" is not delivered. PX4's own offboard-loss failsafe still applies.

**F22 – Low survivors are invisible to (or mis-ranged by) camera+LiDAR fusion (OPEN, MEASURED 2026-11-02b, mission-relevant).** A horizontal 2-D LiDAR at flight height passes over a person lying/sitting on the floor. `tools/arena_fusion_sweep.py --seed 1 --height 1.0` (perfect detections, cylinder bodies): standing survivors fused in 140/140 views, median range error 0.03 m; low survivors (0.45 m) fused in only 13/85 views and **all 13 with the wall's range (> 0.5 m wrong)**. LiDAR plane at 0.3 m: low survivors 74/85 fused, 0.04 m error. Options to decide: reject LiDAR clusters wider than a person, range low targets from the camera (height + tilt), lower/tilted range sensor. Strict xfail `test_low_survivor_is_never_fused_at_the_wrong_range`. Gazebo with the real meshes is the next check.

**F23 – Integration monitor crashed when SLAM had no state yet, which stopped the whole Pi system (FIXED 2026-11-02b).** `f"{slam['x']:+.3f}"` with `x = None` (fresh `outputs/`, or GAZEBO mode before the first scans) raised `TypeError`; `start_nidar_pi.sh` runs `cleanup` when the monitor exits → every Pi process stopped. Found in the GAZEBO dry run. On the real Pi it stayed hidden because an old state file existed. Test `test_integration_monitor_survives_missing_slam_state`.

**F24 – V18 freezes a target's bearing after one large jump (OPEN, SAFETY — blocks closed loop).** `PersistentTargetManager.update` replaces a bearing that differs > `MAX_BEARING_JUMP_DEG` (25°) from `last_valid_bearing_deg` by the old value, and that old value never changes again. Measured: target at +33°, then truly at 0° → reported 33° for 10 s (101 outliers), state ACTIVE, `x_m` 0.545 m → the bridge would keep commanding lateral motion. Triggers: yaw between two processed frames (camera stall during a turn), re-acquisition after a gap. Smooth yaw (5°/frame) is followed with ~9° lag. Range has the same pattern (`MAX_RANGE_JUMP_*`). Fix direction (not applied, needs your decision): accept the new value after N consecutive consistent "outliers", and/or compensate the drone's yaw change. Strict xfail `test_v18_recovers_after_a_single_bearing_jump`.

**F16 – V18 CSV grows without limit (OPEN, low).** 64 MB on the Pi; appended every 50 ms.

**F17 – start_nidar_pi.sh cleanup runs twice (OPEN, cosmetic).** `trap cleanup INT TERM EXIT` + `exit 0` inside cleanup re-triggers it on EXIT (also in the original script); step labels read 1/3, 2/3, 3/4, 4/4. Harmless.

**F18 – Windows `python` is Python 2 from MGLTools; `D:\NIDAR_DRONE_PROJECT\.venv` is a Linux venv (OPEN, documentation).** On Windows always use `py -3 ...`. The `.venv` (bin/, python3.12, created at /home/abhishek/drone_project) cannot be activated from PowerShell.

**F2 – V18 ACTIVE/TEMP_LOST flicker (fixed).** See section 1. Reproduce the old behaviour: `NIDAR_V18_PATH=archive/pre_session_2026-11-01/nidar_v18_headless_persistent_fusion.py python -m src.simulation.nidar_pipeline_sim --no-slam`.

**F3 – V18 path coupling (fixed).** F4 – SLAM Python 3.11 syntax (fixed). F5 – empty logs (fixed).

**SLAM-01 – Scan matcher cannot measure motion (OPEN, blocks GPS-denied navigation).** `ScanMatcher.match` takes the difference of point-cloud centroids and principal axes, and the "error" compares point *i* to point *i* by index. When the sensor moves, the visible part of the room changes, so the centroid moves in ways unrelated to the sensor's motion. The estimates are also reported as *valid* (>80 % accepted), so downstream code would trust them. Fix direction: point-to-point / point-to-line ICP with nearest-neighbour correspondences (e.g. a KD-tree), or a correlative scan matcher on the occupancy grid. The `slam_moving_sensor` scenario is the acceptance test.

**F6 – Fusion on real data pairs a real person with simulated obstacles (OPEN).** On the Pi, the camera sees a real room and V18 matches it against LiDAR tracks from a simulated 8×6 m room. Any range V18 reports from live camera data is therefore fictitious. Do not interpret V18 ranges until a real range sensor is integrated.

**F7 – Key mismatch `fusion_confidence` vs `confidence` (handled).** V18 emits `fusion_confidence`; the bridge reads `confidence`. The Pi's `nidar_airmouse_server.py` maps it (test: `test_fresh_active_target_matches_bridge_contract`).

**F8 – Bridge Pi address hard-coded (fix prepared 2026-11-02, apply in WSL).** Was `PI_HOST = "10.89.74.147"`. Change: `import os` + `PI_HOST = os.environ.get("NIDAR_PI_HOST", "10.218.70.147")` — same variable the Windows camera script uses; laptop copy `ros2_ws_src/` updated, contract tests pass. WSL workspace copy must be edited and rebuilt (`colcon build --packages-select nidar_airmouse`).

**F9 – LiDAR detector reports wall segments as obstacles (OPEN, documented).** Wall clusters become LiDAR tracks; V18 can fuse a person's bearing with a wall. Test: `test_detector_also_reports_wall_segments_as_obstacles`.

**F10 – Camera model not calibrated (PARTLY FIXED 2026-11-02b).** V18 now uses `packet["camera"]["width"]` (Gazebo 640 px works); HFOV still 70° by default (`NIDAR_CAMERA_HFOV_DEG`), not measured for the webcam. The linear pixel→bearing model gives up to ~2° error vs a pinhole camera at 25° (measured in the arena: 23.3° for 25.0°).

**F11 – Stale Docker files (OPEN, low).** `Dockerfile`/`docker-compose.yml` start scripts that no longer exist at the root.

**F12 – COCO detectors miss people from the air (training target).** On 148 held-out VisDrone drone images, stock `yolo26n.pt` recall at conf 0.25 = **6.4 %** for people (AP50 0.17). See section 7.

---

## 5. Test harness — how to run

From the project root:

| Machine | Command |
|---|---|
| Windows PowerShell | `py -3 -m pip install pytest pyyaml` then `py -3 -m pytest -q` (not `python`: it is MGLTools' Python 2) |
| Raspberry Pi | `cd ~/drone_project && source .venv/bin/activate && pip install pytest && python -m pytest -q` |
| WSL | `cd /mnt/d/NIDAR_DRONE_PROJECT && python3 -m pytest -q` |

Expected (2026-11-02f): laptop `165 passed, 3 xfailed`. Earlier (2026-11-02e): laptop `157 passed, 3 xfailed`; mock Pi after bundle c `155 passed, 2 skipped, 3 xfailed`. Earlier (2026-11-02d): laptop `138 passed, 3 xfailed` (one unexplained failure in 19 full runs, not reproduced in the next 18 — a timing-sensitive test under load is suspected; report it if you see one); Pi after installing `nidar_pi_install_2026-11-02b.tgz`: `121 passed, 2 skipped, 3 xfailed` (mock Pi; previous bundle confirmed on the real Pi: 90 passed, 2 skipped, 1 xfailed in 49 s). xfails: SLAM-01, F22, F24 (the skips are WSL-only start-script checks; the xfail is SLAM-01). Fast subset: `-m "not slow"`. Ultralytics, OpenCV and ROS 2 are **not required**: they are stubbed only when absent. Tests need `ros2_ws_src/` (copied this session) for the bridge contract tests; they skip otherwise.

| File | Covers |
|---|---|
| `test_environment.py` | Path coupling between processes, start scripts, Python version, ports, protocol constants |
| `test_protocol_schema.py` | NIDAR_PERCEPTION_V1 on the real recording, `object_record()`, world packets, wire bytes |
| `test_receiver.py` | Real receiver handler over TCP |
| `test_windows_stable_identity.py` | Windows stable-ID logic without camera/model |
| `test_v18_fusion.py` | Fusion gate, persistence state machine, outliers, recovery, JSON output |
| `test_lidar_chain.py` | Simulator, detector, tracker V3, world/LiDAR conventions |
| `test_slam.py` | SLAM components; SLAM-01 as strict xfail |
| `test_airmouse_server.py` | The Pi's port-5010 server: freshness/stop cases, target selection, bridge contract, measured send rate |
| `test_ros2_bridge_contract.py` | Bridge P-controller, timeouts, setpoint clamps, NED rotation, no arming |
| `test_pipeline_simulation.py` | End-to-end scenarios |
| `test_gazebo_lidar_path.py` | Arena ray-caster, ROS→NIDAR angles, LiDAR receiver over TCP, backend freshness |
| `test_gazebo_pi_pipeline.py` | Real V18/SLAM processes in GAZEBO mode, packet camera width, monitor F23, F22/F24 findings |
| `test_gazebo_wsl_nodes.py` | Relay wire format into the real Pi receiver, camera relay → Windows capture, rig routes |
| `test_windows_gazebo_camera.py` | Windows `GazeboCameraCapture` |

---

## 6. Simulation

`src/simulation/nidar_world.py` generates **both** camera packets and LiDAR scans from one 2-D world with ground truth (the old simulator's room and the webcam never described the same objects). `nidar_pipeline_sim.py` pushes them through the *real* receiver (TCP mode), V18, the Pi's `nidar_airmouse_server.py`, bridge and setpoint code with a simulated clock. `NIDAR_V18_PATH` / `NIDAR_AIRMOUSE_SERVER_PATH` select other versions for A/B runs.

```
python -m src.simulation.nidar_pipeline_sim                       # all scenarios + SLAM, report in outputs/simulation/
python -m src.simulation.nidar_pipeline_sim --scenario stale_camera --transport tcp
```

Scenarios: `nominal`, `stale_camera`, `camera_dropout_recovery`, `link_drop_tcp`, `two_person`, `bearing_sweep`, `empty_scene`, `v18_crash` (added 2026-11-02), plus `slam_stationary_sensor` and `slam_moving_sensor`.

Result 2026-11-02 with the fixed Pi files: **all 8 pipeline scenarios PASS; SLAM stationary PASS; SLAM moving FAIL (SLAM-01).** With the Pi's original V18 + server (`outputs/simulation/pi_original_2026-11-02/`): 6 of 8 FAIL — valid command only 60 % of the time with the person in view, and `v18_crash` never stops commanding. Full numbers: `outputs/simulation/SIMULATION_REPORT.md`.

`src/simulation/nidar_replay.py` replays the real 12 Oct recording to any receiver with the exact Windows wire format — use it to test the network link without a camera.

---

## 7. Training (aerial detector)

Scripts in `training/` (see `training/README.md`): `prepare_visdrone.py` → `train_aerial.py` → `evaluate_aerial.py`. Results in `outputs/training/eval_cpu15.md`.

First CPU run (2026-11-01): YOLO26n fine-tuned 15 epochs, 640 px, on 400 VisDrone images, 2 CPU cores, 77 min. Evaluated on 148 **held-out** VisDrone images (10,246 boxes) with one evaluator for all models:

| Class | Stock `yolo26n.pt` AP50 / recall@0.25 | Fine-tuned AP50 / recall@0.25 |
|---|---|---|
| person | 0.174 / **0.064** | 0.304 / **0.272** |
| car | 0.428 / 0.275 | 0.641 / 0.591 |
| large_vehicle | 0.151 / 0.127 | 0.207 / 0.195 |
| two_wheeler | 0.051 / 0.017 | 0.170 / 0.101 |

Cross-check: Ultralytics' own validator gives person AP50 0.297 for the same model (ours 0.304). Model: `models/nidar_aerial_yolo26n_cpu15_640.pt` (5.4 MB).

Caveats: train and evaluation images both come from the VisDrone *val* split (different images, possibly the same scenes), so these numbers are likely optimistic; the proper run trains on the official train split. Recall 27 % is still far from usable for following a person — this proves the pipeline and the direction, not a finished detector. Not yet tested on the NIDAR camera or integrated into `nidar_v11_windows.py`.

---

## 8. Next steps (dependency order)

Gazebo phase (2026-11-02b), see `gazebo/README.md`: **Stage 0** install assets in WSL →
**Stage A** Pi dry run with `send_synthetic_scans.py` → **Stage B** rig in Gazebo (sensor rates,
YOLO on rendered survivors, real-mesh fusion vs F22) → **Stage C** PX4 + x500_nidar_airmouse
(telemetry only, no arming). Fix F24 before any closed-loop test.

Earlier list:

0. **Install the bundle on the Pi** (`tools/install_on_pi.sh`) and run the tests there. Fix the power supply (F15).
1. **Verify the Windows → Pi link** with the replay tool (no camera needed). Success = Pi integration monitor shows perception `LIVE` and frame numbers increasing.
2. **Point the bridge at the right Pi** (F8), start the Pi stack (now including the AirMouse server), run `nidar_bridge` in WSL, and confirm `ros2 topic echo /nidar/airmouse_command` shows `valid=1.0` only while a person is in view.
3. **Verify PX4 telemetry in SITL**: `ros2 topic hz /fmu/out/vehicle_local_position_v1` (check `ros2 topic hz --help` for the QoS flag on your install) and `ros2 run nidar_airmouse telemetry` showing `LIVE`.
4. **Replace the SLAM scan matcher** (SLAM-01) using `slam_moving_sensor` as the acceptance test. Until then SLAM output must not feed control.
5. **Real range sensor**: implement and test the STL-19P packet decoder against `hardware/nidar_hardware_check.py` captures; calibrate the 0° direction to the camera axis.
6. Only after 1–5: enable `ALLOW_OFFBOARD`/`ALLOW_ARM` **in SITL only**, with an explicit stop procedure.
