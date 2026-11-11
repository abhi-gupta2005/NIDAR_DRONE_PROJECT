# NIDAR AirMouse — set up a new laptop and Raspberry Pi

This guide installs everything the project uses on a new Windows laptop and a new Raspberry Pi:
WSL 2 + Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (ros_gz), PX4 SITL, the Micro XRCE-DDS agent,
px4_msgs + the `nidar_airmouse` ROS 2 package, QGroundControl, the AirMouse ground station (PyQt5),
YOLO on Windows (ultralytics + OpenCV), and the Pi software (SLAM V6, explorer brain, fusion, mission server).

There are **four commands** in total. Each installer checks what is already there and skips it, so if
something stops halfway (internet, power), run the **same command again** and it continues.

| # | Where | Command | Time |
|---|---|---|---|
| 1 | Windows PowerShell **as administrator** | `irm https://raw.githubusercontent.com/abhi-gupta2005/NIDAR_DRONE_PROJECT/main/tools/install_windows.ps1 \| iex` | 10–20 min |
| 2 | Ubuntu 24.04 (WSL) | `bash /mnt/d/NIDAR_DRONE_PROJECT/tools/install_wsl.sh` | about 1 hour |
| 3 | Ubuntu (WSL) | `nidar setup --pi <PI_ADDRESS> --user <PI_USER>` then `nidar install-pi` | 5–10 min |
| 4 | Ubuntu (WSL) | `nidar` | a mission |

*(No D: drive on the laptop? The project goes to `C:\NIDAR_DRONE_PROJECT`; use `/mnt/c/...` in step 2. Step 1 prints the exact line.)*

---

## What you need

- **Laptop:** Windows 11 (or Windows 10 22H2), 16 GB RAM recommended (8 GB works, slowly), **40 GB free**,
  administrator rights, virtualization switched on in the BIOS (most laptops have it on), good internet.
- **Raspberry Pi** 3B+ / 4 / 5, a microSD card (16 GB or more), its power supply.
- **One Wi-Fi network** for both (a phone hotspot is fine).

## Step 0 — the project owner puts it on GitHub (once)

On the laptop that already has the project, in WSL:
```bash
bash /mnt/d/NIDAR_DRONE_PROJECT/tools/publish_github.sh
```
It installs git + the GitHub CLI, asks you to log in (it shows a code; type it at github.com/login/device),
creates the repository **NIDAR_DRONE_PROJECT** (public), writes its address into the installer and this guide,
uploads, and prints the exact one-line command for your friend. Later changes: run the same command again.
Left out of GitHub (see `.gitignore`): logs, run outputs, videos, Pi bundles, `.venv`, the 250 MB
`mobileclip2_b.ts` (YOLOE downloads it by itself on its first start).

## Step 1 — Windows (PowerShell as administrator)

Start menu → type **PowerShell** → right-click → **Run as administrator**, then:
```powershell
irm https://raw.githubusercontent.com/abhi-gupta2005/NIDAR_DRONE_PROJECT/main/tools/install_windows.ps1 | iex
```
It installs Git, downloads the project to `D:\NIDAR_DRONE_PROJECT` (or `C:\`), installs Python 3.12 with
the YOLO packages, and installs **WSL 2 + Ubuntu 24.04**.
- If it says **RESTART**: restart Windows.
- Start menu → **Ubuntu 24.04** → it asks for a **new user name and password** (Linux ones; remember the password).

## Step 2 — everything in Ubuntu (one command, about 1 hour)

In the Ubuntu window:
```bash
bash /mnt/d/NIDAR_DRONE_PROJECT/tools/install_wsl.sh
```
Type your Ubuntu password once. It installs, in order: base tools → **ROS 2 Jazzy** (desktop) + **ros_gz /
Gazebo Harmonic** + cv_bridge + colcon → Python packages (PyQt5, numpy, OpenCV, pytest) → **PX4-Autopilot**
(the same commit this project flies, its setup script, `make px4_sitl`) → **Micro XRCE-DDS Agent v2.4.3** →
**~/px4_ros2_ws** (px4_msgs release/1.18 + nidar_airmouse, `colcon build`) → the NIDAR arena worlds and drone
models into PX4's Gazebo folders → **QGroundControl** → the `nidar` command and the ROS environment in every
new terminal → a check of every part and the project's fast tests. Log: `~/nidar_install.log`.

When it says **ALL DONE**, close Ubuntu and open it again. Try the ground station without a Pi:
```bash
python3 /mnt/d/NIDAR_DRONE_PROJECT/gcs/airmouse_gcs.py --demo /mnt/d/NIDAR_DRONE_PROJECT/gcs_records/demo_s1_offline/flight.jsonl.gz --no-ros
```

## Step 3 — the Raspberry Pi

**3a. Write the SD card** with **Raspberry Pi Imager** (download: raspberrypi.com/software):
- Device: your Pi · OS: **Raspberry Pi OS Lite (64-bit)** · Storage: the SD card.
- **Edit settings:** hostname `nidarpi`, a user name + password, **your Wi-Fi** (same as the laptop),
  your country; tab **Services**: **Enable SSH**, "use password authentication". Save → Write.

**3b. Boot the Pi**, wait 2–3 minutes, then find its address. In **Windows PowerShell**:
```powershell
ping -4 nidarpi.local
```
(the number like `192.168.x.x` / `10.x.x.x` is the address; or look in your phone hotspot's "connected devices").

**3c. Connect the laptop to the Pi (once)**, in Ubuntu — replace the address and user:
```bash
nidar setup --pi 192.168.1.50 --user yourname
```
Type `yes` (first connection), then the Pi password **one last time**. It also remembers the Pi
(`~/.nidar_pi`), so every later `nidar` command uses it without `--pi`.

**3d. Install the NIDAR software on the Pi:**
```bash
nidar install-pi
```
It copies the project to `~/drone_project` on the Pi, installs numpy / OpenCV / pyserial / PyYAML from
Raspberry Pi OS, makes `~/drone_project/.venv`, and checks that SLAM V6, the explorer brain and the mission
server load. (For a Pi that already runs NIDAR use `nidar update-pi`; `nidar install-pi --force` copies again.)

## Step 4 — run

```bash
nidar            # autonomous mission: the brain on the Pi explores the Gazebo arena
nidar px4        # PX4 SITL + QGroundControl + route flyer
nidar stop       # stop everything (also from another terminal); or Ctrl+C in the nidar terminal
```
The launcher starts the Pi over SSH, Gazebo, the relays, YOLO in its own Windows window (its **first** start
downloads a ~250 MB model), and the AirMouse ground station; everything of a run is saved in
`logs/run_<time>_<mode>/`. The ground station is watch-only: it never sends a command.

---

## Problems

| You see | Do this |
|---|---|
| `wsl --install` fails / "virtualization" | Switch on virtualization (Intel VT-x / AMD SVM) in the BIOS, restart, run step 1 again. |
| `$'\r': command not found` | The project was copied with Windows line endings: delete the folder and run step 1 again (it clones with the right ones). |
| install_wsl.sh stopped with `[FAILED]` | Read the line above it, fix (internet, disk space), run the same command again. |
| PX4 build: `Killed` / out of memory | In Windows create `C:\Users\<you>\.wslconfig` with `[wsl2]` and `memory=8GB` and `swap=8GB`, PowerShell: `wsl --shutdown`, run step 2 again. |
| `not reachable` | The Pi is off, on another Wi-Fi, or has a new address: `nidar setup --pi NEW_ADDRESS --user yourname`. |
| `refused` | `nidar setup` again (asks the Pi password once). |
| `nidar: command not found` | Open a new Ubuntu window (or `source ~/.bashrc`). |
| Gazebo is slow | Normal in WSL without a graphics driver; close other programs. `nidar --no-gui` runs Gazebo without its window. |

**Private repository instead of public:** the owner adds the friend under GitHub → Settings → Collaborators.
The friend then runs, in PowerShell as administrator:
```powershell
winget install -e --id Git.Git; git clone https://github.com/abhi-gupta2005/NIDAR_DRONE_PROJECT.git D:\NIDAR_DRONE_PROJECT; powershell -ExecutionPolicy Bypass -File D:\NIDAR_DRONE_PROJECT\tools\install_windows.ps1
```
(open a new PowerShell after the Git install if `git` is not found). Then steps 2–4 as above.

**Getting updates later:** owner: `publish_github.sh` again. Friend: PowerShell `git -C D:\NIDAR_DRONE_PROJECT pull`,
then in Ubuntu `bash /mnt/d/NIDAR_DRONE_PROJECT/tools/install_wsl.sh` (rebuilds the ROS package) and `nidar install-pi --force`.

**Safety:** everything here is simulation. Nothing arms a real drone; the SITL-only PX4 parameters the
launcher sets must never be put on a real flight controller.
