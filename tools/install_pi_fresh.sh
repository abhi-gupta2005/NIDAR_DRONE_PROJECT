#!/usr/bin/env bash
# =====================================================================
# NIDAR AirMouse – set up a NEW Raspberry Pi (2026-11-11). Normally started from the laptop (WSL):
#     nidar install-pi            ( = bash tools/nidar_run.sh install-pi )
# which copies the project to ~/drone_project on the Pi and then runs this script there.
#
# Installs (Raspberry Pi OS 64-bit, Debian 12/13): python3-venv, numpy, OpenCV, pyserial, PyYAML, pytest
# from the OS packages (fast on a Pi 3B+, no compiling), makes ~/drone_project/.venv (sees those packages),
# the log/output folders, and checks that the Pi programs import. Starts nothing.
# For a Pi that already runs NIDAR use `nidar update-pi` instead (it keeps your audited files).
# =====================================================================
set -euo pipefail
P="${NIDAR_PROJECT_DIR:-$HOME/drone_project}"
say() { echo "[pi $(date +%H:%M:%S)] $*"; }
[ -f "$P/start_nidar_pi.sh" ] || { say "[ERROR] $P/start_nidar_pi.sh not found: copy the project first (nidar install-pi)"; exit 1; }

if [ -z "${NIDAR_SKIP_APT:-}" ]; then
    say "1/4 OS packages (may ask for the Pi password)"
    sudo apt-get update -q
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q python3-venv python3-pip python3-numpy \
        python3-opencv python3-serial python3-yaml python3-pytest rsync
else
    say "1/4 OS packages: skipped (NIDAR_SKIP_APT)"
fi

say "2/4 Python environment $P/.venv"
[ -x "$P/.venv/bin/python" ] || python3 -m venv --system-site-packages "$P/.venv"
"$P/.venv/bin/python" - <<'PY'
import importlib
for m in ("numpy", "cv2", "serial", "yaml", "pytest"):
    mod = importlib.import_module(m)
    print(f"    {m:7s} {getattr(mod, '__version__', 'ok')}")
PY

say "3/4 folders"
mkdir -p "$P/logs" "$P/outputs/slam/state" "$P/outputs/slam/maps" "$P/outputs/slam/trajectories" "$P/outputs/mission"
chmod +x "$P/start_nidar_pi.sh" "$P/nidar_airmouse_server.py" "$P/integration_monitor.py" \
         "$P"/src/communication/*.py "$P"/tools/*.sh 2> /dev/null || true

say "4/4 do the Pi programs load?"
cd "$P"
PYTHONPATH="$P:$P/src/core:$P/src/tracking:$P/src/perception:$P/src/simulation" "$P/.venv/bin/python" - <<'PY'
import importlib, sys
mods = ["src.core.nidar_slam_v6", "src.mission.nidar_explorer_service", "src.mission.nidar_mission_server",
        "src.mission.survivor_mapper", "src.core.lidar_gazebo_backend"]
bad = 0
for m in mods:
    try:
        importlib.import_module(m)
        print(f"    OK    {m}")
    except Exception as exc:                      # report every failure, then stop
        bad += 1
        print(f"    FAIL  {m}: {exc}")
sys.exit(1 if bad else 0)
PY
say "DONE. The laptop starts everything on this Pi by itself: run  nidar  in WSL."
