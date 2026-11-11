#!/bin/bash

set -u

PROJECT_DIR="$HOME/drone_project"
VENV="$PROJECT_DIR/.venv/bin/activate"
LOG_DIR="$PROJECT_DIR/logs"

mkdir -p "$LOG_DIR"

# 2026-11-01: stdout of the background processes is redirected to log files.
# Without this Python buffers stdout, so a killed receiver left an EMPTY
# receiver.log (observed: logs/receiver.log was 0 bytes).  Unbuffered output
# makes every log line appear immediately.
export PYTHONUNBUFFERED=1

echo "============================================================"
echo "             NIDAR PI SYSTEM STARTUP"
echo "============================================================"
echo

# ============================================================
# ENVIRONMENT
# ============================================================

if [ ! -f "$VENV" ]; then
    echo "[ERROR] Virtual environment not found:"
    echo "$VENV"
    exit 1
fi

source "$VENV"

cd "$PROJECT_DIR"

export PYTHONPATH="$PROJECT_DIR:$PROJECT_DIR/src/core:$PROJECT_DIR/src/tracking:$PROJECT_DIR/src/perception:$PROJECT_DIR/src/simulation"

echo "[OK] Virtual environment activated"
echo "[OK] Project: $PROJECT_DIR"
echo "[OK] PYTHONPATH configured"

# 2026-11-02: LiDAR source for V18 and SLAM.
#   SIMULATOR (default) -> built-in STL-19P simulator, as before
#   GAZEBO              -> scans from the Gazebo simulation in WSL, received on
#                          TCP ${NIDAR_LIDAR_PORT:-5011} by nidar_lidar_receiver.py
#   Use:  NIDAR_LIDAR_BACKEND=GAZEBO ./start_nidar_pi.sh
export NIDAR_LIDAR_BACKEND="${NIDAR_LIDAR_BACKEND:-SIMULATOR}"
case "$NIDAR_LIDAR_BACKEND" in
    SIMULATOR|GAZEBO) ;;
    *) echo "[ERROR] NIDAR_LIDAR_BACKEND=$NIDAR_LIDAR_BACKEND (use SIMULATOR or GAZEBO)"; exit 1 ;;
esac
echo "[OK] LiDAR backend: $NIDAR_LIDAR_BACKEND"

# 2026-11-02e: SLAM version.
#   V5 (default) -> nidar_slam_v5_1 as before (SLAM-01: pose does not follow motion)
#   V6           -> nidar_slam_v6 (scan-to-map matcher) + nidar_mission_server.py
#                   (survivor mapping + ground-station data, TCP ${NIDAR_MISSION_PORT:-5020})
#   Use:  NIDAR_LIDAR_BACKEND=GAZEBO NIDAR_SLAM=V6 ./start_nidar_pi.sh
export NIDAR_SLAM="${NIDAR_SLAM:-V5}"
case "$NIDAR_SLAM" in
    V5|V6) ;;
    *) echo "[ERROR] NIDAR_SLAM=$NIDAR_SLAM (use V5 or V6)"; exit 1 ;;
esac
echo "[OK] SLAM version: $NIDAR_SLAM"
echo

# ============================================================
# CLEAN OLD PROCESSES
# ============================================================

echo "[1/6] Cleaning old NIDAR processes..."

pkill -f "nidar_pi_receiver.py" 2>/dev/null || true
pkill -f "nidar_v18_headless_persistent_fusion.py" 2>/dev/null || true
pkill -f "nidar_slam_v5_1" 2>/dev/null || true
pkill -f "nidar_slam_v6" 2>/dev/null || true
pkill -f "nidar_mission_server.py" 2>/dev/null || true
pkill -f "integration_monitor.py" 2>/dev/null || true
pkill -f "nidar_airmouse_server.py" 2>/dev/null || true
pkill -f "nidar_lidar_receiver.py" 2>/dev/null || true

sleep 2

echo "[OK] Old processes cleared"
echo

# ============================================================
# RECEIVER
# ============================================================

echo "[2/6] Starting NIDAR perception receiver..."

python src/communication/nidar_pi_receiver.py \
    > "$LOG_DIR/receiver.log" 2>&1 &

RECEIVER_PID=$!

sleep 2

if kill -0 "$RECEIVER_PID" 2>/dev/null; then
    echo "[OK] Receiver started | PID=$RECEIVER_PID"
else
    echo "[ERROR] Receiver failed to start"
    tail -50 "$LOG_DIR/receiver.log"
    exit 1
fi

echo

# ============================================================
# GAZEBO LiDAR RECEIVER (only with NIDAR_LIDAR_BACKEND=GAZEBO)
# Must run before V18/SLAM: they read the scans it writes.
# ============================================================

LIDAR_RX_PID=""
if [ "$NIDAR_LIDAR_BACKEND" = "GAZEBO" ]; then
    echo "[2b/6] Starting Gazebo LiDAR receiver (port ${NIDAR_LIDAR_PORT:-5011})..."
    python src/communication/nidar_lidar_receiver.py \
        > "$LOG_DIR/lidar_receiver.log" 2>&1 &
    LIDAR_RX_PID=$!
    sleep 2
    if kill -0 "$LIDAR_RX_PID" 2>/dev/null; then
        echo "[OK] LiDAR receiver started | PID=$LIDAR_RX_PID"
        echo "     V18 and SLAM wait for scans; start nidar_lidar_relay in WSL."
    else
        echo "[ERROR] LiDAR receiver failed to start"
        tail -50 "$LOG_DIR/lidar_receiver.log"
        kill "$RECEIVER_PID" 2>/dev/null || true
        exit 1
    fi
    echo
fi

# ============================================================
# V18 FUSION
# ============================================================

echo "[3/6] Starting NIDAR V18 fusion..."

python src/perception/nidar_v18_headless_persistent_fusion.py \
    > "$LOG_DIR/v18.log" 2>&1 &

V18_PID=$!

sleep 3

if kill -0 "$V18_PID" 2>/dev/null; then
    echo "[OK] V18 started | PID=$V18_PID"
else
    echo "[ERROR] V18 failed to start"
    tail -50 "$LOG_DIR/v18.log"

    kill "$RECEIVER_PID" 2>/dev/null || true
    [ -n "$LIDAR_RX_PID" ] && kill "$LIDAR_RX_PID" 2>/dev/null || true

    exit 1
fi

echo

# ============================================================
# SLAM
# ============================================================

if [ "$NIDAR_SLAM" = "V6" ]; then
    echo "[4/6] Starting NIDAR SLAM V6 (scan-to-map)..."
    python -m src.core.nidar_slam_v6 \
        > "$LOG_DIR/slam.log" 2>&1 &
else
    echo "[4/6] Starting NIDAR SLAM V5.1..."
    python -m src.core.nidar_slam_v5_1 \
        > "$LOG_DIR/slam.log" 2>&1 &
fi

SLAM_PID=$!

sleep 3

if kill -0 "$SLAM_PID" 2>/dev/null; then
    echo "[OK] SLAM started | PID=$SLAM_PID"
else
    echo "[ERROR] SLAM failed to start"
    tail -50 "$LOG_DIR/slam.log"

    kill "$V18_PID" 2>/dev/null || true
    kill "$RECEIVER_PID" 2>/dev/null || true
    [ -n "$LIDAR_RX_PID" ] && kill "$LIDAR_RX_PID" 2>/dev/null || true

    exit 1
fi

echo

# ============================================================
# SYSTEM STATUS
# ============================================================

# ============================================================
# AIRMOUSE SERVER (added to this script 2026-11-02)
# nidar_airmouse_server.py serves the selected V18 target to the WSL ROS 2
# bridge on TCP 5010. Started and stopped WITH the pipeline so it can never
# keep serving a frozen V18 file after V18 has been stopped.
# ============================================================

echo "[5/6] Starting NIDAR AirMouse server (port 5010)..."

python nidar_airmouse_server.py \
    > "$LOG_DIR/airmouse_server.log" 2>&1 &

TARGET_PID=$!

sleep 2

if kill -0 "$TARGET_PID" 2>/dev/null; then
    echo "[OK] AirMouse server started | PID=$TARGET_PID"
else
    echo "[ERROR] AirMouse server failed to start (is another copy already using port 5010?)"
    tail -50 "$LOG_DIR/airmouse_server.log"
    # Not fatal for perception/SLAM; the ROS 2 bridge will see no target.
fi

echo

# ============================================================
# MISSION SERVER (only with NIDAR_SLAM=V6): survivors + data for the ground station
# ============================================================
MISSION_PID=""
if [ "$NIDAR_SLAM" = "V6" ]; then
    echo "[5b/6] Starting NIDAR mission server (port ${NIDAR_MISSION_PORT:-5020})..."
    python src/mission/nidar_mission_server.py \
        > "$LOG_DIR/mission_server.log" 2>&1 &
    MISSION_PID=$!
    sleep 2
    if kill -0 "$MISSION_PID" 2>/dev/null; then
        echo "[OK] Mission server started | PID=$MISSION_PID"
    else
        echo "[ERROR] Mission server failed to start (not fatal for the pipeline)"
        tail -30 "$LOG_DIR/mission_server.log"
        MISSION_PID=""
    fi
    echo
fi

echo "[6/6] NIDAR core services are running."
echo

echo "============================================================"
echo "             NIDAR PI SYSTEM: RUNNING"
echo "============================================================"
echo

echo "Receiver PID : $RECEIVER_PID"
echo "V18 PID      : $V18_PID"
echo "SLAM PID     : $SLAM_PID"
echo "AirMouse PID : ${TARGET_PID:-not running}"
echo "LiDAR        : $NIDAR_LIDAR_BACKEND${LIDAR_RX_PID:+ (receiver PID=$LIDAR_RX_PID)}"
echo "SLAM         : $NIDAR_SLAM${MISSION_PID:+ | mission server PID=$MISSION_PID}"

echo
echo "Logs:"
echo "  $LOG_DIR/receiver.log"
echo "  $LOG_DIR/v18.log"
echo "  $LOG_DIR/slam.log"
echo "  $LOG_DIR/airmouse_server.log"
[ -n "$LIDAR_RX_PID" ] && echo "  $LOG_DIR/lidar_receiver.log"
[ -n "$MISSION_PID" ] && echo "  $LOG_DIR/mission_server.log"

echo
echo "Integration monitor will now run in THIS terminal."
echo
echo "============================================================"
echo "Start the Windows transmitter separately (PowerShell):"
echo
echo "  \$env:NIDAR_PI_HOST = '<PI_IP>'"
echo "  \$env:NIDAR_PI_PORT = '5000'"
echo "  py -3 nidar_v11_windows.py      (Windows laptop; 'python' there is Python 2)"
echo "  or, without a camera, replay the recording:"
echo "  py -3 -m src.simulation.nidar_replay --host <PI_IP> --port 5000"
echo "============================================================"
echo
echo "Press Ctrl+C to stop the complete NIDAR PI system."
echo

# ============================================================
# CLEANUP
# ============================================================

cleanup() {

    echo
    echo
    echo "============================================================"
    echo "             STOPPING NIDAR PI SYSTEM"
    echo "============================================================"

    echo "[1/5] Stopping SLAM..."
    kill "$SLAM_PID" 2>/dev/null || true

    echo "[2/5] Stopping V18..."
    kill "$V18_PID" 2>/dev/null || true

    echo "[3/5] Stopping receiver..."
    kill "$RECEIVER_PID" 2>/dev/null || true

    echo "[4/5] Stopping AirMouse server..."
    kill "${TARGET_PID:-0}" 2>/dev/null || true

    echo "[5/5] Stopping LiDAR receiver and mission server..."
    [ -n "$LIDAR_RX_PID" ] && kill "$LIDAR_RX_PID" 2>/dev/null || true
    [ -n "$MISSION_PID" ] && kill "$MISSION_PID" 2>/dev/null || true

    sleep 2

    # Make sure child processes are gone
    kill -0 "$SLAM_PID" 2>/dev/null && kill -9 "$SLAM_PID" 2>/dev/null || true
    kill -0 "$V18_PID" 2>/dev/null && kill -9 "$V18_PID" 2>/dev/null || true
    kill -0 "$RECEIVER_PID" 2>/dev/null && kill -9 "$RECEIVER_PID" 2>/dev/null || true
    kill -0 "${TARGET_PID:-0}" 2>/dev/null && kill -9 "${TARGET_PID:-0}" 2>/dev/null || true
    [ -n "$LIDAR_RX_PID" ] && kill -0 "$LIDAR_RX_PID" 2>/dev/null && kill -9 "$LIDAR_RX_PID" 2>/dev/null || true
    [ -n "$MISSION_PID" ] && kill -0 "$MISSION_PID" 2>/dev/null && kill -9 "$MISSION_PID" 2>/dev/null || true

    echo
    echo "[OK] Complete NIDAR PI system stopped."
    echo "============================================================"

    exit 0
}

trap cleanup INT TERM EXIT

# ============================================================
# INTEGRATION MONITOR
#
# IMPORTANT:
# The monitor is intentionally NOT redirected to a log and
# NOT put in the background.
#
# Therefore its live dashboard appears directly in this
# terminal.
# ============================================================

python integration_monitor.py

# If integration_monitor exits normally, clean everything up.
cleanup
