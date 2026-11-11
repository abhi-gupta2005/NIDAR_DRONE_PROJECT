#!/usr/bin/env bash

set -u

PROJECT_DIR="/mnt/d/NIDAR_DRONE_PROJECT"
PYTHON="python3"
LOG_DIR="$PROJECT_DIR/logs"

mkdir -p "$LOG_DIR"

# 2026-11-01: stdout of the background processes is redirected to log files.
# Without this Python buffers stdout, so a killed receiver left an EMPTY
# receiver.log (observed: logs/receiver.log was 0 bytes).  Unbuffered output
# makes every log line appear immediately.
export PYTHONUNBUFFERED=1

cd "$PROJECT_DIR" || exit 1

export PYTHONPATH="$PROJECT_DIR:$PROJECT_DIR/src:$PROJECT_DIR/src/core:$PROJECT_DIR/src/perception:$PROJECT_DIR/src/tracking:$PROJECT_DIR/src/simulation:${PYTHONPATH:-}"

echo "============================================================"
echo " NIDAR WSL SYSTEM"
echo "============================================================"
echo "Project : $PROJECT_DIR"
echo "Python  : $($PYTHON --version)"
echo "Mode    : WSL / SIMULATED LiDAR"
echo "============================================================"
echo

cleanup() {
    echo
    echo "[NIDAR] Stopping NIDAR processes..."

    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    echo "[NIDAR] Shutdown complete."
}

trap cleanup INT TERM EXIT

PIDS=()

echo "[1/4] Starting perception receiver..."
$PYTHON src/communication/nidar_pi_receiver.py \
    > "$LOG_DIR/receiver.log" 2>&1 &
RECEIVER_PID=$!
PIDS+=("$RECEIVER_PID")

sleep 1

echo "[2/4] Starting NIDAR V18 fusion..."
$PYTHON src/perception/nidar_v18_headless_persistent_fusion.py \
    > "$LOG_DIR/v18.log" 2>&1 &
V18_PID=$!
PIDS+=("$V18_PID")

sleep 2

echo "[3/4] Starting SLAM V5.1..."
$PYTHON -m src.core.nidar_slam_v5_1 \
    > "$LOG_DIR/slam.log" 2>&1 &
SLAM_PID=$!
PIDS+=("$SLAM_PID")

sleep 2

echo
echo "============================================================"
echo " NIDAR COMPONENTS STARTED"
echo "============================================================"
echo "Receiver PID : $RECEIVER_PID"
echo "V18 PID      : $V18_PID"
echo "SLAM PID     : $SLAM_PID"
echo
echo "Logs:"
echo "  $LOG_DIR/receiver.log"
echo "  $LOG_DIR/v18.log"
echo "  $LOG_DIR/slam.log"
echo "  $LOG_DIR/airmouse_server.log"
echo "[4/5] Starting NIDAR AirMouse server (port 5010)..."
$PYTHON nidar_airmouse_server.py \
    > "$LOG_DIR/airmouse_server.log" 2>&1 &
TARGET_PID=$!
PIDS+=("$TARGET_PID")

sleep 1

echo
echo "[5/5] Starting unified integration monitor..."
echo "============================================================"
echo

$PYTHON integration_monitor.py
