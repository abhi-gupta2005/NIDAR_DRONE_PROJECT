#!/usr/bin/env bash
# =====================================================================
# NIDAR sensor rig in the arena -- Gazebo only, NO PX4, no flight control.
#   bash /mnt/d/NIDAR_DRONE_PROJECT/gazebo/run_rig.sh [seed] [lidar_height_m]
#   e.g. bash run_rig.sh 1 1.0       HEADLESS=1 bash run_rig.sh 1 1.0
# Starts: gz sim (world nidar_airmouse_arena_s<seed>), spawns nidar_rig (entity "abhishek_drone") on the
# launch pad at the given height, and ros_gz_bridge (sensors + rig cmd_vel/odom).
# Then, in other WSL terminals: nidar_lidar_relay.py, nidar_camera_relay.py,
# nidar_rig_driver.py (see gazebo/README.md). Ctrl+C stops everything started here.
# Needs install_into_px4.sh to have been run once.
# =====================================================================
set -uo pipefail
SEED="${1:-1}"
HEIGHT="${2:-1.0}"
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/nidar_gz_env.sh"
W="nidar_airmouse_arena_s$SEED"
WORLD_SDF="$PX4_DIR/Tools/simulation/gz/worlds/$W.sdf"
MODEL_SDF="$PX4_DIR/Tools/simulation/gz/models/nidar_rig/model.sdf"
LAYOUT="$NIDAR_REPO/gazebo/layouts/$W.json"
for f in "$WORLD_SDF" "$MODEL_SDF" "$LAYOUT"; do
    [ -f "$f" ] || { echo "[ERROR] missing $f (run gazebo/install_into_px4.sh)"; exit 1; }
done
# (outputs are captured first: with pipefail, `cmd | grep -q` can report failure via SIGPIPE)
topics="$(gz topic -l 2>/dev/null || true)"
if grep -q "^/world/.*/clock" <<< "$topics"; then
    echo "[ERROR] a Gazebo world is already running (close it or PX4 first)"; exit 1
fi

read -r X Y YAW < <(python3 -c "import json,sys; p=json.load(open(sys.argv[1]))['spawn_pose']; print(p['x'], p['y'], p['yaw'])" "$LAYOUT")
PIDS=()
cleanup() {
    echo; echo "[rig] stopping..."
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
    sleep 1
    for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null || true; done
    exit 0
}
trap cleanup INT TERM

echo "[rig] world $W  spawn ($X, $Y) yaw $YAW  LiDAR/camera height $HEIGHT m"
if [ -n "${HEADLESS:-}" ]; then
    gz sim -r -s "$WORLD_SDF" & PIDS+=($!)
else
    gz sim -r "$WORLD_SDF" & PIDS+=($!)
fi

# First launch downloads the Fuel survivor models before the world is up: allow 5 min.
for i in $(seq 1 300); do
    info="$(gz service -i --service "/world/$W/scene/info" 2>&1 || true)"
    grep -q "Service providers" <<< "$info" && break
    [ "$i" -eq 300 ] && { echo "[ERROR] world did not come up in 300 s"; cleanup; }
    [ $((i % 15)) -eq 0 ] && echo "[rig] waiting for the world ($i s; first launch downloads Fuel models)..."
    sleep 1
done
echo "[rig] world ready (first launch may still be downloading the Fuel survivor models)"

SDF="<sdf version=\"1.9\"><include><uri>file://$MODEL_SDF</uri><pose>$X $Y $HEIGHT 0 0 $YAW</pose></include></sdf>"
gz service -s "/world/$W/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "name: \"abhishek_drone\", allow_renaming: false, sdf: '$SDF'"
sleep 1
topics="$(gz topic -l 2>/dev/null || true)"
grep -q "^/nidar/scan$" <<< "$topics" && echo "[rig] /nidar/scan present" || echo "[WARN] /nidar/scan not listed yet"

ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:="$NIDAR_REPO/gazebo/config/nidar_gz_bridge_rig.yaml" & PIDS+=($!)
echo "[rig] running. Ctrl+C here stops Gazebo and the bridge."
wait
