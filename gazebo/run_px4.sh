#!/usr/bin/env bash
# =====================================================================
# PX4 SITL + x500_nidar_airmouse in the NIDAR arena (WSL).
#   bash /mnt/d/NIDAR_DRONE_PROJECT/gazebo/run_px4.sh [seed]
#   HEADLESS=1 (no Gazebo window, less load), PX4_DAEMON=1 (no pxh> shell: for tools/nidar_run.sh)
# Uses the stock x500 airframe (PX4_SYS_AUTOSTART=4001) with our model
# (PX4_SIM_MODEL=gz_x500_nidar_airmouse), so no PX4 file is edited and no
# airframe or rebuild is needed. This only STARTS the simulator: nothing here
# arms, takes off or enables offboard (px4_setpoint_sim keeps ALLOW_ARM and
# ALLOW_OFFBOARD False).
# Needs: install_into_px4.sh run once; PX4 built before (build/px4_sitl_default).
# =====================================================================
set -uo pipefail
SEED="${1:-1}"
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/nidar_gz_env.sh"
W="nidar_airmouse_arena_s$SEED"
LAYOUT="$NIDAR_REPO/gazebo/layouts/$W.json"
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || { echo "[ERROR] PX4 not built: cd $PX4_DIR && make px4_sitl"; exit 1; }
[ -f "$PX4_DIR/Tools/simulation/gz/worlds/$W.sdf" ] || { echo "[ERROR] run gazebo/install_into_px4.sh first"; exit 1; }
POSE="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['spawn_pose']['PX4_GZ_MODEL_POSE'])" "$LAYOUT")"
echo "[px4] world $W  model x500_nidar_airmouse  pose $POSE"
cd "$PX4_DIR"
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500_nidar_airmouse \
PX4_GZ_WORLD="$W" \
PX4_GZ_MODEL_POSE="$POSE" \
exec ./build/px4_sitl_default/bin/px4 ${PX4_DAEMON:+-d}
