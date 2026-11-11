# NIDAR Gazebo environment for WSL (2026-11-02).
#   source /mnt/d/NIDAR_DRONE_PROJECT/gazebo/nidar_gz_env.sh
# Sets: ROS 2 Jazzy + px4_ros2_ws, Gazebo resource paths (PX4 models/worlds),
#       PX4's server.config (so worlds without <plugin> get IMU/baro/mag/sensors),
#       NIDAR_REPO, PX4_DIR.  Changes nothing on disk.

NIDAR_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export NIDAR_REPO PX4_DIR

# ROS setup scripts read variables that may be unset; under `set -u` (used by
# run_rig.sh / run_px4.sh) that aborted the caller with
# "AMENT_TRACE_SETUP_FILES: unbound variable" (seen 2026-11-02). Turn -u off
# while sourcing them and restore it afterwards.
case $- in *u*) _nidar_restore_u=1; set +u ;; *) _nidar_restore_u=0 ;; esac
NIDAR_ROS_SETUP="${NIDAR_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ -f "$NIDAR_ROS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$NIDAR_ROS_SETUP"
else
    echo "[nidar_gz_env] $NIDAR_ROS_SETUP not found" >&2
fi
if [ -f "$HOME/px4_ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$HOME/px4_ros2_ws/install/setup.bash"
fi
[ "$_nidar_restore_u" = 1 ] && set -u
unset _nidar_restore_u

GZ_MODELS="$PX4_DIR/Tools/simulation/gz/models"
GZ_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
case ":${GZ_SIM_RESOURCE_PATH:-}:" in
    *":$GZ_MODELS:"*) ;;
    *) export GZ_SIM_RESOURCE_PATH="$GZ_MODELS:$GZ_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" ;;
esac
if [ -f "$PX4_DIR/Tools/simulation/gz/server.config" ]; then
    export GZ_SIM_SERVER_CONFIG_PATH="$PX4_DIR/Tools/simulation/gz/server.config"
fi

echo "[nidar_gz_env] NIDAR_REPO=$NIDAR_REPO"
echo "[nidar_gz_env] PX4_DIR=$PX4_DIR  ROS_DISTRO=${ROS_DISTRO:-not set}"
echo "[nidar_gz_env] GZ_SIM_SERVER_CONFIG_PATH=${GZ_SIM_SERVER_CONFIG_PATH:-<not found: worlds without plugins get gz defaults>}"
