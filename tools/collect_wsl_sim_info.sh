#!/usr/bin/env bash
# =====================================================================
# Collect the existing Gazebo / PX4 / ROS 2 simulation setup from WSL.
# READ ONLY on the WSL side: it only reads files and copies them into
#   /mnt/d/NIDAR_DRONE_PROJECT/gazebo_existing/
# Run in WSL:   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/collect_wsl_sim_info.sh
# =====================================================================
set +e
OUT=/mnt/d/NIDAR_DRONE_PROJECT/gazebo_existing
INFO="$OUT/wsl_sim_info.txt"
PX4="$HOME/PX4-Autopilot"
mkdir -p "$OUT"

{
echo "== date: $(date)"
echo "== Ubuntu: $(grep PRETTY_NAME /etc/os-release)"
echo
echo "== Gazebo"
command -v gz && gz sim --versions 2>&1 | head -5
echo
echo "== ROS 2 (sourced for this check)"
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "ROS_DISTRO=${ROS_DISTRO:-not set}"
ros2 pkg list 2>/dev/null | grep -E "^ros_gz|^ros_gz_bridge|^ros_gz_image|^ros_gz_sim|^cv_bridge|^px4_msgs" || echo "(no ros_gz / cv_bridge packages found)"
echo
echo "== PX4"
if [ -d "$PX4" ]; then
  git -C "$PX4" describe --tags --always 2>&1
  git -C "$PX4" status --short 2>/dev/null | head -30
  echo "-- gz launch variables in px4-rc.gzsim:"
  grep -n -E "PX4_GZ_[A-Z_]+|PX4_SIM_MODEL|PX4_GZ_WORLD" "$PX4/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim" 2>/dev/null | head -40
  echo "-- gz airframes:"
  ls "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes" 2>/dev/null | grep -i gz
  echo "-- gz models:"
  ls "$PX4/Tools/simulation/gz/models" 2>/dev/null | tr '\n' ' '; echo
  echo "-- gz worlds:"
  ls "$PX4/Tools/simulation/gz/worlds" 2>/dev/null | tr '\n' ' '; echo
else
  echo "PX4-Autopilot not found at $PX4"
fi
echo
echo "== Anything named *nidar* in the home folder (build/.git excluded)"
find "$HOME" -maxdepth 7 -iname "*nidar*" -not -path "*/build/*" -not -path "*/.git/*" \
     -not -path "*/install/*" -not -path "*/log/*" 2>/dev/null | head -60
echo
echo "== Files that mention the existing world / model names"
grep -rIl -E "nidar_indoor_rescue|x500_nidar_indoor_test" "$HOME" \
     --exclude-dir=build --exclude-dir=.git --exclude-dir=install --exclude-dir=log \
     --exclude-dir=.cache 2>/dev/null | head -40
echo
echo "== How the simulation was launched (shell history)"
grep -E "px4_sitl|PX4_GZ|PX4_SIM_MODEL|gz sim|ros_gz_bridge|parameter_bridge|gazebo_camera_relay|MicroXRCE" \
     "$HOME/.bash_history" 2>/dev/null | tail -40
} > "$INFO" 2>&1

# Copy the existing NIDAR world/model/airframe files (not the build output)
grep -rIl -E "nidar_indoor_rescue|x500_nidar_indoor_test" "$HOME" \
     --exclude-dir=build --exclude-dir=.git --exclude-dir=install --exclude-dir=log \
     --exclude-dir=.cache 2>/dev/null | head -40 | while IFS= read -r f; do
  rel="${f#$HOME/}"
  mkdir -p "$OUT/files/$(dirname "$rel")"
  cp "$f" "$OUT/files/$rel"
done
find "$HOME" -maxdepth 7 -type d -iname "*nidar*" -path "*models*" -not -path "*/build/*" 2>/dev/null | head -10 | while IFS= read -r d; do
  rel="${d#$HOME/}"
  mkdir -p "$OUT/files/$rel"
  cp -r "$d/." "$OUT/files/$rel/"
done

echo "Saved: $INFO"
echo "Copied files: $(find "$OUT/files" -type f 2>/dev/null | wc -l)"
