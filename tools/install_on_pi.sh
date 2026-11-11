#!/usr/bin/env bash
# =====================================================================
# Install the NIDAR update into ~/drone_project (Raspberry Pi).
# Version: 2026-11-09d "explorer + alignment fix + PAD label + brain staleness" (also installs over 2026-11-09c/b, 2026-11-09,
# 2026-11-02e/d/c/b, 2026-11-02, or the files audited before them).
#
# Usage (after unpacking the bundle into ~/nidar_install):
#     bash ~/nidar_install/tools/install_on_pi.sh
#
# Safety rules:
#   1. The Pi files CHANGED by this update (V18, nidar_airmouse_server.py,
#      start_nidar_pi.sh, SLAM V5.1, integration_monitor.py) are only replaced
#      if they are byte-identical to a known version: the one audited on this
#      Pi on 2026-11-02, or the one installed by the 2026-11-02 update. If you
#      edited them since, the script stops and changes nothing.
#   2. Every file that would be overwritten is first copied to
#      ~/drone_project/archive/pre_install_<date-time>/ (same sub-paths).
#   3. Nothing is deleted. Nothing is started.
#   4. It asks for confirmation before copying.
# =====================================================================

set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="${NIDAR_PROJECT_DIR:-$HOME/drone_project}"
STAMP="$(date +%Y-%m-%d_%H%M%S)"
BACKUP_DIR="$PROJECT_DIR/archive/pre_install_$STAMP"

# Accepted sha256 of the Pi files before this update (space separated):
#   1st = audited on the Pi 2026-11-02 (original)
#   2nd = installed by nidar_pi_install_2026-11-02.tgz
#   3rd = installed by nidar_pi_install_2026-11-02b.tgz (only files changed since)
declare -A EXPECTED_ORIGINAL=(
  ["src/perception/nidar_v18_headless_persistent_fusion.py"]="83aad0965c712baac3d7fb27c6834160ab3fede646b66ab37d86d1c8489ac3f9 7b8fb89f534b132aa62c15c5940971db1986ad734c6f2d83fd14257941718a89"
  ["nidar_airmouse_server.py"]="c8ea1a5358d8b2c74a7d600f83591d7d8587e00ecfa3a7f7dfe0a0570fb1f164 0ca1ec2832e12242601500a32644036ba8dd130cb0f4aa1afd106e782e9a9c3d"
  ["start_nidar_pi.sh"]="53afbb0834da78eb7bf2ede987b78d5264443e2071f8a399ee2c54ececc66395 81371cf8ba8d5039cb1f98b8d0ad532cec6fe7dbf07fd8a48e5aaa2470b0e4ce 80ce1232fe67f2816301bb15128951bb25cb37dbc662de3a71e79e4bb3e817bd"
  ["src/core/nidar_slam_v5_1.py"]="003a395e5a3a196418dc10ef969ab3f6f075bf4d1a13002715b3892e807bfb54 7395c198abf213f3430824c64aa50b18100913d274294514a6773b59d5cbcb25"
  ["integration_monitor.py"]="2c34b844f80c5a0e7880b8f8631f5cd3cfa5bd252dceb2e95d898ce7cdf7c409 ddc329a3d32597eb27a7e631cb5b12e0f8b71bce8bdbb8f8e299fb976c601a3d"
)

echo "============================================================"
echo " NIDAR UPDATE 2026-11-09d (explorer, alignment fix, PAD label, brain staleness) -> $PROJECT_DIR"
echo "============================================================"
[ -d "$PROJECT_DIR" ] || { echo "[ERROR] $PROJECT_DIR not found"; exit 1; }

# Only real Python processes RUNNING the NIDAR scripts (not editors, tail, grep...).
RUNNING_RE='^[^ ]*python[0-9.]*( -[^ ]+)* +([^ ]*/)?(nidar_v18_headless_persistent_fusion\.py|nidar_airmouse_server\.py|nidar_pi_receiver\.py|nidar_lidar_receiver\.py|nidar_mission_server\.py|nidar_explorer_service\.py|integration_monitor\.py)|^[^ ]*python[0-9.]* .*-m +src\.core\.nidar_slam_v(5_1|6)'
if pgrep -f -- "$RUNNING_RE" >/dev/null 2>&1; then
  echo "[ERROR] NIDAR processes are running. Stop start_nidar_pi.sh (Ctrl+C) and any"
  echo "        manually started nidar_airmouse_server.py first, then run this again."
  pgrep -af -- "$RUNNING_RE" || true
  exit 1
fi

echo
echo "[1/4] Checking the files this update modifies ..."
blocked=0
for rel in "${!EXPECTED_ORIGINAL[@]}"; do
  target="$PROJECT_DIR/$rel"
  new="$BUNDLE_DIR/$rel"
  if [ ! -f "$target" ]; then
    echo "  MISSING on Pi      : $rel  (will be installed)"
    continue
  fi
  have="$(sha256sum "$target" | cut -d' ' -f1)"
  if cmp -s "$target" "$new"; then
    echo "  already up to date : $rel"
  elif [[ " ${EXPECTED_ORIGINAL[$rel]} " == *" $have "* ]]; then
    echo "  known version      : $rel  (will be updated)"
  else
    echo "  CHANGED SINCE AUDIT: $rel  sha256=$have"
    blocked=1
  fi
done
if [ "$blocked" -ne 0 ]; then
  echo
  echo "[STOP] A file was edited after it was audited. Nothing was changed."
  echo "       Copy it to the laptop (scp) so the update can be re-based on it."
  exit 2
fi

echo
echo "[2/4] Files to install:"
mapfile -t FILES < <(cd "$BUNDLE_DIR" && find . -type f ! -name '*.pyc' | sed 's|^\./||' | sort)
new_count=0; replace_count=0; same_count=0
for rel in "${FILES[@]}"; do
  if [ ! -e "$PROJECT_DIR/$rel" ]; then new_count=$((new_count + 1))
  elif cmp -s "$PROJECT_DIR/$rel" "$BUNDLE_DIR/$rel"; then same_count=$((same_count + 1))
  else replace_count=$((replace_count + 1)); echo "  replace: $rel"; fi
done
echo "  new files: $new_count   replaced: $replace_count   unchanged: $same_count"
echo "  backups of replaced files -> $BACKUP_DIR"

echo
read -r -p "Proceed? [y/N] " answer
[ "${answer:-N}" = "y" ] || [ "${answer:-N}" = "Y" ] || { echo "Cancelled. Nothing changed."; exit 0; }

echo
echo "[3/4] Installing ..."
for rel in "${FILES[@]}"; do
  src="$BUNDLE_DIR/$rel"; dst="$PROJECT_DIR/$rel"
  if [ -e "$dst" ] && ! cmp -s "$src" "$dst"; then
    mkdir -p "$(dirname "$BACKUP_DIR/$rel")"
    cp -p "$dst" "$BACKUP_DIR/$rel"
  fi
  if [ ! -e "$dst" ] || ! cmp -s "$src" "$dst"; then
    mkdir -p "$(dirname "$dst")"
    cp "$src" "$dst"
  fi
done
chmod +x "$PROJECT_DIR/start_nidar_pi.sh" "$PROJECT_DIR/nidar_airmouse_server.py" \
         "$PROJECT_DIR/src/communication/nidar_lidar_receiver.py" \
         "$PROJECT_DIR/tools/nidar_pi_status.sh" "$PROJECT_DIR/tools/install_on_pi.sh" 2>/dev/null || true
mkdir -p "$PROJECT_DIR/logs" "$PROJECT_DIR/outputs/slam/state" "$PROJECT_DIR/outputs/slam/maps" \
         "$PROJECT_DIR/outputs/slam/trajectories"

echo
echo "[4/4] Verifying ..."
for rel in "${!EXPECTED_ORIGINAL[@]}"; do
  cmp -s "$PROJECT_DIR/$rel" "$BUNDLE_DIR/$rel" && echo "  OK  $rel" || { echo "  FAILED  $rel"; exit 3; }
done
grep -q CAMERA_GRACE_SEC "$PROJECT_DIR/src/perception/nidar_v18_headless_persistent_fusion.py" && echo "  OK  V18.1 grace period present"
grep -q V18_STALE_SEC "$PROJECT_DIR/nidar_airmouse_server.py" && echo "  OK  airmouse server freshness checks present"
grep -q "nidar_airmouse_server.py" "$PROJECT_DIR/start_nidar_pi.sh" && echo "  OK  start_nidar_pi.sh launches the airmouse server"
grep -q "NIDAR_LIDAR_BACKEND" "$PROJECT_DIR/start_nidar_pi.sh" && echo "  OK  start_nidar_pi.sh has the GAZEBO LiDAR option (default SIMULATOR)"
grep -q "fmt_num" "$PROJECT_DIR/integration_monitor.py" && echo "  OK  integration monitor survives a missing SLAM state"
grep -q "NIDAR_SLAM" "$PROJECT_DIR/start_nidar_pi.sh" && [ -f "$PROJECT_DIR/src/core/nidar_slam_v6.py" ] && \
  [ -f "$PROJECT_DIR/src/mission/nidar_mission_server.py" ] && echo "  OK  SLAM V6 + mission server available (NIDAR_SLAM=V6)"
grep -q "MIN_INLIER_RATIO" "$PROJECT_DIR/src/mission/survivor_mapper.py" && \
  grep -q "write_journal" "$PROJECT_DIR/src/mission/nidar_mission_server.py" && \
  echo "  OK  robust survivor positions + measurement journal (outputs/mission/measurements_*.csv)"
[ -f "$PROJECT_DIR/src/mission/nidar_explorer_service.py" ] && grep -q "NIDAR_NAV_V1" "$PROJECT_DIR/src/mission/nidar_explorer_service.py" && \
  echo "  OK  explorer service (start by hand: python src/mission/nidar_explorer_service.py --radius 0.25)"

echo
echo "Done. To undo: copy the files from $BACKUP_DIR back into $PROJECT_DIR"
echo "      (new files can simply be left; they are not used by the old scripts)."
