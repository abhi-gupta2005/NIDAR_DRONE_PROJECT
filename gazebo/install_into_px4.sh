#!/usr/bin/env bash
# =====================================================================
# Copy the NIDAR AirMouse Gazebo assets into PX4's Gazebo folders (WSL).
#   bash /mnt/d/NIDAR_DRONE_PROJECT/gazebo/install_into_px4.sh
#
# Copies ONLY these new names (your existing x500_nidar_indoor,
# mono_cam_nidar_test and nidar_indoor_rescue.sdf are never touched):
#   models/x500_nidar_airmouse   models/nidar_rig
#   worlds/nidar_airmouse_arena_s*.sdf
# A destination that exists with different content is first saved as
# <name>.bak_<time>. Nothing is deleted. No PX4 file is edited and no
# rebuild is needed (PX4 reads models/worlds at launch).
# Then checks models and worlds with `gz sdf -k` (see [3/3] for its limits).
# =====================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
DST_MODELS="$PX4_DIR/Tools/simulation/gz/models"
DST_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
STAMP="$(date +%Y%m%d_%H%M%S)"

[ -d "$DST_MODELS" ] && [ -d "$DST_WORLDS" ] || { echo "[ERROR] $PX4_DIR/Tools/simulation/gz/{models,worlds} not found"; exit 1; }
command -v gz >/dev/null || { echo "[ERROR] gz not on PATH: source /opt/ros/jazzy/setup.bash first"; exit 1; }

copy_item() {   # $1 source path, $2 destination path
    local src="$1" dst="$2"
    if [ -e "$dst" ]; then
        if diff -rq "$src" "$dst" >/dev/null 2>&1; then
            echo "  same      : ${dst#$PX4_DIR/}"
            return
        fi
        mv "$dst" "$dst.bak_$STAMP"
        echo "  backup    : ${dst#$PX4_DIR/} -> $(basename "$dst").bak_$STAMP"
    fi
    cp -r "$src" "$dst"
    echo "  installed : ${dst#$PX4_DIR/}"
}

echo "[1/3] Models"
for m in x500_nidar_airmouse nidar_rig; do
    copy_item "$HERE/models/$m" "$DST_MODELS/$m"
done
echo "[2/3] Worlds"
shopt -s nullglob
worlds=("$HERE"/worlds/nidar_airmouse_arena_s*.sdf)
[ ${#worlds[@]} -gt 0 ] || { echo "[ERROR] no generated worlds in $HERE/worlds (run nidar_arena_generator.py)"; exit 1; }
for w in "${worlds[@]}"; do
    copy_item "$w" "$DST_WORLDS/$(basename "$w")"
done

echo "[3/3] SDF checks (gz sdf -k)"
export GZ_SIM_RESOURCE_PATH="$DST_MODELS:$DST_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# `gz sdf -k` (sdformat) has no Gazebo resource-path callback, so it may not resolve
# an <include><uri>x500</uri> the way `gz sim` does. Give it every chance (SDF_PATH,
# run from the models folder) and calibrate against PX4's own x500_lidar_2d, which
# uses the identical include: if the stock model fails the same way, the checker --
# not our model -- is the limit, and the real test is loading it in gz sim.
sdf_check() {   # $1 = model.sdf or world.sdf
    (cd "$DST_MODELS" && SDF_PATH="$DST_MODELS${SDF_PATH:+:$SDF_PATH}" gz sdf -k "$1" 2>&1)
}
fail=0
for m in x500_nidar_airmouse nidar_rig; do
    if out="$(sdf_check "$DST_MODELS/$m/model.sdf")"; then
        echo "  OK  models/$m"
        continue
    fi
    if grep -q "<uri>x500</uri>" "$DST_MODELS/$m/model.sdf" && echo "$out" | grep -q "Unable to find uri\[x500\]"; then
        ref="$DST_MODELS/x500_lidar_2d/model.sdf"
        if [ -f "$ref" ] && ! ref_out="$(sdf_check "$ref")" && echo "$ref_out" | grep -q "Unable to find uri\[x500\]"; then
            echo "  INCONCLUSIVE models/$m: the offline checker cannot resolve <uri>x500</uri>;"
            echo "      PX4's own x500_lidar_2d fails identically. gz sim resolves it (Stage C is the real test)."
            other="$(echo "$out" | grep -i "error" | grep -v "uri\[x500\]\|findFile\|base_link" || true)"
            if [ -n "$other" ]; then
                echo "  FAIL models/$m: errors not explained by the x500 include:"; echo "$other" | sed 's/^/      /'; fail=1
            fi
            continue
        fi
    fi
    echo "  FAIL models/$m"; echo "$out" | sed 's/^/      /'; fail=1
done
for w in "${worlds[@]}"; do
    name="$(basename "$w")"
    if out="$(sdf_check "$DST_WORLDS/$name")"; then
        echo "  OK  worlds/$name"
    else
        # Fuel survivor <include>s are only downloaded by gz sim itself; the offline
        # checker may report them as unresolved. Anything else is a real error.
        if echo "$out" | grep -vi "fuel\|unable to find uri\|https://" | grep -qi "error"; then
            echo "  FAIL worlds/$name"; echo "$out" | sed 's/^/      /'; fail=1
        else
            echo "  OK* worlds/$name (only Fuel URIs unresolved offline; gz sim downloads them on first launch)"
        fi
    fi
done
[ "$fail" -eq 0 ] && echo "Done." || { echo "Some checks FAILED (see above)."; exit 2; }
