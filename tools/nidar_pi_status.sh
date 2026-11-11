#!/usr/bin/env bash
# =====================================================================
# NIDAR Pi status snapshot -- READ ONLY.
#
# Prints the current state of the NIDAR project on this Raspberry Pi and
# saves the same text to ~/nidar_pi_status.txt.  It does not start, stop,
# install, edit or delete anything.
#
# Run on the Pi:   bash ~/nidar_pi_status.sh
# Optional:        NIDAR_PROJECT_DIR=/some/other/path bash ~/nidar_pi_status.sh
# =====================================================================

set +e
PROJECT_DIR="${NIDAR_PROJECT_DIR:-$HOME/drone_project}"
UPDATE_DIR="$HOME/nidar_update"
OUT="$HOME/nidar_pi_status.txt"
VENV_PY="$PROJECT_DIR/.venv/bin/python"
[ -x "$VENV_PY" ] || VENV_PY="$(command -v python3)"

section() { printf '\n==================== %s ====================\n' "$1"; }
have() { command -v "$1" >/dev/null 2>&1; }

main() {
section "SNAPSHOT"
echo "Taken      : $(date '+%Y-%m-%d %H:%M:%S %Z')  (epoch $(date +%s))"
echo "Host       : $(hostname)   user: $(whoami)"
echo "Project    : $PROJECT_DIR  $( [ -d "$PROJECT_DIR" ] && echo '[exists]' || echo '[MISSING]')"

section "SYSTEM"
grep -E '^PRETTY_NAME' /etc/os-release 2>/dev/null
echo "Kernel     : $(uname -r)  arch: $(uname -m)"
[ -r /proc/device-tree/model ] && echo "Board      : $(tr -d '\0' < /proc/device-tree/model)"
echo "Uptime     : $(uptime -p 2>/dev/null || uptime)"
have vcgencmd && echo "CPU temp   : $(vcgencmd measure_temp 2>/dev/null)  throttled: $(vcgencmd get_throttled 2>/dev/null)"
free -h 2>/dev/null | head -2
df -h "$HOME" 2>/dev/null | tail -1 | awk '{print "Disk (home): size "$2"  used "$3"  free "$4"  ("$5")"}'

section "NETWORK"
echo "Addresses  : $(hostname -I 2>/dev/null)"
have iwgetid && echo "Wi-Fi SSID : $(iwgetid -r 2>/dev/null)"
ip route 2>/dev/null | grep default

section "PYTHON"
echo "System python : $(python3 --version 2>&1)"
echo "Project python: $VENV_PY  -> $("$VENV_PY" --version 2>&1)"
"$VENV_PY" -B - <<'PY' 2>&1
# Read installed package metadata only -- importing e.g. ultralytics would write settings files.
from importlib import metadata
groups = {
    "numpy": ["numpy"],
    "opencv": ["opencv-python", "opencv-python-headless", "opencv-contrib-python"],
    "ultralytics": ["ultralytics"],
    "torch": ["torch"],
    "pyserial": ["pyserial"],
    "pytest": ["pytest"],
    "pyyaml": ["PyYAML"],
    "pyqtgraph": ["pyqtgraph"],
}
for label, dists in groups.items():
    found = []
    for d in dists:
        try:
            found.append(f"{d} {metadata.version(d)}")
        except metadata.PackageNotFoundError:
            pass
    print(f"  {label:12s} {', '.join(found) if found else 'NOT INSTALLED'}")
PY

section "PROJECT TOP LEVEL"
ls -la "$PROJECT_DIR" 2>&1 | grep -v -E ' \.\.?$'

section "PYTHON FILES (size, modified, path)"
find "$PROJECT_DIR" -name '*.py' -not -path '*/.venv/*' -not -path '*/__pycache__/*' \
     -printf '%8s  %TY-%Tm-%Td %TH:%TM  %P\n' 2>/dev/null | sort -k4 | head -120

section "OTHER NIDAR SCRIPTS IN HOME (outside the project)"
find "$HOME" -maxdepth 4 \( -name '*nidar*.py' -o -name '*airmouse*.py' -o -name 'start_nidar*.sh' \) \
     -not -path "$PROJECT_DIR/*" -not -path '*/.venv/*' -not -path "$UPDATE_DIR/*" \
     -printf '%8s  %TY-%Tm-%Td %TH:%TM  %p\n' 2>/dev/null | sort -k4

section "nidar_airmouse_server.py (if present anywhere)"
found=0
while IFS= read -r f; do
  found=1
  echo "--- $f  ($(wc -l < "$f") lines, modified $(date -r "$f" '+%Y-%m-%d %H:%M'))"
  grep -nE 'PORT|HOST|bind\(|listen\(|connect|socket\.|json|target|open\(' "$f" | head -40
done < <(find "$HOME" -name 'nidar_airmouse_server.py' -not -path '*/.venv/*' 2>/dev/null)
[ $found -eq 0 ] && echo "not found"

section "KEY FILES: checksum + modified time"
KEY_FILES="src/communication/nidar_pi_receiver.py
src/communication/nidar_target_server.py
src/perception/nidar_v18_headless_persistent_fusion.py
src/core/nidar_slam_v5_1.py
src/core/lidar_interface.py
src/simulation/nidar_lidar_simulator.py
src/tracking/lidar_obstacle_detector.py
src/tracking/lidar_obstacle_tracker_v3.py
integration_monitor.py
start_nidar_pi.sh"
while IFS= read -r rel; do
  f="$PROJECT_DIR/$rel"
  if [ -f "$f" ]; then
    printf '%s  %s  %s\n' "$(sha256sum "$f" | cut -c1-12)" "$(date -r "$f" '+%Y-%m-%d %H:%M')" "$rel"
  else
    printf '%-12s  %-16s  %s\n' "MISSING" "" "$rel"
  fi
done <<< "$KEY_FILES"

section "COMPARED WITH THE WINDOWS COPY I AUDITED (needs ~/nidar_update)"
if [ -d "$UPDATE_DIR/archive/pre_session_2026-11-01" ]; then
  for rel in src/perception/nidar_v18_headless_persistent_fusion.py src/core/nidar_slam_v5_1.py start_nidar_pi.sh; do
    pi="$PROJECT_DIR/$rel"; base="$(basename "$rel")"
    if cmp -s "$pi" "$UPDATE_DIR/archive/pre_session_2026-11-01/$base"; then
      echo "SAME as audited original : $rel"
    elif cmp -s "$pi" "$UPDATE_DIR/$rel"; then
      echo "ALREADY UPDATED          : $rel"
    else
      echo "DIFFERENT (Pi has other edits): $rel"
      diff <(sed 's/\r$//' "$UPDATE_DIR/archive/pre_session_2026-11-01/$base") "$pi" | head -30
    fi
  done
  for rel in src/communication/nidar_pi_receiver.py src/core/lidar_interface.py src/tracking/lidar_obstacle_tracker_v3.py src/tracking/lidar_obstacle_detector.py src/simulation/nidar_lidar_simulator.py integration_monitor.py; do
    if [ -f "$UPDATE_DIR/$rel" ]; then
      cmp -s "$PROJECT_DIR/$rel" "$UPDATE_DIR/$rel" && echo "SAME as Windows copy     : $rel" || echo "DIFFERENT from Windows   : $rel"
    fi
  done
else
  echo "~/nidar_update not found -- comparison skipped (copy nidar_update.tgz first)"
fi

section "KEY SETTINGS IN THE CODE"
grep -nE '^LIDAR_BACKEND' "$PROJECT_DIR/src/core/nidar_slam_v5_1.py" 2>/dev/null
grep -nE '^PORT|NIDAR_PI_PORT' "$PROJECT_DIR/src/communication/nidar_pi_receiver.py" 2>/dev/null
grep -nE '^(FUSION_GATE_DEG|TEMP_LOST_TIMEOUT_SEC|EXPIRE_TIMEOUT_SEC|CAMERA_GRACE_SEC|LOOP_DELAY|CAMERA_HFOV_DEG)|"~/drone_project"|NIDAR_PROJECT_DIR' \
     "$PROJECT_DIR/src/perception/nidar_v18_headless_persistent_fusion.py" 2>/dev/null
grep -q CAMERA_GRACE_SEC "$PROJECT_DIR/src/perception/nidar_v18_headless_persistent_fusion.py" 2>/dev/null \
  && echo "V18 version: V18.1 (grace-period fix present)" || echo "V18 version: original (no grace-period fix)"
grep -nE 'PYTHONUNBUFFERED|nidar_target_server' "$PROJECT_DIR/start_nidar_pi.sh" 2>/dev/null || echo "start_nidar_pi.sh: original (no target server, buffered logs)"

section "RUNNING NIDAR PROCESSES"
ps -eo pid,etime,pcpu,pmem,args 2>/dev/null | grep -E 'nidar|slam|integration_monitor|airmouse|MicroXRCE' | grep -v -E 'grep|nidar_pi_status' || echo "none running"

section "LISTENING TCP PORTS"
ss -ltnp 2>/dev/null | awk 'NR==1 || /:(22|5000|5010|5012|8888)\b/' ; echo "(all listening ports:)"; ss -ltn 2>/dev/null | awk 'NR>1{print $4}' | sort -u | tr '\n' ' '; echo

section "DOCKER"
if have docker; then docker ps -a --format '{{.Names}}  {{.Image}}  {{.Status}}' 2>&1 | head -10; else echo "docker not installed"; fi

section "DATA FILES: freshness"
"$VENV_PY" - "$PROJECT_DIR" <<'PY' 2>&1
import json, os, sys, time
root = sys.argv[1]
now = time.time()
files = [
    ("perception (receiver output)", "src/communication/nidar_latest_perception.json"),
    ("V18 fusion output",            "nidar_v18_headless_persistent_fusion.json"),
    ("SLAM state",                   "outputs/slam/state/nidar_slam_v5_1_state.json"),
]
def age(sec):
    if sec < 120: return f"{sec:.1f} s"
    if sec < 7200: return f"{sec/60:.1f} min"
    if sec < 172800: return f"{sec/3600:.1f} h"
    return f"{sec/86400:.1f} days"
for label, rel in files:
    path = os.path.join(root, rel)
    print(f"-- {label}: {rel}")
    if not os.path.exists(path):
        print("   MISSING"); continue
    print(f"   file modified {age(now - os.path.getmtime(path))} ago, {os.path.getsize(path)} bytes")
    try:
        d = json.load(open(path, encoding="utf-8"))
    except Exception as exc:
        print(f"   cannot parse: {exc}"); continue
    ts = d.get("timestamp")
    print(f"   protocol={d.get('protocol')}  frame={d.get('frame')}  data timestamp age={age(now - float(ts)) if ts else 'n/a'}")
    if "camera" in d:
        print(f"   camera={d['camera']}  objects={len(d.get('objects', []))}")
    if "persistent_objects" in d:
        po = d.get("persistent_objects", [])
        print(f"   camera_source={d.get('camera_source')} lidar_source={d.get('lidar_source')}  "
              f"camera_objects={len(d.get('camera_objects', []))} lidar_tracks={len(d.get('lidar_tracks', []))} "
              f"fused={len(d.get('fused_objects', []))} persistent={[(o.get('stable_id'), o.get('class'), o.get('state')) for o in po][:6]}")
    if "pose" in d:
        print(f"   lidar_source={d.get('lidar_source')} pose={d.get('pose')} stats={d.get('statistics')}")
PY

section "OUTPUT FILES (size, modified)"
for rel in src/communication/nidar_perception_objects.csv nidar_v18_headless_persistent_fusion.csv \
           outputs/slam/trajectories/nidar_slam_v5_1_trajectory.csv outputs/slam/maps/nidar_slam_v5_1_map.pgm; do
  f="$PROJECT_DIR/$rel"
  [ -f "$f" ] && printf '%10s  %s  %s\n' "$(du -h "$f" | cut -f1)" "$(date -r "$f" '+%Y-%m-%d %H:%M')" "$rel" || echo "   missing  $rel"
done

section "LOGS (last 12 lines each)"
for log in "$PROJECT_DIR"/logs/*.log; do
  [ -f "$log" ] || { echo "no logs in $PROJECT_DIR/logs"; break; }
  echo "--- $(basename "$log")  ($(du -h "$log" | cut -f1), modified $(date -r "$log" '+%Y-%m-%d %H:%M'))"
  tail -n 12 "$log" | cut -c1-200
done

section "HARDWARE INTERFACES"
serial_found=0
for d in /dev/serial* /dev/ttyAMA* /dev/ttyS0 /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$d" ] && { ls -l "$d"; serial_found=1; }
done
[ $serial_found -eq 0 ] && echo "no serial devices found"
ls /dev/video* 2>/dev/null | tr '\n' ' ' || true; echo
echo "Groups: $(id -nG)"

section "AUTOSTART (services / cron)"
systemctl list-unit-files --type=service 2>/dev/null | grep -i -E 'nidar|drone|airmouse' || echo "no nidar systemd services"
crontab -l 2>/dev/null | grep -v '^#' | grep . || echo "no crontab entries"

section "GIT"
if [ -d "$PROJECT_DIR/.git" ]; then git -C "$PROJECT_DIR" log --oneline -5; git -C "$PROJECT_DIR" status --short | head -20; else echo "not a git repository"; fi

section "END"
echo "Saved to $OUT"
}

main 2>&1 | tee "$OUT"
