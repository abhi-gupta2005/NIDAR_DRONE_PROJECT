#!/usr/bin/env bash
# =====================================================================
# NIDAR one-command run (WSL), 2026-11-09. ONE terminal instead of nine.
#
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh setup       once: password-free SSH to the Pi,
#                                                                   QGroundControl, copy of PX4's x500 files
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh update-pi   copy + install the newest Pi bundle
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh install-pi  a NEW Pi: copy the project + Python packages
#   (after tools/install_wsl.sh the short command `nidar` does the same: nidar setup, nidar install-pi, nidar)
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh             AUTONOMOUS: brain on the Pi flies the rig
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh px4         PX4 SITL + QGroundControl + route flyer
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/nidar_run.sh stop        stop everything (also from another terminal)
#
# Options: --seed N (1)  --pi HOST (10.218.70.147)  --user NAME (abhishek)  --minutes M (45, brain budget)
#          --radius R (0.30, brain)  --route rooms|survivor:ID (px4)  --no-gui (no Gazebo window)
#          --no-yolo  --no-gcs  --no-qgc  --dry-run (print what would start)
#          --classic-gcs (the previous ground station window instead of the AirMouse app)
#          setup --pi HOST --user NAME also saves the Pi's address + user for every later command (~/.nidar_pi)
#          install-pi --force: copy the project again onto a Pi that already has ~/drone_project
#
# What it does: stops left-overs (here, on the Pi, YOLO on Windows), starts the Pi stack (+ brain) over SSH,
# Gazebo (+ PX4), relays, AirMouse bridge, YOLO in its own Windows window, the ground station (+ QGC), waits
# for SLAM TRACKING, then starts the flight program. Every 5 s one status line; a dropped link is printed as
# "LOST: <which link>". Relays/bridge/rig driver are restarted if they die (never the PX4 flyer). Ctrl+C:
# flight program first (rig stops / PX4 lands), then the Pi (its files are copied here), then the rest.
# Everything of a run lands in logs/run_<time>_<mode>/ (Pi logs + journal + SLAM trajectory in pi/, the ground
# station's flight record in gcs/ -- replay it on the app's Logs screen).
# The Gazebo window opens by default (--no-gui: without). 2026-11-09 PX4 runs were just as slow without it
# (0.03-0.12x real time); the rig runs without QGroundControl ran at 0.5x: QGC in WSL is the big extra load.
# =====================================================================
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Pi address/user: --pi/--user > NIDAR_PI_HOST/NIDAR_PI_USER > saved by `setup --pi --user` (~/.nidar_pi) > defaults
SAVED_HOST="$(sed -n 's/^NIDAR_PI_HOST=//p' "$HOME/.nidar_pi" 2> /dev/null | tail -1)"
SAVED_USER="$(sed -n 's/^NIDAR_PI_USER=//p' "$HOME/.nidar_pi" 2> /dev/null | tail -1)"
MODE="rig"; SEED=1; PI_HOST="${NIDAR_PI_HOST:-${SAVED_HOST:-10.218.70.147}}"; PI_USER="${NIDAR_PI_USER:-${SAVED_USER:-abhishek}}"
MINUTES=45; RADIUS=0.30; ROUTE="rooms"; GUI=1; YOLO=1; GCS=1; QGC=1; DRY=""; CLASSIC_GCS=""; FORCE=""; PI_GIVEN=""
TEST="${NIDAR_RUN_TEST:-}"                       # tests: components are stand-ins
QGC_APP="${NIDAR_QGC_APP:-$HOME/QGroundControl-x86_64.AppImage}"
STATE_DIR="${NIDAR_RUN_STATE:-$HOME/.nidar_run}"
LOGS="${NIDAR_RUN_LOGS:-$REPO/logs}"
MAX_RESTARTS=5

while [ $# -gt 0 ]; do
    case "$1" in
        rig|px4|stop|setup|update-pi|install-pi) MODE="$1" ;;
        --seed) SEED="$2"; shift ;;
        --pi) PI_HOST="$2"; PI_GIVEN=1; shift ;;
        --user) PI_USER="$2"; PI_GIVEN=1; shift ;;
        --force) FORCE=1 ;;
        --minutes) MINUTES="$2"; shift ;;
        --radius) RADIUS="$2"; shift ;;
        --route) ROUTE="$2"; shift ;;
        --gui) GUI=1 ;;
        --no-gui) GUI="" ;;
        --no-yolo) YOLO="" ;;
        --no-gcs) GCS="" ;;
        --classic-gcs) CLASSIC_GCS=1 ;;
        --no-qgc) QGC="" ;;
        --dry-run) DRY=1 ;;
        -h|--help) sed -n '2,32p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "[nidar] unknown argument: $1 (see --help)"; exit 2 ;;
    esac
    shift
done

PI="$PI_USER@$PI_HOST"
W="nidar_airmouse_arena_s$SEED"
say() { echo "[$(date +%H:%M:%S)] $*"; }
launcher_alive() { [ -n "${1:-}" ] && ps -o args= -p "$1" 2> /dev/null | grep -q "nidar_run[.]sh"; }
mkdir -p "$STATE_DIR"

# ---------------------------------------------------------------- environment --
if [ -z "$TEST" ] && [ -z "$DRY" ] && [ "$MODE" != "stop" ]; then
    # shellcheck disable=SC1091
    source "$REPO/gazebo/nidar_gz_env.sh" > /dev/null
fi
export PYTHONUNBUFFERED=1

pi() { ssh -o BatchMode=yes -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 "$PI" "$@"; }
win_ps() { (cd /mnt/c 2>/dev/null || cd /; powershell.exe -NoProfile -ExecutionPolicy Bypass "$@") < /dev/null; }
win_repo() { if command -v wslpath > /dev/null; then wslpath -w "$REPO"; else echo "D:\\NIDAR_DRONE_PROJECT"; fi; }

# Process-name patterns: [x] keeps pkill -f from matching the shell that runs it (over SSH too)
LOCAL_PATTERNS=('nidar_rig_driver[.]py' 'nidar_px4_route_flyer[.]py' 'nidar_lidar_relay[.]py'
                'nidar_camera_relay[.]py' 'nidar_gcs[.]py' 'airmouse_gcs[.]py' 'nidar_status[.]py' 'run_rig[.]sh' 'run_px4[.]sh'
                'parameter_bridg[e]' 'nidar_bridg[e]' 'MicroXRCEAgen[t]' 'QGroundContro[l]' 'gz si[m]'
                'px4_sitl_default/bin/px[4]')
PI_STOP='pkill -INT -f "nidar_explorer_service[.]py"; pkill -TERM -f "start_nidar_pi[.]sh"; sleep 1;
         pkill -f "integration_monito[r].py"; sleep 4;
         for p in "nidar_pi_receive[r].py" "nidar_v18_headless_persistent_fusio[n].py" "nidar_slam_v[56]" \
                  "nidar_mission_serve[r].py" "nidar_airmouse_serve[r].py" "nidar_lidar_receive[r].py" \
                  "nidar_explorer_servic[e].py"; do pkill -f "$p"; done; true'

stop_local_leftovers() {
    for p in "${LOCAL_PATTERNS[@]}"; do pkill -INT -f "$p" 2> /dev/null; done
    sleep 2
    for p in "${LOCAL_PATTERNS[@]}"; do pkill -KILL -f "$p" 2> /dev/null; done
    true
}
stop_pi() { pi "$PI_STOP" > /dev/null 2>&1 || say "[WARN] could not reach the Pi to stop it ($PI)"; }
stop_yolo() { [ -n "$TEST" ] || command -v powershell.exe > /dev/null || return 0
              win_ps -File "$(win_repo)\\tools\\nidar_yolo_stop.ps1" > /dev/null 2>&1 || true; }

# ---------------------------------------------------------------- one-off commands --
if [ "$MODE" = "stop" ]; then
    LP="$(cat "$STATE_DIR/launcher.pid" 2> /dev/null || true)"
    if launcher_alive "$LP"; then
        say "asking the running launcher to stop everything (it lands / stops the flight first)..."
        kill -INT "$LP"
        pkill -INT -P "$LP" 2> /dev/null     # a child it waits on (ssh, status check) must end too
        for ((i = 0; i < 150; i++)); do
            st="$(ps -o stat= -p "$LP" 2> /dev/null)"
            [ -z "$st" ] || [[ "$st" == Z* ]] && { say "stopped"; exit 0; }      # gone (or exited, not yet reaped)
            sleep 1
        done
        say "the launcher did not stop in 150 s: ending it and cleaning up directly"
        pkill -KILL -P "$LP" 2> /dev/null; kill -KILL "$LP" 2> /dev/null
    fi
    rm -f "$STATE_DIR/launcher.pid"
    say "stopping left-overs here, on the Pi ($PI) and YOLO on Windows"
    stop_local_leftovers; stop_pi; stop_yolo
    say "stopped"; exit 0
fi

if [ "$MODE" = "setup" ]; then
    say "1/3 password-free SSH to the Pi ($PI)"
    [ -f "$HOME/.ssh/id_ed25519" ] || ssh-keygen -t ed25519 -N "" -q -f "$HOME/.ssh/id_ed25519"
    if pi true 2> /dev/null; then say "    already works"
    else say "    type the Pi password ONE last time:"; ssh-copy-id -i "$HOME/.ssh/id_ed25519.pub" "$PI"; fi
    if [ -n "$PI_GIVEN" ]; then                      # 2026-11-11: remember the Pi for every later command
        printf 'NIDAR_PI_HOST=%s\nNIDAR_PI_USER=%s\n' "$PI_HOST" "$PI_USER" > "$HOME/.nidar_pi"
        say "    saved: every later command uses Pi $PI_USER@$PI_HOST (~/.nidar_pi)"
    fi
    say "2/3 QGroundControl (PX4's ground station) -> $QGC_APP"
    if [ ! -x "$QGC_APP" ]; then
        sudo apt-get install -y libfuse2t64 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 ||
            sudo apt-get install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
        wget -O "$QGC_APP" https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage &&
            chmod +x "$QGC_APP"
    else say "    already installed"; fi
    say "3/3 copy PX4's x500 / optical-flow / rangefinder files into the project (read-only on the PX4 side)"
    PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"; OUT="$REPO/gazebo_existing/files/px4_models"; mkdir -p "$OUT"
    for m in x500 x500_base x500_flow optical_flow LW20; do
        [ -d "$PX4_DIR/Tools/simulation/gz/models/$m" ] && cp -r "$PX4_DIR/Tools/simulation/gz/models/$m" "$OUT/"
    done
    for f in ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500 \
             ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_flow \
             src/modules/uxrce_dds_client/dds_topics.yaml; do
        [ -f "$PX4_DIR/$f" ] && cp "$PX4_DIR/$f" "$OUT/"
    done
    say "setup done. Start a run with:  bash $REPO/tools/nidar_run.sh"
    exit 0
fi

if [ "$MODE" = "install-pi" ]; then              # 2026-11-11: a NEW Pi (the friend's): project + Python packages
    pi true 2> /dev/null || { say "[ERROR] no password-free SSH to $PI: first run  $0 setup --pi $PI_HOST --user $PI_USER"; exit 1; }
    if pi "test -e ~/drone_project/start_nidar_pi.sh" && [ -z "$FORCE" ]; then
        say "[ERROR] $PI already has ~/drone_project. This is for a NEW Pi; an existing one: $0 update-pi"
        say "        (to copy the project over it anyway: $0 install-pi --force)"; exit 1
    fi
    say "1/2 copying the project to $PI:~/drone_project (code, worlds, tests; no logs, videos or models)"
    tar -C "$REPO" -czf - --exclude=.git --exclude=.venv --exclude=__pycache__ --exclude='logs*' --exclude=outputs \
        --exclude=recordings --exclude=archive --exclude=gazebo_existing --exclude=pi_files --exclude='Claude outputs' \
        --exclude=gcs_reports --exclude=gcs_records --exclude=training --exclude=models --exclude='*.tgz' \
        --exclude='*.mp4' --exclude='*.pt' --exclude='*.ts' --exclude=ros2_ws_src/px4_msgs . |
        pi "mkdir -p ~/drone_project && tar -xzf - -C ~/drone_project" || { say "[ERROR] copying to the Pi failed"; exit 1; }
    say "2/2 Python packages on the Pi (it may ask for the Pi password for sudo)"
    ssh -t -o ConnectTimeout=6 "$PI" "bash ~/drone_project/tools/install_pi_fresh.sh"
    rc=$?
    [ "$rc" -eq 0 ] && say "Pi ready. Run a mission with:  bash $REPO/tools/nidar_run.sh"
    exit "$rc"
fi

if [ "$MODE" = "update-pi" ]; then
    B="$(ls "$REPO"/nidar_pi_install_*.tgz 2> /dev/null | sort | tail -1)"
    [ -n "$B" ] || { say "[ERROR] no nidar_pi_install_*.tgz in $REPO"; exit 1; }
    pi true || { say "[ERROR] no password-free SSH to $PI: run '$0 setup' first"; exit 1; }
    say "stopping NIDAR on the Pi, then installing $(basename "$B")"
    stop_pi
    scp -o BatchMode=yes -q "$B" "$PI:~/" &&
        pi "rm -rf ~/nidar_install && cd ~ && tar xzf $(basename "$B") && echo y | bash ~/nidar_install/tools/install_on_pi.sh"
    exit $?
fi

# ---------------------------------------------------------------- a run --
RUN_DIR="$LOGS/run_$(date +%Y%m%d_%H%M%S)_$MODE"
ENV_SRC="source '$REPO/gazebo/nidar_gz_env.sh' > /dev/null;"
LAYOUT_REL="gazebo/layouts/$W.json"
GCS_LAYOUT=""; [ -f "$REPO/$LAYOUT_REL" ] && GCS_LAYOUT="--layout $LAYOUT_REL"
HL="HEADLESS=1"; [ -n "$GUI" ] && HL=""
declare -A PID CMD RESTART RESTARTS
ORDER=()

command_for() {   # name -> shell command
    if [ -n "$TEST" ] && [ "$1" != "status" ]; then
        echo "echo '[fake $1] running. Ctrl+C here stops'; echo 'INFO  [commander] Ready for takeoff!'; exec sleep 100000"
        return
    fi
    case "$1" in
        gazebo) echo "cd '$REPO' && $HL exec bash gazebo/run_rig.sh $SEED 1.0" ;;
        agent) echo "exec \$(command -v MicroXRCEAgent || echo \$HOME/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent) udp4 -p 8888" ;;
        px4) echo "cd '$REPO' && $HL PX4_DAEMON=1 exec bash gazebo/run_px4.sh $SEED" ;;
        sensors) echo "$ENV_SRC exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$REPO/gazebo/config/nidar_gz_bridge_sensors.yaml" ;;
        lidar) echo "$ENV_SRC exec python3 '$REPO/gazebo/ros2/nidar_lidar_relay.py' --pi-host $PI_HOST" ;;
        camera) echo "$ENV_SRC exec python3 '$REPO/gazebo/ros2/nidar_camera_relay.py'" ;;
        bridge) echo "$ENV_SRC export NIDAR_PI_HOST=$PI_HOST; exec ros2 run nidar_airmouse nidar_bridge" ;;
        gcs) if [ -n "$CLASSIC_GCS" ]; then
                 echo "$ENV_SRC cd '$REPO' && ${BRAIN_MODE:+NIDAR_BRAIN_MODE=$BRAIN_MODE} exec python3 gcs/nidar_gcs.py --pi-host $PI_HOST $GCS_LAYOUT"
             else
                 echo "$ENV_SRC cd '$REPO' && ${BRAIN_MODE:+NIDAR_BRAIN_MODE=$BRAIN_MODE} exec python3 gcs/airmouse_gcs.py --pi-host $PI_HOST $GCS_LAYOUT --record-dir '$RUN_DIR/gcs'"
             fi ;;
        qgc) echo "exec '$QGC_APP'" ;;
        status) echo "exec python3 '$REPO/tools/nidar_status.py' --pi $PI_HOST --watch '$RUN_DIR' ${BRAIN_MODE:+--px4-route}" ;;
        rig) echo "$ENV_SRC exec python3 '$REPO/gazebo/ros2/nidar_rig_driver.py' --brain $PI_HOST --seed $SEED --truth-csv '$RUN_DIR/rig_truth.csv'" ;;
        flyer) echo "$ENV_SRC cd '$REPO' && exec python3 gazebo/ros2/nidar_px4_route_flyer.py --seed $SEED --route $ROUTE" ;;
    esac
}

start() {   # name restart(yes|no)
    local name="$1" restart="$2" cmd
    cmd="$(command_for "$name")"
    if [ -n "$DRY" ]; then echo "  start $name: $cmd"; return; fi
    # own process group (setsid) AND Ctrl+C restored: bash starts background jobs with SIGINT ignored, so
    # without `env --default-signal` the rig driver could not stop cleanly and the PX4 flyer could not LAND
    if [ "$name" = "status" ]; then
        setsid env --default-signal=INT,QUIT bash -c "$cmd" > "$RUN_DIR/$name.log" 2>&1 < /dev/null &
    else
        setsid env --default-signal=INT,QUIT bash -c "$cmd" >> "$RUN_DIR/$name.log" 2>&1 < /dev/null &
    fi
    PID[$name]=$!; CMD[$name]="$cmd"; RESTART[$name]="$restart"; RESTARTS[$name]=${RESTARTS[$name]:-0}
    disown "${PID[$name]}" 2> /dev/null                # no "Killed" job noise; still reaped, still checked
    [[ " ${ORDER[*]} " == *" $name "* ]] || ORDER+=("$name")
    echo "$name ${PID[$name]}" >> "$STATE_DIR/pids"
}

stop_group() {   # name signal
    local p="${PID[$1]:-}"
    [ -n "$p" ] && kill "-$2" -- "-$p" 2> /dev/null
    true
}

wait_log() {   # name pattern timeout_s -> 0 found
    local name="$1" pat="$2" t="$3" i
    for ((i = 0; i < t; i++)); do
        grep -aq "$pat" "$RUN_DIR/$name.log" 2> /dev/null && return 0
        kill -0 "${PID[$name]}" 2> /dev/null || return 2
        ((i > 0 && i % 30 == 0)) && say "    still waiting for $name ($i s)..."
        sleep 1
    done
    return 1
}

CLEANING=""
cleanup() {
    [ -n "$CLEANING" ] && return
    CLEANING=1
    trap '' INT TERM
    echo
    say "STOPPING (do not close this window)"
    for f in rig flyer; do
        if [ -n "${PID[$f]:-}" ] && kill -0 "${PID[$f]}" 2> /dev/null; then
            stop_group "$f" INT
            local lim=5; [ "$f" = "flyer" ] && { lim=90; say "  PX4 flyer: landing first (up to 90 s)..."; }
            for ((i = 0; i < lim; i++)); do kill -0 "${PID[$f]}" 2> /dev/null || break; sleep 1; done
            stop_group "$f" KILL
            if [ "$f" = "flyer" ] && [ -z "$TEST" ]; then say "  letting PX4 finish the landing (15 s)"; sleep 15; fi
        fi
    done
    say "  Pi: stopping the brain and the stack, copying its files of this run to $RUN_DIR/pi/"
    stop_pi
    mkdir -p "$RUN_DIR/pi"
    pi 'cd ~/drone_project && head -c 200000 logs/stack_console.log > logs/stack_start.log 2>/dev/null;
        find logs outputs/mission outputs/slam/trajectories outputs/slam/state -type f -newer logs/.nidar_run_start \
             ! -name stack_console.log 2>/dev/null | tar czf - -T - 2>/dev/null' > "$RUN_DIR/pi_files.tgz" 2> /dev/null &&
        tar xzf "$RUN_DIR/pi_files.tgz" -C "$RUN_DIR/pi" 2> /dev/null && rm -f "$RUN_DIR/pi_files.tgz"
    say "  stopping the WSL programs and Gazebo"
    for n in "${ORDER[@]}"; do stop_group "$n" INT; done
    for ((i = 0; i < 6; i++)); do
        alive=0; for n in "${ORDER[@]}"; do kill -0 "${PID[$n]}" 2> /dev/null && alive=1; done
        [ "$alive" = 0 ] && break; sleep 1
    done
    for n in "${ORDER[@]}"; do stop_group "$n" TERM; done; sleep 1
    for n in "${ORDER[@]}"; do stop_group "$n" KILL; done
    [ -n "$YOLO" ] && { say "  stopping YOLO on Windows"; stop_yolo; }
    rm -f "$STATE_DIR/launcher.pid" "$STATE_DIR/pids"
    say "STOPPED. Everything of this run: ${RUN_DIR#"$REPO/"}  (send me this folder name)"
    exit 0
}

# ---- preflight
if [ -z "$DRY" ]; then
    if launcher_alive "$(cat "$STATE_DIR/launcher.pid" 2> /dev/null)"; then
        say "[ERROR] a run is already active (PID $(cat "$STATE_DIR/launcher.pid")). Stop it: $0 stop"; exit 1
    fi
    if ! pi true 2> "$STATE_DIR/ssh_error.txt"; then        # 2026-11-10: say WHY (unreachable vs key refused)
        if timeout 5 bash -c "</dev/tcp/$PI_HOST/${NIDAR_PI_SSH_PORT:-22}" 2> /dev/null; then
            say "[ERROR] the Pi ($PI_HOST) answers, but password-free login as $PI_USER is refused:"
            say "        $(tail -1 "$STATE_DIR/ssh_error.txt")"
            say "        Fix once:  bash $REPO/tools/nidar_run.sh setup   (asks the Pi password one time)"
        else
            say "[ERROR] the Pi is not reachable at $PI_HOST: it is off, on another Wi-Fi, or has a new address."
            say "        ssh said: $(tail -1 "$STATE_DIR/ssh_error.txt")"
            say "        Power it on, same Wi-Fi as this laptop. New address:  bash $REPO/tools/nidar_run.sh --pi NEW_ADDRESS"
        fi
        exit 1
    fi
    if [ -z "$TEST" ]; then
        [ "$MODE" = "px4" ] && [ ! -x "${PX4_DIR:-$HOME/PX4-Autopilot}/build/px4_sitl_default/bin/px4" ] &&
            { say "[ERROR] PX4 is not built: cd ~/PX4-Autopilot && make px4_sitl"; exit 1; }
        [ -n "$YOLO" ] && ! command -v powershell.exe > /dev/null &&
            { say "[WARN] powershell.exe not reachable from WSL: start YOLO yourself (tools/nidar_yolo.ps1)"; YOLO=""; }
        [ "$MODE" = "px4" ] && [ -n "$QGC" ] && [ ! -x "$QGC_APP" ] &&
            { say "[WARN] QGroundControl not installed (run: $0 setup) - continuing without it"; QGC=""; }
    fi
    mkdir -p "$RUN_DIR"
    echo $$ > "$STATE_DIR/launcher.pid"; : > "$STATE_DIR/pids"
    trap cleanup INT TERM HUP
fi

say "NIDAR $MODE run | arena s$SEED | Pi $PI | logs ${RUN_DIR#"$REPO/"}"
if [ -z "$DRY" ]; then
    say "1. stopping left-overs (here, on the Pi, YOLO on Windows)"
    [ -n "$TEST" ] || stop_local_leftovers
    stop_pi; [ -n "$YOLO" ] && stop_yolo
fi

# ---- Pi
PI_LAYOUT=""
[ -z "$DRY" ] && pi "test -f ~/drone_project/$LAYOUT_REL" 2> /dev/null && PI_LAYOUT="NIDAR_ARENA_LAYOUT=$LAYOUT_REL"
# ( ... &) : the background program must not keep SSH's output open, or `ssh` never returns (2026-11-09: the first
# real runs stopped at step 2 for that reason: `a && b &` puts the whole list, with SSH's output, in the background)
PI_STACK="cd ~/drone_project && mkdir -p logs && touch logs/.nidar_run_start && ($PI_LAYOUT NIDAR_LIDAR_BACKEND=GAZEBO NIDAR_SLAM=V6 setsid nohup ./start_nidar_pi.sh > logs/stack_console.log 2>&1 < /dev/null &)"
PI_BRAIN="cd ~/drone_project && (setsid nohup .venv/bin/python src/mission/nidar_explorer_service.py --radius $RADIUS --mission-min $MINUTES > logs/explorer.log 2>&1 < /dev/null &)"
if [ -n "$DRY" ]; then
    echo "  Pi: $PI_STACK"; [ "$MODE" = "rig" ] && echo "  Pi: $PI_BRAIN"
else
    say "2. Pi: SLAM V6 stack${PI_LAYOUT:+ (layout s$SEED for the ground station only)}"
    pi "$PI_STACK" || { say "[ERROR] could not start the Pi stack"; cleanup; }
    if [ "$MODE" = "rig" ]; then
        say "   Pi: the brain (radius $RADIUS m, $MINUTES min)"; pi "$PI_BRAIN"
    fi
fi

# ---- simulator
BRAIN_MODE=""
if [ "$MODE" = "rig" ]; then
    [ -z "$DRY" ] && say "3. Gazebo + rig${GUI:+ (Gazebo window opens)}${GUI:- (no Gazebo window)}"
    start gazebo no
    if [ -z "$DRY" ]; then
        wait_log gazebo "running. Ctrl+C here stops" 420 || { say "[ERROR] Gazebo did not come up (see $RUN_DIR/gazebo.log)"; cleanup; }
    fi
else
    BRAIN_MODE="route"
    [ -z "$DRY" ] && say "3. uXRCE-DDS agent + PX4 SITL + x500 in the arena${GUI:+ (with window)}"
    start agent yes
    start px4 no
    if [ -z "$DRY" ]; then
        wait_log px4 "Ready for takeoff" 300 || say "[WARN] PX4 did not report 'Ready for takeoff' yet (see $RUN_DIR/px4.log)"
        if [ -z "$TEST" ]; then
            P4="${PX4_DIR:-$HOME/PX4-Autopilot}/build/px4_sitl_default/bin/px4-param"
            # SIMULATED PX4 ONLY (arming without RC / GCS link). Never on a real flight controller.
            for kv in "COM_RC_IN_MODE 4" "NAV_RCL_ACT 0" "NAV_DLL_ACT 0" "COM_RCL_EXCEPT 4"; do
                # shellcheck disable=SC2086
                (cd "${PX4_DIR:-$HOME/PX4-Autopilot}" && "$P4" set $kv > /dev/null 2>&1) || say "[WARN] px4-param set $kv failed"
            done
            say "   PX4 (simulated): RC / data-link failsafes off for SITL"
        fi
    fi
    start sensors yes
fi

[ -z "$DRY" ] && say "4. relays, AirMouse bridge, status reader${GCS:+, ground station}"
start lidar yes
start camera yes
start bridge yes
start status yes
[ -n "$GCS" ] && start gcs no
[ "$MODE" = "px4" ] && [ -n "$QGC" ] && start qgc no
if [ -n "$YOLO" ]; then
    if [ -n "$DRY" ]; then echo "  Windows: tools/nidar_yolo.ps1 -PiHost $PI_HOST (own window)"
    else
        say "5. YOLO in its own Windows window"
        win_ps -Command "Start-Process powershell -ArgumentList '-NoExit','-ExecutionPolicy','Bypass','-File','$(win_repo)\\tools\\nidar_yolo.ps1','-PiHost','$PI_HOST'" > /dev/null 2>&1 ||
            say "[WARN] could not open the YOLO window"
    fi
fi

FLIGHT="rig"; [ "$MODE" = "px4" ] && FLIGHT="flyer"
if [ -n "$DRY" ]; then echo "  (wait for SLAM TRACKING)"; start "$FLIGHT" no; exit 0; fi
say "6. waiting for SLAM TRACKING on the Pi (drone still on the pad)..."
python3 "$REPO/tools/nidar_status.py" --pi "$PI_HOST" --wait-tracking 240 | sed 's/^/   /'
[ "${PIPESTATUS[0]}" -eq 0 ] || { say "[ERROR] SLAM not TRACKING after 240 s - see $RUN_DIR/lidar.log and pi/ logs"; cleanup; }
if [ "$MODE" = "rig" ]; then
    say "7. FLYING: the brain explores by itself (rig driver restarts itself if it dies)"
    start rig yes
else
    say "7. FLYING: PX4 route flyer, route $ROUTE (simulation; Ctrl+C = land)"
    start flyer no
fi
say "Ctrl+C here = stop everything.   Status every 5 s:"

# ---------------------------------------------------------------- supervise --
n_ev=0; done_said=""; tick=0
while :; do
    sleep 1; tick=$((tick + 1))
    for n in "${ORDER[@]}"; do
        p="${PID[$n]:-}"; [ -z "$p" ] && continue
        kill -0 "$p" 2> /dev/null && continue
        case "$n" in
            gazebo|px4) say "LOST: $n stopped (see $RUN_DIR/$n.log) - stopping everything"; cleanup ;;
        esac
        if [ "${RESTART[$n]}" = "yes" ] && [ "${RESTARTS[$n]}" -lt "$MAX_RESTARTS" ]; then
            RESTARTS[$n]=$((RESTARTS[$n] + 1))
            say "LOST: $n stopped - restarting ($((RESTARTS[$n]))/$MAX_RESTARTS, log $n.log)"
            echo "----- restart $((RESTARTS[$n])) $(date +%H:%M:%S) -----" >> "$RUN_DIR/$n.log"
            start "$n" yes
        else
            say "$n ended$([ "${RESTART[$n]}" = "yes" ] && echo " (restarted $MAX_RESTARTS times: giving up)")"
            PID[$n]=""
        fi
    done
    if [ -f "$RUN_DIR/events.log" ]; then
        total=$(wc -l < "$RUN_DIR/events.log")
        if [ "$total" -gt "$n_ev" ]; then
            tail -n "+$((n_ev + 1))" "$RUN_DIR/events.log" | sed 's/^/  ** /'
            n_ev=$total
        fi
        if [ -z "$done_said" ] && grep -q "MISSION COMPLETE" "$RUN_DIR/events.log"; then
            done_said=1
            say "=== MISSION COMPLETE: press 'Export report' in the ground station, then Ctrl+C here ==="
        fi
    fi
    if [ $((tick % 5)) -eq 0 ]; then
        line="$(cat "$RUN_DIR/status.line" 2> /dev/null || echo "status: starting")"
        if [ "$FLIGHT" = "rig" ]; then fl="$(grep -a '^\[RIG\]' "$RUN_DIR/rig.log" 2> /dev/null | tail -1 | cut -c1-110)"
        else fl="$(grep -a 'nidar_px4_route_flyer\|PX4FLY' "$RUN_DIR/flyer.log" 2> /dev/null | tail -1 | cut -c1-140)"; fi
        say "$line"
        [ -n "$fl" ] && echo "           $fl"
    fi
done
