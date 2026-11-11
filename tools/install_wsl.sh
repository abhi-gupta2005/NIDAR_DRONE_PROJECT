#!/usr/bin/env bash
# =====================================================================
# NIDAR AirMouse – install EVERYTHING the laptop side needs in WSL Ubuntu 24.04 (2026-11-11).
#
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/install_wsl.sh             everything (about 1 hour, ~20 GB)
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/install_wsl.sh --dry-run   print what it would do
# (project on C: instead of D:?  bash /mnt/c/NIDAR_DRONE_PROJECT/tools/install_wsl.sh)
#
# Installs, in this order (each step is checked first and skipped when it is already done, so running it
# again after an error or a lost connection simply continues):
#    1. base tools          git, curl, compilers, cmake, Python venv/pip, UTF-8 locale
#    2. ROS 2 Jazzy         ros-jazzy-desktop + ros_gz (Gazebo Harmonic 8) + cv_bridge + colcon
#    3. Python for NIDAR    PyQt5 (ground station), numpy, OpenCV, pytest, PyYAML, matplotlib
#    4. PX4-Autopilot       ~/PX4-Autopilot at the commit this project flies (v1.18.0-beta1-956), its own
#                           setup script (simulation part only) and `make px4_sitl`
#    5. Micro XRCE-DDS      ~/Micro-XRCE-DDS-Agent v2.4.3, built and installed (PX4 <-> ROS 2)
#    6. px4_ros2_ws         ~/px4_ros2_ws: px4_msgs (release/1.18, the one used with that PX4) + nidar_airmouse
#    7. NIDAR in PX4        the arena worlds + drone/rig models copied into PX4's Gazebo folders
#    8. QGroundControl      ~/QGroundControl-x86_64.AppImage + the libraries it needs
#    9. this shell          ROS + NIDAR environment in every new terminal, the short command `nidar`
#   10. check               versions + the project's fast tests
# Nothing here talks to a drone or a flight controller. Log: ~/nidar_install.log
# =====================================================================
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_COMMIT="9fda8a51a4952dd6964fb7e2c86e0a4f748f740b"          # PX4 main 2026-10-03 (v1.18.0-beta1-956)
PX4_MSGS_COMMIT="0be57f29b494be6f8a81cf428a1c50fad203ca6a"     # px4_msgs release/1.18 2026-10-06
XRCE_TAG="v2.4.3"
WS="$HOME/px4_ros2_ws"
QGC_APP="$HOME/QGroundControl-x86_64.AppImage"
LOG="$HOME/nidar_install.log"
PX4=1; DRY=""                  # PX4 is always installed: even the rig world loads PX4's Gazebo folders
for a in "$@"; do
    case "$a" in
        --dry-run) DRY=1 ;;
        -h|--help) sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option $a (see --help)"; exit 2 ;;
    esac
done

say() { echo "[$(date +%H:%M:%S)] $*"; }
run() { if [ -n "$DRY" ]; then echo "    \$ $*"; else "$@"; fi; }
fail() { say "[FAILED] $*"; say "Fix the problem above (full log: $LOG) and run the same command again:"
         say "    bash $REPO/tools/install_wsl.sh      (finished steps are skipped)"; exit 1; }
step() { echo; say "==== $1 ===="; }

if [ -z "$DRY" ]; then
    exec > >(tee -a "$LOG") 2>&1
fi
say "NIDAR AirMouse WSL install — project: $REPO"

# ---------------------------------------------------------------- 0. checks --
. /etc/os-release
if [ "${VERSION_ID:-}" != "24.04" ]; then
    say "[ERROR] this needs Ubuntu 24.04 (ROS 2 Jazzy). This is: ${PRETTY_NAME:-unknown}."
    say "        In Windows PowerShell:  wsl --install -d Ubuntu-24.04"
    [ -n "$DRY" ] || exit 1
fi
grep -qi microsoft /proc/version 2>/dev/null || say "[note] not WSL: fine on a normal Ubuntu 24.04 too."
free_gb=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc 0-9)
[ "${free_gb:-0}" -ge 25 ] || say "[WARN] only ${free_gb} GB free in $HOME: the full install needs about 20 GB."
if [ -z "$DRY" ]; then
    say "sudo is needed for apt (type your Ubuntu password once):"
    sudo -v || fail "sudo"
    ( while kill -0 $$ 2> /dev/null; do sudo -n true; sleep 50; done ) 2> /dev/null &
fi
export DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------- 1. base --
step "1/10 base tools"
run sudo apt-get update -q || fail "apt-get update (internet in WSL?)"
run sudo apt-get install -y -q curl wget git build-essential cmake ninja-build python3-pip python3-venv \
    python3-dev software-properties-common locales lsb-release gnupg rsync unzip || fail "base packages"
run sudo locale-gen en_US en_US.UTF-8 > /dev/null
run sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ---------------------------------------------------------------- 2. ROS 2 --
step "2/10 ROS 2 Jazzy + Gazebo Harmonic (ros_gz)"
if [ -f /opt/ros/jazzy/setup.bash ] && dpkg -s ros-jazzy-ros-gz > /dev/null 2>&1 && dpkg -s ros-jazzy-desktop > /dev/null 2>&1; then
    say "already installed"
else
    run sudo add-apt-repository -y universe > /dev/null
    if ! dpkg -s ros2-apt-source > /dev/null 2>&1 && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
        # the ROS docs' way (ros-apt-source package); the plain key file is the fallback
        v="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2> /dev/null |
             grep -F '"tag_name"' | cut -d'"' -f4)"
        if [ -n "$v" ] && run curl -fsSL -o /tmp/ros2-apt-source.deb \
            "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$v/ros2-apt-source_$v.${UBUNTU_CODENAME:-noble}_all.deb"; then
            run sudo apt-get install -y -q /tmp/ros2-apt-source.deb || fail "ROS apt source"
        else
            run sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
                -o /usr/share/keyrings/ros-archive-keyring.gpg || fail "ROS key"
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME:-noble} main" |
                { if [ -n "$DRY" ]; then cat; else sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null; fi; }
        fi
    fi
    run sudo apt-get update -q || fail "apt-get update after adding ROS"
    run sudo apt-get install -y -q ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-cv-bridge ros-dev-tools \
        python3-colcon-common-extensions python3-rosdep || fail "ROS 2 Jazzy packages"
fi

# ---------------------------------------------------------------- 3. Python --
step "3/10 Python packages (ground station, tests)"
run sudo apt-get install -y -q python3-pyqt5 python3-numpy python3-opencv python3-pytest python3-yaml \
    python3-matplotlib python3-scipy python3-psutil || fail "Python packages"

# ---------------------------------------------------------------- 4. PX4 --
step "4/10 PX4-Autopilot (simulation)"
if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    say "already built: $PX4_DIR"
else
    if [ ! -d "$PX4_DIR/.git" ]; then
        say "downloading PX4 (about 2 GB with its submodules) ..."
        run git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" || fail "git clone PX4"
        if [ -z "$DRY" ]; then
            git -C "$PX4_DIR" checkout -q "$PX4_COMMIT" ||
                { say "[WARN] commit $PX4_COMMIT not found: using PX4 main"; }
        fi
        run git -C "$PX4_DIR" submodule update --init --recursive --jobs 4 || fail "PX4 submodules"
    else
        say "using the PX4 that is already in $PX4_DIR ($(git -C "$PX4_DIR" describe --tags --always 2> /dev/null))"
    fi
    say "PX4's own setup script (simulation tools + Gazebo Harmonic) ..."
    if [ -n "$DRY" ]; then echo "    \$ (cd $PX4_DIR && bash ./Tools/setup/ubuntu.sh --no-nuttx)"
    else (cd "$PX4_DIR" && bash ./Tools/setup/ubuntu.sh --no-nuttx) || fail "PX4 Tools/setup/ubuntu.sh"; fi
    say "building PX4 SITL (15-40 min) ..."
    if [ -n "$DRY" ]; then echo "    \$ (cd $PX4_DIR && make px4_sitl)"
    else (cd "$PX4_DIR" && make px4_sitl) || fail "make px4_sitl"; fi
fi

# ---------------------------------------------------------------- 5. agent --
step "5/10 Micro XRCE-DDS Agent $XRCE_TAG"
if command -v MicroXRCEAgent > /dev/null 2>&1; then
    say "already installed: $(command -v MicroXRCEAgent)"
else
    [ -d "$HOME/Micro-XRCE-DDS-Agent/.git" ] ||
        run git clone -b "$XRCE_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$HOME/Micro-XRCE-DDS-Agent" ||
        fail "git clone Micro-XRCE-DDS-Agent"
    if [ -n "$DRY" ]; then echo "    \$ cmake + make + sudo make install in ~/Micro-XRCE-DDS-Agent/build"
    else
        mkdir -p "$HOME/Micro-XRCE-DDS-Agent/build"
        (cd "$HOME/Micro-XRCE-DDS-Agent/build" && cmake .. && make -j"$(nproc)" && sudo make install && sudo ldconfig /usr/local/lib/) ||
            fail "building Micro-XRCE-DDS-Agent"
    fi
fi

# ---------------------------------------------------------------- 6. workspace --
step "6/10 ROS 2 workspace ~/px4_ros2_ws (px4_msgs + nidar_airmouse)"
run mkdir -p "$WS/src"
if [ ! -d "$WS/src/px4_msgs/.git" ]; then
    run git clone https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs" || fail "git clone px4_msgs"
    [ -n "$DRY" ] || git -C "$WS/src/px4_msgs" checkout -q "$PX4_MSGS_COMMIT" ||
        { say "[WARN] px4_msgs $PX4_MSGS_COMMIT not found: using branch release/1.18";
          git -C "$WS/src/px4_msgs" checkout -q release/1.18 || true; }
fi
run rsync -a --delete --exclude __pycache__ "$REPO/ros2_ws_src/nidar_airmouse/" "$WS/src/nidar_airmouse/" || fail "copy nidar_airmouse"
run mkdir -p "$WS/src/nidar_airmouse/resource"
run touch "$WS/src/nidar_airmouse/resource/nidar_airmouse"          # ament marker (colcon needs it)
if [ -n "$DRY" ]; then echo "    \$ source /opt/ros/jazzy/setup.bash && cd $WS && colcon build"
else
    ( set +u; source /opt/ros/jazzy/setup.bash && cd "$WS" && colcon build --event-handlers console_cohesion- ) ||
        fail "colcon build in $WS"
fi

# ---------------------------------------------------------------- 7. assets --
step "7/10 NIDAR arena worlds + models into PX4's Gazebo folders"
if [ -n "$DRY" ]; then echo "    \$ bash $REPO/gazebo/install_into_px4.sh"
else
    ( set +u; source /opt/ros/jazzy/setup.bash; bash "$REPO/gazebo/install_into_px4.sh" ) || fail "gazebo/install_into_px4.sh"
fi

# ---------------------------------------------------------------- 8. QGC --
step "8/10 QGroundControl (PX4's ground station)"
if [ -x "$QGC_APP" ]; then say "already installed: $QGC_APP"
else
    run sudo apt-get install -y -q libfuse2t64 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 || fail "QGC libraries"
    run wget -q -O "$QGC_APP" https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage || fail "QGC download"
    run chmod +x "$QGC_APP"
    run sudo usermod -aG dialout "${USER:-$(id -un)}"
fi

# ---------------------------------------------------------------- 9. shell --
step "9/10 environment in every new terminal + the 'nidar' command"
BLOCK="# >>> NIDAR AirMouse >>>
[ -f /opt/ros/jazzy/setup.bash ] && source '$REPO/gazebo/nidar_gz_env.sh' > /dev/null 2>&1
alias nidar='bash $REPO/tools/nidar_run.sh'
# <<< NIDAR AirMouse <<<"
if [ -n "$DRY" ]; then printf '    ~/.bashrc gets:\n%s\n' "$BLOCK"
elif grep -q ">>> NIDAR AirMouse >>>" "$HOME/.bashrc" 2> /dev/null; then
    sed -i '/# >>> NIDAR AirMouse >>>/,/# <<< NIDAR AirMouse <<</d' "$HOME/.bashrc"
    printf '\n%s\n' "$BLOCK" >> "$HOME/.bashrc"; say "updated in ~/.bashrc"
else
    printf '\n%s\n' "$BLOCK" >> "$HOME/.bashrc"; say "added to ~/.bashrc"
fi
[ -f "$HOME/.ssh/id_ed25519" ] || run ssh-keygen -t ed25519 -N "" -q -f "$HOME/.ssh/id_ed25519"

# ---------------------------------------------------------------- 10. check --
step "10/10 check"
if [ -n "$DRY" ]; then say "dry run: nothing was installed"; exit 0; fi
ok=1
check() { if eval "$2" > /dev/null 2>&1; then say "  OK    $1"; else say "  MISSING $1"; ok=0; fi; }
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
check "ROS 2 Jazzy"                 "[ \"\$ROS_DISTRO\" = jazzy ]"
check "Gazebo (gz sim)"             "gz sim --versions"
check "ros_gz bridge"               "ros2 pkg prefix ros_gz_bridge"
check "px4_msgs"                    "ros2 pkg prefix px4_msgs"
check "nidar_airmouse (ROS 2)"      "ros2 pkg prefix nidar_airmouse"
check "PyQt5 (ground station)"      "python3 -c 'import PyQt5.QtWidgets'"
check "OpenCV + numpy"              "python3 -c 'import cv2, numpy'"
if [ -n "$PX4" ]; then
    check "PX4 SITL built"          "[ -x '$PX4_DIR/build/px4_sitl_default/bin/px4' ]"
    check "MicroXRCEAgent"          "command -v MicroXRCEAgent"
    check "QGroundControl"          "[ -x '$QGC_APP' ]"
    check "NIDAR arena in PX4"      "ls '$PX4_DIR'/Tools/simulation/gz/worlds/nidar_airmouse_arena_s1.sdf"
fi
say "  running the project's fast tests (1-2 min) ..."
if (cd "$REPO" && QT_QPA_PLATFORM=offscreen python3 -m pytest -q -m "not slow and not hardware" -x -p no:cacheprovider 2>&1 | tail -3); then :; fi
echo
if [ "$ok" = 1 ]; then
    say "ALL DONE. Close this terminal and open a new Ubuntu terminal (the environment loads by itself), then:"
    say "  try the ground station without a Pi:"
    say "     python3 $REPO/gcs/airmouse_gcs.py --demo $REPO/gcs_records/demo_s1_offline/flight.jsonl.gz --no-ros"
    say "  connect your Raspberry Pi (once):   nidar setup --pi <PI_ADDRESS> --user <PI_USER>"
    say "  install the NIDAR software on it:   nidar install-pi"
    say "  run a mission:                      nidar"
else
    say "Some parts are MISSING (see above and $LOG). Run this script again; it continues where it stopped."
fi
