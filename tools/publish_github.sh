#!/usr/bin/env bash
# =====================================================================
# Put this project on GitHub so another laptop can install it with one line (2026-11-11). Run in WSL:
#
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/publish_github.sh                    public repository NIDAR_DRONE_PROJECT
#   bash /mnt/d/NIDAR_DRONE_PROJECT/tools/publish_github.sh NAME --private     other name / private
#
# First time: installs git + the GitHub CLI, logs you in (a code to type at github.com/login/device),
# creates the repository, writes its address into the installer + README, uploads.
# Later: the same command uploads your changes. What is left out: see .gitignore (logs, videos, bundles,
# .venv, the 250 MB mobileclip file, run outputs).
# A PRIVATE repository: add your friend under Settings -> Collaborators; the one-line web install then does not
# work for them (it needs a public file) -> they clone first (docs/SETUP.md, "private repository").
# =====================================================================
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="NIDAR_DRONE_PROJECT"; VIS="--public"; NO_PUSH="${NIDAR_NO_PUSH:-}"
for a in "$@"; do
    case "$a" in
        --private) VIS="--private" ;;
        --public) VIS="--public" ;;
        -h|--help) sed -n '2,16p' "${BASH_SOURCE[0]}"; exit 0 ;;
        -*) echo "unknown option $a"; exit 2 ;;
        *) NAME="$a" ;;
    esac
done
say() { echo "[github] $*"; }

if [ -z "$NO_PUSH" ]; then
    command -v git > /dev/null && command -v gh > /dev/null || { say "installing git + GitHub CLI"; sudo apt-get install -y git gh; }
    gh auth status > /dev/null 2>&1 || { say "log in to GitHub (follow the code it shows):"; gh auth login --hostname github.com --git-protocol https --web; }
    gh auth setup-git
    OWNER="$(gh api user -q .login)"
else
    OWNER="${NIDAR_GH_OWNER:-owner}"
fi
URL="https://github.com/$OWNER/$NAME"
RAW="https://raw.githubusercontent.com/$OWNER/$NAME/main"
say "repository: $URL"

cd "$REPO"
git config --global --get-all safe.directory 2> /dev/null | grep -qxF "$REPO" || git config --global --add safe.directory "$REPO"
[ -d .git ] || git init -q -b main
git config core.fileMode false                     # D: (NTFS) shows every file as executable
git config core.autocrlf false
git config user.name > /dev/null || git config user.name "$OWNER"
git config user.email > /dev/null || git config user.email "$OWNER@users.noreply.github.com"
# the installer and the guides learn where the project lives
for f in tools/install_windows.ps1 README.md docs/SETUP.md; do
    [ -f "$f" ] && sed -i "s#__NIDAR_REPO_URL__#$URL.git#g; s#__NIDAR_RAW_URL__#$RAW#g; s#__NIDAR_REPO_PAGE__#$URL#g" "$f"
done
git add -A
if git diff --cached --quiet; then say "nothing new to upload"
else git commit -q -m "NIDAR AirMouse update $(date +%Y-%m-%d\ %H:%M)"; say "saved: $(git log -1 --format=%h) ($(git ls-files | wc -l) files)"; fi

if [ -n "$NO_PUSH" ]; then say "NIDAR_NO_PUSH set: not uploading"; exit 0; fi
if ! git remote get-url origin > /dev/null 2>&1; then
    if gh repo view "$OWNER/$NAME" > /dev/null 2>&1; then git remote add origin "$URL.git"
    else gh repo create "$NAME" "$VIS" --source . --remote origin --description "NIDAR AirMouse: GPS-denied search drone (ROS 2, PX4, Gazebo, SLAM, YOLO)"; fi
fi
git push -u origin main
echo
say "DONE. On your friend's laptop: PowerShell AS ADMINISTRATOR, then:"
echo "    irm $RAW/tools/install_windows.ps1 | iex"
say "Full guide: $URL/blob/main/docs/SETUP.md"
