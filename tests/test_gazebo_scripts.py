"""
WSL helper scripts in gazebo/ (2026-11-02b), run with stand-ins for ROS and gz:

* nidar_gz_env.sh must survive being sourced under `set -u` (run_rig.sh / run_px4.sh):
  the first WSL run died with "/opt/ros/jazzy/setup.bash: AMENT_TRACE_SETUP_FILES:
  unbound variable" before Gazebo started.
* install_into_px4.sh: `gz sdf -k` cannot resolve <uri>x500</uri> offline; when PX4's
  own x500_lidar_2d fails the same way the result is INCONCLUSIVE (exit 0), while a
  genuine model error still FAILs (exit 2).

Real gz / ROS behaviour is only checked in WSL (gazebo/README.md, Stage 0/B).
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
GZ = ROOT / "gazebo"
pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="bash not available")

FAKE_GZ = r'''#!/bin/bash
# stand-in for `gz sdf -k FILE`: mimics sdformat failing on a bare <uri>x500</uri>
if [ "$1" = "sdf" ] && [ "$2" = "-k" ]; then
  if grep -q "BROKEN" "$3"; then echo "Error: Unable to read file: BROKEN element"; exit 255; fi
  if grep -q "<uri>x500</uri>" "$3"; then
    if [ -n "${FAKE_GZ_RESOLVES_X500:-}" ]; then echo "Valid."; exit 0; fi
    echo "Error [Utils.cc:176] Tried to use callback in sdf::findFile(), but the callback is empty."
    echo "Error Code 14: Msg: Unable to find uri[x500]"
    echo "Error Code 21: Msg: parent frame with name[base_link] specified by joint not found"
    exit 255
  fi
  echo "Valid."; exit 0
fi
exit 0
'''


def run(cmd, env, cwd=ROOT):
    return subprocess.run(["bash", "-c", cmd], cwd=str(cwd), env=env, capture_output=True, text=True, timeout=60)


def test_env_script_survives_set_u_with_ros_setup_using_unset_vars(tmp_path):
    fake_setup = tmp_path / "setup.bash"
    fake_setup.write_text('echo "ros setup sees ${AMENT_TRACE_SETUP_FILES_NOT_SET}"\nexport ROS_DISTRO=jazzy\n')
    env = dict(os.environ, NIDAR_ROS_SETUP=str(fake_setup), HOME=str(tmp_path), PX4_DIR=str(tmp_path / "px4"))
    env.pop("AMENT_TRACE_SETUP_FILES_NOT_SET", None)
    r = run(f'set -uo pipefail; source "{GZ}/nidar_gz_env.sh"; echo "ALIVE $ROS_DISTRO"; '
            f'case $- in *u*) echo "U_RESTORED";; esac; echo "$NIDAR_REPO"', env)
    assert r.returncode == 0, r.stderr
    assert "ALIVE jazzy" in r.stdout and "U_RESTORED" in r.stdout
    assert str(ROOT) in r.stdout


def make_px4(tmp_path, stock_resolves=False):
    px4 = tmp_path / "PX4-Autopilot"
    models = px4 / "Tools/simulation/gz/models"
    (px4 / "Tools/simulation/gz/worlds").mkdir(parents=True)
    (models / "x500_lidar_2d").mkdir(parents=True)
    (models / "x500_lidar_2d/model.sdf").write_text("<sdf><model name='x500_lidar_2d'><include merge='true'>"
                                                    "<uri>x500</uri></include></model></sdf>")
    bindir = tmp_path / "bin"
    bindir.mkdir()
    (bindir / "gz").write_text(FAKE_GZ)
    (bindir / "gz").chmod(0o755)
    env = dict(os.environ, PX4_DIR=str(px4), PATH=f"{bindir}:{os.environ['PATH']}")
    return px4, env


def test_install_reports_x500_include_as_inconclusive_not_fail(tmp_path):
    px4, env = make_px4(tmp_path)
    r = run(f'bash "{GZ}/install_into_px4.sh"', env)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "INCONCLUSIVE models/x500_nidar_airmouse" in r.stdout
    assert "OK  models/nidar_rig" in r.stdout and "FAIL" not in r.stdout
    assert (px4 / "Tools/simulation/gz/models/x500_nidar_airmouse/model.sdf").exists()
    r2 = run(f'bash "{GZ}/install_into_px4.sh"', env)                       # second run: nothing re-copied
    assert r2.returncode == 0 and "same      : Tools/simulation/gz/models/nidar_rig" in r2.stdout


def test_install_still_fails_a_genuinely_broken_model(tmp_path):
    px4, env = make_px4(tmp_path)
    src = tmp_path / "repo_gazebo"
    shutil.copytree(GZ, src, ignore=shutil.ignore_patterns("__pycache__"))
    rig = src / "models/nidar_rig/model.sdf"
    rig.write_text(rig.read_text().replace("<static>false</static>", "<static>false</static><!--BROKEN-->"))
    r = run(f'bash "{src}/install_into_px4.sh"', env)
    assert r.returncode == 2 and "FAIL models/nidar_rig" in r.stdout


def test_install_fails_when_only_our_model_cannot_resolve(tmp_path):
    """Stock model resolves x500 but ours does not -> a real problem, not the checker."""
    px4, env = make_px4(tmp_path)
    ref = px4 / "Tools/simulation/gz/models/x500_lidar_2d/model.sdf"
    ref.write_text("<sdf><model name='x500_lidar_2d'/></sdf>")               # no x500 include -> passes
    r = run(f'bash "{GZ}/install_into_px4.sh"', env)
    assert r.returncode == 2 and "FAIL models/x500_nidar_airmouse" in r.stdout


def test_launch_scripts_source_env_before_using_ros():
    for name in ("run_rig.sh", "run_px4.sh"):
        text = (GZ / name).read_text()
        assert "nidar_gz_env.sh" in text
        assert text.index("nidar_gz_env.sh") < text.find("ros2 ") if "ros2 " in text else True
