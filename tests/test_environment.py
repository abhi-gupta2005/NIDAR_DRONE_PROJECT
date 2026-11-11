"""
Phase 1 -- baseline / configuration consistency.

These tests do not exercise algorithms; they catch the configuration faults
that cost the most debugging time in this project: path coupling between
processes, interpreter requirements, and start scripts pointing at files
that do not exist.
"""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

import pytest

from conftest import PROJECT_ROOT, SRC_ROOT, load_module_from_path


def test_python_version_is_supported():
    # SLAM V5.1 originally required 3.12 (PEP 701 f-strings); after the
    # 2026-11-01 fix the project runs on 3.10+.  Record what we run on.
    assert sys.version_info >= (3, 10), sys.version


def test_numpy_available():
    import numpy as np
    assert np.__version__


def test_slam_source_parses_on_python_3_11_grammar():
    """The f-string in SLAM main() must not reuse the outer quote character."""
    src = (SRC_ROOT / "core" / "nidar_slam_v5_1.py").read_text(encoding="utf-8")
    # Compile in this interpreter (always works on 3.12+) ...
    ast.parse(src)
    # ... and statically reject the 3.12-only construct f"...{x if "y" in z}...".
    offending = re.findall(r'f"[^"\n]*\{[^}\n]*"[^"\n]*"[^}\n]*\}', src)
    assert not offending, f"3.12-only nested double quotes inside f-string: {offending}"


@pytest.mark.parametrize("script", ["start_nidar_pi.sh", "start_nidar_wsl.sh"])
def test_start_scripts_reference_existing_entry_points(script):
    if not (PROJECT_ROOT / script).exists():
        pytest.skip(f"{script} is not on this machine (start_nidar_wsl.sh lives only in the laptop copy)")
    text = (PROJECT_ROOT / script).read_text(encoding="utf-8")
    for rel in ("src/communication/nidar_pi_receiver.py",
                "src/perception/nidar_v18_headless_persistent_fusion.py",
                "nidar_airmouse_server.py",
                "integration_monitor.py"):
        assert rel in text, f"{script} does not start {rel}"
        assert (PROJECT_ROOT / rel).exists(), f"{rel} missing"
    assert "src.core.nidar_slam_v5_1" in text
    assert (SRC_ROOT / "core" / "nidar_slam_v5_1.py").exists()


@pytest.mark.parametrize("script", ["start_nidar_pi.sh", "start_nidar_wsl.sh"])
def test_start_scripts_use_unbuffered_python_output(script):
    """receiver.log was 0 bytes in the field because stdout was block-buffered."""
    if not (PROJECT_ROOT / script).exists():
        pytest.skip(f"{script} is not on this machine (start_nidar_wsl.sh lives only in the laptop copy)")
    text = (PROJECT_ROOT / script).read_text(encoding="utf-8")
    assert "PYTHONUNBUFFERED=1" in text or "python -u" in text or "python3 -u" in text


def test_v18_reads_the_file_the_receiver_writes(tmp_path):
    """
    Historical defect: V18 read ~/drone_project/src/communication/... while the
    receiver wrote next to its own file.  They must resolve to the same path
    wherever the project is checked out.
    """
    receiver = load_module_from_path("rx_env", SRC_ROOT / "communication" / "nidar_pi_receiver.py")
    v18 = load_module_from_path("v18_env", SRC_ROOT / "perception" / "nidar_v18_headless_persistent_fusion.py")
    assert Path(v18.CAMERA_PACKET).resolve() == Path(receiver.LATEST_FILE).resolve()


def test_v18_output_matches_integration_monitor_and_airmouse_server_input():
    v18 = load_module_from_path("v18_env2", SRC_ROOT / "perception" / "nidar_v18_headless_persistent_fusion.py")
    monitor = load_module_from_path("monitor_env", PROJECT_ROOT / "integration_monitor.py")
    airmouse = load_module_from_path("airmouse_env", PROJECT_ROOT / "nidar_airmouse_server.py")
    assert Path(v18.OUTPUT_JSON).resolve() == Path(monitor.V18_FILE).resolve()
    assert Path(v18.OUTPUT_JSON).resolve() == Path(airmouse.JSON_FILE).resolve()


def test_slam_outputs_directories_exist():
    slam_src = (SRC_ROOT / "core" / "nidar_slam_v5_1.py").read_text(encoding="utf-8")
    for rel in ("outputs/slam/state", "outputs/slam/trajectories", "outputs/slam/maps"):
        assert rel in slam_src
        assert (PROJECT_ROOT / rel).is_dir(), f"{rel} must exist: SLAM does not create it"


def test_lidar_backend_is_declared_simulator():
    """Guard: nobody silently switched SLAM to SERIAL without the decoder."""
    from src.core import nidar_slam_v5_1 as slam
    assert slam.LIDAR_BACKEND == "SIMULATOR"
    from src.core.lidar_interface import SerialSTL19PBackend
    with pytest.raises(NotImplementedError):
        SerialSTL19PBackend.__dict__["next_scan"](None)


def test_windows_sender_and_receiver_agree_on_port_and_protocol():
    windows_src = (PROJECT_ROOT / "nidar_v11_windows.py").read_text(encoding="utf-8")
    receiver_src = (SRC_ROOT / "communication" / "nidar_pi_receiver.py").read_text(encoding="utf-8")
    assert '"NIDAR_PERCEPTION_V1"' in windows_src and '"NIDAR_PERCEPTION_V1"' in receiver_src
    assert 'os.environ.get("NIDAR_PI_PORT", "5000")' in windows_src
    assert 'os.environ.get("NIDAR_PI_PORT", "5000")' in receiver_src


def test_bridge_port_matches_airmouse_server_port():
    bridge = (PROJECT_ROOT / "ros2_ws_src" / "nidar_airmouse" / "nidar_airmouse" / "nidar_bridge.py")
    if not bridge.exists():
        pytest.skip("ROS 2 package source not copied into ros2_ws_src/")
    text = bridge.read_text(encoding="utf-8")
    m = re.search(r"PI_PORT\s*=\s*(\d+)", text)
    assert m, "PI_PORT not found in nidar_bridge.py"
    airmouse = load_module_from_path("airmouse_port", PROJECT_ROOT / "nidar_airmouse_server.py")
    assert int(m.group(1)) == airmouse.PORT == 5010
