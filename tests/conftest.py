"""
pytest configuration for the NIDAR AirMouse test harness.

Run from the project root on any machine:
    Windows (.venv active):  python -m pytest -q
    Raspberry Pi / WSL:      python3 -m pytest -q
    only fast tests:         python -m pytest -q -m "not slow"

The harness imports the PRODUCTION modules (receiver, V18, SLAM, LiDAR
chain, Windows stable-ID layer, ROS 2 node logic) and exercises them with
synthetic or recorded data.  Heavy optional dependencies (ultralytics, cv2,
rclpy) are stubbed ONLY when they are not installed, so the same tests run
on the Pi (no ultralytics), on Windows (no ROS 2) and in CI.
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import signal
import sys
import types
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"

# Same import roots as start_nidar_pi.sh / start_nidar_wsl.sh.
for _p in (PROJECT_ROOT, SRC_ROOT, SRC_ROOT / "core", SRC_ROOT / "tracking",
           SRC_ROOT / "perception", SRC_ROOT / "simulation", SRC_ROOT / "communication"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: takes more than a few seconds")
    config.addinivalue_line("markers", "network: opens loopback sockets")
    config.addinivalue_line("markers", "hardware: needs a physical sensor or camera (skipped by default)")
    config.addinivalue_line("markers", "finding: documents a known defect (xfail until fixed)")


# ---------------------------------------------------------------------------
# Optional dependency stubs
# ---------------------------------------------------------------------------

def _stub_module(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


def _ensure_optional_imports():
    """Install minimal stubs for ultralytics / cv2 when they are absent."""
    try:
        importlib.import_module("ultralytics")
    except Exception:
        class _YOLOE:  # pragma: no cover - only used when ultralytics is missing
            def __init__(self, *a, **k):
                raise RuntimeError("ultralytics is not installed in this environment")

        _stub_module("ultralytics", YOLOE=_YOLOE, YOLO=_YOLOE)

    try:
        importlib.import_module("cv2")
    except Exception:
        _stub_module("cv2", CAP_DSHOW=700, CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4,
                     CAP_PROP_BUFFERSIZE=38, FONT_HERSHEY_SIMPLEX=0, LINE_AA=16,
                     VideoCapture=lambda *a, **k: None, imshow=lambda *a, **k: None,
                     waitKey=lambda *a, **k: -1, rectangle=lambda *a, **k: None,
                     putText=lambda *a, **k: None, line=lambda *a, **k: None,
                     getTextSize=lambda *a, **k: ((0, 0), 0), destroyAllWindows=lambda: None)


_ensure_optional_imports()


# ---------------------------------------------------------------------------
# Module loaders (production files, no main loops executed)
# ---------------------------------------------------------------------------

class _PreserveSignals:
    def __enter__(self):
        self.saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        return self

    def __exit__(self, *exc):
        for sig, handler in self.saved.items():
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass


def load_module_from_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    with _PreserveSignals():
        spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="session")
def project_root() -> Path:
    return PROJECT_ROOT


@pytest.fixture()
def sim_clock():
    from src.simulation.nidar_pipeline_sim import SimClock
    return SimClock(start=1_700_000_000.0)


@pytest.fixture()
def v18(sim_clock):
    """Fresh V18 module instance with an injected clock (no main loop)."""
    module = load_module_from_path(
        "nidar_v18_under_test",
        SRC_ROOT / "perception" / "nidar_v18_headless_persistent_fusion.py")
    module.time = sim_clock
    return module


@pytest.fixture()
def receiver_module(tmp_path):
    """Fresh receiver module writing into a temp dir."""
    module = load_module_from_path(
        "nidar_receiver_under_test", SRC_ROOT / "communication" / "nidar_pi_receiver.py")
    module.LATEST_FILE = tmp_path / "nidar_latest_perception.json"
    module.OBJECT_LOG = tmp_path / "nidar_perception_objects.csv"
    return module


@pytest.fixture(scope="session")
def windows_module():
    """The Windows perception script (logic only; no camera, no model)."""
    with _PreserveSignals():
        return importlib.import_module("nidar_v11_windows")


@pytest.fixture(scope="session")
def slam_module():
    with _PreserveSignals():
        return importlib.import_module("src.core.nidar_slam_v5_1")


@pytest.fixture()
def world():
    from src.simulation import nidar_world
    return nidar_world.indoor_scene(seed=1)


@pytest.fixture(scope="session")
def recorded_packets():
    """First 300 packets of the 12 Oct 2026 Windows recording, if present."""
    import json
    path = PROJECT_ROOT / "multi_object_tracking_v10.jsonl"
    if not path.exists():
        pytest.skip("multi_object_tracking_v10.jsonl not present")
    packets = []
    with path.open("r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if i >= 300:
                break
            line = line.strip()
            if line:
                packets.append(json.loads(line))
    return packets
