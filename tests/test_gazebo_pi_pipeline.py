"""
Pi pipeline with the Gazebo LiDAR option (2026-11-02).

Fast tests: the env-gated changes in V18 and SLAM (default behaviour unchanged,
camera width taken from the packet, GAZEBO backend selectable).

Slow tests run the REAL V18 and SLAM processes with NIDAR_LIDAR_BACKEND=GAZEBO,
fed by src/simulation/arena_feed.py (camera packet + scan files written exactly
as the two Pi receivers write them, from the generated arena). They prove the
Pi-side plumbing and stale handling. They do not prove anything about Gazebo
itself, the WSL relays, or YOLO on rendered images.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from src.simulation import arena_raycast as ar

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def arena():
    path = ar.default_layout(1)
    if not path.exists():
        pytest.skip("gazebo/layouts/nidar_airmouse_arena_s1.json not generated")
    return ar.load_arena(path)


# ------------------------------------------------------------------ fast ---

def test_v18_default_lidar_backend_and_hfov_unchanged(v18):
    if "NIDAR_LIDAR_BACKEND" in os.environ or "NIDAR_CAMERA_HFOV_DEG" in os.environ:
        pytest.skip("environment overrides set")
    assert v18.LIDAR_BACKEND == "SIMULATOR"
    assert v18.CAMERA_HFOV_DEG == 70.0 and v18.CAMERA_WIDTH == 1280.0


def test_v18_bearing_uses_the_packet_camera_width(v18):
    assert v18.camera_bearing_from_object({"center": [640.0, 0]}, 640) == pytest.approx(35.0)
    assert v18.camera_bearing_from_object({"center": [320.0, 0]}, 640) == pytest.approx(0.0)
    assert v18.camera_bearing_from_object({"center": [640.0, 0]}) == pytest.approx(0.0), "default 1280"
    assert v18.packet_camera_width({"camera": {"width": 640}}) == 640.0
    for bad in (None, {}, {"camera": None}, {"camera": {"width": "x"}}, {"camera": {"width": 0}},
                {"camera": {"width": float("nan")}}):
        assert v18.packet_camera_width(bad) == v18.CAMERA_WIDTH


def test_v18_fusion_with_640px_camera_matches_lidar(v18):
    """A 640-px frame used to be read as 1280 px: x=480 gave 0 deg instead of +17.5 deg."""
    cam = [{"id": 1, "class": "person", "center": [480.0, 240.0], "confidence": 0.8}]
    lidar = [{"track_id": 7, "bearing_deg": 17.5, "distance_m": 2.0, "confirmed": True}]
    lidar = [t for t in (v18.normalize_track(x) for x in lidar) if t]
    assert lidar, "normalize_track rejected the synthetic track"
    assert v18.fuse_objects(cam, lidar, 640.0), "640-px packet must fuse at +17.5 deg"
    assert not v18.fuse_objects(cam, lidar), "same pixel read as 1280-px frame is ~-8 deg away"


def test_slam_gazebo_backend_is_selectable(slam_module, monkeypatch):
    if "NIDAR_LIDAR_BACKEND" not in os.environ:
        assert slam_module.LIDAR_BACKEND == "SIMULATOR"
    monkeypatch.setattr(slam_module, "LIDAR_BACKEND", "gazebo")
    assert slam_module.create_lidar_backend().name == "GAZEBO_LIDAR"
    monkeypatch.setattr(slam_module, "LIDAR_BACKEND", "nonsense")
    with pytest.raises(RuntimeError):
        slam_module.create_lidar_backend()


# ------------------------------------------------------------------ slow ---

def survivor_view(arena, posture="standing", offset_deg=25.0, standoff=1.5):
    from src.simulation.arena_feed import visible_survivors
    walls = ar.Arena(arena.segments, arena.wall_height_m, [], arena.layout)
    for s in arena.survivors:
        if s["posture"] != posture:
            continue
        for k in range(32):
            a = 2 * math.pi * k / 32
            x, y = s["x"] + standoff * math.cos(a), s["y"] + standoff * math.sin(a)
            yaw = math.atan2(s["y"] - y, s["x"] - x) + math.radians(offset_deg)
            if ar.cast(walls, x, y, yaw, 1.0, samples=72)[0].min() < 0.35:
                continue
            if any(v[0]["id"] == s["id"] for v in visible_survivors(arena, x, y, yaw)):
                return s, (x, y, yaw)
    pytest.skip(f"no clear view of a {posture} survivor")


def run_phases(feeder, pose, phases):
    for seconds, camera, lidar in phases:
        t0 = time.time()
        while time.time() - t0 < seconds:
            feeder.step(pose, camera=camera, lidar=lidar)
            time.sleep(0.1)


@pytest.mark.slow
def test_v18_process_with_gazebo_lidar(arena, tmp_path):
    from src.simulation.arena_feed import ArenaFeeder
    s, pose = survivor_view(arena)
    feeder = ArenaFeeder(arena, tmp_path)
    env = dict(os.environ, NIDAR_PROJECT_DIR=str(tmp_path), NIDAR_LIDAR_BACKEND="GAZEBO",
               NIDAR_SCAN_FILE=str(feeder.scan_file), PYTHONUNBUFFERED="1")
    out = tmp_path / "nidar_v18_headless_persistent_fusion.json"
    log_path = tmp_path / "v18.log"
    with open(log_path, "w") as log:
        proc = subprocess.Popen([sys.executable, str(ROOT / "src/perception/nidar_v18_headless_persistent_fusion.py")],
                                env=env, cwd=str(tmp_path), stdout=log, stderr=subprocess.STDOUT)
        try:
            run_phases(feeder, pose, [(2.0, True, False)])           # camera only
            assert not out.exists(), "V18 must not publish while the LiDAR is silent"
            run_phases(feeder, pose, [(3.0, True, True)])            # camera + LiDAR
            d = json.loads(out.read_text())
            run_phases(feeder, pose, [(2.0, True, False)])           # LiDAR stops
            age = time.time() - json.loads(out.read_text())["timestamp"]
            assert proc.poll() is None, "V18 must survive a LiDAR outage"
        finally:
            proc.terminate()
            proc.wait(5)
    text = log_path.read_text()
    assert "[LiDAR STALE]" in text
    assert d["lidar_source"] == "GAZEBO_LIDAR" and d["camera_source"] == "gazebo_sim"
    true_rng, true_brg = ar.bearing_to(*pose, s["x"], s["y"])
    active = [o for o in d["persistent_objects"] if o["state"] == "ACTIVE"]
    assert active, f"no ACTIVE target; log tail:\n{text[-1500:]}"
    assert active[0]["range_m"] == pytest.approx(true_rng - 0.2, abs=0.15)
    assert active[0]["bearing_deg"] == pytest.approx(true_brg, abs=3.0)    # linear px->deg model, see F22 notes
    assert age > 1.5, "V18 JSON must stop updating when the LiDAR stops (server then reports V18_STALE)"


@pytest.mark.slow
def test_slam_process_reads_gazebo_scans_and_waits_when_stale(arena, tmp_path):
    from src.simulation.arena_feed import ArenaFeeder
    feeder = ArenaFeeder(arena, tmp_path)
    for sub in ("state", "maps", "trajectories"):
        (tmp_path / "outputs" / "slam" / sub).mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, NIDAR_LIDAR_BACKEND="GAZEBO", NIDAR_SCAN_FILE=str(feeder.scan_file),
               PYTHONUNBUFFERED="1", PYTHONPATH=os.pathsep.join([str(ROOT), str(ROOT / "src" / "core")]))
    path = ar.survey_path(arena.layout, step_m=0.05)[:60]
    log_path = tmp_path / "slam.log"
    state = tmp_path / "outputs/slam/state/nidar_slam_v5_1_state.json"
    with open(log_path, "w") as log:
        proc = subprocess.Popen([sys.executable, "-m", "src.core.nidar_slam_v5_1"], env=env, cwd=str(tmp_path),
                                stdout=log, stderr=subprocess.STDOUT)
        try:
            time.sleep(1.5)                                          # no scans yet
            for pose in path:
                feeder.step(pose, camera=False, lidar=True)
                time.sleep(0.1)
            time.sleep(1.5)                                          # scans stop
            alive = proc.poll() is None
        finally:
            proc.terminate()
            proc.wait(5)
    text = log_path.read_text()
    assert alive, f"SLAM must survive a LiDAR outage:\n{text[-1500:]}"
    assert "[SLAM] LiDAR stale" in text
    d = json.loads(state.read_text())
    assert d["lidar_source"] == "GAZEBO_LIDAR"
    assert d["frame"] >= 40


@pytest.mark.slow
@pytest.mark.finding
@pytest.mark.xfail(strict=True, reason=(
    "F22: a low survivor (lying model, 0.45 m) is below a 1.0 m LiDAR plane. The camera sees it, the "
    "LiDAR returns the wall behind it, and V18 sometimes fuses the two -> a person reported at the wall's "
    "range. tools/arena_fusion_sweep.py --seed 1 --height 1.0: lying fused 13/85 views, all 13 with range "
    "error > 0.5 m; standing 140/140 fused, median error 0.03 m. Turns green once V18 refuses LiDAR "
    "clusters that cannot be the camera object (e.g. wider than a person) or ranges low targets another way."))
def test_low_survivor_is_never_fused_at_the_wrong_range():
    sys.path.insert(0, str(ROOT / "tools"))
    import arena_fusion_sweep as sweep
    rows = sweep.sweep(seed=1, height=1.0, standoffs=(1.5, 2.0, 2.5), offsets=(-12, 0, 12), headings=16)
    lying = [r for r in rows if r["posture"] == "lying"]
    assert lying
    wrong = [r for r in lying if r["fused"] and abs(r["range"] - r["surface"]) > 0.5]
    assert not wrong, f"{len(wrong)}/{len(lying)} low-survivor views fused with a wrong range"


# ------------------------------------------------------- monitor (F23) ---

def test_integration_monitor_survives_missing_slam_state(tmp_path):
    """
    F23 (2026-11-02): with no SLAM state file yet (fresh outputs/, or Gazebo LiDAR not
    yet sending) the monitor crashed on f"{None:+.3f}", and start_nidar_pi.sh then
    stopped every Pi process. Run the monitor for ~3 s on an empty project.
    """
    import shutil
    shutil.copy(ROOT / "integration_monitor.py", tmp_path / "integration_monitor.py")
    proc = subprocess.Popen([sys.executable, "integration_monitor.py"], cwd=str(tmp_path),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONUNBUFFERED="1"))
    time.sleep(3.0)
    alive = proc.poll() is None
    proc.terminate()
    out = proc.communicate(timeout=5)[0].decode(errors="replace")
    assert alive and "Traceback" not in out, out[-1500:]
    assert "NIDAR SLAM V5.1" in out and "Pose     : X=--- m" in out


# --------------------------------------------------- V18 lock-out (F24) ---

def _fused(v18, bearing, rng=1.0):
    import contextlib
    import io
    cam = [{"id": 1, "class": "person", "center": [320 + 320 * bearing / 35.0, 240], "confidence": 0.8}]
    lid = [t for t in (v18.normalize_track({"track_id": 5, "bearing_deg": bearing % 360, "distance_m": rng,
                                            "confirmed": True}),) if t]
    with contextlib.redirect_stdout(io.StringIO()):
        return v18.fuse_objects(cam, lid, 640)


def _run(v18, manager, sim_clock, bearing, frames):
    import contextlib
    import io
    out = []
    for _ in range(frames):
        sim_clock.advance(0.1)
        with contextlib.redirect_stdout(io.StringIO()):
            out = manager.update(_fused(v18, bearing))
    return out[0]


def test_v18_follows_a_smoothly_turning_target(v18, sim_clock):
    m = v18.PersistentTargetManager()
    _run(v18, m, sim_clock, 10.0, 10)
    for b in (5.0, 0.0, -5.0, -10.0, -15.0, -20.0, -25.0, -30.0):      # 50 deg/s yaw at 10 Hz
        _run(v18, m, sim_clock, b, 1)
    o = _run(v18, m, sim_clock, -30.0, 20)
    assert o["state"] == "ACTIVE" and o["bearing_deg"] == pytest.approx(-30.0, abs=1.0)


@pytest.mark.finding
@pytest.mark.xfail(strict=True, reason=(
    "F24: V18 PersistentTargetManager replaces a bearing that moved > MAX_BEARING_JUMP_DEG (25 deg) "
    "since the last VALID bearing with the old value -- and the old value never changes again, so after "
    "one jump (drone yaw between two frames, a camera stall during a turn, re-acquisition after a gap) "
    "the target stays ACTIVE with a frozen wrong bearing. Measured: true 0 deg, reported 33 deg for "
    "10 s / 101 outliers; x_m stays 0.545 m, so the bridge would keep commanding lateral motion. "
    "Same pattern for range (MAX_RANGE_JUMP_*). Blocks closed-loop use."))
def test_v18_recovers_after_a_single_bearing_jump(v18, sim_clock):
    m = v18.PersistentTargetManager()
    _run(v18, m, sim_clock, 33.0, 10)
    o = _run(v18, m, sim_clock, 0.0, 30)                                 # 3 s of consistent 0 deg
    assert o["state"] == "ACTIVE"
    assert o["bearing_deg"] == pytest.approx(0.0, abs=3.0), f"bearing frozen at {o['bearing_deg']}"
