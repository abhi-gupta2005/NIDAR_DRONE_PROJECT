"""
One-command launcher tools/nidar_run.sh + status reader tools/nidar_status.py (2026-11-09).

The launcher is run for real in TEST mode: every component is a stand-in process, `ssh`, `scp` and
`powershell.exe` are fakes that log their arguments, and a fake Pi mission server streams states.
Pinned: start order (Pi stack + brain before the flight program, which waits for SLAM TRACKING);
a dead relay is restarted and named ("LOST: lidar"); MISSION COMPLETE is announced; Ctrl+C stops the
flight program first, then the Pi (its files copied), then everything else, then YOLO; `stop` works
from another terminal. Real Gazebo/PX4/Pi behaviour is only checked in WSL.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "tools" / "nidar_run.sh"
sys.path.insert(0, str(ROOT / "tools"))
import nidar_status as ns  # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("bash") is None or shutil.which("setsid") is None,
                                reason="bash/setsid not available")


def state(slam="TRACKING", brain="EXPLORE", survivors=0, lidar_age=0.1, plan=3, ret=None):
    return {"type": "state", "health": {"slam": {"status": slam, "age_s": 0.1, "fps": 5.0},
                                        "lidar": {"age_s": lidar_age}, "perception": {"age_s": 0.2},
                                        "v18": {"age_s": 0.1}},
            "explorer": {"state": brain, "plan_id": plan, "hold_reason": None, "return_reason": ret},
            "survivors": [{"status": "CONFIRMED"}] * survivors, "pose": {"grid": "PAD"}}


# ------------------------------------------------------------------ status reader (pure) --
def test_status_summary_and_events_name_the_lost_link():
    a = ns.summarize(state(), 0.2)
    assert a["tracking"] and a["brain"] == "EXPLORE" and a["link"] == "UP"
    b = ns.summarize(state(lidar_age=7.0), 0.2)
    assert ns.events(a, b) == ["LOST: LiDAR scans (WSL relay -> Pi)"]
    assert ns.events(b, a) == ["BACK: LiDAR scans (WSL relay -> Pi)"]
    down = ns.summarize(state(), 9.0)
    assert ns.events(a, down)[0].startswith("LOST: Pi mission server link")
    assert "NO DATA" in ns.status_line(down)
    c = ns.summarize(state(brain="COMPLETE", survivors=2, ret="nothing left to explore"), 0.2)
    ev = ns.events(a, c)
    assert "BRAIN EXPLORE -> COMPLETE" in ev and "MISSION COMPLETE (nothing left to explore)" in ev
    assert "survivors confirmed: 2" in ev
    assert ns.summarize(None, None)["brain"] == "off"
    assert ns.summarize(state(brain=None), 0.2, "PX4 route")["brain"] == "PX4 route"
    line = ns.status_line(a, 0.93)
    assert "SLAM TRACKING 5.0 Hz" in line and "BRAIN EXPLORE plan 3" in line and "Gazebo speed 0.93x" in line


# ------------------------------------------------------------------ launcher --
def dry(*args):
    r = subprocess.run(["bash", str(RUN), "--dry-run", *args], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    return r.stdout


def test_dry_run_rig_and_px4_order():
    out = dry()
    order = [ln.split(":")[0].strip() for ln in out.splitlines() if ln.strip().startswith(("start", "Pi:"))]
    assert order[:2] == ["Pi", "Pi"]                                   # stack + brain first
    assert order.index("start gazebo") < order.index("start lidar") < order.index("start rig")
    assert order[-1] == "start rig" and "--brain 10.218.70.147" in out and "HEADLESS=1" not in out   # window
    assert "HEADLESS=1" in dry("--no-gui")
    assert "nidar_explorer_service.py --radius 0.30 --mission-min 45" in out
    px4 = dry("px4", "--route", "survivor:S2", "--gui", "--seed", "3")
    assert "start agent" in px4 and "PX4_DAEMON=1" in px4 and "HEADLESS=1" not in px4
    assert "--route survivor:S2" in px4 and "nidar_explorer_service" not in px4
    assert "NIDAR_BRAIN_MODE=route" in px4 and "nidar_airmouse_arena_s3" in px4 and "--px4-route" in px4


class FakeMission(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(8)
        self.port = self.srv.getsockname()[1]
        self.state = state()
        self.halt = False

    def run(self):
        self.srv.settimeout(0.2)
        while not self.halt:
            try:
                c, _ = self.srv.accept()
            except socket.timeout:
                continue
            threading.Thread(target=self.feed, args=(c,), daemon=True).start()

    def feed(self, c):
        try:
            while not self.halt:
                c.sendall((json.dumps(self.state, separators=(",", ":")) + "\n").encode())
                time.sleep(0.2)
        except OSError:
            pass


def fake_bin(tmp_path):
    """ssh runs the remote command in a fake Pi home with its output piped, like a real SSH channel: ssh returns
    only when nothing on the 'Pi' still holds that output (2026-11-09: the real launcher hung at step 2)."""
    b = tmp_path / "bin"
    b.mkdir()
    log = tmp_path / "calls.log"
    pi = tmp_path / "pi_home"
    (pi / "drone_project" / ".venv" / "bin").mkdir(parents=True)
    keep = "trap 'kill $! 2>/dev/null; exit 0' INT TERM; sleep 300 & wait\n"
    (pi / "drone_project" / "start_nidar_pi.sh").write_text("#!/bin/bash\necho stack up\n" + keep)
    (pi / "drone_project" / ".venv" / "bin" / "python").write_text("#!/bin/bash\necho brain up\n" + keep)
    for f in ("start_nidar_pi.sh", ".venv/bin/python"):
        (pi / "drone_project" / f).chmod(0o755)
    (b / "ssh").write_text(f'#!/bin/bash\necho "ssh $*" >> "{log}"\ncmd="${{@: -1}}"\n'
                           f'HOME="{pi}" bash -c "$cmd" < /dev/null | cat\nexit "${{PIPESTATUS[0]}}"\n')
    for name in ("scp", "powershell.exe"):
        (b / name).write_text(f'#!/bin/bash\necho "{name} $*" >> "{log}"\nexit 0\n')
    for name in ("ssh", "scp", "powershell.exe"):
        (b / name).chmod(0o755)
    return b, log


def wait_for(path, text, timeout=40):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if path.exists() and text in path.read_text(errors="replace"):
            return True
        time.sleep(0.2)
    return False


@pytest.mark.network
def test_one_command_run_supervises_and_stops_in_order(tmp_path):
    fm = FakeMission()
    fm.start()
    b, calls = fake_bin(tmp_path)
    env = dict(os.environ, NIDAR_RUN_TEST="1", NIDAR_RUN_LOGS=str(tmp_path / "logs"),
               NIDAR_RUN_STATE=str(tmp_path / "state"), NIDAR_MISSION_PORT=str(fm.port),
               PATH=f"{b}:{os.environ['PATH']}")
    out = tmp_path / "launcher.out"
    with open(out, "w") as fo:
        p = subprocess.Popen(["bash", str(RUN), "--pi", "127.0.0.1"], stdout=fo, stderr=subprocess.STDOUT, env=env)
    try:
        assert wait_for(out, "FLYING"), out.read_text()
        pids = dict(ln.split() for ln in (tmp_path / "state" / "pids").read_text().splitlines())
        for name in ("gazebo", "lidar", "camera", "bridge", "status", "gcs", "rig"):
            assert name in pids, pids
        c = calls.read_text()
        assert "start_nidar_pi.sh" in c and "nidar_explorer_service.py --radius 0.30" in c
        assert "nidar_yolo.ps1" in c                                       # YOLO window opened
        for name in ("rig", "lidar"):                                      # Ctrl+C reaches the programs
            sigign = next(ln for ln in Path(f"/proc/{pids[name]}/status").read_text().splitlines()
                          if ln.startswith("SigIgn"))
            assert not int(sigign.split()[1], 16) & (1 << (signal.SIGINT - 1)), (name, sigign)
        os.kill(int(pids["lidar"]), signal.SIGKILL)                        # a relay dies
        assert wait_for(out, "LOST: lidar stopped - restarting (1/5"), out.read_text()
        fm.state = state(brain="COMPLETE", ret="nothing left to explore")
        assert wait_for(out, "MISSION COMPLETE"), out.read_text()
        r = subprocess.run(["bash", str(RUN), "stop"], env=env, capture_output=True, text=True, timeout=30)
        assert "asking the running launcher to stop" in r.stdout
        p.wait(timeout=40)
        text = out.read_text()
        assert "STOPPED." in text and p.returncode == 0
        c = calls.read_text()
        assert c.count("nidar_explorer_service[.]py") >= 2                # Pi stopped before and after
        assert "nidar_yolo_stop.ps1" in c and "tar czf" in c               # YOLO stopped, Pi files fetched
        run_dir = next((tmp_path / "logs").glob("run_*_rig"))
        assert (run_dir / "pi" / "logs" / "explorer.log").read_text().strip() == "brain up"
        left = subprocess.run(["pgrep", "-f", str(tmp_path / "pi_home")], capture_output=True, text=True).stdout
        assert left.strip() == "", left                                    # nothing left running on the "Pi"
        time.sleep(0.5)
        alive = [n for n, pid in pids.items() if subprocess.run(["kill", "-0", pid]).returncode == 0]
        assert alive == [], alive
        assert not (tmp_path / "state" / "launcher.pid").exists()
        assert list((tmp_path / "logs").glob("run_*_rig/events.log"))
    finally:
        if p.poll() is None:
            p.kill()
        fm.halt = True


def test_refuses_without_password_free_ssh(tmp_path):
    b, _ = fake_bin(tmp_path)
    (b / "ssh").write_text("#!/bin/bash\nexit 255\n")
    env = dict(os.environ, NIDAR_RUN_TEST="1", NIDAR_RUN_LOGS=str(tmp_path / "logs"),
               NIDAR_RUN_STATE=str(tmp_path / "state"), PATH=f"{b}:{os.environ['PATH']}")
    # nothing answers on the Pi's SSH port: say it is unreachable (2026-11-10), not "run setup"
    r = subprocess.run(["bash", str(RUN), "--pi", "127.0.0.1"], env=dict(env, NIDAR_PI_SSH_PORT="1"),
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 1 and "not reachable" in r.stdout and "--pi NEW_ADDRESS" in r.stdout
    # the Pi answers but refuses the key: run setup
    import socket
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    try:
        r = subprocess.run(["bash", str(RUN), "--pi", "127.0.0.1"],
                           env=dict(env, NIDAR_PI_SSH_PORT=str(srv.getsockname()[1])),
                           capture_output=True, text=True, timeout=30)
    finally:
        srv.close()
    assert r.returncode == 1 and "refused" in r.stdout and "nidar_run.sh setup" in r.stdout


def test_gazebo_speed_comes_from_simulated_time():
    text = "header {\n}\nsim_time {\n  sec: 120\n  nsec: 500000000\n}\nreal_time {\n  sec: 400\n}\n" \
           "real_time_factor: 0.05\n"
    assert ns.parse_sim_time(text) == pytest.approx(120.5)
    assert ns.parse_sim_time("sim_time {\n  nsec: 4000000\n}") == pytest.approx(0.004)
    m = ns.SpeedMeter()
    assert m.update(100.0, 50.0) is None
    assert m.update(110.0, 54.3) == pytest.approx(0.43)                  # not the 0.05 that /stats claimed
    assert m.update(111.0, None) is None


def test_install_pi_sets_up_a_new_pi_and_setup_remembers_it(tmp_path):
    """2026-11-11: a friend's NEW Pi: `setup --pi --user` remembers the Pi (~/.nidar_pi) for every later command;
    `install-pi` copies the project + makes the venv; refuses a Pi that already has the project."""
    b = tmp_path / "bin"
    b.mkdir()
    pi_home = tmp_path / "pi_home"
    pi_home.mkdir()
    (b / "ssh").write_text(f'#!/bin/bash\ncmd="${{@: -1}}"\nHOME="{pi_home}" bash -c "$cmd"\n')
    (b / "ssh").chmod(0o755)
    home = tmp_path / "home"
    home.mkdir()
    (home / ".bashrc").write_text("# user stuff\n")
    qgc = tmp_path / "qgc"
    qgc.write_text("#!/bin/bash\n")
    qgc.chmod(0o755)
    env = dict(os.environ, NIDAR_RUN_TEST="1", NIDAR_RUN_STATE=str(tmp_path / "state"), HOME=str(home),
               NIDAR_SKIP_APT="1", NIDAR_QGC_APP=str(qgc), PATH=f"{b}:{os.environ['PATH']}")
    (home / ".ssh").mkdir()
    (home / ".ssh" / "id_ed25519").write_text("x")
    r = subprocess.run(["bash", str(RUN), "setup", "--pi", "192.168.4.20", "--user", "friend"], env=env,
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr
    saved = (home / ".nidar_pi").read_text()
    assert "NIDAR_PI_HOST=192.168.4.20" in saved and "NIDAR_PI_USER=friend" in saved
    assert (home / ".bashrc").read_text() == "# user stuff\n"
    out = subprocess.run(["bash", str(RUN), "--dry-run"], env=env, capture_output=True, text=True, timeout=60).stdout
    assert "friend@192.168.4.20" in out and "abhishek@" not in out          # the saved Pi is used by itself
    r = subprocess.run(["bash", str(RUN), "install-pi"], env=env, capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout + r.stderr
    proj = pi_home / "drone_project"
    assert (proj / "start_nidar_pi.sh").exists() and (proj / "src/core/nidar_slam_v6.py").exists()
    assert (proj / ".venv/bin/python").exists() and (proj / "logs").is_dir()
    assert not (proj / ".git").exists() and not list(proj.glob("*.pt"))
    assert "OK    src.mission.nidar_explorer_service" in r.stdout and "Pi ready" in r.stdout
    r = subprocess.run(["bash", str(RUN), "install-pi"], env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 1 and "update-pi" in r.stdout and "--force" in r.stdout
