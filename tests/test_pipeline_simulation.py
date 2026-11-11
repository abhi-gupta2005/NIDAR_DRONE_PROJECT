"""
End-to-end scenarios through the real receiver / V18 / target server / bridge
/ setpoint code on the co-simulated world (src/simulation/nidar_pipeline_sim.py).
Shortened durations keep the suite fast; the CLI runs the full versions.
"""

from __future__ import annotations

import dataclasses

import pytest

from src.simulation import nidar_pipeline_sim as psim


def run(name, transport="direct", **overrides):
    sc = dataclasses.replace(psim.SCENARIOS[name], **overrides)
    sim = psim.PipelineSimulation(sc, transport=transport, seed=0)
    return sim.run()


def failing(report):
    return {k: v["detail"] for k, v in report.checks.items() if not v["pass"]}


def test_nominal_direct_passes_all_checks():
    rep = run("nominal", duration_s=8.0)
    assert not failing(rep), failing(rep)
    m = rep.metrics
    assert m["active_coverage_ratio"] == 1.0
    assert m["command_valid_ratio_while_visible"] == 1.0
    assert m["range_error_m"]["median"] < 0.3 and m["bearing_error_deg"]["median"] < 2.0


def test_nominal_file_transport_matches_direct():
    rep = run("nominal", transport="file", duration_s=6.0)
    assert not failing(rep), failing(rep)
    assert rep.camera_frames_seen_by_v18 >= 60


@pytest.mark.network
@pytest.mark.slow
def test_nominal_tcp_transport_through_real_receiver():
    rep = run("nominal", transport="tcp", duration_s=6.0)
    assert not failing(rep), failing(rep)


def test_stale_camera_timeline_matches_v18_design():
    rep = run("stale_camera", duration_s=13.0, camera_stop_at_s=6.0)
    assert not failing(rep), failing(rep)
    tl = rep.metrics["stale_timeline_s_after_last_frame"]
    assert tl["first_zero_command"] <= 1.0
    assert 1.5 <= tl["first_predicted"] <= 3.1
    assert 3.0 <= tl["all_expired"] <= 4.0
    assert tl["target_server_stale"] is not None and tl["target_server_stale"] <= 1.3


def test_short_dropout_keeps_identity_and_recovers_fast():
    rep = run("camera_dropout_recovery", duration_s=12.0, camera_stop_at_s=5.0, camera_resume_at_s=6.5)
    assert not failing(rep), failing(rep)
    rec = rep.metrics["recovery"]["camera_resumed"]
    assert rec["first_valid_command_after_s"] <= 0.5
    assert rep.metrics["stable_ids_per_world_object"] == {"1": [1]}


@pytest.mark.network
@pytest.mark.slow
def test_link_drop_recovers_with_new_identity_after_expiry():
    rep = run("link_drop_tcp", transport="tcp", duration_s=14.0, link_drop_at_s=5.0, link_restore_at_s=9.0)
    assert not failing(rep), failing(rep)
    ids = rep.metrics["stable_ids_per_world_object"]["1"]
    assert len(ids) == 2, "a 4 s outage exceeds EXPIRE_TIMEOUT_SEC: a new stable id is expected"


def test_two_people_keep_separate_identities():
    rep = run("two_person", duration_s=12.0)
    assert not failing(rep), failing(rep)
    assert rep.metrics["identity_switches"] == 0


def test_empty_scene_produces_no_commands():
    rep = run("empty_scene", duration_s=4.0)
    assert not failing(rep), failing(rep)
    assert rep.metrics["command_valid_ticks"] == 0
    assert rep.metrics["target_status_counts"].get("TRACKING", 0) == 0


def test_bearing_model_error_is_bounded_but_present():
    """The linear pixel->bearing mapping in V18 is a documented approximation."""
    rep = run("bearing_sweep", duration_s=10.0)
    assert not failing(rep), failing(rep)
    err = rep.metrics["bearing_error_deg"]
    assert err["p95"] <= 3.0
    assert err["median"] > 0.3, "with perfect sensors the residual comes from the linear bearing model"


def test_v18_crash_stops_commands_within_stale_limit():
    """V18 dies with the person ACTIVE: the port-5010 server must stop forwarding it (2026-11-02 defect)."""
    rep = run("v18_crash", duration_s=10.0, v18_stop_at_s=5.0)
    assert not failing(rep), failing(rep)
    assert rep.metrics["v18_crash_first_zero_command_s"] <= 1.5
    assert rep.metrics["v18_crash_valid_command_ticks_after_2s"] == 0
