"""Acceptance run against a REAL MC -- skipped unless GCS_PANEL_REAL_MC is set.

    GCS_PANEL_REAL_MC=192.168.100.11 GCS_PANEL_DRONE_ID=1 pytest tests/test_real_mc.py

Drives the panel's core (the same code the GUI buttons call) against real
gcs_mavlink_node + mission_orchestrator_node behind real mavlink-routerd
Channel B/C configs, on the default ports. Needs the MC in PREFLIGHT for the
activate steps. Creates and deletes its own uniquely named plans.

GCS_PANEL_REAL_MC_PHASE_GATE=1 additionally sends MC_CMD_PHASE_TRANSITION ->
INGRESS and expects activate to be refused with REJECTED_PHASE, then returns
to PREFLIGHT -- only on a bench rig where that transition is safe and allowed.
"""

import os
import time

import pytest

from gcs_test_panel.core import defs, plan
from gcs_test_panel.core.channel_b import ChannelB
from gcs_test_panel.core.channel_c import ChannelC
from gcs_test_panel.core.shared import MavftpError
from gcs_test_panel.core.telemetry import Telemetry

HOST = os.environ.get("GCS_PANEL_REAL_MC")
DRONE_ID = int(os.environ.get("GCS_PANEL_DRONE_ID", "1"))
pytestmark = pytest.mark.skipif(not HOST, reason="set GCS_PANEL_REAL_MC=<MC IP> to run against a real MC")


@pytest.fixture(scope="module")
def links():
    tele = Telemetry(DRONE_ID)
    chb = ChannelB(HOST, DRONE_ID)
    chc = ChannelC(HOST, DRONE_ID)
    chb.worker.add_listener(lambda m, d, t: d == "rx" and tele.handle(m, t))
    chb.start()
    chc.start()
    yield chb, chc, tele
    chb.stop()
    chc.stop()


def _wait(pred, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.1)
    return pred()


def test_01_mc_heartbeat(links):
    chb, _, _ = links
    assert _wait(chb.link_up, 6), "no MC HEARTBEAT on Channel B (is mavlink-router-channel-b up?)"


def test_02_telemetry_rates(links):
    _, _, tele = links
    time.sleep(6)
    assert tele.status is not None, "no MC_STATUS (does fcc_bridge_node publish /mc/fcc/vehicle_state?)"
    assert tele.rate("MC_STATUS") >= defs.MC_STATUS_MIN_HZ * 0.95
    if tele.health is not None:
        assert tele.rate("MC_HEALTH") >= defs.MC_HEALTH_MIN_HZ * 0.95


def test_03_command_ack_bound(links):
    chb, _, _ = links
    results = [chb.phase_transition(0) for _ in range(5)]
    for r in results:
        assert r.result == 0, r.result_name
        assert r.within_bound, f"ACK took {r.latency_s * 1000:.0f} ms"


def test_04_channel_b_upload_then_arm(links):
    chb, _, _ = links
    r = chb.upload_mission(plan.to_mission_items(plan.example_plan("e2e_b")))
    assert r.passed, r.log
    assert r.upload_id is not None and r.upload_id > 0
    ack = chb.arm_authorise(r.upload_id)
    assert ack.passed


def test_05_channel_c_crud_activate(links):
    _, chc, tele = links
    stamp = int(time.time())
    a, b = f"e2e_a_{stamp}", f"e2e_b_{stamp}"
    body = plan.dumps(plan.example_plan(a)).encode()
    chc.create_plan(a, body)
    with pytest.raises(MavftpError, match="FileExists"):
        chc.create_plan(a, body)
    entries, _ = chc.list_plans()
    assert f"{a}.json" in [e.name for e in entries]
    assert chc.read_plan(a) == body
    updated = body.replace(b'"lat": 1.3521', b'"lat": 1.3522')
    chc.update_plan(a, updated)
    assert chc.read_plan(a) == updated

    assert tele.status is None or tele.status.mission_phase == 0, "activate needs PREFLIGHT"
    _, ack = chc.activate(a)
    assert defs.PLAN_ACK_RESULTS[ack.result] == "ACCEPTED", ack
    assert chc.list_plans()[1] == f"{a}.json"
    with pytest.raises(MavftpError, match="FileProtected"):
        chc.delete_plan(a)

    chc.create_plan(b, plan.dumps(plan.example_plan(b)).encode())
    _, ack = chc.activate(b)
    assert ack.result == 0
    chc.delete_plan(a)
    with pytest.raises(MavftpError, match="FileNotFound"):
        chc.read_plan(a)


@pytest.mark.skipif(not os.environ.get("GCS_PANEL_REAL_MC_PHASE_GATE"), reason="opt-in: drives a phase transition")
def test_06_activate_refused_outside_preflight(links):
    chb, chc, tele = links
    name = f"e2e_gate_{int(time.time())}"
    chc.create_plan(name, plan.dumps(plan.example_plan(name)).encode())
    try:
        assert chb.phase_transition(3).passed
        assert _wait(lambda: tele.status is not None and tele.status.mission_phase == 3, 5)
        _, ack = chc.activate(name)
        assert defs.PLAN_ACK_RESULTS[ack.result] == "REJECTED_PHASE"
    finally:
        chb.phase_transition(0)
        _wait(lambda: tele.status is not None and tele.status.mission_phase == 0, 5)
