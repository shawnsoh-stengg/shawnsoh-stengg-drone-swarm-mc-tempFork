import time

import pytest

from gcs_test_panel.core import defs, plan
from gcs_test_panel.core.channel_b import ChannelB
from gcs_test_panel.core.channel_c import ChannelC
from gcs_test_panel.core.shared import MavftpError


@pytest.fixture
def chb(ports, fake_mc):
    b_recv, b_send, _, _ = ports
    ch = ChannelB("127.0.0.1", 1, b_recv, b_send, bind_host="127.0.0.1")
    ch.start()
    yield ch
    ch.stop()


@pytest.fixture
def chc(ports, fake_mc):
    _, _, c_recv, c_send = ports
    ch = ChannelC("127.0.0.1", 1, c_recv, c_send, timeout=2.0, bind_host="127.0.0.1")
    ch.start()
    yield ch
    ch.stop()


def _wait(pred, timeout=4.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_link_comes_up_after_gcs_heartbeat(chb):
    assert _wait(chb.link_up, 4.0)


def test_command_ack_latency_graded(chb, fake_mc):
    r = chb.phase_transition(3)
    # Receipt ACK is IN_PROGRESS; the verdict arrives as a second ACK.
    assert r.result == defs.MAV_RESULT_IN_PROGRESS and r.passed
    assert r.outcome == 0
    assert r.latency_s < defs.COMMAND_ACK_BOUND_S
    assert fake_mc.received_commands[-1][0] == defs.MC_CMD_PHASE_TRANSITION
    assert fake_mc.received_commands[-1][1][0] == 3.0


def test_phase_transition_denied_carries_reason(chb, fake_mc):
    fake_mc.phase_reject_reason = "not in MUSTER"
    r = chb.phase_transition(5)
    # FR-061 grades the receipt only, so a denied transition still passes.
    assert r.result == defs.MAV_RESULT_IN_PROGRESS and r.passed
    assert r.outcome == 2
    assert r.reason == "not in MUSTER"


def test_slow_ack_fails_bound(chb, fake_mc):
    fake_mc.ack_delay_s = 0.6
    r = chb.end_mission()
    assert r.result == 0 and not r.passed


def test_indoor_gnss_anchor_params(chb, fake_mc):
    r = chb.indoor_gnss_anchor(1.4180151, 103.8629088, 6.0)
    assert r.result == 0 and r.passed
    cmd, params = fake_mc.received_commands[-1]
    assert cmd == defs.MC_CMD_INDOOR_GNSS_SIM_ANCHOR
    # float32 on the wire: lat to ~1e-7 deg, lon (near 100 deg) to ~8e-6 deg.
    assert abs(params[0] - 1.4180151) < 2e-7
    assert abs(params[1] - 103.8629088) < 1e-5
    assert params[2] == 6.0


def test_set_mode_params(chb, fake_mc):
    chb.emergency_set_mode(defs.PX4_MANUAL_MODES["POSCTL"])
    cmd, params = fake_mc.received_commands[-1]
    assert cmd == defs.MAV_CMD_DO_SET_MODE and params[1] == 3.0


def test_mission_upload_with_re_request_and_upload_id(chb, fake_mc):
    fake_mc.drop_mission_items = 1
    items = plan.to_mission_items(plan.example_plan())
    progress = []
    r = chb.upload_mission(items, progress=lambda s, n, text: progress.append(text))
    assert r.passed, r.log
    assert r.upload_id == 1
    assert r.re_requests == 1
    got = fake_mc.uploads[-1]
    assert [m.command for m in got] == [i.command for i in items]
    assert got[0].x == items[0].x and got[0].mission_type == defs.MAV_MISSION_TYPE_MC


def test_channel_c_crud_and_activate(chc, fake_mc):
    body = plan.dumps(plan.example_plan("alpha")).encode()
    chc.create_plan("alpha", body)
    with pytest.raises(MavftpError, match="FileExists"):
        chc.create_plan("alpha", body)
    entries, active = chc.list_plans()
    assert [e.name for e in entries] == ["alpha.json"] and active == ""
    assert chc.read_plan("alpha") == body
    chc.update_plan("alpha", body.replace(b"alpha", b"alpha2"))
    assert b"alpha2" in chc.read_plan("alpha.json")
    _, ack = chc.activate("alpha")
    assert ack.result == 0
    assert chc.list_plans()[1] == "alpha.json"
    with pytest.raises(MavftpError, match="FileProtected"):
        chc.delete_plan("alpha")
    assert chc.link_up() is True


def test_channel_c_activate_rejected_outside_preflight(chc, fake_mc):
    chc.create_plan("beta", b'{"name": "beta"}')
    fake_mc.phase = 7
    _, ack = chc.activate("beta")
    assert defs.PLAN_ACK_RESULTS[ack.result] == "REJECTED_PHASE"


def test_channel_c_timeout_marks_link_down(ports):
    _, _, c_recv, c_send = ports
    ch = ChannelC("127.0.0.1", 1, c_recv, c_send, timeout=0.5, bind_host="127.0.0.1")
    ch.start()
    try:
        with pytest.raises(MavftpError, match="timed out"):
            ch.list_plans()
        assert ch.link_up() is False
    finally:
        ch.stop()


def test_long_deny_reason_is_reassembled_from_chunks(chb, fake_mc):
    reason = "pre-arm check failed: visual localisation required (GNSS degraded, no /mc/local_pose)"
    assert len(reason) > 50
    fake_mc.phase_reject_reason = reason
    r = chb.phase_transition(1)
    assert r.outcome == 2
    assert r.reason == reason


def test_anchor_confirmation_statustext_arrives_whole(chb, fake_mc):
    from gcs_test_panel.core.telemetry import Telemetry
    tele = Telemetry(1)
    chb.worker.add_listener(lambda m, d, t: d == "rx" and tele.handle(m, t))
    chb.indoor_gnss_anchor(1.4180151, 103.8629088, 6.0)
    assert _wait(lambda: any(t.startswith("gnss_sim_node:") for _, _, t in tele.statustexts))
    text = next(t for _, _, t in tele.statustexts if t.startswith("gnss_sim_node:"))
    assert text.startswith("gnss_sim_node: anchored false origin lat=1.41801")
    assert "lon=103.86290" in text and text.endswith("alt_msl=6.00")
