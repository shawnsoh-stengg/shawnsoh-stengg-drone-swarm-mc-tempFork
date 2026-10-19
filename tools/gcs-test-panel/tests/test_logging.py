import json
import time
import zipfile

from gcs_test_panel.core.channel_b import ChannelB
from gcs_test_panel.core.replay import read_tlog
from gcs_test_panel.core.session import Session
from gcs_test_panel.core.telemetry import RateMeter, Telemetry


def test_rate_meter():
    m = RateMeter(5.0)
    for i in range(20):
        m.add(100 + i * 0.5)
    assert abs(m.rate(109.6) - 2.0) < 0.15
    assert m.rate(130) == 0.0  # stream stopped -> decays to zero


def test_session_tlog_events_and_replay(tmp_path, ports, fake_mc):
    b_recv, b_send, _, _ = ports
    s = Session(tmp_path, "s1")
    s.set_test_case("FT-001 AC2")
    ch = ChannelB("127.0.0.1", 1, b_recv, b_send, tlog=s.tlog("channel_b"), bind_host="127.0.0.1")
    ch.start()
    try:
        deadline = time.time() + 5
        while time.time() < deadline and ch.worker.rx_count < 6:
            time.sleep(0.1)
        r = ch.phase_transition(0)
        s.event("command", "phase", r.passed, latency_ms=r.latency_s * 1000)
    finally:
        ch.stop()
    zpath = s.export()
    s.close()

    events = [json.loads(line) for line in (tmp_path / "s1" / "events.jsonl").read_text().splitlines()]
    assert events[-1]["kind"] == "session_end"
    cmd = next(e for e in events if e["kind"] == "command")
    assert cmd["test_case"] == "FT-001 AC2" and cmd["pass"] is True

    msgs = list(read_tlog(tmp_path / "s1" / "channel_b.tlog"))
    types = {(m.get_type(), d) for m, d, _ in msgs}
    assert ("HEARTBEAT", "tx") in types and ("COMMAND_LONG", "tx") in types
    assert ("MC_STATUS", "rx") in types and ("COMMAND_ACK", "rx") in types

    tele = Telemetry(1)
    for m, d, t in msgs:
        if d == "rx":
            tele.handle(m, t)
    assert tele.status is not None and tele.health is not None

    with zipfile.ZipFile(zpath) as zf:
        assert {"s1/channel_b.tlog", "s1/events.jsonl", "s1/session.json"} <= set(zf.namelist())


def test_deduper_drops_exact_repeats_only():
    from gcs_test_panel.core.link import Deduper

    d = Deduper(1.0)
    assert not d.is_duplicate(b"frame-a", 10.0)
    assert d.is_duplicate(b"frame-a", 10.001)
    assert not d.is_duplicate(b"frame-b", 10.002)
    assert not d.is_duplicate(b"frame-a", 12.0)  # outside the window: a new message
    assert d.dropped == 1
