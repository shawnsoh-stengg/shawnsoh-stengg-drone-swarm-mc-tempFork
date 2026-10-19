"""Headless GUI test: drives a whole session through the real widgets
against the fake MC (QT_QPA_PLATFORM=offscreen). Set GCS_PANEL_SHOTS=<dir>
to also save a PNG of every tab."""

import json
import os
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from gcs_test_panel.app import build_parser  # noqa: E402
from gcs_test_panel.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    app.setStyle("Fusion")
    yield app


def pump(app, until, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        app.processEvents()
        if until():
            return True
        time.sleep(0.02)
    app.processEvents()
    return until()


def test_full_session(qapp, tmp_path, ports, fake_mc, monkeypatch):
    for name in ("warning", "question", "information", "critical"):
        monkeypatch.setattr(QMessageBox, name, lambda *a, **k: QMessageBox.Yes)
    b_recv, b_send, c_recv, c_send = ports
    args = build_parser().parse_args([
        "--mc-host", "127.0.0.1", "--bind", "127.0.0.1", "--log-dir", str(tmp_path), "--test-case", "UI-1",
        "--b-recv-port", str(b_recv), "--b-send-port", str(b_send),
        "--c-recv-port", str(c_recv), "--c-send-port", str(c_send),
    ])
    win = MainWindow(args)
    win.show()
    win.connect_links()
    assert pump(qapp, lambda: win.chb.link_up(), 5)
    assert pump(qapp, lambda: win.telemetry.status is not None, 3)

    # Channel C: create the editor's plan, then activate it.
    win.plans._push_c("create")
    assert pump(qapp, lambda: win.plans.table.rowCount() == 1)
    win.plans.table.selectRow(0)
    win.plans._lib("activate")
    assert pump(qapp, lambda: fake_mc.active == "bench_test.json")
    assert pump(qapp, lambda: "bench_test.json" in win.plans.active_label.text())

    # Channel B: upload the same plan; the upload_id lands in the ARM_AUTHORISE box.
    win.plans._push_b()
    assert pump(qapp, lambda: win.commands.upload_id.value() == 1)
    assert win.plans.progress.value() == win.plans.progress.maximum()

    win.commands._arm()
    win.commands.phase_combo.setCurrentIndex(3)  # INGRESS
    win.commands._phase()
    assert pump(qapp, lambda: win.commands.table.rowCount() == 2)
    assert win.commands.table.item(0, 5).text() == "PASS"
    assert pump(qapp, lambda: win.telemetry.status.mission_phase == 3, 3)
    assert pump(qapp, lambda: win.commands.table.item(0, 6).text() == "entered INGRESS", 4)
    assert pump(qapp, lambda: "Current: <b>INGRESS" in win.commands.phase_hint.text(), 2)

    # Emergency path goes through the (patched) confirm dialog.
    win.commands._set_mode()
    assert pump(qapp, lambda: win.commands.table.rowCount() == 3)
    assert pump(qapp, lambda: win.telemetry.status.mission_phase == 255, 3)

    # FR-073: stop the GCS heartbeat; the MC raises GCS_LINK_LOST at 10 s.
    win.commands.hb_check.setChecked(False)
    assert pump(qapp, lambda: any(r.get("kind") == "link_loss_test" for r in _events(win)), 14)
    win.commands.hb_check.setChecked(True)

    shots = os.environ.get("GCS_PANEL_SHOTS")
    if shots:
        win.monitor.set_plan(win.plans.current_plan(show_errors=False))
        for i in range(win.tabs.count()):
            win.tabs.setCurrentIndex(i)
            pump(qapp, lambda: False, 0.6)
            win.grab().save(str(Path(shots) / f"tab{i}.png"))
        win.tabs.setCurrentIndex(0)

    bundle = win.session.export()
    session_dir = win.session.dir
    win.close()

    events = [json.loads(line) for line in (session_dir / "events.jsonl").read_text().splitlines()]
    kinds = [e["kind"] for e in events]
    for kind in ("connect", "plan_create", "plan_activate", "mission_upload", "command", "heartbeat", "link_loss_test"):
        assert kind in kinds, kind
    assert all(e["test_case"] == "UI-1" for e in events if e["kind"] != "session_start")
    loss = next(e for e in events if e["kind"] == "link_loss_test")
    assert loss["pass"] is True, loss
    assert bundle.exists()


def _events(win):
    return [json.loads(line) for line in (win.session.dir / "events.jsonl").read_text().splitlines()]
