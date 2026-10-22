"""Main window: connection/session dock, the tabs, lamps, and all wiring (TEST-007).

Data flow:
  link reader threads -> Bridge.message (queued) -> on_message():
      inspector + Telemetry (which raises phase/fault/rate events)
  QTimer (4 Hz) -> lamps, monitor tiles/plots/map, link edges
  button -> Runner (worker thread, blocking request/reply) -> result slot
      -> result table + Session.event() -> events.jsonl + Event log view
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QCheckBox,
    QDockWidget,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..core import defs
from ..core import plan as planmod
from ..core.channel_b import ChannelB, CommandResult, UploadResult
from ..core.channel_c import ChannelC
from ..core.replay import Replayer
from ..core.session import Session
from ..core.shared import MavftpError
from ..core.telemetry import Telemetry
from .bridge import Bridge, Runner
from .commands import CommandsTab
from .events import EventLog
from .inspector import InspectorTab
from .monitor import MonitorTab
from .plans import PlansTab
from .widgets import AMBER, GREEN, GREY, RED, Lamp


class _ReplaySignals(Bridge):
    replay_done = Signal(int)


class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setWindowTitle("GCS Test Panel — Channel B + C (TEST-007)")
        self.resize(1440, 900)

        self.bridge = _ReplaySignals()
        self.runner = Runner()
        self.session = Session(Path(args.log_dir))
        self.session.add_listener(self.bridge.event.emit)
        self.chb: Optional[ChannelB] = None
        self.chc: Optional[ChannelC] = None
        self.replayer: Optional[Replayer] = None
        self.replay_now: Optional[float] = None
        self.telemetry = Telemetry(args.drone_id, self._telemetry_event)
        self.hb_stopped_at: Optional[float] = None
        self._b_was_up: Optional[bool] = None
        self._b_ever_up = False
        self._dups_reported = False
        self._tick = 0

        self.bridge.message.connect(self.on_message)
        self.bridge.event.connect(self.on_event)
        self.bridge.replay_done.connect(self.on_replay_done)

        # --- tabs ---
        self.monitor = MonitorTab()
        self.commands = CommandsTab()
        self.plans = PlansTab()
        self.inspector = InspectorTab()
        self.events = EventLog()
        self.tabs = QTabWidget()
        self.tabs.addTab(self.monitor, "Monitor")
        self.tabs.addTab(self.commands, "Channel B — Commands")
        self.tabs.addTab(self.plans, "Mission plans (B upload / C library)")
        self.tabs.addTab(self.inspector, "Message inspector")
        self.tabs.addTab(self.events, "Event log")
        self.setCentralWidget(self.tabs)
        self.monitor.bind(self.telemetry)

        self.commands.command_requested.connect(self.on_command)
        self.commands.heartbeat_toggled.connect(self.on_heartbeat_toggled)
        self.plans.library_requested.connect(self.on_library)
        self.plans.upload_requested.connect(self.on_upload)
        self.plans.plan_changed.connect(self.monitor.set_plan)
        self.bridge.upload_progress.connect(self.plans.upload_progress)
        self.monitor.set_plan(self.plans.current_plan(show_errors=False))

        self._build_dock()
        self._build_menu()

        self.lamp_b = Lamp("Channel B: disconnected")
        self.lamp_c = Lamp("Channel C: disconnected")
        self.session_label = QLabel(f"Session: {self.session.dir}")
        self.dup_label = QLabel("")
        self.dup_label.setToolTip("MC router config predates the 2026-10-18 double-delivery fix; see core/link.py's docstring")
        self.statusBar().addWidget(self.dup_label)
        self.statusBar().addWidget(self.lamp_b)
        self.statusBar().addWidget(self.lamp_c)
        self.statusBar().addPermanentWidget(self.session_label)

        self.session.event("session_start", f"session started, logging to {self.session.dir}")
        if args.test_case:
            self.test_case.setText(args.test_case)
            self.session.set_test_case(args.test_case)

        self.timer = QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

        if args.connect:
            QTimer.singleShot(0, self.connect_links)

    # --- layout ---

    def _build_dock(self) -> None:
        dock = QDockWidget("Connection & session", self)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        w = QWidget()
        v = QVBoxLayout(w)

        conn = QGroupBox("MC link")
        f = QFormLayout(conn)
        self.mc_host = QLineEdit(self.args.mc_host)
        self.mc_host.setToolTip("The drone's MC_ETH_IP (192.168.100.11-class), or 127.0.0.1 for local testing")
        self.drone_id = QSpinBox()
        self.drone_id.setRange(1, 254)
        self.drone_id.setValue(self.args.drone_id)
        self.drone_id.setToolTip("The MC's DRONE_ID == its MAVLink system_id")
        self.bind_host = QLineEdit(self.args.bind)
        self.ports = {}
        f.addRow("MC host", self.mc_host)
        f.addRow("Drone ID", self.drone_id)
        f.addRow("Local bind", self.bind_host)
        for key, label, value in (
            ("b_recv", "B listen (GCS_CHANNEL_B)", self.args.b_recv_port),
            ("b_send", "B send → MC (GCS_MC_RECV)", self.args.b_send_port),
            ("c_recv", "C listen (ORCHESTRATOR_CHANNEL_C)", self.args.c_recv_port),
            ("c_send", "C send → MC (ORCHESTRATOR_MC_RECV)", self.args.c_send_port),
        ):
            spin = QSpinBox()
            spin.setRange(1, 65535)
            spin.setValue(value)
            self.ports[key] = spin
            f.addRow(label, spin)
        row = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        row.addWidget(self.connect_btn)
        f.addRow(row)
        f.addRow(QLabel(f"Channel A (:{defs.QGC_CHANNEL_A_PORT}) is QGC's — never bound here."))
        v.addWidget(conn)

        sess = QGroupBox("Test session")
        sf = QVBoxLayout(sess)
        form = QFormLayout()
        self.test_case = QLineEdit()
        self.test_case.setPlaceholderText("e.g. FT-001 AC2")
        self.test_case.editingFinished.connect(lambda: self.session.set_test_case(self.test_case.text().strip()))
        self.tester = QLineEdit(self.session.tester)
        self.tester.editingFinished.connect(self._tester_changed)
        form.addRow("Test case", self.test_case)
        form.addRow("Tester", self.tester)
        sf.addLayout(form)
        self.note = QPlainTextEdit()
        self.note.setPlaceholderText("Observation / note — stamped with the test case into events.jsonl")
        self.note.setMaximumHeight(90)
        sf.addWidget(self.note)
        add = QPushButton("Add note")
        add.clicked.connect(self._add_note)
        sf.addWidget(add)
        export = QPushButton("Export session (.zip)…")
        export.clicked.connect(self.export_session)
        sf.addWidget(export)
        v.addWidget(sess)
        v.addStretch(1)
        dock.setWidget(w)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)

    def _build_menu(self) -> None:
        m = self.menuBar().addMenu("&File")
        a = QAction("Replay .tlog…", self)
        a.triggered.connect(self.start_replay)
        m.addAction(a)
        a = QAction("Stop replay", self)
        a.triggered.connect(self.stop_replay)
        m.addAction(a)
        m.addSeparator()
        a = QAction("Export session…", self)
        a.triggered.connect(self.export_session)
        m.addAction(a)
        a = QAction("Open log folder location", self)
        a.triggered.connect(lambda: QMessageBox.information(self, "Session folder", str(self.session.dir)))
        m.addAction(a)
        m.addSeparator()
        a = QAction("Quit", self)
        a.triggered.connect(self.close)
        m.addAction(a)

    # --- session ---

    def _tester_changed(self) -> None:
        self.session.tester = self.tester.text().strip()
        self.session.write_meta(tester=self.session.tester)

    def _add_note(self) -> None:
        text = self.note.toPlainText().strip()
        if text:
            self.session.note(text)
            self.note.clear()

    def export_session(self) -> None:
        default = str(self.session.dir.with_suffix(".zip"))
        path, _ = QFileDialog.getSaveFileName(self, "Export session", default, "Zip (*.zip)")
        if path:
            dest = self.session.export(Path(path))
            QMessageBox.information(self, "Exported", f"Session bundle written to\n{dest}")

    # --- connection ---

    def toggle_connection(self) -> None:
        if self.chb is None:
            self.connect_links()
        else:
            self.disconnect_links()

    def connect_links(self) -> None:
        if self.replayer is not None:
            self.stop_replay()
        host = self.mc_host.text().strip()
        drone_id = self.drone_id.value()
        bind = self.bind_host.text().strip() or "0.0.0.0"
        p = {k: s.value() for k, s in self.ports.items()}
        chb = chc = None
        try:
            chb = ChannelB(host, drone_id, p["b_recv"], p["b_send"], tlog=self.session.tlog("channel_b"), bind_host=bind)
            chc = ChannelC(host, drone_id, p["c_recv"], p["c_send"], tlog=self.session.tlog("channel_c"), bind_host=bind)
        except OSError as e:
            if chb is not None:
                chb.link.close()
            QMessageBox.critical(
                self, "Cannot open link",
                f"{e}\n\nAnother process (a CLI tool, another panel instance) is probably "
                "already bound to one of the listen ports.",
            )
            return
        chb.heartbeat_enabled = self.commands.hb_check.isChecked()
        chb.worker.add_listener(lambda m, d, t: self.bridge.message.emit("channel_b", m, d, t))
        chc.worker.add_listener(lambda m, d, t: self.bridge.message.emit("channel_c", m, d, t))
        self.telemetry = Telemetry(drone_id, self._telemetry_event)
        self.monitor.bind(self.telemetry)
        self.monitor.set_plan(self.plans.current_plan(show_errors=False))
        self.chb, self.chc = chb, chc
        chb.start()
        chc.start()
        self._b_was_up = None
        self._b_ever_up = False
        self._dups_reported = False
        settings = {"mc_host": host, "drone_id": drone_id, "bind": bind, **p}
        self.session.write_meta(connection=settings)
        self.session.event("connect", f"connected to MC {host} drone {drone_id}", **settings)
        self.connect_btn.setText("Disconnect")
        for w in (self.mc_host, self.drone_id, self.bind_host, *self.ports.values()):
            w.setEnabled(False)
        self.on_library("list", "auto", None)

    def disconnect_links(self) -> None:
        for ch in (self.chb, self.chc):
            if ch is not None:
                ch.stop()
        if self.chb is not None:
            self.session.event("disconnect", "disconnected")
        self.chb = self.chc = None
        self.connect_btn.setText("Connect")
        for w in (self.mc_host, self.drone_id, self.bind_host, *self.ports.values()):
            w.setEnabled(True)

    # --- incoming ---

    def on_message(self, channel: str, msg, direction: str, t: float) -> None:
        if channel == "replay":
            self.replay_now = t
        self.inspector.add(channel, msg, direction, t)
        if direction == "rx":
            self.telemetry.handle(msg, t)

    def on_event(self, record: dict) -> None:
        self.events.add(record)

    def _telemetry_event(self, kind: str, summary: str, passed=None, **data) -> None:
        if self.replayer is not None:
            # Replayed alarms are shown but not written into this session's log.
            self.events.add({"t": self.replay_now or time.time(), "test_case": "replay",
                             "kind": kind, "summary": summary, "pass": passed})
            return
        self.session.event(kind, summary, passed, **data)
        if kind == "fault" and data.get("raised") and "GCS_LINK_LOST" in data["faults"] and self.hb_stopped_at:
            last_hb = self.chb.last_heartbeat_sent if self.chb is not None else None
            elapsed = time.time() - (last_hb if last_hb is not None else self.hb_stopped_at)
            ok = defs.GCS_LINK_TIMEOUT_S <= elapsed <= defs.GCS_LINK_TIMEOUT_S + defs.GCS_LINK_LOST_TOLERANCE_S
            self.session.event(
                "link_loss_test",
                f"GCS_LINK_LOST raised {elapsed:.1f} s after the last GCS heartbeat "
                f"(expected {defs.GCS_LINK_TIMEOUT_S:.0f}–{defs.GCS_LINK_TIMEOUT_S + defs.GCS_LINK_LOST_TOLERANCE_S:.0f} s)",
                ok, elapsed_s=round(elapsed, 2),
            )

    # --- Channel B ---

    def _need(self, ch, name: str) -> bool:
        if ch is None:
            QMessageBox.information(self, "Not connected", f"Connect to the MC first ({name}).")
            return False
        return True

    def on_command(self, label: str, fn) -> None:
        if not self._need(self.chb, "Channel B"):
            return
        chb = self.chb
        self.runner.run(lambda: fn(chb), lambda r: self._command_done(label, r),
                        lambda e: self._command_failed(label, e))

    def _command_done(self, label: str, r: CommandResult) -> None:
        effect_item = self.commands.add_result(label, r)
        latency = f"{r.latency_s * 1000:.0f} ms" if r.latency_s is not None else "no ACK"
        self.session.event(
            "command", f"{label}: ACK {r.result_name} ({latency}) — received by gcs_mavlink_node", r.passed,
            command=r.command, command_name=r.name, params=list(r.params), result=r.result_name, reason=r.reason,
            latency_ms=round(r.latency_s * 1000, 1) if r.latency_s is not None else None,
        )
        if r.result is not None:
            self._watch_effect(label, r, effect_item)

    def _watch_effect(self, label: str, r: CommandResult, item) -> None:
        """gcs_mavlink_node ACKs every command on receipt; whether
        mission_manager_node acted on it is only visible as a phase change in
        MC_STATUS. Watch for EFFECT_WINDOW_S and say what happened."""
        if r.command == defs.MC_CMD_INDOOR_GNSS_SIM_ANCHOR:
            self._watch_anchor_result(item)
            return
        status = self.telemetry.status
        if status is None:
            self.commands.set_effect(item, "unknown — no MC_STATUS (does fcc_bridge publish vehicle_state?)", None)
            return
        start = status.mission_phase
        start_name = defs.phase_name(start)
        cmd = r.command
        if cmd == defs.MC_CMD_PHASE_TRANSITION:
            target = int(round(r.params[0]))
            valid = defs.PHASE_NEXT.get(start) == target
            if r.outcome not in (None, 0):
                # mission_manager_node's own verdict -- no need to infer it.
                text = f"rejected by mission_manager_node: {r.reason or 'no reason given'}"
                if valid and start in defs.PHASE_GUARD_HINTS:
                    text += ". " + defs.PHASE_GUARD_HINTS[start]
                self.commands.set_effect(item, text, False if valid else None)
                self.session.event("command_effect", f"{label}: {text}", False if valid else None,
                                   command=cmd, start_phase=start, end_phase=start, reason=r.reason)
                return
            if target == start:
                self.commands.set_effect(item, f"already in {start_name} — no self-transition row, ignored", None)
                return
            reject_text = (
                f"stayed in {start_name}: guard failed. " + defs.PHASE_GUARD_HINTS.get(start, "")
                if valid else
                f"stayed in {start_name}: rejected as expected — no {start_name} → "
                f"{defs.phase_name(target)} row (FR-002); only → {defs.phase_name(defs.PHASE_NEXT[start])} is accepted"
                if start in defs.PHASE_NEXT else f"stayed in {start_name}: no transitions from here"
            )
            expect, ok_text, fail_verdict = target, f"entered {defs.phase_name(target)}", (False if valid else None)
        elif cmd == defs.MC_CMD_ARM_AUTHORISE:
            if r.params and r.params[0] == 0:
                self.commands.set_effect(item, "authorisation cancelled (upload ID 0)", None)
                return
            if start != 0:
                self.commands.set_effect(item, f"recorded (no phase effect outside PREFLIGHT; now {start_name})", None)
                return
            expect, ok_text, fail_verdict = 1, "pre-arm checks passed — MC entered ARMING and commanded FCC arm", None
            reject_text = ("authorisation recorded, still PREFLIGHT: at least one pre-arm check is failing "
                           "(see the STATUSTEXT lines in the Event log tab)")
        elif cmd in (defs.MAV_CMD_COMPONENT_ARM_DISARM, defs.MAV_CMD_NAV_RETURN_TO_LAUNCH, defs.MAV_CMD_DO_SET_MODE):
            expect, ok_text, fail_verdict = 255, "MC forced FAULT and forwarded the command to the FCC", False
            reject_text = f"stayed in {start_name}: EMERGENCY command had no visible effect"
        elif cmd == defs.MC_CMD_END_MISSION:
            expect, ok_text, fail_verdict = 8, "MC entered RETURN", None
            reject_text = f"stayed in {start_name} (END_MISSION only acts once a mission is under way)"
        else:
            self.commands.set_effect(item, "not observable on Channel B (handled by health_monitor_node)", None)
            return
        if start == expect:
            self.commands.set_effect(item, f"already in {start_name}", None)
            return

        self.commands.set_effect(item, "watching MC_STATUS…", None)
        deadline = time.time() + defs.EFFECT_WINDOW_S
        timer = QTimer(self)
        timer.setInterval(200)

        def check():
            s = self.telemetry.status
            phase = s.mission_phase if s is not None else None
            if phase == expect:
                verdict, text = True, ok_text
            elif time.time() < deadline:
                return
            elif phase is not None and phase != start:
                verdict, text = fail_verdict, f"phase moved to {defs.phase_name(phase)} instead"
            else:
                verdict, text = fail_verdict, reject_text
            timer.stop()
            timer.deleteLater()
            self.commands.set_effect(item, text, verdict)
            self.session.event("command_effect", f"{label}: {text}", verdict, command=cmd, start_phase=start,
                               end_phase=phase)

        timer.timeout.connect(check)
        timer.start()

    def _watch_anchor_result(self, item) -> None:
        """gnss_sim_node answers an anchor only on /mc/status_text, which
        arrives here as a (chunked) STATUSTEXT: 'anchored'/'re-anchored' or
        'rejected'. It can land before this runs, so look back a little."""
        since = time.time() - 2.0
        deadline = time.time() + defs.EFFECT_WINDOW_S
        self.commands.set_effect(item, "waiting for gnss_sim_node's STATUSTEXT…", None)
        timer = QTimer(self)
        timer.setInterval(200)

        def check():
            for t, _sev, text in list(self.telemetry.statustexts):
                if t < since or not text.startswith("gnss_sim_node:"):
                    continue
                if "anchored false origin" in text:
                    verdict = True
                elif "rejected" in text:
                    verdict = False
                else:
                    continue
                break
            else:
                if time.time() < deadline:
                    return
                verdict, text = False, ("no reply from gnss_sim_node: is drone-mc-indoor-nav running "
                                        "(sudo systemctl start drone-mc-indoor-nav)?")
            timer.stop()
            timer.deleteLater()
            self.commands.set_effect(item, text, verdict)
            self.session.event("command_effect", f"anchor: {text}", verdict,
                               command=defs.MC_CMD_INDOOR_GNSS_SIM_ANCHOR)

        timer.timeout.connect(check)
        timer.start()

    def _command_failed(self, label: str, e: BaseException) -> None:
        self.commands.add_result(label, None, str(e))
        self.session.event("command", f"{label}: error {e}", False)

    def on_heartbeat_toggled(self, on: bool) -> None:
        if self.chb is not None:
            self.chb.heartbeat_enabled = on
        self.hb_stopped_at = None if on else time.time()
        self.session.event(
            "heartbeat",
            "GCS heartbeat resumed" if on else
            f"GCS heartbeat stopped — MC should raise GCS_LINK_LOST after {defs.GCS_LINK_TIMEOUT_S:.0f} s",
        )

    def on_upload(self, plan: dict) -> None:
        if not self._need(self.chb, "Channel B"):
            return
        items = planmod.to_mission_items(plan)
        chb = self.chb
        progress = self.bridge.upload_progress.emit
        self.session.event("mission_upload_start", f"Channel B upload of '{plan['name']}' ({len(items)} items)")
        self.runner.run(
            lambda: chb.upload_mission(items, progress=progress),
            lambda r: self._upload_done(plan, r),
            lambda e: self.session.event("mission_upload", f"upload error: {e}", False),
        )

    def _upload_done(self, plan: dict, r: UploadResult) -> None:
        if r.upload_id is not None:
            self.commands.upload_id.setValue(r.upload_id)
        self.monitor.set_plan(plan)
        self.session.event(
            "mission_upload",
            f"Channel B upload '{plan['name']}': {r.result_name}, {r.count} items in {r.duration_s:.2f} s, "
            f"upload_id={r.upload_id}, re-requests={r.re_requests}",
            r.passed, upload_id=r.upload_id, items=r.count, re_requests=r.re_requests,
            duration_s=round(r.duration_s, 3),
        )

    # --- Channel C ---

    def on_library(self, action: str, name: str, data) -> None:
        if self.chc is None:
            if name != "auto":
                self._need(None, "Channel C")
            return
        chc = self.chc
        silent = action == "list" and name == "auto"
        calls = {
            "list": lambda: chc.list_plans(),
            "read": lambda: chc.read_plan(name),
            "create": lambda: chc.create_plan(name, data),
            "update": lambda: chc.update_plan(name, data),
            "delete": lambda: chc.delete_plan(name),
            "activate": lambda: chc.activate(name),
        }
        self.runner.run(calls[action], lambda r: self._library_done(action, name, r, silent),
                        lambda e: self._library_failed(action, name, e, silent))

    def _library_done(self, action: str, name: str, r, silent: bool) -> None:
        if action == "list":
            entries, active = r
            self.plans.set_library(entries, active)
            if not silent:
                self.session.event("plan_list", f"{len(entries)} plan(s) on MC, active: {active or '(none)'}", True,
                                   plans=[e.name for e in entries], active=active)
            return
        if action == "read":
            self.plans.set_editor_text(r.decode("utf-8", "replace"))
            self.session.event("plan_read", f"read {name} ({len(r)} bytes)", True)
            return
        if action == "activate":
            request_id, ack = r
            result = defs.PLAN_ACK_RESULTS.get(ack.result, str(ack.result))
            err = ack.error_message if isinstance(ack.error_message, str) else ack.error_message.decode("ascii", "replace")
            err = err.rstrip("\x00")
            # A REJECTED_* reply is the MC working correctly -- whether it was
            # the expected outcome depends on the test case, so no verdict.
            self.session.event(
                "plan_activate", f"activate {name}: {result}" + (f" — {err}" if err else ""),
                True if ack.result == 0 else None, client_request_id=request_id, result=result, error=err,
            )
            if ack.result == 0:
                self.commands.note_plan_activated()
        else:
            self.session.event(f"plan_{action}", f"{action} {name}: OK", True)
        self.on_library("list", "auto", None)

    def _library_failed(self, action: str, name: str, e: BaseException, silent: bool) -> None:
        timed_out = isinstance(e, MavftpError) and "timed out" in str(e)
        if silent and not timed_out:
            return
        # A Nak is a valid protocol reply (e.g. FileProtected on deleting the
        # active plan) -- logged without a verdict; only no reply is a failure.
        verdict = False if timed_out or not isinstance(e, MavftpError) else None
        if not silent or timed_out:
            self.session.event(f"plan_{action}", f"{action} {name or 'library'}: {e}", verdict)

    # --- replay ---

    def start_replay(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Replay .tlog", str(self.session.dir.parent), "MAVLink tlog (*.tlog);;All files (*)")
        if not path:
            return
        speeds = ["1× (real time)", "4×", "20×", "As fast as possible"]
        choice, ok = QInputDialog.getItem(self, "Replay speed", "Speed:", speeds, 0, False)
        if not ok:
            return
        self.replay(Path(path), {0: 1.0, 1: 4.0, 2: 20.0, 3: 0.0}[speeds.index(choice)])

    def replay(self, path: Path, speed: float) -> None:
        if self.chb is not None:
            self.disconnect_links()
        self.stop_replay()
        self.telemetry = Telemetry(self.drone_id.value(), self._telemetry_event)
        self.monitor.bind(self.telemetry)
        self.inspector._clear()
        self.replay_now = None
        self.replayer = Replayer(
            path, lambda m, d, t: self.bridge.message.emit("replay", m, d, t), speed,
            on_done=self.bridge.replay_done.emit,
        )
        self.replayer.start()
        self.lamp_b.set_state(AMBER, f"REPLAY: {path.name}")
        self.lamp_c.set_state(GREY, "")

    def stop_replay(self) -> None:
        if self.replayer is not None:
            self.replayer.stop()
            self.replayer = None
            self.replay_now = None

    def on_replay_done(self, count: int) -> None:
        self.lamp_b.set_state(GREY, f"REPLAY finished: {count} messages")

    # --- periodic refresh ---

    def refresh(self) -> None:
        self._tick += 1
        replaying = self.replayer is not None
        now = self.replay_now if replaying and self.replay_now else time.time()
        b_up = False
        if not replaying:
            b_up = self._refresh_lamps(now)
        self.monitor.refresh(now, b_up or replaying)
        status = self.telemetry.status
        self.commands.set_current_phase(status.mission_phase if status is not None else None)
        if self._tick % 4 == 0:
            self.inspector.refresh_stats(now)

    def _refresh_lamps(self, now: float) -> bool:
        chb, chc = self.chb, self.chc
        if chb is None:
            self.lamp_b.set_state(GREY, "Channel B: disconnected")
            self.lamp_c.set_state(GREY, "Channel C: disconnected")
            self.commands.set_heartbeat_status("", False)
            return False
        up = chb.link_up(now)
        if up:
            self.lamp_b.set_state(GREEN, f"Channel B: MC heartbeat {now - chb.last_mc_heartbeat:.1f} s ago")
        elif chb.last_mc_heartbeat is not None:
            self.lamp_b.set_state(RED, f"Channel B: MC heartbeat lost ({now - chb.last_mc_heartbeat:.0f} s)")
        else:
            self.lamp_b.set_state(AMBER, "Channel B: waiting for MC heartbeat")
        if self._b_was_up is not None and up != self._b_was_up:
            if up and not self._b_ever_up:
                self.session.event("link", "Channel B up — first MC heartbeat received", True)
            else:
                self.session.event("link", "Channel B MC heartbeat " + ("restored" if up else "lost"), up)
        self._b_ever_up |= up
        self._b_was_up = up
        dups = chb.worker.dedup.dropped + chc.worker.dedup.dropped
        self.dup_label.setText(f"duplicate frames dropped: {dups}" if dups else "")
        if dups and not self._dups_reported:
            self._dups_reported = True
            self.session.event(
                "link", "duplicate MC frames arriving (two mavlink-router endpoints deliver to this "
                "socket) — dropped before display; the .tlog keeps both copies",
            )

        ok = chc.link_up()
        if ok is None:
            self.lamp_c.set_state(GREY, "Channel C: no request yet")
        elif ok:
            self.lamp_c.set_state(GREEN, f"Channel C: last reply {now - chc.last_reply_time:.0f} s ago")
        else:
            self.lamp_c.set_state(RED, "Channel C: last request timed out")

        if self.hb_stopped_at is not None:
            off = now - self.hb_stopped_at
            self.commands.set_heartbeat_status(
                f"GCS heartbeat OFF for {off:.0f} s — MC should declare GCS_LINK_LOST at {defs.GCS_LINK_TIMEOUT_S:.0f} s",
                off >= defs.GCS_LINK_TIMEOUT_S,
            )
        else:
            self.commands.set_heartbeat_status("sending", False)
        return up

    def closeEvent(self, event) -> None:
        self.stop_replay()
        self.disconnect_links()
        self.runner.shutdown()
        self.session.close()
        super().closeEvent(event)
