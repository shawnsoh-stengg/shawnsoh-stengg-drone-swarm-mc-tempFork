"""Channel B commands tab: COMMAND_LONG + GCS heartbeat control (TEST-007)."""

from __future__ import annotations

import math
import struct
from datetime import datetime
from typing import Callable

from PySide6.QtCore import QSettings, Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..core import defs
from ..core.channel_b import CommandResult
from .widgets import GREEN, RED


class CommandsTab(QWidget):
    # (label, callable(ChannelB) -> CommandResult)
    command_requested = Signal(str, object)
    heartbeat_toggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        top = QHBoxLayout()
        root.addLayout(top)

        # --- PHASE-tier commands ---
        phase_box = QGroupBox("Mission commands (PHASE tier, queued by mission_manager_node)")
        g = QGridLayout(phase_box)

        self.phase_combo = QComboBox()
        for value, name in defs.MISSION_PHASES.items():
            if value != 255:
                self.phase_combo.addItem(f"{value} — {name}", value)
        b = QPushButton("Send MC_CMD_PHASE_TRANSITION")
        b.clicked.connect(self._phase)
        g.addWidget(QLabel("Target phase"), 0, 0)
        g.addWidget(self.phase_combo, 0, 1)
        g.addWidget(b, 0, 2)
        self.phase_hint = QLabel("Current phase unknown — no MC_STATUS yet.")
        self.phase_hint.setWordWrap(True)
        self.phase_hint.setStyleSheet("color:#555;")
        g.addWidget(self.phase_hint, 4, 0, 1, 3)
        self._current_phase = None

        self.upload_id = QSpinBox()
        self.upload_id.setRange(0, 2**31 - 1)
        self.upload_id.setToolTip(
            "Filled automatically: from the last Channel B upload's MISSION_ACK, or a placeholder "
            "after a Channel C activate (its upload_id isn't reported back). 0 CANCELS the "
            "authorisation: mission_manager_node only checks param1 != 0."
        )
        b = QPushButton("Send MC_CMD_ARM_AUTHORISE")
        b.clicked.connect(self._arm)
        g.addWidget(QLabel("Mission upload ID"), 1, 0)
        g.addWidget(self.upload_id, 1, 1)
        g.addWidget(b, 1, 2)

        self.override_enable = QCheckBox("enable")
        self.override_enable.setChecked(True)
        self.override_type = QComboBox()
        for value, name in defs.OVERRIDE_TYPES.items():
            self.override_type.addItem(f"{value} — {name}", value)
        ov = QHBoxLayout()
        ov.addWidget(self.override_enable)
        ov.addWidget(self.override_type, 1)
        b = QPushButton("Send MC_CMD_GCS_OVERRIDE")
        b.clicked.connect(self._override)
        g.addWidget(QLabel("GCS override"), 2, 0)
        g.addLayout(ov, 2, 1)
        g.addWidget(b, 2, 2)

        b = QPushButton("Send MC_CMD_END_MISSION")
        b.clicked.connect(lambda: self._send("MC_CMD_END_MISSION", lambda ch: ch.end_mission()))
        g.addWidget(b, 3, 2)
        top.addWidget(phase_box, 3)

        # --- EMERGENCY tier ---
        emer = QGroupBox("EMERGENCY (bypasses queue, forces FAULT)")
        emer.setStyleSheet(f"QGroupBox {{ border: 2px solid {RED}; border-radius: 4px; margin-top: 10px; }}"
                           f"QGroupBox::title {{ color: {RED}; subcontrol-origin: margin; left: 8px; }}")
        e = QGridLayout(emer)
        disarm = QPushButton("DISARM")
        rtl = QPushButton("RTL")
        self.mode_combo = QComboBox()
        for name, value in defs.PX4_MANUAL_MODES.items():
            self.mode_combo.addItem(f"{name} ({value})", value)
        setmode = QPushButton("DO_SET_MODE")
        for w in (disarm, rtl, setmode):
            w.setStyleSheet(f"QPushButton {{ background:{RED}; color:white; font-weight:600; padding:6px; }}")
        disarm.clicked.connect(lambda: self._emergency("MAV_CMD_COMPONENT_ARM_DISARM (disarm)", lambda ch: ch.emergency_disarm()))
        rtl.clicked.connect(lambda: self._emergency("MAV_CMD_NAV_RETURN_TO_LAUNCH", lambda ch: ch.emergency_rtl()))
        setmode.clicked.connect(self._set_mode)
        e.addWidget(disarm, 0, 0)
        e.addWidget(rtl, 0, 1)
        e.addWidget(self.mode_combo, 1, 0)
        e.addWidget(setmode, 1, 1)
        top.addWidget(emer, 2)

        # --- indoor GNSS sim anchor (FT-006) ---
        anc_box = QGroupBox("Indoor GNSS sim anchor (gnss_sim_node false origin, FT-006)")
        a = QHBoxLayout(anc_box)
        self.anchor_lat = self._coord_box(-90.0, 90.0, 7, "°")
        self.anchor_lon = self._coord_box(-180.0, 180.0, 7, "°")
        self.anchor_alt = self._coord_box(-500.0, 9000.0, 2, " m")
        for label, box in (("Lat", self.anchor_lat), ("Lon", self.anchor_lon), ("Alt MSL", self.anchor_alt)):
            a.addWidget(QLabel(label))
            a.addWidget(box, 1)
        self._settings = QSettings("drone-mc", "gcs-test-panel")
        for key, box in (("anchor/lat", self.anchor_lat), ("anchor/lon", self.anchor_lon), ("anchor/alt", self.anchor_alt)):
            try:
                box.setValue(float(self._settings.value(key, 0.0)))
            except (TypeError, ValueError):
                pass
        b = QPushButton("Send MC_CMD_INDOOR_GNSS_SIM_ANCHOR")
        b.clicked.connect(self._anchor)
        a.addWidget(b)
        root.addWidget(anc_box)

        # --- heartbeat / FR-073 ---
        hb_box = QGroupBox("GCS heartbeat (FR-073 / SFR-007 link-loss test)")
        h = QHBoxLayout(hb_box)
        self.hb_check = QCheckBox("Send 1 Hz GCS HEARTBEAT")
        self.hb_check.setChecked(True)
        self.hb_check.toggled.connect(self.heartbeat_toggled.emit)
        self.hb_status = QLabel("")
        h.addWidget(self.hb_check)
        h.addWidget(self.hb_status, 1)
        root.addWidget(hb_box)

        # --- results ---
        res_box = QGroupBox("Command results")
        rv = QVBoxLayout(res_box)
        note = QLabel(
            f"<b>ACK</b> = gcs_mavlink_node received and decoded it (graded against FR-061/FR-003's "
            f"{defs.COMMAND_ACK_BOUND_S * 1000:.0f} ms). For MC_CMD_PHASE_TRANSITION it is IN_PROGRESS, "
            "followed by mission_manager_node's verdict (→ ACCEPTED / → DENIED, with its reason). "
            "Other commands are ACKed on receipt only. <b>Effect</b> checks MC_STATUS for what actually happened."
        )
        note.setWordWrap(True)
        rv.addWidget(note)
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(
            ["Time", "Command", "Params", "ACK", "ACK latency (ms)", "ACK ≤500 ms", "Effect on MC"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(6, QHeaderView.Stretch)
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        rv.addWidget(self.table)
        root.addWidget(res_box, 1)

    # --- actions ---

    def _send(self, label: str, fn: Callable) -> None:
        self.command_requested.emit(label, fn)

    def _phase(self) -> None:
        phase = self.phase_combo.currentData()
        self._send(f"MC_CMD_PHASE_TRANSITION → {defs.phase_name(phase)}", lambda ch: ch.phase_transition(phase))

    def note_plan_activated(self) -> None:
        """A Channel C activate is a new upload on the MC, but MC_MISSION_PLAN_ACK
        doesn't carry the upload_id mission_orchestrator_node assigned it. The MC
        only checks ARM_AUTHORISE's param1 != 0, so any non-zero value
        authorises; keep a real Channel B upload_id if one is already there."""
        if self.upload_id.value() == 0:
            self.upload_id.setValue(1)

    def _arm(self) -> None:
        upload_id = self.upload_id.value()
        if upload_id == 0:
            answer = QMessageBox.question(
                self, "Cancel arm authorisation?",
                "Upload ID 0 doesn't authorise arming: mission_manager_node treats "
                "param1 = 0 as \"not authorised\" and clears any authorisation already "
                "given.\n\nUpload or activate a plan first (the ID fills in "
                "automatically), or enter its upload ID.\n\nSend 0 anyway, to cancel "
                "the authorisation?",
                QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel,
            )
            if answer == QMessageBox.Yes:
                self._send("MC_CMD_ARM_AUTHORISE upload 0 (cancel)", lambda ch: ch.arm_authorise(0))
            return
        answer = QMessageBox.warning(
            self, "Authorise arming",
            "If every pre-arm check passes, mission_manager_node enters ARMING and "
            "ARMS THE FCC immediately — no further command is sent.\n\n"
            "Props off / area clear?\n\nNote: send this AFTER the mission upload or "
            "plan activate — a new upload resets the authorisation.",
            QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel,
        )
        if answer == QMessageBox.Yes:
            self._send(f"MC_CMD_ARM_AUTHORISE upload {upload_id}", lambda ch: ch.arm_authorise(upload_id))

    @staticmethod
    def _coord_box(lo: float, hi: float, decimals: int, suffix: str) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setRange(lo, hi)
        box.setDecimals(decimals)
        box.setSuffix(suffix)
        box.setSingleStep(10 ** -decimals)
        return box

    def _anchor(self) -> None:
        lat, lon, alt = self.anchor_lat.value(), self.anchor_lon.value(), self.anchor_alt.value()
        if lat == 0.0 and lon == 0.0:
            QMessageBox.warning(self, "Anchor", "Lat/Lon are 0, 0 (Null Island). Enter the bench's real position.")
            return
        f32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]  # noqa: E731
        err_m = math.hypot((f32(lat) - lat) * 111_320.0, (f32(lon) - lon) * 111_320.0 * math.cos(math.radians(lat)))
        answer = QMessageBox.question(
            self, "Anchor indoor GNSS sim",
            f"Set gnss_sim_node's false origin to\n\n  lat {lat:.7f}\n  lon {lon:.7f}\n  alt {alt:.2f} m MSL\n\n"
            f"Sent as float32: {f32(lat):.7f}, {f32(lon):.7f} ({err_m:.2f} m rounding).\n\n"
            "Before sending: drone-mc-indoor-nav running, optical flow arriving (real H-Flow or "
            "`ft006 flow`), MAV_USEHILGPS=1, no real GPS fix (the node rejects the anchor if it sees one).\n\n"
            "Every re-anchor re-zeroes the local position and clears a latched INDOOR_GNSS_SIM_LOST.",
            QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        for key, v in (("anchor/lat", lat), ("anchor/lon", lon), ("anchor/alt", alt)):
            self._settings.setValue(key, v)
        self._send(f"MC_CMD_INDOOR_GNSS_SIM_ANCHOR {lat:.7f}, {lon:.7f}, {alt:.2f} m",
                   lambda ch: ch.indoor_gnss_anchor(lat, lon, alt))

    def _override(self) -> None:
        enable = self.override_enable.isChecked()
        kind = self.override_type.currentData()
        self._send(
            f"MC_CMD_GCS_OVERRIDE {'on' if enable else 'off'} {defs.OVERRIDE_TYPES[kind]}",
            lambda ch: ch.gcs_override(enable, kind),
        )

    def _set_mode(self) -> None:
        mode = self.mode_combo.currentData()
        self._emergency(f"MAV_CMD_DO_SET_MODE {self.mode_combo.currentText()}", lambda ch: ch.emergency_set_mode(mode))

    def _emergency(self, label: str, fn: Callable) -> None:
        answer = QMessageBox.warning(
            self, "Emergency command",
            f"Send {label}?\n\nThe MC treats this as EMERGENCY tier: it bypasses the "
            "command queue and forces the FAULT phase.",
            QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel,
        )
        if answer == QMessageBox.Yes:
            self._send(label, fn)

    # --- results ---

    def add_result(self, label: str, r: CommandResult | None, error: str = "") -> QTableWidgetItem:
        """Adds a row; returns its Effect cell for the caller to fill in later
        (the item keeps its identity as newer rows are inserted above it)."""
        self.table.insertRow(0)
        params = ", ".join(f"{p:.10g}" for p in r.params[:3]) if r else ""
        latency = f"{r.latency_s * 1000:.0f}" if r and r.latency_s is not None else "—"
        verdict = "PASS" if r and r.passed else "FAIL"
        values = [datetime.now().strftime("%H:%M:%S"), label, params, r.result_name if r else error, latency, verdict]
        for col, v in enumerate(values):
            item = QTableWidgetItem(v)
            if col == 5:
                item.setForeground(Qt.white)
                item.setBackground(Qt.darkGreen if verdict == "PASS" else Qt.red)
            elif col == 4 and r and r.latency_s is not None and not r.within_bound:
                item.setForeground(Qt.red)
            self.table.setItem(0, col, item)
        effect = QTableWidgetItem("")
        self.table.setItem(0, 6, effect)
        return effect

    @staticmethod
    def set_effect(item: QTableWidgetItem, text: str, ok: bool | None) -> None:
        item.setText(text)
        item.setToolTip(text)
        if ok is not None:
            item.setForeground(Qt.darkGreen if ok else Qt.red)

    def set_current_phase(self, phase: int | None) -> None:
        if phase == self._current_phase:
            return
        self._current_phase = phase
        if phase is None:
            self.phase_hint.setText("Current phase unknown — no MC_STATUS yet.")
            return
        nxt = defs.PHASE_NEXT.get(phase)
        text = f"Current: <b>{defs.phase_name(phase)}</b>. "
        if nxt is None:
            text += "No transition is accepted from here."
        else:
            text += f"The MC accepts only <b>→ {defs.phase_name(nxt)}</b> from here; anything else is rejected (FR-002). "
            idx = self.phase_combo.findData(nxt)
            if idx >= 0:
                self.phase_combo.setCurrentIndex(idx)
        text += " " + defs.PHASE_GUARD_HINTS.get(phase, "")
        self.phase_hint.setText(text)
    def set_heartbeat_status(self, text: str, alarm: bool) -> None:
        self.hb_status.setText(text)
        self.hb_status.setStyleSheet(f"color:{RED if alarm else GREEN}; font-weight:600;")
