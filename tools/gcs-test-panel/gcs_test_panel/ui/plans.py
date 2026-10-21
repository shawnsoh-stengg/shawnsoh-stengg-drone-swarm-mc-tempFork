"""Mission plans tab: one plan editor, two ways into the MC (TEST-007).

Left: Channel C plan library (MAVFTP list/read/delete + activate).
Right: JSON editor with validation, and the two push paths -- create/update
on Channel C, or upload over Channel B's MC_MISSION_UPLOAD handshake.
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QInputDialog,
    QLabel,
    QListWidget,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..core import plan as planmod
from .widgets import AMBER, GREEN, RED


class PlansTab(QWidget):
    # Channel C requests: (action, name, data)
    library_requested = Signal(str, str, object)
    # Channel B upload: plan dict
    upload_requested = Signal(dict)
    # editor content changed and parses: plan dict or None
    plan_changed = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        root = QHBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        split = QSplitter(Qt.Horizontal)
        root.addWidget(split)

        # --- Channel C library ---
        lib = QGroupBox("Channel C — plan library on the MC (MAVFTP)")
        lv = QVBoxLayout(lib)
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Plan file", "Bytes", "Active"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.doubleClicked.connect(lambda _: self._lib("read"))
        lv.addWidget(self.table)
        row = QHBoxLayout()
        for text, action in (("Refresh", "list"), ("Open in editor", "read"), ("Activate", "activate"), ("Delete", "delete")):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, a=action: self._lib(a))
            row.addWidget(b)
        lv.addLayout(row)
        self.auto_refresh = QCheckBox("Auto-refresh every 5 s (also keeps the Channel C lamp current)")
        lv.addWidget(self.auto_refresh)
        self._timer = QTimer(self)
        self._timer.setInterval(5000)
        self._timer.timeout.connect(lambda: self.library_requested.emit("list", "auto", None))
        self.auto_refresh.toggled.connect(lambda on: self._timer.start() if on else self._timer.stop())
        self.active_label = QLabel("Active plan: —")
        self.active_label.setStyleSheet("font-weight:600;")
        lv.addWidget(self.active_label)
        split.addWidget(lib)

        # --- editor ---
        ed = QGroupBox("Plan editor (docs/GCS-MAVLINK-API-REFERENCE.md \"Mission plan JSON format\")")
        ev = QVBoxLayout(ed)
        file_row = QHBoxLayout()
        for text, slot in (("New from example", self._new), ("Load file…", self._load), ("Save file…", self._save), ("Validate", self._validate_clicked)):
            b = QPushButton(text)
            b.clicked.connect(slot)
            file_row.addWidget(b)
        ev.addLayout(file_row)
        self.editor = QPlainTextEdit()
        font = QFont("Monospace")
        font.setStyleHint(QFont.TypeWriter)
        self.editor.setFont(font)
        self.editor.textChanged.connect(self._on_text)
        ev.addWidget(self.editor, 1)
        self.validity = QLabel("")
        self.validity.setWordWrap(True)
        ev.addWidget(self.validity)

        push = QHBoxLayout()
        b = QPushButton("Create on MC (Channel C)")
        b.clicked.connect(lambda: self._push_c("create"))
        push.addWidget(b)
        b = QPushButton("Update on MC (Channel C)")
        b.clicked.connect(lambda: self._push_c("update"))
        push.addWidget(b)
        b = QPushButton("Upload via Channel B (MC_MISSION_UPLOAD)")
        b.clicked.connect(self._push_b)
        push.addWidget(b)
        ev.addLayout(push)

        up = QGroupBox("Channel B upload progress")
        uv = QVBoxLayout(up)
        self.progress = QProgressBar()
        self.upload_log = QListWidget()
        self.upload_log.setMaximumHeight(140)
        uv.addWidget(self.progress)
        uv.addWidget(self.upload_log)
        ev.addWidget(up)
        split.addWidget(ed)
        split.setSizes([420, 760])

        self.editor.setPlainText(planmod.dumps(planmod.example_plan()))

    # --- editor ---

    def current_plan(self, show_errors: bool = True) -> dict | None:
        try:
            plan, _ = planmod.parse(self.editor.toPlainText())
            return plan
        except planmod.PlanError as e:
            if show_errors:
                QMessageBox.warning(self, "Plan invalid", str(e))
            return None

    def _on_text(self) -> None:
        try:
            plan, warnings = planmod.parse(self.editor.toPlainText())
        except planmod.PlanError as e:
            self.validity.setText(f"✗ {e}")
            self.validity.setStyleSheet(f"color:{RED};")
            self.plan_changed.emit(None)
            return
        if warnings:
            self.validity.setText("⚠ valid, with warnings: " + "; ".join(warnings))
            self.validity.setStyleSheet(f"color:{AMBER};")
        else:
            n = len(planmod.to_mission_items(plan))
            self.validity.setText(f"✓ valid — {n} Channel B mission items")
            self.validity.setStyleSheet(f"color:{GREEN};")
        self.plan_changed.emit(plan)

    def _validate_clicked(self) -> None:
        if self.current_plan() is not None:
            QMessageBox.information(self, "Plan valid", self.validity.text())

    def _new(self) -> None:
        name, ok = QInputDialog.getText(self, "New plan", "Plan name:", text="bench_test")
        if ok and name:
            self.editor.setPlainText(planmod.dumps(planmod.example_plan(name)))

    def _load(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Load plan", "", "Plan JSON (*.json);;All files (*)")
        if path:
            self.editor.setPlainText(Path(path).read_text(encoding="utf-8"))

    def _save(self) -> None:
        plan = self.current_plan()
        if plan is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save plan", f"{plan['name']}.json", "Plan JSON (*.json)")
        if path:
            Path(path).write_text(planmod.dumps(plan), encoding="utf-8")

    def set_editor_text(self, text: str) -> None:
        self.editor.setPlainText(text)

    # --- Channel C ---

    def selected_name(self) -> str | None:
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            return None
        return self.table.item(rows[0].row(), 0).text()

    def _lib(self, action: str) -> None:
        if action == "list":
            self.library_requested.emit("list", "", None)
            return
        name = self.selected_name()
        if name is None:
            QMessageBox.information(self, "No plan selected", "Select a plan in the library first (Refresh to list them).")
            return
        if action == "delete" and QMessageBox.question(self, "Delete plan", f"Delete {name} from the MC?") != QMessageBox.Yes:
            return
        self.library_requested.emit(action, name, None)

    def _push_c(self, action: str) -> None:
        plan = self.current_plan()
        if plan is None:
            return
        plan = planmod.stamp(plan, created=action == "create")
        self.editor.setPlainText(planmod.dumps(plan))
        self.library_requested.emit(action, plan["name"], planmod.dumps(plan).encode())

    def set_library(self, entries, active: str) -> None:
        self.table.setRowCount(0)
        for e in entries:
            row = self.table.rowCount()
            self.table.insertRow(row)
            is_active = e.name == active
            for col, v in enumerate((e.name, str(e.size), "● active" if is_active else "")):
                item = QTableWidgetItem(v)
                if is_active:
                    item.setForeground(Qt.darkGreen)
                    f = item.font()
                    f.setBold(True)
                    item.setFont(f)
                self.table.setItem(row, col, item)
        self.active_label.setText(f"Active plan: {active or '(none)'}")

    # --- Channel B ---

    def _push_b(self) -> None:
        plan = self.current_plan()
        if plan is None:
            return
        self.upload_log.clear()
        self.progress.setRange(0, max(1, len(planmod.to_mission_items(plan))))
        self.progress.setValue(0)
        self.upload_requested.emit(plan)

    def upload_progress(self, seq: int, count: int, text: str) -> None:
        if seq >= 0:
            self.progress.setValue(seq + 1)
        self.upload_log.addItem(text)
        self.upload_log.scrollToBottom()
