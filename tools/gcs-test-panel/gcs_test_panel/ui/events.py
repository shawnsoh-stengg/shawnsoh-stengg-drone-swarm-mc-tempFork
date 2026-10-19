"""Event log view -- what events.jsonl records, live (TEST-007)."""

from __future__ import annotations

from datetime import datetime

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class EventLog(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        bar = QHBoxLayout()
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["All events", "Pass/fail only", "Failures only"])
        self.filter_combo.currentIndexChanged.connect(self._apply_filter)
        self.counts = QLabel("")
        bar.addWidget(QLabel("Show:"))
        bar.addWidget(self.filter_combo)
        bar.addStretch(1)
        bar.addWidget(self.counts)
        root.addLayout(bar)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Time", "Test case", "Kind", "Result", "Summary"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setWordWrap(False)
        root.addWidget(self.table)
        self.passed = 0
        self.failed = 0

    def add(self, record: dict) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        verdict = record.get("pass")
        result = "" if verdict is None else "PASS" if verdict else "FAIL"
        if verdict is True:
            self.passed += 1
        elif verdict is False:
            self.failed += 1
        stamp = datetime.fromtimestamp(record["t"]).strftime("%H:%M:%S.%f")[:-3]  # local, like the other tables
        values = [stamp, record.get("test_case", ""), record["kind"], result, record["summary"]]
        for col, v in enumerate(values):
            item = QTableWidgetItem(v)
            if col == 3 and verdict is not None:
                item.setForeground(Qt.white)
                item.setBackground(QColor("#2e9e44") if verdict else QColor("#d13438"))
            self.table.setItem(row, col, item)
        self.table.setRowHidden(row, not self._visible(verdict))
        self.table.scrollToBottom()
        self.counts.setText(f"pass {self.passed} · fail {self.failed}")

    def _visible(self, verdict) -> bool:
        mode = self.filter_combo.currentIndex()
        return mode == 0 or (mode == 1 and verdict is not None) or (mode == 2 and verdict is False)

    def _apply_filter(self) -> None:
        for row in range(self.table.rowCount()):
            text = self.table.item(row, 3).text()
            verdict = None if not text else text == "PASS"
            self.table.setRowHidden(row, not self._visible(verdict))
