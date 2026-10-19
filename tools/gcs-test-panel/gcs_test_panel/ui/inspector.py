"""Raw message inspector + per-type stats (TEST-007)."""

from __future__ import annotations

import collections
import struct
from datetime import datetime

from PySide6.QtCore import QAbstractTableModel, QModelIndex, QSortFilterProxyModel, Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QSplitter,
    QTableView,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..core.telemetry import RateMeter

FTP_OPCODES = {
    0: "None", 1: "TerminateSession", 2: "ResetSessions", 3: "ListDirectory", 4: "OpenFileRO",
    5: "ReadFile", 6: "CreateFile", 7: "WriteFile", 8: "RemoveFile", 11: "OpenFileWO",
    15: "BurstReadFile", 128: "Ack", 129: "Nak",
}


def summarise(msg) -> str:
    if msg.get_type() == "FILE_TRANSFER_PROTOCOL":
        raw = bytes(msg.payload)
        seq, session, opcode, size, req, burst, _, offset = struct.unpack("<HBBBBBBI", raw[:12])
        data = raw[12:12 + size]
        text = f"seq={seq} op={FTP_OPCODES.get(opcode, opcode)} req={FTP_OPCODES.get(req, req)} size={size} offset={offset}"
        if opcode == 129 and data:
            text += f" err={data[0]}"
        elif data and all(32 <= c < 127 or c in (0, 9) for c in data[:64]):
            text += f" data={data[:64].decode('ascii', 'replace')!r}"
        return text
    try:
        d = msg.to_dict()
    except Exception:
        return str(msg)
    d.pop("mavpackettype", None)
    parts = []
    for k, v in d.items():
        if isinstance(v, (bytes, bytearray)):
            v = v.rstrip(b"\x00").decode("ascii", "replace")
        if isinstance(v, str):
            v = v.rstrip("\x00")
        if isinstance(v, float):
            v = f"{v:g}"
        parts.append(f"{k}={v}")
    return ", ".join(parts)[:300]


class MessageModel(QAbstractTableModel):
    HEADERS = ["Time", "Ch", "Dir", "Type", "Src", "Fields"]

    def __init__(self, max_rows: int = 5000):
        super().__init__()
        self.rows: list[tuple] = []
        self.max_rows = max_rows

    def rowCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else len(self.rows)

    def columnCount(self, parent=QModelIndex()):
        return len(self.HEADERS)

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def data(self, index, role=Qt.DisplayRole):
        row = self.rows[index.row()]
        if role == Qt.DisplayRole:
            return row[index.column()]
        if role == Qt.ForegroundRole and row[2] == "tx":
            return QColor("#1f6feb")
        return None

    def append(self, new_rows: list[tuple]) -> None:
        if not new_rows:
            return
        overflow = len(self.rows) + len(new_rows) - self.max_rows
        if overflow > 0:
            drop = min(overflow + self.max_rows // 10, len(self.rows))
            self.beginRemoveRows(QModelIndex(), 0, drop - 1)
            del self.rows[:drop]
            self.endRemoveRows()
        first = len(self.rows)
        self.beginInsertRows(QModelIndex(), first, first + len(new_rows) - 1)
        self.rows.extend(new_rows)
        self.endInsertRows()

    def clear(self) -> None:
        self.beginResetModel()
        self.rows.clear()
        self.endResetModel()


class InspectorTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pending: list[tuple] = []
        self._stats: dict[tuple[str, str, str], list] = {}
        self._now = None

        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        bar = QHBoxLayout()
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("Filter (type, channel, field text — e.g. COMMAND_, MC_STATUS, channel_c)")
        self.pause = QCheckBox("Pause")
        self.hide_hb = QCheckBox("Hide HEARTBEAT")
        clear = QPushButton("Clear")
        bar.addWidget(QLabel("Filter:"))
        bar.addWidget(self.filter, 1)
        bar.addWidget(self.hide_hb)
        bar.addWidget(self.pause)
        bar.addWidget(clear)
        root.addLayout(bar)

        split = QSplitter(Qt.Vertical)
        self.model = MessageModel()
        self.proxy = QSortFilterProxyModel()
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterKeyColumn(-1)
        self.proxy.setFilterCaseSensitivity(Qt.CaseInsensitive)
        self.filter.textChanged.connect(self.proxy.setFilterFixedString)
        self.view = QTableView()
        self.view.setModel(self.proxy)
        self.view.verticalHeader().hide()
        self.view.horizontalHeader().setStretchLastSection(True)
        self.view.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        for col, width in enumerate((95, 80, 30, 190, 55)):
            self.view.setColumnWidth(col, width)
        split.addWidget(self.view)

        self.stats = QTableWidget(0, 5)
        self.stats.setHorizontalHeaderLabels(["Channel", "Dir", "Type", "Count", "Rate (Hz)"])
        self.stats.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.stats.verticalHeader().hide()
        self.stats.setEditTriggers(QTableWidget.NoEditTriggers)
        self.stats.setSortingEnabled(True)
        split.addWidget(self.stats)
        split.setSizes([500, 180])
        root.addWidget(split)

        clear.clicked.connect(self._clear)
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._flush)
        self._timer.start()

    def add(self, channel: str, msg, direction: str, t: float) -> None:
        key = (channel, direction, msg.get_type())
        stat = self._stats.setdefault(key, [0, RateMeter()])
        stat[0] += 1
        stat[1].add(t)
        self._now = t
        if self.pause.isChecked() or (self.hide_hb.isChecked() and msg.get_type() == "HEARTBEAT"):
            return
        self._pending.append((
            datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3],
            channel.replace("channel_", ""),
            direction,
            msg.get_type(),
            f"{msg.get_srcSystem()}/{msg.get_srcComponent()}",
            summarise(msg),
        ))

    def _flush(self) -> None:
        pending, self._pending = self._pending, []
        at_bottom = self.view.verticalScrollBar().value() >= self.view.verticalScrollBar().maximum() - 2
        self.model.append(pending)
        if pending and at_bottom:
            self.view.scrollToBottom()

    def refresh_stats(self, now: float) -> None:
        self.stats.setSortingEnabled(False)
        self.stats.setRowCount(len(self._stats))
        for row, ((ch, d, mtype), (count, meter)) in enumerate(sorted(self._stats.items())):
            for col, v in enumerate((ch, d, mtype)):
                self.stats.setItem(row, col, QTableWidgetItem(v))
            item = QTableWidgetItem()
            item.setData(Qt.DisplayRole, count)
            self.stats.setItem(row, 3, item)
            item = QTableWidgetItem()
            item.setData(Qt.DisplayRole, round(meter.rate(now), 2))
            self.stats.setItem(row, 4, item)
        self.stats.setSortingEnabled(True)

    def _clear(self) -> None:
        self.model.clear()
        self._stats.clear()
        self.stats.setRowCount(0)
