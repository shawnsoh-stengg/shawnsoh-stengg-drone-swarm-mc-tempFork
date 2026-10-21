"""Small reusable widgets: status lamps and value tiles (TEST-007)."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget

GREEN = "#2e9e44"
RED = "#d13438"
AMBER = "#d18f00"
GREY = "#8a8a8a"


class Lamp(QWidget):
    """A coloured dot plus a label -- green/red/amber/grey."""

    def __init__(self, text: str = "", parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self._dot = QLabel()
        self._dot.setFixedSize(14, 14)
        self._text = QLabel(text)
        layout.addWidget(self._dot)
        layout.addWidget(self._text)
        layout.addStretch(1)
        self.set_state(GREY)

    def set_state(self, colour: str, text: str | None = None, tooltip: str | None = None) -> None:
        self._dot.setStyleSheet(f"background:{colour}; border-radius:7px; border:1px solid #444;")
        if text is not None:
            self._text.setText(text)
        if tooltip is not None:
            self.setToolTip(tooltip)


class Tile(QFrame):
    """A titled value box; `alarm` turns it red, `warn` amber."""

    def __init__(self, title: str, big: bool = False, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setObjectName("tile")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 6)
        layout.setSpacing(0)
        self._title = QLabel(title)
        self._title.setStyleSheet("color:#666; font-size:11px;")
        self._value = QLabel("—")
        self._value.setTextInteractionFlags(Qt.TextSelectableByMouse)
        size = 22 if big else 15
        self._base = f"font-size:{size}px; font-weight:600;"
        self._value.setStyleSheet(self._base)
        layout.addWidget(self._title)
        layout.addWidget(self._value)

    def set(self, value: str, alarm: bool = False, warn: bool = False, tooltip: str = "") -> None:
        self._value.setText(value)
        colour = RED if alarm else AMBER if warn else "palette(text)"
        self._value.setStyleSheet(self._base + f"color:{colour};")
        self.setToolTip(tooltip)
