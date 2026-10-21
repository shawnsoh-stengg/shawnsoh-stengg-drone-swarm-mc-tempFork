"""Standard MAVLink .tlog writer.

Same format QGroundControl, MAVProxy and pymavlink's mavlogfile use: each
record is an 8-byte big-endian microsecond UTC timestamp followed by the raw
MAVLink frame. Both directions are logged into one file per channel (as
MAVProxy does), so `mavlogdump.py` shows what the panel sent alongside what the
MC replied, and the panel's own replay mode can re-drive its monitors from it.
"""

from __future__ import annotations

import struct
import threading
import time
from pathlib import Path


class TlogWriter:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(self.path, "ab")
        self._lock = threading.Lock()

    def write(self, frame: bytes, timestamp: float | None = None) -> None:
        usec = int((time.time() if timestamp is None else timestamp) * 1e6)
        with self._lock:
            if self._f.closed:
                return
            self._f.write(struct.pack(">Q", usec) + bytes(frame))
            self._f.flush()

    def close(self) -> None:
        with self._lock:
            self._f.close()
