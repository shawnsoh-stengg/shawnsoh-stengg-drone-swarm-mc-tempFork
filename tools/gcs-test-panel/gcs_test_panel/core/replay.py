"""Replay a recorded .tlog through the same listeners a live link feeds (TEST-007).

Messages keep their recorded timestamps, so rate meters and plots read the
same as they did live. `speed` scales the wall-clock pacing (0 = as fast as
possible).
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Callable, Optional

import pymavlink.mavutil as mavutil

from . import defs
from .link import RX, TX, Deduper


def read_tlog(path: Path):
    """Yield (msg, direction, timestamp) for every frame in `path`, with the
    same duplicate suppression a live link applies."""
    mlog = mavutil.mavlink_connection(str(path), dialect=None, robust_parsing=True)
    dedup = Deduper()
    try:
        while True:
            msg = mlog.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_type() == "BAD_DATA":
                continue
            direction = TX if msg.get_srcSystem() == defs.GCS_SYSTEM_ID else RX
            t = getattr(msg, "_timestamp", 0.0)
            if direction == RX and dedup.is_duplicate(bytes(msg.get_msgbuf()), t):
                continue
            yield msg, direction, t
    finally:
        mlog.close()


class Replayer:
    def __init__(
        self,
        path: Path,
        listener: Callable[[object, str, float], None],
        speed: float = 1.0,
        on_done: Optional[Callable[[int], None]] = None,
    ):
        self.path = Path(path)
        self.listener = listener
        self.speed = speed
        self.on_done = on_done
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="tlog-replay", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        count = 0
        t_first = None
        wall0 = time.monotonic()
        for msg, direction, t in read_tlog(self.path):
            if self._stop.is_set():
                break
            if t_first is None:
                t_first = t
            if self.speed > 0:
                delay = (t - t_first) / self.speed - (time.monotonic() - wall0)
                if delay > 0 and self._stop.wait(delay):
                    break
            self.listener(msg, direction, t)
            count += 1
        if self.on_done is not None:
            self.on_done(count)
