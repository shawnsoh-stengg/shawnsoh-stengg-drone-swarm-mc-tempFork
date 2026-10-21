"""Decoded telemetry state + rate meters (TEST-007).

Holds the latest MC_STATUS/MC_HEALTH, per-type message rates and the
alarm edges the event log records (phase changes, fault bits set/cleared,
rate dropping below its requirement). Pure Python, fed from either a live
LinkWorker or a .tlog replay, so the GUI's monitor and the replay path share
one implementation.
"""

from __future__ import annotations

import collections
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from . import defs
from .statustext import StatustextAssembler


class RateMeter:
    """Messages/second over a sliding window."""

    def __init__(self, window_s: float = 5.0):
        self.window_s = window_s
        self._times: collections.deque[float] = collections.deque()

    def add(self, t: float) -> None:
        self._times.append(t)
        self._trim(t)

    def _trim(self, now: float) -> None:
        while self._times and now - self._times[0] > self.window_s:
            self._times.popleft()

    def rate(self, now: float) -> float:
        # Mean interval between the samples in the window -- exact for a
        # periodic sender, unlike counting samples per window (a 2 Hz stream
        # reads 1.8 or 2.0 depending on where the window edge falls). If the
        # stream then goes quiet for longer than an interval, the silence
        # itself caps the rate, so a stopped stream decays to 0.
        self._trim(now)
        n = len(self._times)
        if n < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0:
            return 0.0
        rate = (n - 1) / span
        silence = now - self._times[-1]
        if silence > 1.5 / rate:
            rate = min(rate, 1.0 / silence)
        return rate


@dataclass
class Detection:
    t: float
    detection_id: int
    class_name: str
    confidence: float
    lat: float
    lon: float
    alt_m: float
    pos_uncertainty_m: float
    drone_id: int


def local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Equirectangular east/north metres from an origin -- plenty for a
    few-km test area, and needs no map tiles or network in the field."""
    r = 6371000.0
    east = math.radians(lon - lon0) * r * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * r
    return east, north


class Telemetry:
    """Event callback signature: (kind, summary, passed, data)."""

    def __init__(self, drone_id: int, on_event: Optional[Callable[..., None]] = None):
        self.drone_id = drone_id
        self.on_event = on_event
        self.status = None
        self.health = None
        self.status_time: Optional[float] = None
        self.health_time: Optional[float] = None
        self.rates: dict[str, RateMeter] = collections.defaultdict(RateMeter)
        self.detections: collections.deque[Detection] = collections.deque(maxlen=500)
        self.track: collections.deque[tuple[float, float, float, float]] = collections.deque(maxlen=20000)
        self.series: collections.deque[tuple[float, float, int, int]] = collections.deque(maxlen=20000)
        self.statustexts: collections.deque[tuple[float, int, str]] = collections.deque(maxlen=200)
        self._statustext_chunks = StatustextAssembler()
        self._prev_phase: Optional[int] = None
        self._prev_fault = 0
        self._prev_alive: Optional[int] = None
        self._rate_low: dict[str, bool] = {}
        self._lock = threading.Lock()

    def _emit(self, kind: str, summary: str, passed: Optional[bool] = None, **data) -> None:
        if self.on_event is not None:
            self.on_event(kind, summary, passed, **data)

    def handle(self, msg, t: float) -> None:
        """Feed one received message (from the MC side of either channel)."""
        mtype = msg.get_type()
        if msg.get_srcSystem() != self.drone_id:
            return
        with self._lock:
            self.rates[mtype].add(t)
            if mtype == "MC_STATUS":
                self._on_status(msg, t)
            elif mtype == "MC_HEALTH":
                self._on_health(msg, t)
            elif mtype == "MC_DETECTION":
                name = msg.class_name
                if isinstance(name, bytes):
                    name = name.decode("ascii", "replace")
                self.detections.append(
                    Detection(
                        t, msg.detection_id, name.rstrip("\x00"), msg.confidence / 10000.0,
                        msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0, msg.pos_uncertainty / 100.0,
                        msg.drone_id,
                    )
                )
            elif mtype == "STATUSTEXT":
                text = self._statustext_chunks.feed(msg, t)
                if text is not None:  # None: a chunk of a longer text, more to come
                    self.statustexts.append((t, msg.severity, text))
                    self._emit("statustext", f"STATUSTEXT[{msg.severity}] {text}")

    def _on_status(self, msg, t: float) -> None:
        self.status = msg
        self.status_time = t
        if msg.lat != 0 or msg.lon != 0:
            self.track.append((t, msg.lat / 1e7, msg.lon / 1e7, msg.alt_msl / 1000.0))
        self.series.append((t, msg.alt_msl / 1000.0, msg.battery_remaining, msg.mission_phase))
        if msg.mission_phase != self._prev_phase:
            prev = defs.phase_name(self._prev_phase) if self._prev_phase is not None else "-"
            self._emit(
                "phase", f"mission phase {prev} -> {defs.phase_name(msg.mission_phase)}",
                phase=msg.mission_phase,
            )
            self._prev_phase = msg.mission_phase
        self._fault_edges(msg.fault_mask)

    def _on_health(self, msg, t: float) -> None:
        self.health = msg
        self.health_time = t
        self._fault_edges(msg.fault_mask)
        if self._prev_alive is not None and msg.node_alive_mask != self._prev_alive:
            lost = defs.decode_bits(self._prev_alive & ~msg.node_alive_mask, defs.NODE_ALIVE_BITS)
            back = defs.decode_bits(msg.node_alive_mask & ~self._prev_alive, defs.NODE_ALIVE_BITS)
            # Observations, not verdicts: whether a node dropping out is a
            # failure depends on the test (e.g. a deliberate kill-restart).
            if lost:
                self._emit("node", f"node(s) not alive: {', '.join(lost)}", nodes=lost, alive=False)
            if back:
                self._emit("node", f"node(s) alive again: {', '.join(back)}", nodes=back, alive=True)
        self._prev_alive = msg.node_alive_mask

    def _fault_edges(self, mask: int) -> None:
        if mask == self._prev_fault:
            return
        raised = defs.decode_bits(mask & ~self._prev_fault, defs.FAULT_BITS)
        cleared = defs.decode_bits(self._prev_fault & ~mask, defs.FAULT_BITS)
        # No verdict here either -- an expected fault (the heartbeat-off test)
        # is graded by the test that provoked it, not by the fault itself.
        if raised:
            self._emit("fault", f"fault raised: {', '.join(raised)}", faults=raised, mask=mask, raised=True)
        if cleared:
            self._emit("fault", f"fault cleared: {', '.join(cleared)}", faults=cleared, mask=mask, raised=False)
        self._prev_fault = mask

    def rate(self, mtype: str, now: Optional[float] = None) -> float:
        with self._lock:
            return self.rates[mtype].rate(time.time() if now is None else now) if mtype in self.rates else 0.0

    def check_rates(self, now: Optional[float] = None, link_up: bool = True) -> dict[str, tuple[float, float, bool]]:
        """Grade MC_STATUS/MC_HEALTH against FR-062/FR-071. Returns
        {type: (rate, required, ok)} and records low/recovered edges --
        only while the link is up, so a deliberately-dropped link doesn't
        double-report as a rate failure."""
        now = time.time() if now is None else now
        out = {}
        for mtype, required in (("MC_STATUS", defs.MC_STATUS_MIN_HZ), ("MC_HEALTH", defs.MC_HEALTH_MIN_HZ)):
            r = self.rate(mtype, now)
            ok = r >= required * 0.95
            out[mtype] = (r, required, ok)
            if not link_up:
                continue
            was_low = self._rate_low.get(mtype)
            if was_low is None:
                # Don't grade until the stream is established (a rate needs
                # a few samples; one message reads as 0 Hz).
                if mtype in self.rates and len(self.rates[mtype]._times) >= 3:
                    self._rate_low[mtype] = False
                continue
            if not ok and not was_low:
                self._emit("rate", f"{mtype} rate {r:.2f} Hz below {required:.0f} Hz", False, rate=r)
            elif ok and was_low:
                self._emit("rate", f"{mtype} rate recovered ({r:.2f} Hz)", True, rate=r)
            self._rate_low[mtype] = not ok
        return out
