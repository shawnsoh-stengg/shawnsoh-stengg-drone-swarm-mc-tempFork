"""One MAVLink UDP link per channel, owned by a single reader thread (TEST-007).

The CLI tools (TEST-004, FT-001) read their socket inline: send, then block in
recv_match() for the reply. A GUI can't do that -- telemetry has to keep
flowing to the monitors while a command waits for its ACK -- so here exactly
one thread reads each socket and fans every message out:

- to listeners (monitors, inspector, tlog) -- every message, both directions;
- to short-lived Subscriptions -- the request/reply helpers subscribe
  *before* sending, so a fast reply can never slip past between the send and
  the wait.

Duplicate frames are dropped before fan-out. With a router config from
before 2026-10-18, mavlink-router delivers every MC->GCS frame through two
endpoints when the GCS sends from its own listen port: the fixed
GCS_CHANNEL_B / ORCHESTRATOR_CHANNEL_C target (GCS_IP:port) and the
GCS_MC_RECV / ORCHESTRATOR_MC_RECV Server endpoint, which learns that same
address:port as a client. Same bytes, same MAVLink seq, twice. The current
configs make the Server endpoint ingress-only, so the filter is now a guard
against stale on-device configs. The .tlog still records every copy (it is
the raw record); replay applies the same filter.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Iterable, Optional

import pymavlink.mavutil as mavutil

from .tlog import TlogWriter

RX = "rx"
TX = "tx"

Listener = Callable[[object, str, float], None]


class Deduper:
    """Drops a frame whose exact bytes were already seen within `window_s`.
    Keyed on the whole frame (header incl. seq + payload + CRC), so two
    genuinely separate messages can't collide unless they are identical
    down to the 8-bit sequence number inside one window."""

    def __init__(self, window_s: float = 1.0):
        self.window_s = window_s
        self.dropped = 0
        self._seen: dict[bytes, float] = {}
        self._last_purge = 0.0

    def is_duplicate(self, frame: bytes, t: float) -> bool:
        if t - self._last_purge > self.window_s:
            self._seen = {k: v for k, v in self._seen.items() if t - v <= self.window_s}
            self._last_purge = t
        prev = self._seen.get(frame)
        self._seen[frame] = t
        if prev is not None and t - prev <= self.window_s:
            self.dropped += 1
            return True
        return False


class SplitPortLink(mavutil.mavudp):
    """Binds `recv_port` locally and always sends to `(mc_host, send_port)`.

    Same two-port shape as TEST-004's ChannelCLink, generalised over the
    source identity so Channel B (255/0) and Channel C (255/190) share it.
    `mav.send` is wrapped in a lock: the heartbeat thread, the command worker
    and the UI can all send concurrently, and MAVLink.send() mutates the
    shared sequence counter.
    """

    def __init__(
        self,
        recv_port: int,
        mc_host: str,
        send_port: int,
        source_system: int,
        source_component: int,
        bind_host: str = "0.0.0.0",
    ):
        self._dest = (mc_host, send_port)
        self._on_tx: Optional[Callable[[bytes], None]] = None
        super().__init__(
            f"{bind_host}:{recv_port}",
            input=True,
            source_system=source_system,
            source_component=source_component,
        )
        send = self.mav.send
        lock = threading.Lock()

        def locked_send(mavmsg, force_mavlink1=False):
            with lock:
                send(mavmsg, force_mavlink1=force_mavlink1)

        self.mav.send = locked_send

    def write(self, buf):
        try:
            self.port.sendto(buf, self._dest)
        except OSError:
            return
        if self._on_tx is not None:
            self._on_tx(bytes(buf))


class Subscription:
    """A queue of received messages matching `types` (None = all)."""

    def __init__(self, worker: "LinkWorker", types: Optional[Iterable[str]]):
        self._worker = worker
        self.types = set(types) if types is not None else None
        self.queue: "queue.Queue[object]" = queue.Queue()

    def offer(self, msg) -> None:
        if self.types is None or msg.get_type() in self.types:
            self.queue.put(msg)

    def get(self, timeout: float):
        """Next matching message, or None after `timeout` seconds."""
        try:
            return self.queue.get(timeout=max(timeout, 0.0))
        except queue.Empty:
            return None

    def wait_for(self, predicate: Callable[[object], bool], timeout: float):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            msg = self.get(remaining)
            if msg is not None and predicate(msg):
                return msg

    def drain(self) -> None:
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                return

    def close(self) -> None:
        self._worker._remove_subscription(self)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class LinkWorker:
    def __init__(self, name: str, link: SplitPortLink, tlog: Optional[TlogWriter] = None):
        self.name = name
        self.link = link
        self.tlog = tlog
        self.rx_count = 0
        self.tx_count = 0
        self.bad_count = 0
        self.last_rx_time: Optional[float] = None
        self.dedup = Deduper()
        self._listeners: list[Listener] = []
        self._subs: list[Subscription] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # Separate parser for our own outgoing frames, so the inspector and
        # event log see decoded TX messages too without touching the RX
        # parser's state.
        self._tx_parser = mavutil.mavlink.MAVLink(None)
        self._tx_parser.robust_parsing = True
        link._on_tx = self._handle_tx

    # --- fan-out ---

    def add_listener(self, listener: Listener) -> None:
        with self._lock:
            self._listeners.append(listener)

    def subscribe(self, types: Optional[Iterable[str]] = None) -> Subscription:
        sub = Subscription(self, types)
        with self._lock:
            self._subs.append(sub)
        return sub

    def _remove_subscription(self, sub: Subscription) -> None:
        with self._lock:
            if sub in self._subs:
                self._subs.remove(sub)

    def _notify(self, msg, direction: str, t: float) -> None:
        with self._lock:
            listeners = list(self._listeners)
            subs = list(self._subs) if direction == RX else []
        for sub in subs:
            sub.offer(msg)
        for listener in listeners:
            try:
                listener(msg, direction, t)
            except Exception:  # a broken UI callback must never kill the reader
                pass

    # --- TX path (called from SplitPortLink.write, any thread) ---

    def _handle_tx(self, buf: bytes) -> None:
        t = time.time()
        self.tx_count += 1
        if self.tlog is not None:
            self.tlog.write(buf, t)
        try:
            msgs = self._tx_parser.parse_buffer(buf) or []
        except Exception:
            msgs = []
        for msg in msgs:
            self._notify(msg, TX, t)

    # --- RX thread ---

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"{self.name}-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.link.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.link.recv_match(blocking=True, timeout=0.25)
            except OSError:
                if self._stop.is_set():
                    return
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            t = time.time()
            if msg.get_type() == "BAD_DATA":
                self.bad_count += 1
                continue
            frame = bytes(msg.get_msgbuf())
            if self.tlog is not None:
                self.tlog.write(frame, t)
            if self.dedup.is_duplicate(frame, t):
                continue
            self.rx_count += 1
            self.last_rx_time = t
            self._notify(msg, RX, t)
