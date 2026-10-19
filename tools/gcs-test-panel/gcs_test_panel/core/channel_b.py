"""Channel B client -- GCS <-> gcs_mavlink_node (TEST-007).

Covers what the GCS operator side of Channel B does
(docs/GCS-MAVLINK-API-REFERENCE.md):

- a 1 Hz MAV_TYPE_GCS heartbeat, switchable off so a tester can deliberately
  trip the >10 s GCS_LINK_LOST failsafe (FR-073/SFR-007);
- COMMAND_LONG with a timed COMMAND_ACK, graded against FR-061's 500 ms;
- the MC_MISSION_UPLOAD handshake (MISSION_COUNT -> MISSION_REQUEST_INT* ->
  MISSION_ACK, mission_type=3).

No Qt in here -- the GUI, HSR-003's CLI and pytest all drive this directly.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import defs
from .link import LinkWorker, SplitPortLink
from .plan import MissionItem
from .statustext import StatustextAssembler
from .tlog import TlogWriter


@dataclass
class CommandResult:
    command: int
    params: tuple
    result: Optional[int]  # MAV_RESULT of the receipt ACK, None on timeout
    latency_s: Optional[float]
    # MC_CMD_PHASE_TRANSITION only: gcs_mavlink_node answers IN_PROGRESS on
    # receipt, then relays mission_manager_node's verdict as a second
    # COMMAND_ACK (ACCEPTED/DENIED) plus, when denied, a STATUSTEXT reason.
    outcome: Optional[int] = None
    reason: str = ""

    @property
    def name(self) -> str:
        return defs.COMMAND_NAMES.get(self.command, str(self.command))

    @property
    def result_name(self) -> str:
        if self.result is None:
            return "TIMEOUT"
        name = defs.MAV_RESULT_NAMES.get(self.result, str(self.result))
        if self.result == defs.MAV_RESULT_IN_PROGRESS:
            final = "no final ACK" if self.outcome is None else defs.MAV_RESULT_NAMES.get(self.outcome, str(self.outcome))
            name += f" → {final}"
        return name

    @property
    def within_bound(self) -> bool:
        return self.latency_s is not None and self.latency_s <= defs.COMMAND_ACK_BOUND_S

    @property
    def passed(self) -> bool:
        """FR-061: a receipt ACK within the bound -- not whether the MC acted."""
        return self.result in (0, defs.MAV_RESULT_IN_PROGRESS) and self.within_bound


@dataclass
class UploadResult:
    count: int
    result: Optional[int]  # MAV_MISSION_RESULT, None on timeout/abort
    upload_id: Optional[int]  # MISSION_ACK.opaque_id -- gcs_mavlink_node puts upload_id there
    duration_s: float
    re_requests: int = 0
    error: str = ""
    log: list[str] = field(default_factory=list)

    @property
    def result_name(self) -> str:
        if self.result is None:
            return self.error or "TIMEOUT"
        return defs.MAV_MISSION_RESULT_NAMES.get(self.result, str(self.result))

    @property
    def passed(self) -> bool:
        return self.result == 0


def mission_ack_opaque_id(msg) -> Optional[int]:
    """MISSION_ACK.opaque_id (uint32 extension at payload offset 4).

    pymavlink 2.4.49's bundled common.xml predates that field, so it can't
    decode it -- but gcs_mavlink_node's send_mission_ack() fills it with the
    upload_id that MC_CMD_ARM_AUTHORISE later needs, so read the raw payload.
    MAVLink 2 strips trailing zero bytes; pad back before unpacking.
    """
    # Received messages keep their original wire bytes. MAVLink 2 header is
    # 10 bytes; payload length is byte 1.
    buf = bytes(msg.get_msgbuf())
    if not buf or buf[0] != 0xFD:
        return None
    plen = buf[1]
    body = buf[10 : 10 + plen].ljust(8, b"\x00")
    return struct.unpack_from("<I", body, 4)[0]


class ChannelB:
    def __init__(
        self,
        mc_host: str,
        drone_id: int,
        recv_port: int = defs.CH_B_RECV_PORT,
        send_port: int = defs.CH_B_SEND_PORT,
        tlog: Optional[TlogWriter] = None,
        bind_host: str = "0.0.0.0",
    ):
        self.drone_id = drone_id
        self.link = SplitPortLink(
            recv_port, mc_host, send_port, defs.GCS_SYSTEM_ID, defs.GCS_COMPONENT_ID, bind_host
        )
        self.worker = LinkWorker("channel_b", self.link, tlog)
        self.heartbeat_enabled = True
        self.last_mc_heartbeat: Optional[float] = None
        # The MC's GCS_LINK_LOST timer runs from the last heartbeat it got,
        # so the link-loss test is timed from this, not from the checkbox.
        self.last_heartbeat_sent: Optional[float] = None
        self._hb_stop = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None
        self._upload_lock = threading.Lock()
        self.worker.add_listener(self._track_mc_heartbeat)

    # --- lifecycle ---

    def start(self) -> None:
        self.worker.start()
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, name="channel_b-hb", daemon=True)
        self._hb_thread.start()

    def stop(self) -> None:
        self._hb_stop.set()
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=2.0)
        self.worker.stop()

    def _heartbeat_loop(self) -> None:
        while not self._hb_stop.is_set():
            if self.heartbeat_enabled:
                self.send_heartbeat()
            self._hb_stop.wait(1.0)

    def send_heartbeat(self) -> None:
        # Exactly type=6/autopilot=8 -- gcs_mavlink_node drops anything else.
        self.link.mav.heartbeat_send(defs.MAV_TYPE_GCS, defs.MAV_AUTOPILOT_INVALID, 0, 0, 4)
        self.last_heartbeat_sent = time.time()

    def _is_from_mc(self, msg) -> bool:
        return msg.get_srcSystem() == self.drone_id and msg.get_srcComponent() == defs.MC_COMPONENT_ID

    def _track_mc_heartbeat(self, msg, direction, t) -> None:
        if direction == "rx" and msg.get_type() == "HEARTBEAT" and self._is_from_mc(msg):
            self.last_mc_heartbeat = t

    def link_up(self, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else now
        return self.last_mc_heartbeat is not None and now - self.last_mc_heartbeat < defs.MC_HEARTBEAT_STALE_S

    # --- COMMAND_LONG ---

    def send_command(self, command: int, params=(), timeout: float = 2.0) -> CommandResult:
        p = list(params) + [0.0] * (7 - len(params))
        is_ack = lambda m: m.get_type() == "COMMAND_ACK" and self._is_from_mc(m) and m.command == command  # noqa: E731
        with self.worker.subscribe(["COMMAND_ACK", "STATUSTEXT"]) as sub:
            t0 = time.monotonic()
            self.link.mav.command_long_send(self.drone_id, defs.MC_COMPONENT_ID, command, 0, *p[:7])
            ack = sub.wait_for(is_ack, timeout)
            latency = time.monotonic() - t0 if ack is not None else None
            r = CommandResult(command, tuple(p[:7]), ack.result if ack is not None else None, latency)
            if r.result == defs.MAV_RESULT_IN_PROGRESS:
                final = sub.wait_for(lambda m: is_ack(m) and m.result != defs.MAV_RESULT_IN_PROGRESS, timeout)
                r.outcome = final.result if final is not None else None
                if r.outcome not in (None, 0):
                    # The reason STATUSTEXT follows the DENIED ACK immediately,
                    # possibly as several v2 chunks.
                    chunks = StatustextAssembler()
                    deadline = time.monotonic() + 0.5
                    while (left := deadline - time.monotonic()) > 0:
                        m = sub.wait_for(lambda m: m.get_type() == "STATUSTEXT" and self._is_from_mc(m), left)
                        if m is None:
                            break
                        text = chunks.feed(m, time.monotonic())
                        if text is not None:
                            r.reason = text
                            break
        return r

    def phase_transition(self, phase: int) -> CommandResult:
        return self.send_command(defs.MC_CMD_PHASE_TRANSITION, (float(phase),))

    def arm_authorise(self, upload_id: int) -> CommandResult:
        return self.send_command(defs.MC_CMD_ARM_AUTHORISE, (float(upload_id),))

    def end_mission(self) -> CommandResult:
        return self.send_command(defs.MC_CMD_END_MISSION)

    def gcs_override(self, enable: bool, override_type: int) -> CommandResult:
        return self.send_command(defs.MC_CMD_GCS_OVERRIDE, (1.0 if enable else 0.0, float(override_type)))

    def indoor_gnss_anchor(self, lat_deg: float, lon_deg: float, alt_msl_m: float) -> CommandResult:
        """Same command `ft006 anchor` publishes, but over Channel B. Params are
        float32 on the wire (and in mc_msgs/GcsCommand), so lon near 100 deg
        resolves to ~0.85 m -- identical to the FT-006 helper's precision."""
        return self.send_command(defs.MC_CMD_INDOOR_GNSS_SIM_ANCHOR, (lat_deg, lon_deg, alt_msl_m))

    def emergency_disarm(self) -> CommandResult:
        return self.send_command(defs.MAV_CMD_COMPONENT_ARM_DISARM, (0.0,))

    def emergency_rtl(self) -> CommandResult:
        return self.send_command(defs.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def emergency_set_mode(self, custom_main: int) -> CommandResult:
        return self.send_command(
            defs.MAV_CMD_DO_SET_MODE, (float(defs.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(custom_main))
        )

    # --- MC_MISSION_UPLOAD ---

    def upload_mission(
        self,
        items: list[MissionItem],
        progress: Optional[Callable[[int, int, str], None]] = None,
        silence_timeout: float = defs.MISSION_UPLOAD_SILENCE_TIMEOUT_S,
    ) -> UploadResult:
        """Run the upload handshake as the sender. The MC drives it: it
        requests each seq (re-requesting after 1 s of silence) and closes
        with MISSION_ACK. Serialised -- one upload at a time per link."""
        def note(text: str, seq: int = -1) -> None:
            log.append(text)
            if progress is not None:
                progress(seq, len(items), text)

        log: list[str] = []
        with self._upload_lock, self.worker.subscribe(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"]) as sub:
            t0 = time.monotonic()
            requested: dict[int, int] = {}
            self.link.mav.mission_count_send(
                self.drone_id, defs.MC_COMPONENT_ID, len(items), defs.MAV_MISSION_TYPE_MC
            )
            note(f"MISSION_COUNT count={len(items)} sent")
            while True:
                msg = sub.wait_for(
                    lambda m: self._is_from_mc(m) and getattr(m, "mission_type", None) == defs.MAV_MISSION_TYPE_MC,
                    silence_timeout,
                )
                if msg is None:
                    note(f"no reply for {silence_timeout:.0f} s -- giving up")
                    return UploadResult(
                        len(items), None, None, time.monotonic() - t0,
                        sum(n - 1 for n in requested.values()), "TIMEOUT", log,
                    )
                if msg.get_type() == "MISSION_ACK":
                    upload_id = mission_ack_opaque_id(msg)
                    result = UploadResult(
                        len(items), msg.type, upload_id, time.monotonic() - t0,
                        sum(n - 1 for n in requested.values()), "", log,
                    )
                    note(f"MISSION_ACK {result.result_name} upload_id={upload_id}")
                    return result
                seq = msg.seq
                requested[seq] = requested.get(seq, 0) + 1
                if seq >= len(items):
                    note(f"MC requested seq {seq} beyond count {len(items)}", seq)
                    continue
                item = items[seq]
                self.link.mav.mission_item_int_send(
                    self.drone_id, defs.MC_COMPONENT_ID, seq, item.frame, item.command,
                    0, 1, item.param1, item.param2, item.param3, item.param4,
                    item.x, item.y, item.z, defs.MAV_MISSION_TYPE_MC,
                )
                again = " (re-request)" if requested[seq] > 1 else ""
                note(f"item {seq + 1}/{len(items)} {item.describe()} sent{again}", seq)
