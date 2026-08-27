"""Channel C UDP transport and MAVFTP/activate client (TEST-004).

Talks to mission_orchestrator_node the same way the real Mission Orchestrator
app would: MAVFTP (FILE_TRANSFER_PROTOCOL, standard common.xml) for
create/update/list/read/delete, and the two custom messages
(MC_MISSION_PLAN_ACTIVATE/ACK) for activate
(MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md Sec4-Sec7).

Uses pymavlink.mavftp_op.FTP_OP for the MAVFTP header pack/unpack (the same
module pymavlink's own higher-level mavftp.MAVFTP class builds on) rather
than pymavlink.mavftp.MAVFTP itself: that class's command dispatch has no
case for OpenFileWO (see its __mavlink_packet's req_opcode table), so it
cannot drive the "update" half of create/update at all, and its cmd_put/
cmd_get are built around a callback-driven, multi-block-in-flight pipeline
sized for large files over lossy links -- overkill for the single-digit-KB
JSON plans this interface ever carries (mission_orchestrator_node.cpp's own
handle_burst_read() comment makes the same "small file, no need to pipeline"
call). A plain one-request-one-reply loop is simpler and exercises the same
session/CRC-checked wire format either way.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

import pymavlink.mavutil as mavutil
from pymavlink.mavftp import FtpError
from pymavlink.mavftp_op import (
    FTP_OP,
    OP_Ack,
    OP_BurstReadFile,
    OP_CreateFile,
    OP_ListDirectory,
    OP_Nack,
    OP_OpenFileRO,
    OP_OpenFileWO,
    OP_RemoveFile,
    OP_TerminateSession,
    OP_WriteFile,
)

# kFtpMaxData (mission_orchestrator_node.hpp) / MAX_Payload (pymavlink) —
# 251-byte FILE_TRANSFER_PROTOCOL.payload minus the 12-byte FTP_OP header.
MAX_FTP_DATA = 239
# Comfortably under MAX_FTP_DATA; matches the order of magnitude pymavlink's
# own default write_size (80) uses, just larger since this link isn't a
# bandwidth-constrained radio.
WRITE_CHUNK_SIZE = 200

ORCHESTRATOR_SYSTEM_ID = 255  # ground-class system, same convention as QGC (interface spec Sec3.3)
ORCHESTRATOR_COMPONENT_ID = 190  # MAV_COMP_ID_MISSIONPLANNER
MC_COMPONENT_ID = 191  # MAV_COMP_ID_ONBOARD_COMPUTER (interface spec Sec3.3)


class MavftpError(RuntimeError):
    """An MAVFTP Nak, or a timeout waiting for a reply."""

    def __init__(self, message: str, ftp_error: Optional[FtpError] = None):
        super().__init__(message)
        self.ftp_error = ftp_error


@dataclass
class DirectoryEntry:
    name: str
    size: int


class ChannelCLink(mavutil.mavudp):
    """A UDP socket bound to Channel C's Orchestrator-facing ports.

    Binds locally to `recv_port` (ORCHESTRATOR_MAV_PORT_RECV, 14571 by
    default) and always sends to `(mc_host, send_port)`
    (ORCHESTRATOR_MAV_PORT_SEND, 14572) -- config/mavlink-router-channel-c.conf's
    ORCHESTRATOR_CHANNEL_C (Normal, fixed target) and ORCHESTRATOR_MC_RECV
    (Server, learns the sender) endpoints, respectively. Deliberately does
    NOT use mavutil's own 'udpin:'/'udpout:' connection strings, which only
    support a single port doing double duty as both source and destination
    (mavudp.write()'s Server-mode "reply to whichever address last sent us a
    packet" vs. Normal-mode "always send to one fixed destination") -- using
    the two distinct, documented ports explicitly is also how this tool
    demonstrates Channel C isolation from the test side (TEST-004 AC), not
    just relying on mavlink-router's dynamic-client fallback to work it out.
    """

    def __init__(self, recv_port: int, mc_host: str, send_port: int):
        self._dest = (mc_host, send_port)
        super().__init__(
            f"0.0.0.0:{recv_port}",
            input=True,
            source_system=ORCHESTRATOR_SYSTEM_ID,
            source_component=ORCHESTRATOR_COMPONENT_ID,
        )

    def write(self, buf):
        try:
            self.port.sendto(buf, self._dest)
        except OSError:
            pass


def _ftp_op_from_message(msg) -> FTP_OP:
    hdr = bytearray(msg.payload[0:12])
    seq, session, opcode, size, req_opcode, burst_complete, _pad, offset = struct.unpack("<HBBBBBBI", hdr)
    payload = bytearray(msg.payload[12 : 12 + size])
    return FTP_OP(seq, session, opcode, size, req_opcode, burst_complete, offset, payload)


def _ftp_error_name(code: Optional[int]) -> str:
    if code is None:
        return "Unknown"
    try:
        return FtpError(code).name
    except ValueError:
        return f"Unknown({code})"


def _as_ftp_error(code: Optional[int]) -> Optional[FtpError]:
    if code is None:
        return None
    try:
        return FtpError(code)
    except ValueError:
        return None


def _virtual_path(filename: str) -> str:
    """MAVFTP paths are relative to /var/drone-mc/mission_plans/ (interface
    spec Sec4.3) -- mission_orchestrator_node's resolve_plan_path() and its
    "/_active" special case both require the leading '/'; the server does
    not add it itself."""
    return filename if filename.startswith("/") else f"/{filename}"


class ChannelCClient:
    """CRUD + activate client, one instance per drone MC.

    FTP requests target_system=0 (MAVLink broadcast), not `mc_system_id` --
    required, not cosmetic. mavlink-router only routes a message to a
    specific sysid/compid once it has *seen* that identity emit a message on
    some endpoint (Endpoint::_add_sys_comp_id(), called for every message's
    src fields before routing); mission_orchestrator_node never sends
    anything unsolicited (no periodic heartbeat), so its endpoint's "known
    components" table would otherwise never contain 1/191, and a request
    addressed to it by real sysid would be silently dropped as "unknown
    sysid/compid" forever (confirmed with a real mavlink-router build:
    identical drops on three consecutive attempts, `docker/mavlink-router`'s
    pinned commit's src/endpoint.cpp accept_msg()). target_sysid==0 hits
    that same function's unconditional broadcast-accept branch instead, and
    mission_orchestrator_node.cpp's own handle_ftp_request() already checks
    `target_system != system_id_ && target_system != 0` -- broadcast was
    already an anticipated, accepted case there, just never reachable from
    a client using its literal target sysid through this router. Once the
    node's first reply flows back through, mavlink-router learns it for
    real, so this only matters for a cold start -- but a test tool has to
    assume every run is a cold start.

    The actual `mc_system_id` is instead used defensively, to verify a
    reply really came from the drone this tool was pointed at (`--mc-host`
    already selects that; this is a second check, not the addressing
    mechanism).
    """

    def __init__(self, link: ChannelCLink, mc_system_id: int, timeout: float = 5.0):
        self.link = link
        self.mc_system_id = mc_system_id
        self.target_component = MC_COMPONENT_ID
        self.timeout = timeout
        self._seq = 0

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) % 256
        return seq

    def _send_ftp(self, op: FTP_OP) -> None:
        payload = bytearray(op.pack())
        pad_to = 12 + MAX_FTP_DATA
        if len(payload) < pad_to:
            payload.extend(bytearray(pad_to - len(payload)))
        self.link.mav.file_transfer_protocol_send(
            0, 0, self.target_component, bytes(payload)
        )

    def _recv_ftp(self, req_opcode: int, deadline: float) -> FTP_OP:
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise MavftpError(f"timed out waiting for a reply to opcode {req_opcode}")
            msg = self.link.recv_match(
                type=["FILE_TRANSFER_PROTOCOL"], blocking=True, timeout=remaining
            )
            if msg is None:
                continue
            if msg.get_srcSystem() != self.mc_system_id:
                continue
            if (
                msg.target_system != self.link.source_system
                or msg.target_component != self.link.source_component
            ):
                continue
            op = _ftp_op_from_message(msg)
            if op.req_opcode != req_opcode:
                continue
            return op

    def _request(self, opcode: int, data, offset: int = 0) -> FTP_OP:
        """Send one FTP_OP and return whatever the server replies (Ack or Nak) —
        callers decide how to interpret a Nak (e.g. ListDirectory's own EOF
        is a Nak, not an error)."""
        size = len(data) if data is not None else 0
        op = FTP_OP(self._next_seq(), 0, opcode, size, 0, 0, offset, data)
        self._send_ftp(op)
        return self._recv_ftp(opcode, time.time() + self.timeout)

    def _request_ack(self, opcode: int, data, offset: int = 0) -> FTP_OP:
        """Like _request(), but raises MavftpError on any Nak."""
        reply = self._request(opcode, data, offset=offset)
        if reply.opcode == OP_Nack:
            code = reply.payload[0] if reply.payload else None
            raise MavftpError(
                f"Nak({_ftp_error_name(code)}) for opcode {opcode}",
                ftp_error=_as_ftp_error(code),
            )
        return reply

    # --- Create / Update (interface spec Sec6.2) ---

    def create_or_update(self, filename: str, data: bytes, update: bool) -> None:
        open_opcode = OP_OpenFileWO if update else OP_CreateFile
        enc_name = bytearray(_virtual_path(filename), "ascii")
        self._request_ack(open_opcode, enc_name)
        try:
            offset = 0
            while offset < len(data):
                chunk = data[offset : offset + WRITE_CHUNK_SIZE]
                self._request_ack(OP_WriteFile, bytearray(chunk), offset=offset)
                offset += len(chunk)
        finally:
            # TerminateSession triggers finish_write_session() server-side
            # (JSON validation + truncate-to-final-length) even on our own
            # exception path, mirroring mission_orchestrator_node's own
            # "the file stays on disk either way" semantics. Best-effort: a
            # failure here (e.g. the server is what just timed out) must not
            # mask whatever real error sent us into this finally block.
            try:
                self._request(OP_TerminateSession, None)
            except MavftpError:
                pass

    # --- List, and the _active convention (interface spec Sec6.3) ---

    def list_directory(self) -> list[DirectoryEntry]:
        entries: list[DirectoryEntry] = []
        offset = 0
        while True:
            reply = self._request(OP_ListDirectory, None, offset=offset)
            if reply.opcode == OP_Nack:
                code = reply.payload[0] if reply.payload else None
                if code == FtpError.EndOfFile:
                    return entries
                raise MavftpError(
                    f"Nak({_ftp_error_name(code)}) listing plans", ftp_error=_as_ftp_error(code)
                )
            for raw in bytes(reply.payload).split(b"\x00"):
                if not raw or raw[0:1] != b"F":
                    continue
                name, _, size_str = raw[1:].partition(b"\t")
                entries.append(DirectoryEntry(name=name.decode("ascii"), size=int(size_str)))
                offset += 1

    def read_active_filename(self) -> str:
        """ReadFile("/_active") — the reserved, synthetic entry tracking
        which plan is currently active (empty string if none has ever been)."""
        data = self._read_file("_active")
        return data.decode("ascii")

    # --- Read (interface spec Sec6.2, via BurstReadFile per TEST-004 AC) ---

    def read_plan(self, filename: str) -> bytes:
        return self._read_file(filename)

    def _read_file(self, filename: str) -> bytes:
        enc_name = bytearray(_virtual_path(filename), "ascii")
        self._request_ack(OP_OpenFileRO, enc_name)  # Ack payload: uint32 file size, unused here
        try:
            # A single BurstReadFile request gets a *push* of as many Ack
            # replies as the file needs (handle_burst_read() on the server
            # loops internally without waiting for further requests) — send
            # it directly rather than through _request(), which would
            # consume-and-discard that first reply instead of feeding it
            # into the same accumulation loop as the rest.
            self._send_ftp(FTP_OP(self._next_seq(), 0, OP_BurstReadFile, 0, 0, 0, 0, None))
            chunks: dict[int, bytes] = {}
            deadline = time.time() + self.timeout
            while True:
                reply = self._recv_ftp(OP_BurstReadFile, deadline)
                deadline = time.time() + self.timeout
                if reply.opcode == OP_Ack:
                    chunks[reply.offset] = bytes(reply.payload)
                    if reply.burst_complete:
                        break
                    continue
                code = reply.payload[0] if reply.payload else None
                if code == FtpError.EndOfFile:
                    break
                raise MavftpError(
                    f"Nak({_ftp_error_name(code)}) reading {filename}", ftp_error=_as_ftp_error(code)
                )
            return b"".join(chunks[off] for off in sorted(chunks))
        finally:
            try:
                self._request(OP_TerminateSession, None)
            except MavftpError:
                pass

    # --- Delete (interface spec Sec6.5) ---

    def remove_plan(self, filename: str) -> None:
        enc_name = bytearray(_virtual_path(filename), "ascii")
        self._request_ack(OP_RemoveFile, enc_name)

    # --- Activate (interface spec Sec6.4, ICD Sec12.2) ---

    def activate(self, filename: str, client_request_id: int):
        # MC_MISSION_PLAN_ACTIVATE carries no target_system field at all, so
        # mavlink-router already treats it as target_sysid=-1 -- the same
        # unconditional-broadcast-accept branch target_sysid=0 hits for FTP
        # above -- no equivalent workaround needed here.
        padded = filename.encode("ascii")
        self.link.mav.mc_mission_plan_activate_send(client_request_id, padded)
        deadline = time.time() + self.timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise MavftpError("timed out waiting for MC_MISSION_PLAN_ACK")
            msg = self.link.recv_match(
                type=["MC_MISSION_PLAN_ACK"], blocking=True, timeout=remaining
            )
            if (
                msg is not None
                and msg.get_srcSystem() == self.mc_system_id
                and msg.client_request_id == client_request_id
            ):
                return msg
