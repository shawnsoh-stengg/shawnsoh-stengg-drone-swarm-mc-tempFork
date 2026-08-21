"""
ICD-MC-001 §5 (IFD-003) MC <-> Swarm Peers wire protocol.

Header pack/unpack plus the two message payloads mock_peer_node needs:
SWARM_HEARTBEAT (0x01) and PEER_STATE (0x30)/PEER_STATE_REQUEST (0x31). This
is the reference-quality implementation of the wire format ahead of
swarm_coordinator_node's own UDP layer, which CLAUDE.md notes is still a
§2.4/§5/§6 scaffold with no mc_msgs dependency yet.

CRC-32 uses the same polynomial as Python's stdlib zlib.crc32 (standard
CRC-32/ISO-HDLC), matching ICD §5.2's "CRC-32 of header bytes 0-7 + payload".

PEER_STATE_REQUEST (0x31) gap note: ICD-MC-001.md §5.3 lists this message
type but §5's per-message tables never define its payload. Since it is
always unicast to the addressee already known from its own manifest entry,
this module treats it as header-only (payload_length=0) -- the simplest
interpretation consistent with what the ICD *does* specify.

PEER_STATE (0x30) byte-count note: ICD-MC-001.md §5's own prose says
"Payload: 52 bytes", but its field table lists is_leader at offset 52 and a
3-byte reserved block at offset 53-55, i.e. 56 bytes end-to-end. This module
follows the field table (56 bytes), not the summary line.
"""
from __future__ import annotations

from collections import namedtuple
import struct
import zlib

PROTOCOL_VERSION = 0x01

HEARTBEAT_PORT = 14900
COORDINATION_PORT = 14901

MSG_SWARM_HEARTBEAT = 0x01
MSG_ISR_CANDIDATE = 0x10
MSG_ISR_RESULT = 0x11
MSG_ISR_RESULT_ACK = 0x12
MSG_VORONOI_PROPOSAL = 0x20
MSG_VORONOI_ACCEPT = 0x21
MSG_PEER_STATE = 0x30
MSG_PEER_STATE_REQUEST = 0x31

FLAG_REQUEST_ACK = 0x01

_HEADER_PREFIX_FMT = '<BBBBHH'  # version, sender_id, message_type, flags, sequence, payload_length
_HEADER_PREFIX_LEN = 8
_HEADER_LEN = 12  # prefix (8) + crc32 (4)

_HEARTBEAT_FMT = '<QiiifffBBBBHH'
_HEARTBEAT_FIELDS = (
    'timestamp_us', 'lat_e7', 'lon_e7', 'alt_mm', 'vx', 'vy', 'vz',
    'mission_phase', 'battery_pct', 'sensor_health', 'isr_role', 'fault_mask', 'reserved',
)

# timestamp_us, x, y, z, vx, vy, vz, heading, q[4], is_leader, reserved[3]
_PEER_STATE_FMT = '<Q11fB3x'
_PEER_STATE_FIELDS = (
    'timestamp_us', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading',
    'q0', 'q1', 'q2', 'q3', 'is_leader',
)

SwarmHeader = namedtuple(
    'SwarmHeader', ['version', 'sender_id', 'message_type', 'flags', 'sequence', 'payload_length']
)


def pack_header(
    sender_id: int, message_type: int, payload: bytes, flags: int = 0, sequence: int = 0
) -> bytes:
    prefix = struct.pack(
        _HEADER_PREFIX_FMT, PROTOCOL_VERSION, sender_id, message_type, flags, sequence,
        len(payload),
    )
    crc = zlib.crc32(prefix + payload) & 0xFFFFFFFF
    return prefix + struct.pack('<I', crc) + payload


def unpack_header(data: bytes):
    """
    Split a raw frame into (SwarmHeader, payload).

    Raises ValueError on a truncated frame or CRC mismatch.
    """
    if len(data) < _HEADER_LEN:
        raise ValueError(f'frame too short for a swarm-protocol header: {len(data)} bytes')
    prefix = data[:_HEADER_PREFIX_LEN]
    version, sender_id, message_type, flags, sequence, payload_length = struct.unpack(
        _HEADER_PREFIX_FMT, prefix
    )
    (crc,) = struct.unpack('<I', data[_HEADER_PREFIX_LEN:_HEADER_LEN])
    payload = data[_HEADER_LEN:_HEADER_LEN + payload_length]
    if len(payload) != payload_length:
        raise ValueError(
            f'payload_length={payload_length} but only {len(payload)} bytes follow the header'
        )
    expected_crc = zlib.crc32(prefix + payload) & 0xFFFFFFFF
    if crc != expected_crc:
        raise ValueError(f'crc32 mismatch: header says 0x{crc:08x}, computed 0x{expected_crc:08x}')
    if version != PROTOCOL_VERSION:
        raise ValueError(f'unsupported protocol_version {version}')
    header = SwarmHeader(version, sender_id, message_type, flags, sequence, payload_length)
    return header, payload


def pack_heartbeat(**fields) -> bytes:
    values = [fields.get(name, 0) for name in _HEARTBEAT_FIELDS]
    return struct.pack(_HEARTBEAT_FMT, *values)


def unpack_heartbeat(payload: bytes) -> dict:
    values = struct.unpack(_HEARTBEAT_FMT, payload)
    return dict(zip(_HEARTBEAT_FIELDS, values))


def pack_peer_state(**fields) -> bytes:
    q = fields.get('q', (1.0, 0.0, 0.0, 0.0))
    values = [
        fields.get('timestamp_us', 0),
        fields.get('x', 0.0), fields.get('y', 0.0), fields.get('z', 0.0),
        fields.get('vx', 0.0), fields.get('vy', 0.0), fields.get('vz', 0.0),
        fields.get('heading', 0.0),
        q[0], q[1], q[2], q[3], 1 if fields.get('is_leader', False) else 0,
    ]
    return struct.pack(_PEER_STATE_FMT, *values)


def unpack_peer_state(payload: bytes) -> dict:
    values = struct.unpack(_PEER_STATE_FMT, payload)
    out = dict(zip(_PEER_STATE_FIELDS, values))
    out['q'] = (out.pop('q0'), out.pop('q1'), out.pop('q2'), out.pop('q3'))
    out['is_leader'] = bool(out['is_leader'])
    return out


def pack_peer_state_request() -> bytes:
    return b''
