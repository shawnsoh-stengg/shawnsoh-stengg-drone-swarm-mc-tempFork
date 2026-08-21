"""
Hand-packed MAVLink v2 subset for mock_gcs_node (ICD-MC-001 §4, IFD-002).

Follows the same toolchain decision gcs_mavlink_node.cpp/mc_mavlink_defs.xml
already made for this project: no pymavlink/mavgen build dependency, just
hand-packed frames using constants derived from an *offline* mavgen run (not
computed by hand). Concretely, for this module: mavgen was run once against
mc_mavlink_defs.xml (+ common.xml for the two standard messages) to read off
each message's post-field-sort wire order, struct format and CRC_EXTRA, then
every value below was cross-checked byte-for-byte against pymavlink's own
encoder before being committed. If mc_mavlink_defs.xml's field list for
MC_STATUS ever changes, MC_STATUS's entry in _SPECS below must be re-derived
the same way -- it will not update itself.

Only the messages mock_gcs_node actually speaks are implemented: HEARTBEAT
(0), COMMAND_LONG (76), COMMAND_ACK (77), and MC_STATUS (50100). Add more
_SPECS entries the same way (offline mavgen run against mc_mavlink_defs.xml)
if a future mock needs them.
"""
from __future__ import annotations

from collections import namedtuple
import struct

MAVLINK_STX = 0xFD

MSG_HEARTBEAT = 0
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
MSG_MC_STATUS = 50100

# Each entry: crc_extra (mavgen-derived), struct format string and field
# names IN WIRE ORDER (post field-size-descending sort, stable on ties) --
# NOT the same order as encode()'s keyword arguments, which follow the
# message's *declared* (mc_mavlink_defs.xml / common.xml) field order.
_SPECS = {
    MSG_HEARTBEAT: {
        'crc_extra': 50,
        'fmt': '<IBBBBB',
        'wire_fields': (
            'custom_mode', 'type', 'autopilot', 'base_mode', 'system_status', 'mavlink_version',
        ),
    },
    MSG_COMMAND_LONG: {
        'crc_extra': 152,
        'fmt': '<fffffffHBBB',
        'wire_fields': (
            'param1', 'param2', 'param3', 'param4', 'param5', 'param6', 'param7',
            'command', 'target_system', 'target_component', 'confirmation',
        ),
    },
    MSG_COMMAND_ACK: {
        'crc_extra': 143,
        'fmt': '<HBBiBB',
        'wire_fields': (
            'command', 'result', 'progress', 'result_param2', 'target_system', 'target_component',
        ),
    },
    MSG_MC_STATUS: {
        'crc_extra': 24,
        'fmt': '<QiiiHHhHBBBBB',
        'wire_fields': (
            'time_usec', 'lat', 'lon', 'alt_msl', 'heading', 'groundspeed', 'climb_rate',
            'fault_mask', 'battery_remaining', 'mission_phase', 'localisation_mode',
            'peer_count', 'isr_role',
        ),
    },
}

DecodedFrame = namedtuple('DecodedFrame', ['msgid', 'sysid', 'compid', 'seq', 'fields'])


def _crc16_x25(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ ((tmp << 4) & 0xFF)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def encode(msgid: int, seq: int, sysid: int, compid: int, **fields) -> bytes:
    """
    Pack one MAVLink v2 frame.

    Keyword-only; argument order doesn't matter but names must match a
    _SPECS wire_fields entry, e.g. encode(MSG_HEARTBEAT, seq, sysid, compid,
    type=6, autopilot=8, base_mode=0, custom_mode=0, system_status=4,
    mavlink_version=3).
    """
    spec = _SPECS[msgid]
    values = [fields[name] for name in spec['wire_fields']]
    payload = bytearray(struct.pack(spec['fmt'], *values))
    while payload and payload[-1] == 0:  # MAVLink v2 trailing-zero-byte truncation
        payload.pop()
    msgid_bytes = msgid.to_bytes(3, 'little')
    header = bytes([len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF]) + msgid_bytes
    crc = _crc16_x25(header + bytes(payload) + bytes([spec['crc_extra']]))
    return bytes([MAVLINK_STX]) + header + bytes(payload) + struct.pack('<H', crc)


def decode(data: bytes):
    """
    Decode one MAVLink v2 frame.

    Return a DecodedFrame, or None if the frame is truncated, uses a msgid
    this module doesn't know (so CRC_EXTRA can't be checked), or fails its
    checksum.
    """
    if len(data) < 12 or data[0] != MAVLINK_STX:
        return None
    length = data[1]
    seq = data[4]
    sysid = data[5]
    compid = data[6]
    msgid = int.from_bytes(data[7:10], 'little')
    if len(data) < 10 + length + 2:
        return None
    payload = data[10:10 + length]
    (crc_recv,) = struct.unpack('<H', data[10 + length:10 + length + 2])
    spec = _SPECS.get(msgid)
    if spec is None:
        return None
    header_for_crc = data[1:10]
    expected_crc = _crc16_x25(header_for_crc + payload + bytes([spec['crc_extra']]))
    if expected_crc != crc_recv:
        return None
    full_len = struct.calcsize(spec['fmt'])
    padded_payload = payload + b'\x00' * (full_len - len(payload))
    values = struct.unpack(spec['fmt'], padded_payload)
    fields = dict(zip(spec['wire_fields'], values))
    return DecodedFrame(msgid, sysid, compid, seq, fields)
