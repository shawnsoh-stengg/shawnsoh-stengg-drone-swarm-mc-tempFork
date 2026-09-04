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

Messages implemented: HEARTBEAT (0), COMMAND_LONG (76), COMMAND_ACK (77),
and MC_STATUS (50100) for mock_gcs_node; MISSION_COUNT (44),
MISSION_REQUEST_INT (51), MISSION_ITEM_INT (73), and MISSION_ACK (47) for
TEST-002's mavlink_gcs_client (standard common.xml messages this time, not
mc_mavlink_defs.xml -- same cross-check methodology: field order/CRC_EXTRA
read from pymavlink's compiled common.xml dialect, then verified
byte-for-byte against pymavlink's own encoder). Add more _SPECS entries the
same way if a future mock needs them.
"""
from __future__ import annotations

from collections import namedtuple
import struct

MAVLINK_STX = 0xFD

MSG_HEARTBEAT = 0
MSG_MISSION_ACK = 47
MSG_MISSION_COUNT = 44
MSG_MISSION_ITEM_INT = 73
MSG_MISSION_REQUEST_INT = 51
MSG_PARAM_SET = 23
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
    MSG_PARAM_SET: {
        'crc_extra': 168,
        'fmt': '<fBB16sB',
        'wire_fields': (
            'param_value', 'target_system', 'target_component', 'param_id', 'param_type',
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
    # Same opaque_id v2 extension pattern as MISSION_ACK below -- 0 on
    # upload to the vehicle (this module's only direction), so its trailing
    # zero bytes get truncated on the wire either way, but decode() needs
    # the full field list to unpack a genuine 9-byte frame without erroring.
    MSG_MISSION_COUNT: {
        'crc_extra': 221,
        'fmt': '<HBBBI',
        'wire_fields': (
            'count', 'target_system', 'target_component', 'mission_type', 'opaque_id',
        ),
    },
    # NOTE: this payload's own field is named 'seq' (mission item sequence
    # number), which collides with encode()'s frame-sequence-number
    # parameter of the same name -- use encode_with_fields() for this and
    # MISSION_ITEM_INT, not encode().
    MSG_MISSION_REQUEST_INT: {
        'crc_extra': 196,
        'fmt': '<HBBB',
        'wire_fields': ('seq', 'target_system', 'target_component', 'mission_type'),
    },
    MSG_MISSION_ITEM_INT: {
        'crc_extra': 38,
        'fmt': '<ffffiifHHBBBBBB',
        'wire_fields': (
            'param1', 'param2', 'param3', 'param4', 'x', 'y', 'z', 'seq', 'command',
            'target_system', 'target_component', 'frame', 'current', 'autocontinue',
            'mission_type',
        ),
    },
    # MISSION_ACK carries a 5th field, 'opaque_id' (uint32, the mission
    # upload_id echoed back on ACCEPT -- gcs_mavlink_node.cpp's
    # send_mission_ack() passes mission_upload_msg_.upload_id here), added
    # to common.xml as a MAVLink v2 *extension* field after this message's
    # original 4-field definition. Extension fields are always appended
    # after the size-sorted base fields in declaration order, never
    # participating in the sort themselves -- confirmed against this
    # environment's actual generated mavlink_msg_mission_ack.h (not
    # pymavlink's bundled common.xml, which is older and has no opaque_id
    # at all); CRC_EXTRA is unaffected by extension fields per the MAVLink
    # spec (computed from base fields only), matches the pre-extension value.
    MSG_MISSION_ACK: {
        'crc_extra': 153,
        'fmt': '<BBBBI',
        'wire_fields': (
            'target_system', 'target_component', 'type', 'mission_type', 'opaque_id',
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


def _encode_frame(msgid: int, seq: int, sysid: int, compid: int, fields: dict) -> bytes:
    spec = _SPECS[msgid]
    values = [fields[name] for name in spec['wire_fields']]
    payload = bytearray(struct.pack(spec['fmt'], *values))
    while payload and payload[-1] == 0:  # MAVLink v2 trailing-zero-byte truncation
        payload.pop()
    msgid_bytes = msgid.to_bytes(3, 'little')
    header = bytes([len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF]) + msgid_bytes
    crc = _crc16_x25(header + bytes(payload) + bytes([spec['crc_extra']]))
    return bytes([MAVLINK_STX]) + header + bytes(payload) + struct.pack('<H', crc)


def encode(msgid: int, seq: int, sysid: int, compid: int, **fields) -> bytes:
    """
    Pack one MAVLink v2 frame.

    Keyword-only; argument order doesn't matter but names must match a
    _SPECS wire_fields entry, e.g. encode(MSG_HEARTBEAT, seq, sysid, compid,
    type=6, autopilot=8, base_mode=0, custom_mode=0, system_status=4,
    mavlink_version=3).
    """
    return _encode_frame(msgid, seq, sysid, compid, fields)


def encode_with_fields(msgid: int, seq: int, sysid: int, compid: int, fields: dict) -> bytes:
    """
    Pack one MAVLink v2 frame, like encode(), from a plain dict of fields.

    Needed for messages whose own payload has a field literally named
    'seq' (MISSION_REQUEST_INT, MISSION_ITEM_INT) -- passing that as a
    keyword argument to encode() would collide with its own frame
    sequence-number parameter of the same name.
    """
    return _encode_frame(msgid, seq, sysid, compid, fields)


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
    if len(payload) < full_len:
        # MAVLink v2 trailing-zero-byte truncation: a shorter payload means
        # the sender's trailing fields (in our wire order) were all zero.
        padded_payload = payload + b'\x00' * (full_len - len(payload))
    else:
        # Sender's real dialect has trailing extension fields this spec
        # doesn't model (e.g. GPS_RAW_INT's yaw/hdg_acc/...) -- ignore them
        # rather than erroring; our spec only needs the fields it declares.
        padded_payload = payload[:full_len]
    values = struct.unpack(spec['fmt'], padded_payload)
    fields = dict(zip(spec['wire_fields'], values))
    return DecodedFrame(msgid, sysid, compid, seq, fields)
