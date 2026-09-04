#!/usr/bin/env python3
"""
mavlink_gcs_client -- hand-rolled GCS-side MAVLink driver for TEST-002's SITL scenarios.

Talks directly to a real `gcs_mavlink_node` instance's loopback socket
(ICD-MC-001 Channel B) -- the same "GCS" role `mock_gcs_node.py` plays for
its own simpler heartbeat/command tests, but this client additionally
speaks the real mission-upload protocol (`MISSION_COUNT`/
`MISSION_REQUEST_INT`/`MISSION_ITEM_INT`/`MISSION_ACK`) those tests never
needed, using `mavlink_lite`'s hand-packed encode/decode (same
cross-checked-against-pymavlink methodology, not a new toolchain
dependency). A plain function (not a `Node` mock like the other
`mc_test_utils` doubles), driven directly from a pytest scenario, matching
`test_mock_gcs_node.py`'s own "hand-roll the protocol in the test process"
precedent.
"""
import socket
import time

from mc_test_utils import mavlink_lite as ml
import yaml

# gcs_mavlink_node.cpp's own kCmd* mission-item command IDs (ICD-MC-001 §4.6).
CMD_INGRESS_WAYPOINT = 50200
CMD_MUSTER_POINT = 50201
CMD_FORMATION_OFFSET = 50202
CMD_SEARCH_AREA_VERTEX = 50203
CMD_SWARM_PEER_ENTRY = 50204

# gcs_mavlink_node.cpp's kMavMissionTypeMc / standard MAV_MISSION_ACCEPTED.
MAV_MISSION_TYPE_MC = 3
MAV_MISSION_ACCEPTED = 0

# mission_manager_node.cpp's kMcCmd* GcsCommand IDs (ICD-MC-001 §10.5).
MC_CMD_PHASE_TRANSITION = 50001
MC_CMD_ARM_AUTHORISE = 50002
MC_CMD_END_MISSION = 50003

# gcs_mavlink_node.hpp's own kComponentId (MAV_COMP_ID_ONBOARD_COMPUTER).
DEFAULT_TARGET_COMPONENT = 191

# Standard MAVLink common.xml constants for inject_battery_failure() below.
MAV_CMD_INJECT_FAILURE = 420
FAILURE_UNIT_SYSTEM_BATTERY = 100
FAILURE_TYPE_OFF = 1
MAV_PARAM_TYPE_INT32 = 6
PX4_ONBOARD_SYSTEM_ID = 1
PX4_ONBOARD_COMPONENT_ID = 1  # MAV_COMP_ID_AUTOPILOT1


def inject_battery_failure(px4_address, px4_port, gcs_system_id=255, gcs_component_id=190):
    """
    Force PX4 SITL's simulated battery to empty, bypassing fcc_bridge_node entirely.

    Talks directly to PX4's own onboard MAVLink instance (the "local
    14580/remote 14540" link this project's research confirmed exists
    alongside fcc_bridge_node's own 14540/14580 link -- both are real,
    independent PX4 MAVLink instances, so this cannot hijack
    MavlinkUdpBackend's learned peer address the way sending a spoofed
    frame at fcc_bridge_node's own socket would (see PROJECT-PLAN-5-MONTH.md's
    TEST-002 entry). Sequence, verified live against a real PX4 SITL
    instance during TEST-002's own research: PARAM_SET(SYS_FAILURE_EN=1)
    then COMMAND_LONG(MAV_CMD_INJECT_FAILURE, battery, OFF).
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        param_frame = ml.encode(
            ml.MSG_PARAM_SET, 0, gcs_system_id, gcs_component_id,
            param_value=1.0, target_system=PX4_ONBOARD_SYSTEM_ID,
            target_component=PX4_ONBOARD_COMPONENT_ID, param_id=b'SYS_FAILURE_EN',
            param_type=MAV_PARAM_TYPE_INT32)
        sock.sendto(param_frame, (px4_address, px4_port))
        time.sleep(0.2)  # let PX4 apply the param before the command references it

        command_frame = ml.encode(
            ml.MSG_COMMAND_LONG, 1, gcs_system_id, gcs_component_id,
            param1=float(FAILURE_UNIT_SYSTEM_BATTERY), param2=float(FAILURE_TYPE_OFF),
            param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0,
            command=MAV_CMD_INJECT_FAILURE, target_system=PX4_ONBOARD_SYSTEM_ID,
            target_component=PX4_ONBOARD_COMPONENT_ID, confirmation=0)
        sock.sendto(command_frame, (px4_address, px4_port))
    finally:
        sock.close()


def ingress_or_muster_item(command, lat_deg, lon_deg, alt_m):
    """Build one ingress-waypoint or muster-point mission item (ICD §4.6)."""
    return {
        'command': command,
        'x': round(lat_deg * 1e7),
        'y': round(lon_deg * 1e7),
        'z': float(alt_m),
    }


def formation_offset_item(drone_id, n_m, e_m, d_m):
    """Build one formation-offset mission item (ICD §4.6, N/E scaled x1000 mm)."""
    return {
        'command': CMD_FORMATION_OFFSET,
        'param1': float(drone_id),
        'x': round(n_m * 1000),
        'y': round(e_m * 1000),
        'z': float(d_m),
    }


def search_area_vertex_item(vertex_index, total_vertices, lat_deg, lon_deg):
    """Build one search-area-vertex mission item (ICD §4.6)."""
    return {
        'command': CMD_SEARCH_AREA_VERTEX,
        'param1': float(vertex_index),
        'param2': float(total_vertices),
        'x': round(lat_deg * 1e7),
        'y': round(lon_deg * 1e7),
    }


def swarm_peer_entry_item(drone_id, ip_octet3, ip_octet4):
    """Build one swarm-peer-manifest mission item (ICD §4.6/§11.1)."""
    # x/y are MISSION_ITEM_INT's real int32 fields (unlike z, a real float)
    # -- gcs_mavlink_node.cpp's decode_swarm_peer_entry_item() std::lround()s
    # all three, but struct.pack('i', ...) itself requires an actual int.
    return {
        'command': CMD_SWARM_PEER_ENTRY,
        'x': int(drone_id),
        'y': int(ip_octet3),
        'z': float(ip_octet4),
    }


def load_mission_items(fixture_path, num_drones=1):
    """
    Build a full mission-item list from a TP-MC-001 §3.4 fixture YAML file.

    Order matches gcs_mavlink_node's own flat MISSION_ITEM_INT seq stream
    expectations: ingress waypoint, muster point, search-polygon vertices,
    then `num_drones` formation offsets and swarm-manifest entries (see
    test_data/mission_upload_3drone.yaml's own comments for the ICD §4.6
    field mappings and why ingress/muster use near-(0,0) coordinates).
    """
    with open(fixture_path) as f:
        fixture = yaml.safe_load(f)

    items = [
        ingress_or_muster_item(CMD_INGRESS_WAYPOINT, **_deg_alt(fixture['ingress_waypoint'])),
        ingress_or_muster_item(CMD_MUSTER_POINT, **_deg_alt(fixture['muster_point'])),
    ]
    polygon = fixture['search_polygon']
    for index, vertex in enumerate(polygon):
        items.append(search_area_vertex_item(
            index, len(polygon), vertex['lat_deg'], vertex['lon_deg']))
    for offset in fixture['formation_offsets'][:num_drones]:
        items.append(formation_offset_item(
            offset['drone_id'], offset['n_m'], offset['e_m'], offset['d_m']))
    for peer in fixture['swarm_manifest'][:num_drones]:
        items.append(swarm_peer_entry_item(
            peer['drone_id'], peer['ip_octet3'], peer['ip_octet4']))
    return items


def _deg_alt(point):
    return {'lat_deg': point['lat_deg'], 'lon_deg': point['lon_deg'], 'alt_m': point['alt_m']}


class MavlinkGcsClient:
    """
    Drives gcs_mavlink_node's real MAVLink Channel B protocol from a test.

    One instance per SITL scenario; not thread-safe, matching the single
    sequential-handshake usage pattern every TP-ST scenario in this suite
    needs (no concurrent mission uploads).
    """

    def __init__(
        self, mc_address='127.0.0.1', mc_port=14560, gcs_system_id=255,
        gcs_component_id=190, target_system=1, target_component=DEFAULT_TARGET_COMPONENT,
    ):
        self._mc_addr = (mc_address, mc_port)
        self._sysid = gcs_system_id
        self._compid = gcs_component_id
        self._target_system = target_system
        self._target_component = target_component
        self._seq = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('0.0.0.0', 0))
        self._sock.settimeout(0.5)

    def close(self):
        self._sock.close()

    def _next_seq(self):
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def send_heartbeat(self):
        """Send one GCS HEARTBEAT -- gcs_mavlink_node learns/refreshes its reply target from it."""
        frame = ml.encode(
            ml.MSG_HEARTBEAT, self._next_seq(), self._sysid, self._compid,
            type=6, autopilot=8, base_mode=0, custom_mode=0, system_status=4,
            mavlink_version=3)
        self._sock.sendto(frame, self._mc_addr)

    def send_command(self, command, params=None):
        """Send one COMMAND_LONG, decoded by mission_manager_node into a GcsCommand."""
        values = list(params or [])
        values += [0.0] * (7 - len(values))
        frame = ml.encode(
            ml.MSG_COMMAND_LONG, self._next_seq(), self._sysid, self._compid,
            param1=values[0], param2=values[1], param3=values[2], param4=values[3],
            param5=values[4], param6=values[5], param7=values[6], command=command,
            target_system=self._target_system, target_component=self._target_component,
            confirmation=0)
        self._sock.sendto(frame, self._mc_addr)

    def arm_authorise(self, authorised=True):
        self.send_command(MC_CMD_ARM_AUTHORISE, [1.0 if authorised else 0.0])

    def request_phase_transition(self, phase):
        self.send_command(MC_CMD_PHASE_TRANSITION, [float(phase)])

    def end_mission(self):
        self.send_command(MC_CMD_END_MISSION)

    def upload_mission(self, items, timeout_s=5.0):
        """
        Run the full mission-upload handshake and return whether it was accepted.

        MISSION_COUNT -> (MISSION_REQUEST_INT -> MISSION_ITEM_INT)xN ->
        MISSION_ACK (SDD §11.2). `items` is a list of dicts shaped like
        this module's *_item() helpers' return values.
        """
        count_frame = ml.encode(
            ml.MSG_MISSION_COUNT, self._next_seq(), self._sysid, self._compid,
            count=len(items), target_system=self._target_system,
            target_component=self._target_component, mission_type=MAV_MISSION_TYPE_MC,
            opaque_id=0)  # 0 on upload to the vehicle -- see mavlink_lite.py's own note
        self._sock.sendto(count_frame, self._mc_addr)

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                data, _addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            decoded = ml.decode(data)
            if decoded is None:
                continue
            if decoded.msgid == ml.MSG_MISSION_REQUEST_INT:
                seq = decoded.fields['seq']
                if seq >= len(items):
                    continue
                item = items[seq]
                item_frame = ml.encode_with_fields(
                    ml.MSG_MISSION_ITEM_INT, self._next_seq(), self._sysid, self._compid, {
                        'param1': item.get('param1', 0.0),
                        'param2': item.get('param2', 0.0),
                        'param3': item.get('param3', 0.0),
                        'param4': item.get('param4', 0.0),
                        'x': item['x'],
                        'y': item['y'],
                        'z': item.get('z', 0.0),
                        'seq': seq,
                        'command': item['command'],
                        'target_system': self._target_system,
                        'target_component': self._target_component,
                        'frame': 5,  # MAV_FRAME_GLOBAL_INT
                        'current': 0,
                        'autocontinue': 1,
                        'mission_type': MAV_MISSION_TYPE_MC,
                    })
                self._sock.sendto(item_frame, self._mc_addr)
            elif decoded.msgid == ml.MSG_MISSION_ACK:
                return decoded.fields['type'] == MAV_MISSION_ACCEPTED
        return False
