"""Channel B/C wire constants (TEST-007).

Every value here is copied from an authoritative source, cited inline, rather
than invented for the panel -- docs/GCS-MAVLINK-API-REFERENCE.md,
ICD-MC-001.md Sec10.6-Sec10.9, ws/src/gcs_comms/mc_mavlink_defs.xml and
ws/src/gcs_comms/src/gcs_mavlink_node.cpp. If one of those changes, change it
here too: nothing regenerates this file.
"""

from __future__ import annotations

# --- Ports (config/mavlink-router-channel-{b,c}.conf, docs/GCS-MAVLINK-API-REFERENCE.md) ---
# Each channel needs two sockets: mavlink-router's GCS-facing endpoint sends
# MC->GCS traffic to a FIXED address:port, not back to whatever ephemeral
# port a single udpout socket used (tools/ft001_ac2_channel_b's docstring).
CH_B_RECV_PORT = 14551  # GCS_CHANNEL_B -- the GCS's own receive port
CH_B_SEND_PORT = 14552  # GCS_MC_RECV -- the MC's own receive port
CH_C_RECV_PORT = 14571  # ORCHESTRATOR_CHANNEL_C
CH_C_SEND_PORT = 14572  # ORCHESTRATOR_MC_RECV
QGC_CHANNEL_A_PORT = 14550  # never bound by this panel -- QGC owns it

DEFAULT_MC_HOST = "192.168.100.11"  # MC_ETH_IP (CLAUDE.md network table)
DEFAULT_DRONE_ID = 1

# --- Identities ---
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = 0  # Channel B operator (tools/ft001_ac2_channel_b)
ORCHESTRATOR_COMPONENT_ID = 190  # MAV_COMP_ID_MISSIONPLANNER, Channel C
MC_COMPONENT_ID = 191  # MAV_COMP_ID_ONBOARD_COMPUTER, both channels

# --- Channel B heartbeat (ICD Sec4.3; gcs_mavlink_node.cpp handle_heartbeat()) ---
# gcs_mavlink_node ignores any heartbeat that isn't exactly type=6/autopilot=8.
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_TYPE_ONBOARD_CONTROLLER = 18  # what the MC heartbeats as
GCS_LINK_TIMEOUT_S = 10.0  # gcs_link_timeout_s default (FR-073/SFR-007)
# Grading window for the heartbeat-off test: GCS_LINK_LOST should appear no
# earlier than the timeout and within this much after it -- gcs_mavlink_node
# checks link age on a 1 Hz timer, and the fault then has to reach MC_STATUS
# (2 Hz) via health_monitor_node. A panel-side choice, not a requirement value.
GCS_LINK_LOST_TOLERANCE_S = 3.0

# --- Timing bounds the panel grades against ---
COMMAND_ACK_BOUND_S = 0.5  # FR-061/FR-003
MC_STATUS_MIN_HZ = 2.0  # FR-062
MC_HEALTH_MIN_HZ = 1.0  # FR-071
MC_HEARTBEAT_STALE_S = 3.0  # lamp goes red after 3 missed 1 Hz MC heartbeats

# --- COMMAND_LONG ids (docs/GCS-MAVLINK-API-REFERENCE.md "COMMAND_LONG handling") ---
MC_CMD_PHASE_TRANSITION = 50001
MC_CMD_ARM_AUTHORISE = 50002
MC_CMD_END_MISSION = 50003
MC_CMD_GCS_OVERRIDE = 50004
MC_CMD_INDOOR_GNSS_SIM_ANCHOR = 50005  # gnss_sim_node (IGS-004/FT-006)
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400

COMMAND_NAMES = {
    MC_CMD_PHASE_TRANSITION: "MC_CMD_PHASE_TRANSITION",
    MC_CMD_ARM_AUTHORISE: "MC_CMD_ARM_AUTHORISE",
    MC_CMD_END_MISSION: "MC_CMD_END_MISSION",
    MC_CMD_GCS_OVERRIDE: "MC_CMD_GCS_OVERRIDE",
    MC_CMD_INDOOR_GNSS_SIM_ANCHOR: "MC_CMD_INDOOR_GNSS_SIM_ANCHOR",
    MAV_CMD_NAV_RETURN_TO_LAUNCH: "MAV_CMD_NAV_RETURN_TO_LAUNCH",
    MAV_CMD_DO_SET_MODE: "MAV_CMD_DO_SET_MODE",
    MAV_CMD_COMPONENT_ARM_DISARM: "MAV_CMD_COMPONENT_ARM_DISARM",
}

# PX4 custom_main modes that make MAV_CMD_DO_SET_MODE an EMERGENCY-tier
# command (param2 = custom_main; PX4 v1.17 px4_custom_mode.h).
PX4_MANUAL_MODES = {
    "MANUAL": 1,
    "ALTCTL": 2,
    "POSCTL": 3,
    "STABILIZED": 7,
}
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}
MAV_RESULT_IN_PROGRESS = 5

# --- Enums (ICD Sec10.6/Sec10.7, mc_mavlink_defs.xml) ---
MISSION_PHASES = {
    0: "PREFLIGHT",
    1: "ARMING",
    2: "TAKEOFF",
    3: "INGRESS",
    4: "MUSTER",
    5: "ISR_SELECTION",
    6: "VORONOI_PARTITION",
    7: "SEARCH",
    8: "RETURN",
    9: "LAND",
    255: "FAULT",
}

# mission_manager_node's transition table (mission_manager_node.cpp
# build_transition_table(), SDD Sec4.1): from each phase exactly one forward
# row exists -- a MC_CMD_PHASE_TRANSITION to anything else is rejected
# (FR-002). FAULT has no rows at all (terminal until restart, EXEC-006).
PHASE_NEXT = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5, 5: 6, 6: 7, 7: 8, 8: 9, 9: 0}

# What that one row's guard needs, for the operator hint. Most rows are also
# evaluated autonomously every control tick, so the GCS command is often not
# what moves the phase -- the guard becoming true is.
PHASE_GUARD_HINTS = {
    0: "→ ARMING needs MC_CMD_ARM_AUTHORISE (sent AFTER the plan upload/activate — "
       "an upload resets it) and all 7 pre-arm checks passing. The MC then enters "
       "ARMING and ARMS THE FCC by itself — no phase command needed.",
    1: "→ TAKEOFF happens by itself once the FCC reports armed.",
    2: "→ INGRESS needs this GCS command AND the drone within tolerance of the takeoff altitude.",
    255: "FAULT is terminal: every transition is rejected until mission-exec restarts.",
}

# mission_exec/pre_arm_checks.cpp all_pre_arm_checks() -- failures are
# published on /mc/status_text, which gcs_mavlink_node relays as STATUSTEXT
# (Monitor tab's event log); a rejected PHASE_TRANSITION(ARMING) also names
# the first failing check in its DENIED reason.
PRE_ARM_CHECKS = (
    "FCC link alive (vehicle_state.fcc_link_ok)",
    "FCC disarmed",
    "GCS ARM authorisation set",
    "battery ≥ low-battery threshold + margin",
    "every live swarm peer in PREFLIGHT/ARMING",
    "mission upload complete: waypoints, formation offsets, search polygon, swarm manifest all non-empty",
    "GNSS ok, or visual localisation alive",
)

# How long to watch MC_STATUS for a command's effect: MC_STATUS is 2 Hz and
# mission_manager_node drains commands on its control timer.
EFFECT_WINDOW_S = 3.0

OVERRIDE_TYPES = {
    0: "OVERRIDE_NONE",
    1: "OVERRIDE_LOW_BATTERY_RTL",
    2: "OVERRIDE_GCS_LINK_HOLD",
}

LOCALISATION_MODES = {0: "GNSS", 1: "VISUAL_ODOMETRY"}

# ICD Sec10.8
FAULT_BITS = {
    0: "FCC_LINK_LOST",
    1: "GCS_LINK_LOST",
    2: "MANAGER_DEAD",
    3: "LOW_BATTERY",
    4: "PEER_LOST",
    5: "GPU_OVERTEMP",
    6: "DISK_FULL",
    7: "CAMERA_FAIL",
    8: "GNSS_DEGRADED",
    9: "VISUAL_ODOM_FAIL",
    10: "GIMBAL_LINK_LOST",
    11: "OFFBOARD_STREAM_STALE",
    12: "VISION_MEM_HIGH",
    13: "INDOOR_GNSS_SIM_LOST",
}

# ICD Sec10.9
NODE_ALIVE_BITS = {
    0: "fcc_bridge_node",
    1: "gcs_mavlink_node",
    2: "swarm_coordinator_node",
    3: "formation_controller_node",
    4: "mission_manager_node",
    5: "search_planner_node",
    6: "visual_localisation_node",
    7: "camera_node",
    8: "perception_down_node",
    9: "health_monitor_node",
    10: "perception_front_node",
    11: "video_mux_node",
}

# mc_mavlink_defs.xml MC_MISSION_PLAN_ACK.result
PLAN_ACK_RESULTS = {
    0: "ACCEPTED",
    1: "REJECTED_INVALID_JSON",
    2: "REJECTED_NOT_FOUND",
    3: "REJECTED_PHASE",
}

# mc_msgs/MissionUpload.skip_phase_mask -- the only flyable stage sets.
SKIP_SWARM_PHASES = 0x70  # MUSTER | ISR_SELECTION | VORONOI_PARTITION
SKIP_SEARCH = 0x80
SKIP_PHASE_MASKS = {
    0: "full swarm mission",
    SKIP_SWARM_PHASES: "solo: ingress -> search -> return",
    SKIP_SWARM_PHASES | SKIP_SEARCH: "transit: ingress -> return",
}

# --- Channel B mission upload (gcs_mavlink_node.cpp, docs/GCS-MAVLINK-API-REFERENCE.md) ---
MAV_MISSION_TYPE_MC = 3  # kMavMissionTypeMc -- not in common.xml's enum
CMD_INGRESS_WAYPOINT = 50200
CMD_MUSTER_POINT = 50201
CMD_FORMATION_OFFSET = 50202
CMD_SEARCH_AREA_VERTEX = 50203
CMD_SWARM_PEER_ENTRY = 50204
CMD_MISSION_CONFIG = 50205  # param1 = skip_phase_mask
MISSION_ITEM_NAMES = {
    CMD_INGRESS_WAYPOINT: "ingress waypoint",
    CMD_MUSTER_POINT: "muster point",
    CMD_FORMATION_OFFSET: "formation offset",
    CMD_SEARCH_AREA_VERTEX: "search-area vertex",
    CMD_SWARM_PEER_ENTRY: "swarm peer",
    CMD_MISSION_CONFIG: "mission config",
}
MAV_MISSION_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "ERROR",
    2: "UNSUPPORTED_FRAME",
    3: "UNSUPPORTED",
    4: "NO_SPACE",
    5: "INVALID",
    13: "INVALID_SEQUENCE",
    14: "DENIED",
    15: "OPERATION_CANCELLED",
}
# The MC re-requests an item after 1 s of silence and aborts after 5 s
# (mission_request_retry_s / mission_upload_timeout_s defaults).
MISSION_UPLOAD_SILENCE_TIMEOUT_S = 6.0

MAV_FRAME_GLOBAL_INT = 5
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_MISSION = 2


def decode_bits(mask: int, names: dict[int, str]) -> list[str]:
    return [names.get(bit, f"BIT{bit}") for bit in range(16) if mask & (1 << bit)]


def phase_name(value: int) -> str:
    return MISSION_PHASES.get(value, f"UNKNOWN({value})")
