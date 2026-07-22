# Interface Control Document
## Swarm Search Drone — Mission Computer (AI MC)

| Field | Value |
|---|---|
| Document | MC-ICD-001 |
| Version | 0.1-DRAFT |
| Date | 2026-06-20 |
| Parent SRS | MC-SRS-001 v0.1-DRAFT |
| Platform | Jetson Orin NX 16GB · JetPack 6.2.2 · Ubuntu 22.04 aarch64 |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Interface Summary](#2-interface-summary)
3. [IFD-001: MC ↔ FCC (uXRCE-DDS + Mission Protocol)](#3-ifd-001-mc--fcc-uxrce-dds--mission-protocol)
4. [IFD-002: MC ↔ GCS (MAVLink 2/UDP)](#4-ifd-002-mc--gcs-mavlink-2udp)
5. [IFD-003: MC ↔ Swarm Peers (Swarm Protocol/UDP)](#5-ifd-003-mc--swarm-peers-swarm-protocoludp)
6. [IFD-004: MC ↔ Downward Camera (CSI / ROS 2)](#6-ifd-004-mc--downward-camera-csi--ros-2)
7. [IFD-006: MC ↔ Gimbal Camera (Ethernet / RTSP H.265)](#7-ifd-006-mc--gimbal-camera-ethernet--rtsp-h265)
8. [IFD-007: MC ↔ GCS Video Uplink (RTSP H.265 + Metadata)](#8-ifd-007-mc--gcs-video-uplink-rtsp-h265--metadata)
9. [IFD-005: Internal ROS 2 Topics](#9-ifd-005-internal-ros-2-topics)
10. [Enumerations and Constants](#10-enumerations-and-constants)
11. [Configuration Reference](#11-configuration-reference)

---

## 1. Introduction

### 1.1 Purpose

This document defines the data formats, message structures, transport parameters, timing constraints, and error handling for all interfaces involving the Mission Computer (MC). It is the primary reference for software developers implementing or integrating MC nodes.

### 1.2 Scope

Seven interface domains are defined:

| ID | Interface | Transport |
|---|---|---|
| IFD-001 | MC ↔ FCC | uXRCE-DDS (serial / ethernet) + MAVLink mission protocol (UDP :14555, always ethernet) |
| IFD-002 | MC ↔ GCS | MAVLink 2/UDP via mavlink-router |
| IFD-003 | MC ↔ Swarm Peers | Custom binary protocol / UDP / MANET |
| IFD-004 | MC ↔ Downward Camera | CSI → gscam2 → ROS 2 |
| IFD-006 | MC ↔ Gimbal Camera | Ethernet (HIF-007) → RTSP/H.265 → `perception_front_node` |
| IFD-007 | MC ↔ GCS Video Uplink | RTSP/H.265 + metadata (independent of IFD-002) |
| IFD-005 | Internal (MC-internal) | ROS 2 topics / services (CycloneDDS, same-host only). Cross-host DDS, if/when needed, rides `zenoh-bridge-dds` over a static-peer Zenoh mesh instead — see §11.4a. |

### 1.3 Conventions

- All multi-byte fields are **little-endian** unless stated otherwise.
- All timestamps are **UTC microseconds since Unix epoch** (`uint64_t`) unless stated otherwise.
- All angles are **radians** unless stated otherwise.
- All positions are **metres** and coordinate frames are stated per field.
- `[N]` denotes a fixed-length array of N elements.
- ROS 2 message package prefix `mc_msgs` refers to the custom message package in the shared colcon workspace.

---

## 2. Interface Summary

```
┌───────────────────────────────────────────────────────────────────┐
│                    Mission Computer (MC)                          │
│                    192.168.10.11                                   │
│                                                                   │
│  IFD-004        IFD-001               IFD-002          IFD-003   │
│  Camera ──CSI──► gscam2   FCC ──DDS──► fcc_bridge   GCS ──MAV──► │
│  /camera/         node   ttyTHS1 /    _node        mavlink-      │
│  image_raw               192.168.12.x              router        │
│                                                                   │
│  IFD-006                              IFD-007                    │
│  Gimbal camera ──eth0(switch)──► perception_ video_mux ─RTSP/H.265─► │
│  192.168.10.200                    front_node        + metadata      │
│                                        (:8555)          to GCS        │
│                                                                   │
│  IFD-005: Internal ROS 2 topics (/mc/*)                           │
│  All nodes communicate via CycloneDDS on the shared DDS domain    │
└───────────────────────────────────────────────────────────────────┘
```

---

## 3. IFD-001: MC ↔ FCC (uXRCE-DDS + Mission Protocol)

**SRS refs:** IR-001 to IR-007

### 3.1 Transport

| Parameter | Primary (Serial) | Secondary (Ethernet) |
|---|---|---|
| Protocol | uXRCE-DDS over UART | uXRCE-DDS over UDP |
| Device / Address | `/dev/ttyTHS1` | `192.168.12.31` (FCC) |
| Baud rate | 921600 | — |
| UDP port (agent) | — | 8888 |
| DDS domain ID | 0 | 0 |
| Agent binary | `MicroXRCEAgent serial ...` | `MicroXRCEAgent udp4 ...` |
| Selection | Deployment config param `FCC_TRANSPORT` | |

The Micro XRCE-DDS Agent runs in the `fcc-bridge` container and bridges between the serial/UDP transport and the DDS domain. All PX4 uORB topics are exposed as ROS 2-compatible DDS topics under the `/fmu/` namespace.

**Mission protocol sub-channel (always Ethernet, independent of `FCC_TRANSPORT`):**

| Parameter | Value |
|---|---|
| Protocol | MAVLink 2 over UDP (standard mission microservice) |
| Address | `192.168.12.31:14555` (FCC) |
| Carried over | The existing HIF-002 Ethernet link (already MAVLink-capable; no new hardware) |
| Purpose | `upload_mission()` / `update_mission_item()` only — see §3.3 `Mission Protocol` below |

Stock PX4 has no DDS-writable mission topic — mission items live in FCC-side dataman, reachable only through the standard MAVLink mission protocol. This sub-channel is therefore always active on 192.168.12.x regardless of whether `FCC_TRANSPORT` currently routes state/commands over serial or Ethernet (IR-007). It is distinct from the uXRCE-DDS Agent port (`:8888`) and the GCS↔FCC Channel A port (`:14550`, §11.2), and it is bidirectional MC↔FCC only — the MC does not use Channel A (§4).

**`mavlink-router` is not involved in IFD-001** — the mission-protocol socket is owned directly by `XrceDdsBackend`/`MavlinkUdpBackend`, not routed.

### 3.2 Topics Subscribed by MC (FCC → MC)

All topics are in the `/fmu/out/` namespace, published by the PX4 Micro XRCE-DDS client.

---

#### `vehicle_local_position`

Rate: ≥ 10 Hz (configured in PX4 `uxrce_dds_client` parameters)

| Field | Type | Unit | Description |
|---|---|---|---|
| `timestamp` | `uint64` | µs | PX4 monotonic timestamp |
| `timestamp_sample` | `uint64` | µs | Sample timestamp |
| `x` | `float32` | m | North position (NED) |
| `y` | `float32` | m | East position (NED) |
| `z` | `float32` | m | Down position (NED) |
| `vx` | `float32` | m/s | North velocity |
| `vy` | `float32` | m/s | East velocity |
| `vz` | `float32` | m/s | Down velocity |
| `heading` | `float32` | rad | Vehicle heading from North; NaN if unknown |
| `xy_valid` | `bool` | — | Horizontal position valid |
| `z_valid` | `bool` | — | Vertical position valid |
| `v_xy_valid` | `bool` | — | Horizontal velocity valid |
| `v_z_valid` | `bool` | — | Vertical velocity valid |
| `dist_bottom` | `float32` | m | Distance to ground (rangefinder); NaN if unavailable |

---

#### `vehicle_global_position`

Rate: ≥ 5 Hz

| Field | Type | Unit | Description |
|---|---|---|---|
| `timestamp` | `uint64` | µs | PX4 monotonic timestamp |
| `lat` | `float64` | deg | Latitude (WGS-84) |
| `lon` | `float64` | deg | Longitude (WGS-84) |
| `alt` | `float32` | m | Altitude MSL |
| `alt_ellipsoid` | `float32` | m | Altitude above WGS-84 ellipsoid |
| `eph` | `float32` | m | Standard deviation horizontal position error |
| `epv` | `float32` | m | Standard deviation vertical position error |

---

#### `vehicle_status`

Rate: ≥ 5 Hz

| Field | Type | Values | Description |
|---|---|---|---|
| `timestamp` | `uint64` | µs | PX4 monotonic timestamp |
| `nav_state` | `uint8` | See §10.1 | Current navigation state |
| `arming_state` | `uint8` | See §10.2 | Arming state |
| `vehicle_type` | `uint8` | 1=fixed-wing, 2=rotary | Vehicle type |
| `failure_detector_status` | `uint16` | bitmask | Failure detector flags |
| `system_id` | `uint8` | — | MAVLink system ID of this FCC |
| `component_id` | `uint8` | — | MAVLink component ID |
| `is_vtol` | `bool` | — | Is VTOL |

---

#### `battery_status`

Rate: ≥ 2 Hz

| Field | Type | Unit | Description |
|---|---|---|---|
| `timestamp` | `uint64` | µs | PX4 monotonic timestamp |
| `voltage_v` | `float32` | V | Battery voltage |
| `current_a` | `float32` | A | Battery current draw; -1 if unknown |
| `remaining` | `float32` | 0.0–1.0 | State of charge; -1 if unknown |
| `warning` | `uint8` | See §10.3 | Battery warning level |
| `cell_count` | `uint8` | — | Number of cells |
| `connected` | `bool` | — | Battery connected |

---

#### `vehicle_attitude`

Rate: ≥ 50 Hz (used by visual localisation node for IMU fusion)

| Field | Type | Unit | Description |
|---|---|---|---|
| `timestamp` | `uint64` | µs | PX4 monotonic timestamp |
| `timestamp_sample` | `uint64` | µs | Sample timestamp |
| `q` | `float32[4]` | — | Attitude quaternion `[w, x, y, z]` (NED → body) |
| `delta_q_reset` | `float32[4]` | — | Quaternion delta since last reset |
| `quat_reset_counter` | `uint8` | — | Reset counter |

---

#### `vehicle_odometry`

Rate: ≥ 20 Hz (used by visual localisation for IMU integration)

| Field | Type | Unit | Description |
|---|---|---|---|
| `timestamp` | `uint64` | µs | PX4 monotonic timestamp |
| `timestamp_sample` | `uint64` | µs | Sample timestamp |
| `pose_frame` | `uint8` | See §10.4 | Reference frame of position/orientation |
| `position` | `float32[3]` | m | Position `[x, y, z]` in `pose_frame` |
| `q` | `float32[4]` | — | Attitude quaternion `[w, x, y, z]` |
| `velocity_frame` | `uint8` | See §10.4 | Reference frame of velocity |
| `velocity` | `float32[3]` | m/s | Velocity `[vx, vy, vz]` in `velocity_frame` |
| `angular_velocity` | `float32[3]` | rad/s | Angular velocity `[roll, pitch, yaw]` in body frame |
| `position_variance` | `float32[3]` | m² | Position variance |
| `orientation_variance` | `float32[3]` | rad² | Orientation variance |
| `velocity_variance` | `float32[3]` | m²/s² | Velocity variance |

---

#### `sensor_combined`

Rate: ≥ 100 Hz

| Field | Type | Unit | Description |
|---|---|---|---|
| `timestamp` | `uint64` | µs | PX4 monotonic timestamp |
| `gyro_rad` | `float32[3]` | rad/s | Angular rate `[x, y, z]` in body frame |
| `gyro_integral_dt` | `uint32` | µs | Integration interval for `gyro_rad` |
| `accelerometer_timestamp_relative` | `int32` | µs | Accelerometer sample age relative to `timestamp` |
| `accelerometer_m_s2` | `float32[3]` | m/s² | Acceleration `[x, y, z]` in body frame |
| `accelerometer_integral_dt` | `uint32` | µs | Integration interval for `accelerometer_m_s2` |
| `accelerometer_clipping` | `uint8` | bitmask | Clipping per axis (bit 0=X, 1=Y, 2=Z) |
| `gyro_clipping` | `uint8` | bitmask | Gyro clipping per axis |

---

### 3.3 Topics Published by MC (MC → FCC)

`vehicle_command` and `vehicle_visual_odometry` are in the `/fmu/in/` namespace, consumed by the PX4 Micro XRCE-DDS client. Mission item transport (`upload_mission()` / `update_mission_item()`) does **not** use `/fmu/in/` — it uses the MAVLink mission protocol over the dedicated `192.168.12.31:14555` socket described in §3.1, because stock PX4 has no DDS-writable mission topic.

---

#### Mission Protocol (MAVLink 2/UDP `192.168.12.31:14555`)

Driven by: `fcc_bridge_node`'s `FccBackend` implementation, on behalf of `search_planner_node`/`mission_manager_node` (full upload) or `formation_controller_node` (single-item replace). Neither exchange has a PX4-imposed minimum rate — a gap does not cause a mode change (contrast the old OFFBOARD `trajectory_setpoint`, which required ≥ 2 Hz to avoid a PX4-initiated mode exit).

**`MissionItem` fields** (payload of each `MISSION_ITEM_INT`):

| Field | Type | Unit | Description |
|---|---|---|---|
| `seq` | `uint16` | — | Item index; `0` is reserved as `FORMATION_SLOT` during INGRESS (SDD §5) |
| `frame` | `uint8` | — | `MAV_FRAME_GLOBAL_INT` |
| `command` | `uint16` | — | `MAV_CMD_NAV_WAYPOINT` (16); `MAV_CMD_NAV_LOITER_UNLIM` (17) for terminal items |
| `lat`, `lon` | `int32` | 1e7 deg | Target position |
| `alt` | `float32` | m MSL | Target altitude |
| `acceptance_radius` | `float32` | m | See SDD §5.2 for the widened radius used on `FORMATION_SLOT` |

**`upload_mission(items)` — full handshake** (`search_planner_node` via `mission_manager_node`, FR-013; infrequent, on mission change only):

```
MC  → FCC : MISSION_COUNT(count = items.size())
FCC → MC  : MISSION_REQUEST_INT(seq = 0)
MC  → FCC : MISSION_ITEM_INT(seq = 0, ...)
              ⋮  (repeats for each seq)
FCC → MC  : MISSION_REQUEST_INT(seq = count-1)
MC  → FCC : MISSION_ITEM_INT(seq = count-1, ...)
FCC → MC  : MISSION_ACK(MAV_MISSION_ACCEPTED)
```

**`update_mission_item(seq, item)` — partial update microservice** (`formation_controller_node`, FR-020/FR-023; ≥ 5 Hz on `FORMATION_SLOT` during INGRESS — a full handshake at this rate would be far too heavyweight):

```
MC  → FCC : MISSION_WRITE_PARTIAL_LIST(start = seq, end = seq)
FCC → MC  : MISSION_REQUEST_INT(seq)
MC  → FCC : MISSION_ITEM_INT(seq, ...)
FCC → MC  : MISSION_ACK(MAV_MISSION_ACCEPTED)
```

**Timeout:** no response to `MISSION_REQUEST_INT`/`MISSION_ACK` within 500 ms aborts the transaction; `formation_controller_node` retries on its next 5 Hz tick rather than blocking (SDD §5.3 — mission-mode has no PX4-side staleness safety net, so the MC must not stall its control loop waiting on this handshake).

---

#### `vehicle_command`

Published by: `fcc_bridge_node` on behalf of `mission_manager_node` or `health_monitor_node`

| Field | Type | Description |
|---|---|---|
| `timestamp` | `uint64` | MC monotonic timestamp (µs) |
| `command` | `uint32` | MAVLink command ID (see §10.5) |
| `param1`–`param7` | `float32` | Command parameters (MAVLink-defined per command) |
| `target_system` | `uint8` | PX4 system ID (typically 1) |
| `target_component` | `uint8` | PX4 component ID (typically 1) |
| `source_system` | `uint8` | MC system ID |
| `source_component` | `uint8` | MC component ID (= `MAV_COMP_ID_ONBOARD_COMPUTER`) |
| `from_external` | `bool` | Set `true` when relaying a GCS command |

**Commonly used `command` values:**

| Command | ID | Purpose | Key params |
|---|---|---|---|
| `MAV_CMD_COMPONENT_ARM_DISARM` | 400 | Arm / Disarm | param1: 1=arm, 0=disarm |
| `MAV_CMD_DO_SET_MODE` | 176 | Set flight mode | param1: MAV_MODE_FLAG; param2: PX4 custom mode |
| `MAV_CMD_NAV_TAKEOFF` | 22 | Takeoff | param7: altitude (m MSL) |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | 20 | RTL | — |
| `MAV_CMD_NAV_LOITER_UNLIM` | 17 | HOLD (loiter) | — |

**PX4 custom mode values for `MAV_CMD_DO_SET_MODE`:**

| Mode | param1 | param2 (custom_main) | param3 (custom_sub) |
|---|---|---|---|
| HOLD | 1 | 4 | 3 |
| MISSION | 1 | 4 | 0 |
| RTL | 1 | 5 | 0 |
| TAKEOFF | 1 | 2 | 6 |
| OFFBOARD ⚠ | 1 | 6 | 0 |

> ⚠ **Pending firmware verification.** `custom_main = 6` is `PX4_CUSTOM_MAIN_MODE_OFFBOARD` per PX4's `px4_custom_mode.h`; verify against the actual v1.14 firmware source before relying on this row operationally (the existing HOLD/MISSION/RTL/TAKEOFF rows are unverified against firmware too — TAKEOFF's `custom_main=2` looks inconsistent with the AUTO-submode pattern the other three use, a pre-existing discrepancy noted here, not introduced by this row).
>
> The MC commands `OFFBOARD` only for a phase/leg the active mission plan explicitly flags via `offboard_phase_mask` (§9.2, `mc_msgs/MissionUpload`); `mission_manager_node` is the sole node authorised to issue the mode command (FR-012, FR-026). Formation keeping (FR-020/023) stays in `MISSION` mode by default and tracks the leader via the mission protocol's `FORMATION_SLOT` item (§3.3 above) — `OFFBOARD` is only used where a leg opts in, via `formation_controller_node` streaming `mc_msgs/FccOffboardSetpoint` (§9.1/§9.2) instead. See `OFFBOARD-MODE-SWITCHING-BACKLOG.md` for the full entry/exit sequencing and fallback design (FR-026–029, SFR-009).

---

#### `vehicle_visual_odometry`

Published by: `fcc_bridge_node` relaying from `visual_localisation_node` (active only when GNSS quality below threshold).

Same field layout as `vehicle_odometry` (§3.2) with `pose_frame = POSE_FRAME_NED` and quality/variance populated by the visual odometry estimator.

---

### 3.4 FccBackend Abstraction

The `FccBackend` class in the `fcc_bridge_node` provides a transport-agnostic ROS 2 interface. Application nodes **never** reference `/fmu/in/` or `/fmu/out/` topics directly.

| FccBackend ROS 2 Topic | Direction | Underlying Transport |
|---|---|---|
| `/mc/fcc/vehicle_state` | FCC → MC | uXRCE-DDS; aggregated from `vehicle_local_position`, `vehicle_global_position`, `vehicle_status`, `battery_status`, `vehicle_attitude` |
| `/mc/fcc/odometry` | FCC → MC | uXRCE-DDS `vehicle_odometry` |
| `/mc/fcc/imu` | FCC → MC | uXRCE-DDS `sensor_combined` |
| `/mc/fcc/mission_upload` | MC → FCC | MAVLink mission protocol, full handshake (§3.3 Mission Protocol) |
| `/mc/fcc/mission_item_update` | MC → FCC | MAVLink mission protocol, partial update (§3.3 Mission Protocol) |
| `/mc/fcc/send_command` | MC → FCC | uXRCE-DDS `vehicle_command` |
| `/mc/fcc/mission_ack` | FCC → MC | `MISSION_ACK` result from the mission protocol handshake, one per upload/item-update |
| `/mc/fcc/visual_odometry` | MC → FCC | uXRCE-DDS `vehicle_visual_odometry` |
| `/mc/fcc/offboard_setpoint` | MC → FCC | uXRCE-DDS `offboard_control_mode` + `trajectory_setpoint`; published only while OFFBOARD is confirmed active (FR-027/028) |

`FccBackend::stream_offboard_setpoint(const OffboardSetpoint &)` is the corresponding abstract method (mirrors `update_mission_item()`'s contract: the backend does not buffer or extrapolate between calls — the caller, `formation_controller_node`, is responsible for sustaining ≥ 2 Hz while OFFBOARD is the commanded mode). It does not itself send `DO_SET_MODE OFFBOARD` — mode commands stay in `send_command()`, called exclusively by `mission_manager_node` (FR-012, FR-026).

---

### 3.5 Timing and Error Handling

| Condition | Timeout | Action | SRS Ref |
|---|---|---|---|
| FCC uXRCE-DDS silence | 1 s | Declare `FCC_LINK_LOST`, command HOLD (if link recovers within 3 s), notify GCS | FR-015 |
| Formation `mission_item_update` gap | > 2 s (leader state stale, FR-024) | `formation_controller_node` commands HOLD itself — PX4 does **not** exit `MISSION` mode on a stale item, so the MC must self-supervise (SDD §5.3) | FR-024 |
| Mission protocol handshake (no `MISSION_REQUEST_INT`/`MISSION_ACK`) | 500 ms | Abort the transaction; retry on the next control-loop tick rather than blocking | IR-007 |
| vehicle_command → actuation | — | Must complete ≤ 100 ms | IR-006 |

---

## 4. IFD-002: MC ↔ GCS (MAVLink 2/UDP)

**SRS refs:** IR-010 to IR-014, FR-060 to FR-064

### 4.1 Transport

| Parameter | Value |
|---|---|
| Protocol | MAVLink 2 over UDP |
| GCS address | `192.168.10.10` |
| GCS receive port (MC → GCS) | `14551` |
| MC receive port (GCS → MC) | `14552` |
| Router daemon | `mavlink-router v4` in `mavlink-router` container |
| MAVLink system_id (MC) | Drone ID (1-based, from swarm manifest; typically 11 for MC #1) |
| MAVLink component_id (MC) | `MAV_COMP_ID_ONBOARD_COMPUTER` (= 191) |
| MAVLink system_id (GCS) | 255 |

> **Channel isolation:** This is Channel B (GCS ↔ MC). Channel A (GCS ↔ FCC, port 14550) is routed independently. The MC's mavlink-router must not forward Channel B traffic onto Channel A or vice versa.
>
> `mavlink-router` meshes every endpoint declared in one config together — it has no concept of isolated endpoint groups within a single process. Guaranteed isolation therefore comes from running two independent `mavlink-routerd` processes, one per channel, each reading its own conf file (`docker/mavlink-router/entrypoint.sh`) — not from a shared config with routing rules. `drone-mc-mavlink-router.service` runs both processes inside the one container.

### 4.2 mavlink-router Configuration

```ini
# /etc/drone-mc/mavlink-router-channel-a.conf
# Channel A: GCS ↔ FCC only. No MC-facing endpoint exists in this process —
# nothing lets an MC application node inject onto Channel A.

[UdpEndpoint GCS_CHANNEL_A]
Mode = Server
Port = 14550       # bound on the MC's own address (192.168.10.11); GCS sends here

[UdpEndpoint FCC_CHANNEL_A]
Mode = Normal
Address = 192.168.10.31
Port = 14550
```

```ini
# /etc/drone-mc/mavlink-router-channel-b.conf
# Channel B: GCS ↔ MC only — a separate process from Channel A above.

[UdpEndpoint GCS_CHANNEL_B]
Mode = Normal
Address = 192.168.10.10
Port = 14551

[UdpEndpoint GCS_MC_RECV]
Mode = Server
Port = 14552

[UdpEndpoint GCS_MAVLINK_NODE]
Mode = Normal
Address = 127.0.0.1
Port = 14560       # internal: gcs_mavlink_node listens here
```

### 4.3 Standard MAVLink Messages — GCS → MC

| Message | ID | Rate / Trigger | Description |
|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz | GCS liveness. `type=6` (GCS), `autopilot=8` (invalid) |
| `COMMAND_LONG` | 76 | On demand | GCS commands to MC (see §4.5) |
| `COMMAND_INT` | 75 | On demand | GCS commands with position payload |
| `MISSION_COUNT` | 44 | On demand | Begin mission upload (item count) |
| `MISSION_ITEM_INT` | 73 | On demand | Mission waypoint/item with int-encoded lat/lon |
| `MISSION_ACK` | 47 | On demand | Acknowledge end of mission upload |
| `PARAM_SET` | 23 | On demand | Set MC runtime parameter |
| `PARAM_REQUEST_LIST` | 21 | On demand | Request all MC parameters |

### 4.4 Standard MAVLink Messages — MC → GCS

| Message | ID | Rate | Description |
|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz | MC liveness. `type=18` (ONBOARD_CONTROLLER), `system_status` reflects current phase |
| `COMMAND_ACK` | 77 | On command receipt | Acknowledge GCS command; result = MAV_RESULT |
| `MISSION_REQUEST_INT` | 51 | During upload | Request next mission item |
| `MISSION_ACK` | 47 | On upload complete | Confirm mission accepted |
| `STATUSTEXT` | 253 | Event-driven | Human-readable status/warning/error strings |
| `PARAM_VALUE` | 22 | On request | MC parameter value |

### 4.5 COMMAND_LONG — GCS → MC Command Catalogue

All MC-directed commands use `target_system = MC system_id` and `target_component = 191` (`MAV_COMP_ID_ONBOARD_COMPUTER`).

| Command | MAV_CMD ID | param1 | param2 | param3–7 | Description |
|---|---|---|---|---|---|
| `MC_PHASE_TRANSITION` | 50001¹ | Phase ID (§10.6) | — | — | Command MC to transition to specified mission phase |
| `MC_ARM_AUTHORISE` | 50002¹ | Mission upload ID | — | — | Grant ARM authorisation for current mission |
| `MC_END_MISSION` | 50003¹ | — | — | — | Command transition to RETURN phase |
| `MC_GCS_OVERRIDE` | 50004¹ | 1=enable, 0=disable | Override type (§10.7) | — | Assert GCS authority override (e.g., suppress low-batt RTL) |
| `MAV_CMD_COMPONENT_ARM_DISARM` | 400 | 0=disarm | — | — | Emergency disarm from GCS |
| `MAV_CMD_DO_SET_MODE` | 176 | — | — | — | Direct flight mode override (emergency) |

¹ IDs in the MAVLink user-defined range (50000–60000). Full definitions in `mc_mavlink_defs.xml` in the shared colcon workspace.

### 4.6 Custom MAVLink Messages — MC → GCS

Custom messages are defined in `mc_mavlink_defs.xml`. IDs are in the user-defined range 50100–50199.

---

#### `MC_STATUS` (ID: 50100)

Replaces/supplements the standard `GLOBAL_POSITION_INT` for MC-specific fields. Rate: ≥ 2 Hz (FR-062).

| Field | Type | Unit | Description |
|---|---|---|---|
| `time_usec` | `uint64_t` | µs | UTC timestamp |
| `lat` | `int32_t` | degE7 | Latitude × 10⁷ |
| `lon` | `int32_t` | degE7 | Longitude × 10⁷ |
| `alt_msl` | `int32_t` | mm | Altitude MSL |
| `heading` | `uint16_t` | cdeg | Vehicle heading (0–36000) |
| `groundspeed` | `uint16_t` | cm/s | Groundspeed |
| `climb_rate` | `int16_t` | cm/s | Climb rate (positive = up) |
| `battery_remaining` | `uint8_t` | % | Battery state of charge (0–100) |
| `mission_phase` | `uint8_t` | — | Current phase (§10.6) |
| `localisation_mode` | `uint8_t` | — | 0=GNSS, 1=VisualOdometry |
| `fault_mask` | `uint16_t` | bitmask | Active faults (§10.8) |
| `peer_count` | `uint8_t` | — | Number of swarm peers currently ALIVE |
| `isr_role` | `uint8_t` | — | 0=non-ISR, 1=ISR |

---

#### `MC_DETECTION` (ID: 50101)

Event-driven. Published within 1 s of detection (FR-043).

| Field | Type | Unit | Description |
|---|---|---|---|
| `time_usec` | `uint64_t` | µs | UTC timestamp of frame capture |
| `detection_id` | `uint32_t` | — | Monotonically increasing detection counter |
| `class_id` | `uint16_t` | — | Detection class label ID (see class registry) |
| `confidence` | `uint16_t` | 0–10000 | Confidence × 10000 (i.e., 9500 = 0.95) |
| `lat` | `int32_t` | degE7 | Estimated geodetic latitude of target × 10⁷ |
| `lon` | `int32_t` | degE7 | Estimated geodetic longitude of target × 10⁷ |
| `alt` | `int32_t` | mm | Estimated target altitude MSL |
| `pos_uncertainty` | `uint16_t` | cm | 1-sigma horizontal position uncertainty |
| `drone_id` | `uint8_t` | — | Reporting drone ID |
| `class_name` | `char[24]` | — | Null-terminated class label string |

---

#### `MC_MISSION_UPLOAD` (ID: 50102)

Sent by GCS as a payload to `COMMAND_LONG(50002)` via a multi-message sequence. The upload uses the standard MAVLink mission protocol (`MISSION_COUNT` → `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` × N → `MISSION_ACK`) with a custom mission type field `MAV_MISSION_TYPE = 3` (MC mission).

Mission item encoding for MC-specific items:

| `command` | Meaning | `x` (lat ×10⁷) | `y` (lon ×10⁷) | `z` | `param1` | `param2` |
|---|---|---|---|---|---|---|
| 50200 | Ingress waypoint | lat | lon | alt (m) | speed (m/s) | — |
| 50201 | Muster point | lat | lon | alt (m) | — | — |
| 50202 | Formation offset | N offset (m) | E offset (m) | D offset (m) | drone_id | — |
| 50203 | Search area vertex | lat | lon | — | vertex_index | total_vertices |
| 50204 | Swarm peer entry | drone_id | IP octet 3 | IP octet 4 | — | — |

---

#### `MC_HEALTH` (ID: 50103)

Rate: ≥ 1 Hz (FR-071).

| Field | Type | Unit | Description |
|---|---|---|---|
| `time_usec` | `uint64_t` | µs | UTC timestamp |
| `cpu_load` | `uint8_t` | % | Total CPU utilisation |
| `mem_used` | `uint8_t` | % | RAM utilisation |
| `disk_used` | `uint8_t` | % | Disk utilisation (primary partition) |
| `gpu_temp` | `uint8_t` | °C | GPU temperature |
| `cpu_temp` | `uint8_t` | °C | CPU package temperature |
| `fault_mask` | `uint16_t` | bitmask | Active faults (§10.8) |
| `node_alive_mask` | `uint16_t` | bitmask | Node liveness flags (§10.9) |
| `uptime_s` | `uint32_t` | s | MC uptime in seconds |

---

### 4.7 STATUSTEXT Severity Levels

| Severity | Value | Usage |
|---|---|---|
| `MAV_SEVERITY_EMERGENCY` | 0 | Critical fault, RTL commanded |
| `MAV_SEVERITY_ALERT` | 1 | Safety failsafe triggered |
| `MAV_SEVERITY_CRITICAL` | 2 | Node failure, restart attempted |
| `MAV_SEVERITY_ERROR` | 3 | Recoverable error (peer lost, link degraded) |
| `MAV_SEVERITY_WARNING` | 4 | Warning condition (low battery, GNSS degraded) |
| `MAV_SEVERITY_NOTICE` | 5 | Phase transition, significant state change |
| `MAV_SEVERITY_INFO` | 6 | Routine informational messages |
| `MAV_SEVERITY_DEBUG` | 7 | Debug (suppressed in operational builds) |

### 4.8 Timing and Error Handling

| Condition | Timeout | Action | SRS Ref |
|---|---|---|---|
| GCS link silence (MC side) | 10 s | Command FCC HOLD; set `GCS_LINK_LOST` fault | FR-073, SFR-007 |
| GCS HEARTBEAT not seen | 10 s | Treat as GCS link lost | — |
| GCS command → MC ACK | — | ≤ 500 ms | FR-061 |
| Low battery | `battery_remaining < LOW_BATTERY_THRESHOLD` | Command RTL unless GCS override active | FR-072, SFR-008 |
| HEARTBEAT transmit rate | — | 1 Hz | IR-011 |

---

## 5. IFD-003: MC ↔ Swarm Peers (Swarm Protocol/UDP)

**SRS refs:** IR-020 to IR-023, FR-030 to FR-036

### 5.1 Transport

| Parameter | Value |
|---|---|
| Protocol | Custom binary / UDP / IPv4 |
| Network | MANET mesh · 192.168.10.x |
| Peer IPs | From GCS-uploaded swarm manifest |
| Heartbeat port | `14900` (all drones listen and send) |
| Coordination port | `14901` (ISR + Voronoi messages) |
| Addressing | Unicast to known peer IPs (from manifest) |
| Max peer message latency | 500 ms before LOST declared | 

### 5.2 Protocol Header

Every swarm message begins with a fixed 12-byte header (little-endian):

```
Offset  Size  Type      Field              Description
------  ----  --------  -----------------  ----------------------------------------
0       1     uint8     protocol_version   = 0x01
1       1     uint8     sender_id          Drone ID (1-based, from swarm manifest)
2       1     uint8     message_type       See §5.3
3       1     uint8     flags              bit0=REQUEST_ACK; bits 1-7 reserved=0
4       2     uint16    sequence           Wrapping sequence number (per message type)
6       2     uint16    payload_length     Byte length of payload following this header
8       4     uint32    crc32              CRC-32 of header bytes 0-7 + payload
   (header total = 12 bytes)
Payload follows immediately after the header.
```

### 5.3 Message Type Catalogue

| Type ID | Name | Port | Direction | Description |
|---|---|---|---|---|
| `0x01` | `SWARM_HEARTBEAT` | 14900 | Broadcast | Periodic liveness and state |
| `0x10` | `ISR_CANDIDATE` | 14901 | Unicast (all peers) | Self-nomination for ISR election |
| `0x11` | `ISR_RESULT` | 14901 | Unicast (coordinator → all) | Election outcome |
| `0x12` | `ISR_RESULT_ACK` | 14901 | Unicast (peer → coordinator) | Acknowledge ISR result |
| `0x20` | `VORONOI_PROPOSAL` | 14901 | Unicast (all peers) | Position set for Voronoi computation |
| `0x21` | `VORONOI_ACCEPT` | 14901 | Unicast (all peers) | Cell acceptance confirmation |
| `0x30` | `PEER_STATE` | 14901 | Unicast (on request) | Detailed peer state for formation keeping |
| `0x31` | `PEER_STATE_REQUEST` | 14901 | Unicast | Request PEER_STATE from a specific drone |

---

#### `SWARM_HEARTBEAT` (0x01)

Rate: 1 Hz. Payload: 40 bytes.

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 8 | `uint64` | `timestamp_us` | UTC timestamp (µs) |
| 8 | 4 | `int32` | `lat_e7` | Latitude × 10⁷ (degE7) |
| 12 | 4 | `int32` | `lon_e7` | Longitude × 10⁷ (degE7) |
| 16 | 4 | `int32` | `alt_mm` | Altitude MSL (mm) |
| 20 | 4 | `float32` | `vx` | North velocity (m/s) |
| 24 | 4 | `float32` | `vy` | East velocity (m/s) |
| 28 | 4 | `float32` | `vz` | Down velocity (m/s) |
| 32 | 1 | `uint8` | `mission_phase` | Current phase (§10.6) |
| 33 | 1 | `uint8` | `battery_pct` | Battery remaining (0–100) |
| 34 | 1 | `uint8` | `sensor_health` | Sensor health bitmask (§10.10) |
| 35 | 1 | `uint8` | `isr_role` | 0=non-ISR, 1=ISR |
| 36 | 2 | `uint16` | `fault_mask` | Active faults (§10.8) |
| 38 | 2 | `uint16` | `reserved` | = 0x0000 |

---

#### `ISR_CANDIDATE` (0x10)

Sent by each drone during `ISR_SELECTION` phase. Payload: 12 bytes.

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 8 | `uint64` | `timestamp_us` | UTC timestamp (µs) |
| 8 | 1 | `uint8` | `battery_pct` | Battery remaining (0–100) |
| 9 | 1 | `uint8` | `sensor_flags` | Sensor capability flags (§10.10) |
| 10 | 2 | `uint16` | `dist_to_centroid_m` | Distance to search area centroid (m) |

---

#### `ISR_RESULT` (0x11)

Sent by coordinator after election. Payload: variable (2 + N bytes where N = swarm size).

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 8 | `uint64` | `timestamp_us` | UTC timestamp (µs) |
| 8 | 1 | `uint8` | `isr_count` | Number of ISR drones elected |
| 9 | 1 | `uint8` | `total_drones` | Total drones in swarm |
| 10 | N | `uint8[N]` | `isr_ids` | Drone IDs elected for ISR (1-based) |

---

#### `ISR_RESULT_ACK` (0x12)

Payload: 9 bytes.

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 8 | `uint64` | `timestamp_us` | UTC timestamp (µs) |
| 8 | 1 | `uint8` | `accepted` | 1 = accepted, 0 = rejected (reason in flags) |

---

#### `VORONOI_PROPOSAL` (0x20)

Sent by each participating drone to share its position for Voronoi computation. Payload: 24 bytes.

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 8 | `uint64` | `timestamp_us` | UTC timestamp (µs) |
| 8 | 1 | `uint8` | `round` | Proposal round (0-based, for re-partition) |
| 9 | 3 | `uint8[3]` | `reserved` | = 0x000000 |
| 12 | 4 | `int32` | `lat_e7` | Drone latitude × 10⁷ |
| 16 | 4 | `int32` | `lon_e7` | Drone longitude × 10⁷ |
| 20 | 4 | `float32` | `alt_m` | Altitude MSL (m) |

> The search area polygon is not re-transmitted in each proposal; all drones reference the polygon uploaded in the GCS mission upload. Consistency is assumed (same upload epoch is checked via mission upload ID in `ISR_CANDIDATE`).

---

#### `VORONOI_ACCEPT` (0x21)

Sent after each drone independently computes its Voronoi cell. Payload: variable (20 + V×8 bytes, V = cell vertices).

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 8 | `uint64` | `timestamp_us` | UTC timestamp (µs) |
| 8 | 1 | `uint8` | `round` | Corresponding proposal round |
| 9 | 1 | `uint8` | `vertex_count` | Number of vertices in this drone's cell (V) |
| 10 | 2 | `uint16` | `reserved` | = 0x0000 |
| 12 | V×8 | `int32[V][2]` | `vertices` | Cell boundary: `[lat_e7, lon_e7]` per vertex, ordered CCW |

---

#### `PEER_STATE` (0x30)

Detailed state for formation keeping. Payload: 52 bytes.

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 8 | `uint64` | `timestamp_us` | UTC timestamp (µs) |
| 8 | 4 | `float32` | `x` | North position NED (m) |
| 12 | 4 | `float32` | `y` | East position NED (m) |
| 16 | 4 | `float32` | `z` | Down position NED (m) |
| 20 | 4 | `float32` | `vx` | North velocity (m/s) |
| 24 | 4 | `float32` | `vy` | East velocity (m/s) |
| 28 | 4 | `float32` | `vz` | Down velocity (m/s) |
| 32 | 4 | `float32` | `heading` | Heading from North (rad) |
| 36 | 4 | `float32[4]` | `q` | Attitude quaternion `[w,x,y,z]` |
| 52 | 1 | `uint8` | `is_leader` | 1 = this drone is the formation leader |
| 53 | 3 | `uint8[3]` | `reserved` | = 0x000000 |

> `PEER_STATE` is published at ≥ 5 Hz during `INGRESS` phase by all drones (FR-022).

---

### 5.4 Peer Management State Machine

```
State: UNKNOWN → ALIVE → LOST → DEAD

UNKNOWN:  Drone in manifest, no messages received yet.
ALIVE:    Heartbeat received within last 3 s.
LOST:     3 consecutive heartbeats missed (>3 s silence).
           → Notify GCS (FR-032)
           → If in SEARCH: trigger re-partition (FR-036)
DEAD:     Peer has been LOST for >30 s. Removed from active set.
```

---

## 6. IFD-004: MC ↔ Downward Camera (CSI / ROS 2)

**SRS refs:** IR-030 to IR-033

### 6.1 Physical Interface

| Parameter | Value |
|---|---|
| Camera | Sony IMX219 |
| Interface | CSI-2 (MIPI Camera Serial Interface) |
| CSI lanes | 2 |
| Max resolution | 3280 × 2464 |
| Operational resolution | 1280 × 720 (configurable) |
| Driver | `gscam2` ROS 2 node (GStreamer backend) |
| GStreamer pipeline | `nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGR ! appsink` |

### 6.2 ROS 2 Topics Published by `camera_node`

#### `/camera/image_raw`

Type: `sensor_msgs/msg/Image`

| Field | Value |
|---|---|
| `header.frame_id` | `"camera_optical"` |
| `height` | 720 (configurable) |
| `width` | 1280 (configurable) |
| `encoding` | `"bgr8"` |
| `is_bigendian` | 0 |
| `step` | `width × 3` bytes |
| `data` | Raw pixel data |
| Publish rate | ≥ 15 FPS during SEARCH and INGRESS phases |

#### `/camera/camera_info`

Type: `sensor_msgs/msg/CameraInfo`

| Field | Value |
|---|---|
| `header.frame_id` | `"camera_optical"` |
| `width`, `height` | Matches `/camera/image_raw` |
| `distortion_model` | `"plumb_bob"` |
| `D` | `[k1, k2, p1, p2, k3]` — loaded from calibration file |
| `K` | 3×3 intrinsic matrix — loaded from calibration file |
| `R` | Rectification matrix (identity if monocular) |
| `P` | 3×4 projection matrix |

Calibration file location: `/etc/drone-mc/camera_calibration.yaml` (bind-mounted into `vision` container).

### 6.3 Coordinate Frame

The camera optical frame (`camera_optical`) is defined as:
- X: right in image plane
- Y: down in image plane
- Z: into scene (optical axis)

Transform from camera body frame to vehicle body frame is stored in the ROS 2 TF tree as a static transform published by `camera_node` at startup.

---

## 7. IFD-006: MC ↔ Gimbal Camera (Ethernet / RTSP H.265)

**SRS refs:** IR-040, FR-080, FR-081, FR-084

### 7.1 Transport

| Parameter | Value |
|---|---|
| Physical interface | HIF-007 — ETH port 1 (JST-GH) on the baseboard's internal Ethernet switch, shared with HIF-002/HIF-004 via Jetson `eth0` |
| Protocol | RTSP 1.0, video codec H.265/HEVC |
| Camera address | `192.168.10.200:8554` |
| Jetson address | `192.168.10.11` (existing `eth0` interface, §HID-007) |
| Stream URL | `rtsp://192.168.10.200:8554/stream1` |
| Resolution / rate | 1920×1080 @ 30 FPS (camera-side configurable) |
| Consumer | `perception_front_node` (`video_uplink` package) via GStreamer `rtspsrc` |

This interface is entirely separate from IFD-004 (downward CSI camera) — different physical link, different node, different package. It does not use `gscam2`.

### 7.2 ROS 2 Topics Published by `perception_front_node`

#### `/mc/perception/front/detections`

Type: `mc_msgs/DetectionArray` (same type as IFD-004's detection message, §9.2 `mc_msgs/Detection`). Event-driven, within 1 s of detection (FR-081).

#### `/mc/perception/front/tracks`

Type: `mc_msgs/TrackArray`. Published at ≥ 5 Hz whenever the gimbal video pipeline is active (FR-081).

| Field | Type | Unit | Description |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | Timestamp of the source frame |
| `tracks` | `mc_msgs/Track[]` | — | Active tracks (§9.2 `mc_msgs/Track`) |

### 7.3 Timing and Error Handling

| Condition | Timeout | Action | SRS Ref |
|---|---|---|---|
| Gimbal RTSP link silence | 3 s | Declare `GIMBAL_LINK_LOST`, clear `GIMBAL_OK` sensor flag, `perception_front_node` attempts `rtspsrc` reconnect | FR-084 |
| Gimbal link loss | — | Does **not** trigger any flight failsafe — perception-only, no effect on FCC control | FR-084 |

---

## 8. IFD-007: MC ↔ GCS Video Uplink (RTSP H.265 + Metadata)

**SRS refs:** IR-041, IR-042, FR-083

### 8.1 Transport

| Parameter | Value |
|---|---|
| Protocol | RTSP 1.0 over the existing MANET link (HIF-004), video codec H.265/HEVC, embedded KLV-style metadata track |
| Server | `video_mux_node` (`video_uplink` package), bound on the MC's GCS-facing address |
| Stream URL (GCS pulls) | `rtsp://192.168.10.11:8555/mc_video` |
| Independent of | IFD-002 (MAVLink GCS channel) — separate port, separate process; a stalled video uplink does not affect telemetry or command/ack traffic |

`video_mux_node` subscribes to `perception_front_node`'s decoded/annotated frame stream and `/mc/perception/front/tracks` + `/mc/perception/front/detections`, re-encodes to H.265, muxes in a metadata track (per-frame UTC timestamp, detection class, confidence, track ID, geodetic position), and serves the result as a single RTSP session.

### 8.2 Metadata Track Contents

| Field | Type | Unit | Description |
|---|---|---|---|
| `time_usec` | `uint64` | µs | UTC timestamp of muxed frame |
| `track_id` | `uint32` | — | Track identifier (persists across frames for the same target) |
| `class_id` | `uint16` | — | Detection class label ID |
| `confidence` | `uint16` | 0–10000 | Confidence × 10000 |
| `lat`, `lon` | `int32` | degE7 | Estimated target geodetic position |
| `alt` | `int32` | mm | Estimated target altitude MSL |

### 8.3 ROS 2 Topics Published by `video_mux_node`

#### `/mc/video/uplink_status`

Type: `mc_msgs/VideoUplinkStatus`. Rate: ≥ 1 Hz.

| Field | Type | Unit | Description |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | UTC timestamp |
| `streaming` | `bool` | — | RTSP session currently serving a connected GCS client |
| `bitrate_kbps` | `uint32` | kbps | Current output bitrate |
| `client_connected` | `bool` | — | GCS has an active RTSP session open |

### 8.4 Timing and Error Handling

| Condition | Timeout | Action | SRS Ref |
|---|---|---|---|
| `perception_front_node` frame/track silence | 3 s | `video_mux_node` stops serving new frames, sets `streaming=false` in `/mc/video/uplink_status`; existing RTSP session is torn down | FR-083 |
| GCS RTSP client disconnect | — | `video_mux_node` continues encoding (ready for reconnect); no fault raised | — |

---

## 9. IFD-005: Internal ROS 2 Topics

**SRS refs:** SW-003, SW-004, SW-010 to SW-019

All internal topics use the `/mc/` namespace. All custom message types are defined in the `mc_msgs` ROS 2 package.

### 9.1 Topic Catalogue

| Topic | Message Type | Publisher | Subscribers | Rate |
|---|---|---|---|---|
| `/mc/fcc/vehicle_state` | `mc_msgs/VehicleState` | `fcc_bridge_node` | `mission_manager`, `formation_controller`, `gcs_mavlink`, `health_monitor` | ≥ 10 Hz |
| `/mc/fcc/odometry` | `nav_msgs/Odometry` | `fcc_bridge_node` | `visual_localisation` | ≥ 20 Hz |
| `/mc/fcc/imu` | `sensor_msgs/Imu` | `fcc_bridge_node` | `visual_localisation` | ≥ 100 Hz |
| `/mc/fcc/mission_upload` | `mc_msgs/FccMission` | `mission_manager` | `fcc_bridge_node` | On change (path replace) |
| `/mc/fcc/mission_item_update` | `mc_msgs/FccMissionItem` | `formation_controller` | `fcc_bridge_node` | ≥ 5 Hz (MISSION, INGRESS) |
| `/mc/fcc/send_command` | `mc_msgs/FccCommand` | `mission_manager`, `health_monitor` | `fcc_bridge_node` | Event-driven |
| `/mc/fcc/mission_ack` | `mc_msgs/FccMissionAck` | `fcc_bridge_node` | `mission_manager`, `formation_controller` | Event-driven (one per forwarded upload/item-update) |
| `/mc/fcc/visual_odometry` | `mc_msgs/VisualOdometry` | `visual_localisation` | `fcc_bridge_node` | ≥ 10 Hz (GNSS-denied) |
| `/mc/fcc/offboard_setpoint` | `mc_msgs/FccOffboardSetpoint` | `formation_controller` | `fcc_bridge_node` | ≥ 5 Hz while OFFBOARD confirmed active (FR-027/028); idle otherwise |
| `/mc/mission_phase` | `mc_msgs/MissionPhase` | `mission_manager` | `swarm_coordinator`, `gcs_mavlink`, `health_monitor`, `formation_controller` | On change + 1 Hz keepalive |
| `/mc/manager/heartbeat` | `std_msgs/Header` | `mission_manager` | `health_monitor` (watchdog) | ≥ 1 Hz |
| `/mc/gcs/command` | `mc_msgs/GcsCommand` | `gcs_mavlink_node` | `mission_manager` | Event-driven |
| `/mc/gcs/mission_upload` | `mc_msgs/MissionUpload` | `gcs_mavlink_node` | `mission_manager`, `swarm_coordinator` | Event-driven |
| `/mc/gcs/link_status` | `mc_msgs/LinkStatus` | `gcs_mavlink_node` | `mission_manager`, `health_monitor` | 1 Hz |
| `/mc/swarm/peer_states` | `mc_msgs/SwarmPeerArray` | `swarm_coordinator` | `formation_controller`, `mission_manager` | 1 Hz + on change |
| `/mc/swarm/voronoi_cell` | `mc_msgs/VoronoiCell` | `swarm_coordinator` | `search_planner`, `mission_manager` | On change |
| `/mc/swarm/isr_role` | `mc_msgs/IsrRole` | `swarm_coordinator` | `mission_manager` | On change |
| `/mc/search/coverage_path` | `nav_msgs/Path` | `search_planner` | `mission_manager` | On change |
| `/mc/local_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `visual_localisation` | `perception_down`, `mission_manager`, `gcs_mavlink` | ≥ 10 Hz |
| `/mc/localisation_mode` | `mc_msgs/LocalisationMode` | `visual_localisation` | `gcs_mavlink`, `health_monitor` | On change |
| `/mc/detections` | `mc_msgs/DetectionArray` | `perception_down` (renamed from `detection_node`) | `gcs_mavlink` | Event-driven |
| `/mc/perception/front/detections` | `mc_msgs/DetectionArray` | `perception_front` | `video_mux`, `gcs_mavlink` | Event-driven |
| `/mc/perception/front/tracks` | `mc_msgs/TrackArray` | `perception_front` | `video_mux` | ≥ 5 Hz |
| `/mc/video/uplink_status` | `mc_msgs/VideoUplinkStatus` | `video_mux` | `health_monitor` | ≥ 1 Hz |
| `/mc/health` | `mc_msgs/SystemHealth` | `health_monitor` | `gcs_mavlink`, `mission_manager` | ≥ 1 Hz |
| `/mc/fault` | `mc_msgs/FaultEvent` | `health_monitor` | `mission_manager` | Event-driven |

### 9.2 Custom Message Definitions

#### `mc_msgs/VehicleState`

```
# Aggregated FCC state — published by fcc_bridge_node
std_msgs/Header header
# Position
float64 lat             # deg  WGS-84
float64 lon             # deg  WGS-84
float32 alt_msl         # m    MSL
float32 x               # m    NED North
float32 y               # m    NED East
float32 z               # m    NED Down
# Velocity
float32 vx              # m/s  NED
float32 vy              # m/s  NED
float32 vz              # m/s  NED
float32 groundspeed     # m/s
# Attitude
float32 heading         # rad  from North
float32[4] q            # [w,x,y,z] NED-to-body
# Status
uint8   nav_state       # PX4 nav state (see §10.1)
uint8   arming_state    # PX4 arming state (see §10.2)
# Battery
float32 battery_v       # V
float32 battery_pct     # 0.0 – 1.0
uint8   battery_warning # see §10.3
# GNSS
uint8   fix_type        # 0=no fix, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
float32 hdop            # horizontal DOP
float32 vdop            # vertical DOP
bool    gnss_ok         # fix_type >= 3 AND hdop < threshold
# Link
bool    fcc_link_ok     # uXRCE-DDS bridge alive
```

#### `mc_msgs/FccMissionItem`

```
# Single PX4 mission item — mirrors fcc_bridge::MissionItem. Published
# standalone on /mc/fcc/mission_item_update (FORMATION_SLOT tracking,
# >=5 Hz during INGRESS) and reused as the element type of
# mc_msgs/FccMission.items (FCC-005).
std_msgs/Header header
uint16  seq
uint8   frame               # MAV_FRAME (5 = FRAME_GLOBAL_INT)
uint16  command              # MAV_CMD (16 = NAV_WAYPOINT, 17 = NAV_LOITER_UNLIM)
int32   lat                  # 1e7 deg
int32   lon                  # 1e7 deg
float32 alt                  # m MSL
float32 acceptance_radius    # m
```

#### `mc_msgs/FccMission`

```
# Full-replace mission upload on /mc/fcc/mission_upload — on-change only,
# not periodic (SDD §3.4 item 2).
std_msgs/Header header
uint32 upload_id             # correlates with the eventual /mc/fcc/mission_ack (FCC-006)
mc_msgs/FccMissionItem[] items
```

#### `mc_msgs/FccCommand`

```
# Vehicle command forwarded to the FCC on /mc/fcc/send_command (arm/disarm,
# mode change, RTL, HOLD, ...) — event-driven, mirrors
# FccBackend::send_command().
std_msgs/Header header
uint32 cmd                   # MAV_CMD
float32[7] params            # param1-param7
```

#### `mc_msgs/FccMissionAck`

```
# Ack for a mission upload or mission-item-update request forwarded by
# fcc_bridge_node (the /mc/fcc/mission_upload and /mc/fcc/mission_item_update
# subscriptions, FCC-005) — reports whether the FCC's
# MISSION_ACK/MAV_MISSION_ACCEPTED response arrived before timeout (FCC-006).
# Event-driven: published once per forwarded request, no periodic republish.
std_msgs/Header header

uint8 UPLOAD      = 0
uint8 ITEM_UPDATE = 1
uint8 request_type       # UPLOAD or ITEM_UPDATE, see constants above

uint32 upload_id         # valid iff request_type == UPLOAD; echoes FccMission.upload_id
uint16 seq                # valid iff request_type == ITEM_UPDATE; echoes FccMissionItem.seq

bool success              # true = MAV_MISSION_ACCEPTED before timeout; false = timeout/reject
```

#### `mc_msgs/FccOffboardSetpoint`

```
# Continuous OFFBOARD position setpoint, published on /mc/fcc/offboard_setpoint
# while an OFFBOARD-flagged leg is active (FR-027/028). NED frame, matches
# px4_msgs::msg::TrajectorySetpoint / OffboardControlMode field layout on the
# XrceDdsBackend side. Velocity/acceleration fields are deliberately omitted
# for this initial cut — formation-tracking only needs position+yaw, the same
# as the FORMATION_SLOT mission-item-replace path this supersedes on
# OFFBOARD-flagged legs (SDD §5.2). Extend later if a use case needs velocity
# control.
std_msgs/Header header
float32 x            # N, m
float32 y            # E, m
float32 z            # D, m
float32 yaw          # rad, NED
```

#### `mc_msgs/MissionPhase`

```
# Current mission phase
std_msgs/Header header
uint8 phase             # See §10.6
string phase_name       # Human-readable phase name
uint64 phase_entered_us # UTC us when this phase was entered
uint8  source           # 0=GCS_COMMAND 1=AUTONOMOUS 2=FAILSAFE
```

#### `mc_msgs/GcsCommand`

```
# Decoded GCS MAVLink command
std_msgs/Header header
uint32 command          # MAV_CMD ID
float32[7] params       # param1–param7
uint8  target_system
uint8  target_component
uint8  confirmation
```

#### `mc_msgs/MissionUpload`

```
# Full mission data uploaded by GCS
std_msgs/Header header
uint32 upload_id                          # Unique ID for this upload
# Ingress waypoints
geometry_msgs/PoseStamped[] ingress_waypoints
# Formation offsets [drone_id, N, E, D]
mc_msgs/FormationOffset[] formation_offsets
# Search area polygon (geodetic)
geographic_msgs/GeoPoint[] search_polygon
# Swarm manifest
mc_msgs/SwarmPeer[] swarm_manifest
# ARM authorisation
bool arm_authorised
float32 low_battery_threshold             # 0.0–1.0 (default 0.20)
# bit N = Mission Phase ID N (§10.6) requires OFFBOARD for its FCC-facing
# legs; 0 = all MISSION mode (pre-FR-026 behaviour). See FR-020/026-029.
uint16 offboard_phase_mask
```

#### `mc_msgs/SwarmPeerArray`

```
# Array of all known peer states
std_msgs/Header header
mc_msgs/SwarmPeer[] peers
```

#### `mc_msgs/SwarmPeer`

```
uint8  drone_id
string ip_address
uint8  mission_phase      # see §10.6
float64 lat               # deg
float64 lon               # deg
float32 alt_msl           # m
float32 vx                # m/s NED
float32 vy
float32 vz
float32 battery_pct       # 0.0–1.0
uint8   sensor_flags      # see §10.10
uint16  fault_mask        # see §10.8
uint8   peer_status       # 0=UNKNOWN 1=ALIVE 2=LOST 3=DEAD
bool    is_leader
bool    isr_role
uint64  last_heartbeat_us # UTC us of last received heartbeat
```

#### `mc_msgs/VoronoiCell`

```
# Assigned Voronoi cell boundary
std_msgs/Header header
uint8   drone_id
uint8   round             # partition round (0 = initial, 1+ = re-partition)
geographic_msgs/GeoPoint[] vertices  # CCW order
uint32  upload_id         # corresponding mission upload ID
```

#### `mc_msgs/DetectionArray`

```
# AI detection results
std_msgs/Header header
mc_msgs/Detection[] detections
```

#### `mc_msgs/Detection`

```
uint32  detection_id      # Monotonic counter
uint64  frame_timestamp_us # UTC us of source frame
uint16  class_id
string  class_name
float32 confidence        # 0.0–1.0
float64 lat               # deg — estimated target geodetic position
float64 lon               # deg
float32 alt_msl           # m
float32 pos_uncertainty_m # 1-sigma horizontal position uncertainty
# Bounding box in image (pixels)
uint32  bbox_x
uint32  bbox_y
uint32  bbox_w
uint32  bbox_h
```

#### `mc_msgs/Track`

```
# Single tracked target from perception_front_node (gimbal camera)
uint32  track_id          # Persists across frames for the same target
uint16  class_id
string  class_name
float32 confidence        # 0.0-1.0
float64 lat                # deg - estimated target geodetic position
float64 lon                # deg
float32 alt_msl           # m
uint32  frames_tracked     # Consecutive frames this track has been held
```

#### `mc_msgs/TrackArray`

```
# Active tracks - published by perception_front_node on
# /mc/perception/front/tracks (ICD IFD-006 section 7.2)
std_msgs/Header header
mc_msgs/Track[] tracks
```

#### `mc_msgs/VideoUplinkStatus`

```
# video_mux_node health - published on /mc/video/uplink_status (ICD IFD-007 section 8.3)
std_msgs/Header header
bool    streaming
uint32  bitrate_kbps
bool    client_connected
```

#### `mc_msgs/SystemHealth`

```
std_msgs/Header header
uint8   cpu_pct           # 0–100
uint8   mem_pct           # 0–100
uint8   disk_pct          # 0–100
uint8   gpu_temp_c        # °C
uint8   cpu_temp_c        # °C
uint16  fault_mask        # see §10.8
uint16  node_alive_mask   # see §10.9
uint32  uptime_s
```

#### `mc_msgs/FaultEvent`

```
std_msgs/Header header
uint16  fault_id          # see §10.8 (single fault bit)
uint8   severity          # MAV_SEVERITY level
string  description
uint8   failsafe_action   # 0=NONE 1=HOLD 2=RTL 3=LAND
```

#### `mc_msgs/LocalisationMode`

```
std_msgs/Header header
uint8  mode               # 0=GNSS_PRIMARY 1=VISUAL_ODOMETRY
uint8  reason             # 0=NOMINAL 1=GNSS_FIX_LOST 2=HDOP_HIGH 3=GNSS_REACQUIRED
float32 hdop              # Current HDOP
uint8   fix_type          # Current GNSS fix type
```

#### `mc_msgs/LinkStatus`

```
std_msgs/Header header
bool    gcs_link_ok
uint64  last_heartbeat_us  # UTC us of last GCS heartbeat received
float32 link_quality       # 0.0–1.0 (RSSI-derived if available)
uint32  rx_packet_count
uint32  tx_packet_count
uint32  rx_drop_count
```

---

## 10. Enumerations and Constants

### 10.1 PX4 Navigation State (`nav_state`)

| Value | Name | Description |
|---|---|---|
| 0 | `MANUAL` | Manual mode |
| 3 | `AUTO_MISSION` | Autonomous waypoint mission |
| 4 | `AUTO_LOITER` | Hold / loiter |
| 5 | `AUTO_RTL` | Return to launch |
| 14 | `OFFBOARD` | Offboard control (MC setpoints) |
| 17 | `AUTO_TAKEOFF` | Autonomous takeoff |
| 22 | `AUTO_PRECLAND` | Precision landing |

### 10.2 PX4 Arming State (`arming_state`)

| Value | Name |
|---|---|
| 0 | `INIT` |
| 1 | `STANDBY` |
| 2 | `ARMED` |
| 3 | `STANDBY_ERROR` |
| 4 | `SHUTDOWN` |
| 5 | `IN_AIR_RESTORE` |

### 10.3 Battery Warning Level

| Value | Name | Description |
|---|---|---|
| 0 | `NONE` | Normal |
| 1 | `LOW` | Low battery warning |
| 2 | `CRITICAL` | Critical — RTL imminent |
| 3 | `EMERGENCY` | Emergency — land immediately |
| 4 | `FAILED` | Battery failure |

### 10.4 Odometry Reference Frame

| Value | Name |
|---|---|
| 0 | `POSE_FRAME_UNKNOWN` |
| 1 | `POSE_FRAME_NED` |
| 2 | `POSE_FRAME_FRD` |
| 3 | `VELOCITY_FRAME_BODY_FRD` |

### 10.5 Commonly Used MAVLink Command IDs

| ID | Name |
|---|---|
| 17 | `MAV_CMD_NAV_LOITER_UNLIM` |
| 20 | `MAV_CMD_NAV_RETURN_TO_LAUNCH` |
| 22 | `MAV_CMD_NAV_TAKEOFF` |
| 176 | `MAV_CMD_DO_SET_MODE` |
| 400 | `MAV_CMD_COMPONENT_ARM_DISARM` |
| 50001 | `MC_CMD_PHASE_TRANSITION` (custom) |
| 50002 | `MC_CMD_ARM_AUTHORISE` (custom) |
| 50003 | `MC_CMD_END_MISSION` (custom) |
| 50004 | `MC_CMD_GCS_OVERRIDE` (custom) |

### 10.6 Mission Phase IDs

| Value | Name |
|---|---|
| 0 | `PREFLIGHT` |
| 1 | `ARMING` |
| 2 | `TAKEOFF` |
| 3 | `INGRESS` |
| 4 | `MUSTER` |
| 5 | `ISR_SELECTION` |
| 6 | `VORONOI_PARTITION` |
| 7 | `SEARCH` |
| 8 | `RETURN` |
| 9 | `LAND` |
| 255 | `FAULT` |

### 10.7 GCS Override Types

| Value | Name | Effect |
|---|---|---|
| 0 | `OVERRIDE_NONE` | Disable all overrides |
| 1 | `OVERRIDE_LOW_BATTERY_RTL` | Suppress low-battery RTL |
| 2 | `OVERRIDE_GCS_LINK_HOLD` | Suppress GCS-link-loss HOLD |

### 10.8 Fault Mask Bits

| Bit | Name | Trigger |
|---|---|---|
| 0 | `FCC_LINK_LOST` | uXRCE-DDS silent > 1 s |
| 1 | `GCS_LINK_LOST` | GCS MAVLink silent > 10 s |
| 2 | `MANAGER_DEAD` | Manager heartbeat silent > 5 s after 3 restarts |
| 3 | `LOW_BATTERY` | Battery remaining < threshold |
| 4 | `PEER_LOST` | ≥ 1 peer declared LOST |
| 5 | `GPU_OVERTEMP` | GPU temperature > 85 °C |
| 6 | `DISK_FULL` | Disk usage > 90% |
| 7 | `CAMERA_FAIL` | Camera node not publishing |
| 8 | `GNSS_DEGRADED` | GNSS fix lost or HDOP > 2.0 |
| 9 | `VISUAL_ODOM_FAIL` | Visual odometry not publishing when GNSS degraded |
| 10 | `GIMBAL_LINK_LOST` | Gimbal camera RTSP link silent > 3 s (perception-only, no flight failsafe) |
| 11 | `OFFBOARD_STREAM_STALE` | OFFBOARD setpoint stream age > 500 ms while OFFBOARD confirmed active (SFR-009) — `health_monitor_node` publishes this fault only; `mission_manager_node` executes the fallback (FR-029) |
| 12–15 | Reserved | — |

### 10.9 Node Alive Mask Bits

| Bit | Node |
|---|---|
| 0 | `fcc_bridge_node` |
| 1 | `gcs_mavlink_node` |
| 2 | `swarm_coordinator_node` |
| 3 | `formation_controller_node` |
| 4 | `mission_manager_node` |
| 5 | `search_planner_node` |
| 6 | `visual_localisation_node` |
| 7 | `camera_node` |
| 8 | `perception_down_node` (renamed from `detection_node`) |
| 9 | `health_monitor_node` |
| 10 | `perception_front_node` |
| 11 | `video_mux_node` |
| 12–15 | Reserved |

### 10.10 Sensor Health / Capability Flags

| Bit | Name | Description |
|---|---|---|
| 0 | `CAMERA_OK` | Downward camera publishing |
| 1 | `AI_INFERENCE_OK` | Detection pipeline running |
| 2 | `VISUAL_ODOM_OK` | Visual odometry running |
| 3 | `GNSS_OK` | GNSS fix adequate |
| 4 | `IMU_OK` | IMU data flowing |
| 5 | `GIMBAL_OK` | Gimbal camera RTSP link alive |
| 6–7 | Reserved | — |

---

## 11. Configuration Reference

All deployment parameters are set via environment variables in the Docker Compose or systemd environment files. No runtime configuration changes require code recompilation.

### 11.1 Network Configuration (Fixed)

| Parameter | Value |
|---|---|
| MC IP | `192.168.10.11` |
| FCC IP | `192.168.10.31` |
| GCS IP | `192.168.10.10` |
| MC ↔ FCC internal subnet | `192.168.12.x` |
| Swarm peer IPs | `192.168.10.12` … `192.168.10.1N` |
| Gimbal IP camera (HIF-007, shared switch — reuses MC IP `192.168.10.11` on `eth0`) | `192.168.10.200` (first of the `.200`+ range reserved for camera/gimbal peripherals) |

### 11.2 Port Reference

| Port | Protocol | Interface | Direction |
|---|---|---|---|
| `14550` | UDP | MAVLink — GCS ↔ FCC (Channel A) | Bidirectional (MC does not use) |
| `14551` | UDP | MAVLink — GCS → MC (Channel B) | GCS → mavlink-router on MC |
| `14552` | UDP | MAVLink — MC → GCS (Channel B) | MC → GCS |
| `14560` | UDP | MAVLink — mavlink-router → gcs_mavlink_node | Internal (loopback) |
| `8888` | UDP | uXRCE-DDS Agent | FCC → MC (ethernet fallback) |
| `14555` | UDP | MAVLink mission protocol — MC ↔ FCC | Bidirectional, always on `192.168.12.x` (IR-007) |
| `14900` | UDP | Swarm heartbeat | Broadcast among peers |
| `14901` | UDP | Swarm coordination | Unicast among peers |
| `8554` | TCP/UDP | RTSP — Gimbal camera source stream | Gimbal camera → `perception_front_node`, on `192.168.10.200` (HIF-007, shared switch) |
| `8555` | TCP/UDP | RTSP — MC video uplink | `video_mux_node` → GCS, on `192.168.10.11` (IFD-007) |

### 11.3 Serial Configuration

| Parameter | Value |
|---|---|
| Device | `/dev/ttyTHS1` |
| Baud rate | `921600` |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |

### 11.4 Deployment Environment Variables

| Variable | Default | Description |
|---|---|---|
| `FCC_TRANSPORT` | `serial` | `serial` or `ethernet` — selects uXRCE-DDS transport |
| `FCC_SERIAL_DEVICE` | `/dev/ttyTHS1` | Serial device for FCC (serial transport) |
| `FCC_SERIAL_BAUD` | `921600` | Baud rate |
| `FCC_ETH_IP` | `192.168.12.31` | FCC IP (ethernet transport) |
| `FCC_ETH_PORT` | `8888` | uXRCE-DDS UDP port |
| `FCC_MISSION_PORT` | `14555` | MAVLink mission-protocol UDP port (always active on `192.168.12.x`, independent of `FCC_TRANSPORT`) |
| `DRONE_ID` | `1` | This drone's ID (1-based) |
| `GCS_IP` | `192.168.10.10` | GCS IP address |
| `GCS_MAV_PORT_RECV` | `14551` | MC receive port for GCS MAVLink |
| `GCS_MAV_PORT_SEND` | `14552` | MC send port to GCS MAVLink |
| `GCS_MAVLINK_LOOPBACK_PORT` | `14560` | `gcs_mavlink_node`'s Channel B loopback port (`GCS_MAVLINK_NODE` endpoint, §4.2) |
| `SWARM_HB_PORT` | `14900` | Swarm heartbeat port |
| `SWARM_COORD_PORT` | `14901` | Swarm coordination port |
| `LOW_BATTERY_THRESHOLD` | `0.20` | RTL trigger (0.0–1.0) |
| `GCS_LINK_TIMEOUT_S` | `10` | GCS link-loss HOLD trigger (s) |
| `FCC_LINK_TIMEOUT_S` | `1` | FCC link-loss fault trigger (s) |
| `MANAGER_WD_TIMEOUT_S` | `5` | Manager watchdog timeout (s) |
| `SIMULATION_MODE` | `false` | `true` replaces hardware with SITL equivalents |
| `SITL_FCC_PORT` | `14540` | `MavlinkUdpBackend`'s local UDP bind port (`SIMULATION_MODE=true` only) — PX4 SITL's address is learned dynamically from its first datagram, not configured (FCC-007) |
| `ROS_DOMAIN_ID` | `0` | CycloneDDS domain ID (same-host discovery; also the domain `zenoh-bridge-dds` bridges cross-host, §11.4a) |
| `CYCLONEDDS_URI` | `/etc/drone-mc/cyclonedds.xml` | CycloneDDS config file (same-host / `lo` only) |
| `CAMERA_CALIBRATION_FILE` | `/etc/drone-mc/camera_calibration.yaml` | Calibration file path |
| `CAMERA_WIDTH` | `1280` | Camera capture width (px) |
| `CAMERA_HEIGHT` | `720` | Camera capture height (px) |
| `CAMERA_FPS` | `30` | Camera capture frame rate |

### 11.4a Cross-Host DDS Bridge Configuration

`zenoh-bridge-dds` (SW-008, SW-039) is not configured via `/etc/drone-mc/environment` — its peer list, mode, and topic allow-list live entirely in the per-drone-edited `/etc/drone-mc/zenoh-bridge-dds.json5` (`config/zenoh-bridge-dds.json5` in the repo), so there is exactly one file to edit per drone rather than two that could drift apart. It picks up `$ROS_DOMAIN_ID` from the shared environment file to know which local DDS domain to bridge. Default listen/connect port: `7447` (TCP).

---

*MC-ICD-001 · v0.1-DRAFT · 2026-06-20*
