# ROS 2 API Reference

Internal ROS 2 topic and message reference for the `ws/src/` colcon
workspace, grounded in the current code (not just the design docs). For the
external wire formats (MAVLink Channels A/B/C, uXRCE-DDS to the FCC) see
`ICD-MC-001.md` — this document covers only the ROS 2 graph *between* MC
nodes (plus the `fcc_bridge` backend's own DDS topics to PX4, since those
are still ROS 2 topics on the wire).

No `.srv` or `.action` interfaces are defined anywhere in the workspace yet
— all inter-node communication today is topic-based.

## Implementation status

Only four nodes currently wire up real publishers/subscribers:
`fcc_bridge_node`, `gcs_mavlink_node`, `mission_orchestrator_node`, and
`mission_manager_node`. Every other node in the tables below
(`health_monitor_node`, `swarm_coordinator_node`,
`formation_controller_node`, `perception_down_node`,
`visual_localisation_node`, `perception_front_node`, `video_mux_node`,
`search_planner_node`) exists only as an empty constructor stub — no
publishers, subscribers, or timers are wired up in their `.cpp` files yet.
Their rows below are the **designed** interface (sourced from the
`mc_msgs/msg/*.msg` header comments and `ICD-MC-001.md` §9.2), not something
you can observe on a running `ros2 topic list` today.

Each table row is marked:

- ✅ **implemented** — the publisher/subscriber exists in the node's `.cpp`
  today.
- 🧭 **designed** — documented in `mc_msgs` / the ICD as this node's
  eventual interface, not yet wired up in code.

## `/mc/` topic map by node

### `fcc_bridge_node` (package `fcc_bridge`)

| Topic | Type | Dir | QoS | Rate / trigger | Status |
|---|---|---|---|---|---|
| `/mc/fcc/vehicle_state` | `mc_msgs/VehicleState` | pub | `KeepLast(10)` | `vehicle_state_rate_hz` param, default 10 Hz | ✅ |
| `/mc/fault` | `mc_msgs/FaultEvent` | pub | `KeepLast(10)` | on FCC/GCS link-loss detection | ✅ |
| `/mc/fcc/mission_ack` | `mc_msgs/FccMissionAck` | pub | `KeepLast(10)` | once per forwarded upload/item-update request | ✅ |
| `/mc/fcc/mission_upload` | `mc_msgs/FccMission` | sub | `KeepLast(10)` | on change (full mission replace) | ✅ |
| `/mc/fcc/mission_item_update` | `mc_msgs/FccMissionItem` | sub | `KeepLast(10)` | on change (single-item replace, e.g. FORMATION_SLOT) | ✅ |
| `/mc/fcc/send_command` | `mc_msgs/FccCommand` | sub | `KeepLast(10)` | event-driven (arm/disarm, mode change, RTL, HOLD) | ✅ |
| `/mc/fcc/offboard_setpoint` | `mc_msgs/FccOffboardSetpoint` | sub | `KeepLast(10)` | continuous, while an OFFBOARD-flagged leg is active | ✅ |
| `/mc/fcc/visual_odometry` | `mc_msgs/VisualOdometry` | sub | — | ≥10 Hz while `/mc/localisation_mode == VISUAL_ODOMETRY` | 🧭 (backend method `FccBackend::publish_visual_odometry()` exists; no ROS subscription wired to call it yet) |

`fcc_bridge_node` also talks to PX4 directly over DDS via its `FccBackend`
(`XrceDdsBackend` in production, `MavlinkUdpBackend` for SITL). The
`XrceDdsBackend` topics are real ROS 2 topics but are private to the
backend — no other MC node subscribes to them:

| Topic | Type | Dir | QoS |
|---|---|---|---|
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | sub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/out/vehicle_global_position` | `px4_msgs/VehicleGlobalPosition` | sub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | sub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | sub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | sub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | sub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` | sub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | pub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | pub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | pub | best-effort, `KeepLast(5)`, volatile |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | pub | best-effort, `KeepLast(5)`, volatile |

QoS matches PX4's uXRCE-DDS client defaults on `/fmu/*` topics — best-effort
reliability and volatile durability are required for the DDS↔uORB bridge to
see the node at all. The MC↔FCC mission protocol (Channel C,
`MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK`) rides a separate raw UDP
socket, not a ROS 2 topic — see `ICD-MC-001.md` §3.3.

### `gcs_mavlink_node` (package `gcs_comms`)

Channel B: GCS→MC, raw MAVLink v2 over a loopback socket (`GCS_MAVLINK_LOOPBACK_PORT`, default 14560).

| Topic | Type | Dir | QoS | Rate / trigger | Status |
|---|---|---|---|---|---|
| `/mc/gcs/command` | `mc_msgs/GcsCommand` | pub | `KeepLast(10)` | on decoded GCS `COMMAND_LONG` | ✅ |
| `/mc/gcs/link_status` | `mc_msgs/LinkStatus` | pub | `KeepLast(10)` | on heartbeat timer (1000 ms) | ✅ |
| `/mc/gcs/mission_upload` | `mc_msgs/MissionUpload` | pub | `KeepLast(10)` | on decoded GCS mission upload | ✅ |
| `/mc/fcc/vehicle_state` | `mc_msgs/VehicleState` | sub | `KeepLast(10)` | mirrored to GCS via `mc_status_timer_` (500 ms) | ✅ |
| `/mc/mission_phase` | `mc_msgs/MissionPhase` | sub | `KeepLast(10)` | " | ✅ |
| `/mc/localisation_mode` | `mc_msgs/LocalisationMode` | sub | `KeepLast(10)` | " | ✅ |
| `/mc/health` | `mc_msgs/SystemHealth` | sub | `KeepLast(10)` | " | ✅ |
| `/mc/detections` | `mc_msgs/DetectionArray` | sub | `KeepLast(10)` | relayed to GCS as they arrive | ✅ |

MAVROS runs alongside `gcs_mavlink_node` as a second OS process in the
`gcs-comms` container (GCS-005), mirroring Channel A telemetry directly —
it is not part of this node's ROS graph and has no `mc_msgs` topics of its
own; see `CLAUDE.md`'s MAVLink channel separation note.

### `mission_orchestrator_node` (package `gcs_comms`)

Channel C: MAVLink mission-plan-library protocol over its own loopback
socket (`ORCHESTRATOR_MAVLINK_LOOPBACK_PORT`, default 14580); see
`MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md`.

| Topic | Type | Dir | QoS | Rate / trigger | Status |
|---|---|---|---|---|---|
| `/mc/gcs/mission_upload` | `mc_msgs/MissionUpload` | pub | `KeepLast(10)` | on activating a stored plan | ✅ |
| `/mc/mission_phase` | `mc_msgs/MissionPhase` | sub | `KeepLast(10)` | gates activation to `PREFLIGHT` only | ✅ |

Note both `gcs_mavlink_node` and `mission_orchestrator_node` publish
`/mc/gcs/mission_upload` — they are alternative sources of the same topic
(interactive GCS upload vs. activating a stored plan from the library), not
a conflict.

### `mission_manager_node` (package `mission_exec`)

| Topic | Type | Dir | QoS | Rate / trigger | Status |
|---|---|---|---|---|---|
| `/mc/mission_phase` | `mc_msgs/MissionPhase` | pub | `KeepLast(10)` | on phase change + 1 Hz keepalive | ✅ |
| `/mc/manager/heartbeat` | `std_msgs/Header` | pub | `KeepLast(10)` | liveness heartbeat (watchdog, FR-074) | ✅ |
| `/mc/fcc/send_command` | `mc_msgs/FccCommand` | pub | `KeepLast(10)` | event-driven | ✅ |
| `/mc/fcc/mission_item_update` | `mc_msgs/FccMissionItem` | pub | `KeepLast(10)` | event-driven | ✅ |
| `/mc/fcc/mission_upload` | `mc_msgs/FccMission` | pub | `KeepLast(10)` | on change | ✅ |
| `/mc/status_text` | `mc_msgs/StatusText` | pub | `KeepLast(10)` | e.g. pre-arm-check failure reasons (EXEC-002) | ✅ |
| `/mc/gcs/command_ack` | `mc_msgs/GcsCommandAck` | pub | `KeepLast(10)` | ≤500 ms after a Phase-tier GCS command (EXEC-003) | ✅ |
| `/mc/fcc/vehicle_state` | `mc_msgs/VehicleState` | sub | `KeepLast(10)` | — | ✅ |
| `/mc/fcc/mission_ack` | `mc_msgs/FccMissionAck` | sub | `KeepLast(10)` | — | ✅ |
| `/mc/gcs/command` | `mc_msgs/GcsCommand` | sub | `KeepLast(10)` | — | ✅ |
| `/mc/gcs/mission_upload` | `mc_msgs/MissionUpload` | sub | `KeepLast(10)` | persisted to disk on receipt (FR-074) | ✅ |
| `/mc/swarm/peer_states` | `mc_msgs/SwarmPeerArray` | sub | `KeepLast(10)` | — | ✅ |
| `/mc/swarm/isr_role` | `mc_msgs/IsrRole` | sub | `KeepLast(10)` | gates `ISR_SELECTION → VORONOI_PARTITION` | ✅ |
| `/mc/swarm/voronoi_cell` | `mc_msgs/VoronoiCell` | sub | `KeepLast(10)` | — | ✅ |
| `/mc/search/coverage_path` | `nav_msgs/Path` | sub | `KeepLast(10)` | boustrophedon waypoints from `search_planner_node` | ✅ |
| `/mc/fault` | `mc_msgs/FaultEvent` | sub | `KeepLast(10)` | drives `FAULT → RTL` escape | ✅ |
| `/mc/local_pose` | `geometry_msgs/PoseWithCovarianceStamped` | sub | `KeepLast(10)` | liveness-only (staleness check), pose itself unused here | ✅ |

### `health_monitor_node` (package `health_monitor`) — 🧭 designed only

| Topic | Type | Dir | Rate | Purpose |
|---|---|---|---|---|
| `/mc/health` | `mc_msgs/SystemHealth` | pub | ≥1 Hz | Aggregate CPU/mem/disk/GPU-temp + `fault_mask`/`node_alive_mask` |

### `swarm_coordinator_node` (package `swarm_coord`) — 🧭 designed only

| Topic | Type | Dir | Rate | Purpose |
|---|---|---|---|---|
| `/mc/swarm/peer_states` | `mc_msgs/SwarmPeerArray` | pub | 1 Hz + on change | All known peer states |
| `/mc/swarm/voronoi_cell` | `mc_msgs/VoronoiCell` | pub | on change | This drone's assigned search cell |
| `/mc/swarm/isr_role` | `mc_msgs/IsrRole` | pub | on change | ISR election outcome for this drone |

### `formation_controller_node` (package `swarm_coord`) — 🧭 designed only

Consumes `mc_msgs/FormationOffset` entries (nested in
`mc_msgs/MissionUpload.formation_offsets`) to drive per-drone formation
offsets during `INGRESS`; no standalone topic of its own is documented yet.

### `perception_down_node` (package `vision`) — 🧭 designed only

| Topic | Type | Dir | Rate | Purpose |
|---|---|---|---|---|
| `/mc/detections` | `mc_msgs/DetectionArray` | pub | event-driven | Geolocated AI detections, downward camera |

### `visual_localisation_node` (package `vision`) — 🧭 designed only

| Topic | Type | Dir | Rate | Purpose |
|---|---|---|---|---|
| `/mc/local_pose` | `geometry_msgs/PoseWithCovarianceStamped` | pub | ≥10 Hz | Visual odometry position estimate |
| `/mc/localisation_mode` | `mc_msgs/LocalisationMode` | pub | on change | GNSS↔VO handover state |
| `/mc/fcc/visual_odometry` | `mc_msgs/VisualOdometry` | pub | ≥10 Hz while active | Forwarded into `fcc_bridge_node` (see gap noted above) |

### `perception_front_node` (package `video_downlink`) — 🧭 designed only

| Topic | Type | Dir | Rate | Purpose |
|---|---|---|---|---|
| `/mc/perception/front/detections` | `mc_msgs/DetectionArray` | pub | event-driven | Front gimbal camera detections |
| `/mc/perception/front/tracks` | `mc_msgs/TrackArray` | pub | ≥5 Hz | Persistent target tracks |

### `video_mux_node` (package `video_downlink`) — 🧭 designed only

| Topic | Type | Dir | Rate | Purpose |
|---|---|---|---|---|
| `/mc/video/downlink_status` | `mc_msgs/VideoDownlinkStatus` | pub | ≥1 Hz | Stream A (gimbal) health |
| `/mc/video/down_downlink_status` | `mc_msgs/VideoDownlinkStatus` | pub | ≥1 Hz | Stream B (downward) health |

### `search_planner_node` (package `mission_exec`) — 🧭 designed only

| Topic | Type | Dir | Rate | Purpose |
|---|---|---|---|---|
| `/mc/search/coverage_path` | `nav_msgs/Path` | pub | on partition (re)computed | Boustrophedon sweep waypoints for this drone's Voronoi cell |

## `mc_msgs` message reference

All custom messages live in `ws/src/mc_msgs/msg/`. Field-level detail below
is a summary — the `.msg` files themselves carry the authoritative,
section-cited comments (ICD-MC-001 §9.2 unless noted).

| Message | Key fields | Notes |
|---|---|---|
| `VehicleState` | `lat/lon/alt_msl`, `x/y/z` (NED), `vx/vy/vz`, `heading`, `q[4]`, `nav_state`, `arming_state`, `battery_v/battery_pct/battery_warning`, `fix_type/hdop/vdop/gnss_ok`, `fcc_link_ok` | Authoritative flight state, `fcc_bridge` output |
| `FccCommand` | `cmd` (MAV_CMD), `params[7]` | Vehicle command to FCC |
| `FccMission` | `upload_id`, `items[]` (`FccMissionItem`) | Full-replace mission upload |
| `FccMissionItem` | `seq`, `frame`, `command`, `lat/lon` (1e7 deg), `alt`, `acceptance_radius` | Single PX4 mission item; also used standalone for item-replace |
| `FccMissionAck` | `request_type` (`UPLOAD`/`ITEM_UPDATE`), `upload_id`, `seq`, `success` | Ack for an upload or item-update request |
| `FccOffboardSetpoint` | `x/y/z` (NED), `yaw` | Continuous OFFBOARD position setpoint; no velocity/accel yet (deliberate initial cut) |
| `VisualOdometry` | `x/y/z`, `q[4]`, `vx/vy/vz`, `position_variance[3]`, `orientation_variance`, `reset_counter`, `quality` | GNSS-denied pose estimate, NED frame |
| `GcsCommand` | `command` (MAV_CMD), `params[7]`, `target_system`, `target_component`, `confirmation` | Decoded GCS MAVLink command |
| `GcsCommandAck` | `command`, `result` (`RESULT_ACCEPTED`/`RESULT_REJECTED`), `reason` | Outcome-level ack, distinct from `gcs_mavlink_node`'s immediate `COMMAND_ACK` |
| `LinkStatus` | `gcs_link_ok`, `last_heartbeat_us`, `link_quality`, `rx/tx_packet_count`, `rx_drop_count` | GCS↔MC link health |
| `MissionUpload` | `upload_id`, `ingress_waypoints[]`, `formation_offsets[]`, `search_polygon[]`, `swarm_manifest[]`, `arm_authorised`, `low_battery_threshold`, `offboard_phase_mask` | Full mission data from GCS or the orchestrator |
| `FormationOffset` | `drone_id`, `n/e/d` (NED offset, m) | Element of `MissionUpload.formation_offsets` |
| `MissionPhase` | `phase` (0–9, or 255=`FAULT`), `phase_name`, `phase_entered_us`, `source` (`GCS_COMMAND`/`AUTONOMOUS`/`FAILSAFE`) | Current mission state machine phase |
| `StatusText` | `severity` (MAV_SEVERITY), `text` | Human-readable status for GCS; not yet relayed as a real MAVLink `STATUSTEXT` |
| `FaultEvent` | `fault_id` (single bit, see mask constants), `severity`, `description`, `failsafe_action` | One fault occurrence |
| `SystemHealth` | `cpu_pct/mem_pct/disk_pct`, `gpu_temp_c/cpu_temp_c`, `fault_mask`, `node_alive_mask`, `uptime_s` | Aggregate health snapshot |
| `SwarmPeer` | `drone_id`, `ip_address`, `mission_phase`, `lat/lon/alt_msl`, `vx/vy/vz`, `battery_pct`, `sensor_flags` (layout TBD upstream), `fault_mask`, `peer_status`, `is_leader`, `isr_role`, `last_heartbeat_us` | One peer's last-known state |
| `SwarmPeerArray` | `peers[]` (`SwarmPeer`) | All known peers |
| `IsrRole` | `role` (`PENDING`/`ISR`/`NOT_ISR`), `score`, `coordinator_id`, `result_acked`, `all_peers_acked`, `election_round` | This drone's ISR election outcome |
| `VoronoiCell` | `drone_id`, `round`, `vertices[]` (`GeoPoint`, CCW), `upload_id` | Assigned search cell boundary |
| `LocalisationMode` | `mode` (`GNSS_PRIMARY`/`VISUAL_ODOMETRY`), `reason`, `hdop`, `fix_type` | GNSS↔VO handover state |
| `Detection` | `detection_id`, `frame_timestamp_us`, `class_id/class_name`, `confidence`, `lat/lon/alt_msl`, `pos_uncertainty_m`, `bbox_x/y/w/h` | One geolocated AI detection |
| `DetectionArray` | `detections[]` (`Detection`) | Detection batch |
| `Track` | `track_id`, `class_id/class_name`, `confidence`, `lat/lon/alt_msl`, `frames_tracked` | One persistent tracked target |
| `TrackArray` | `tracks[]` (`Track`) | Track batch |
| `VideoDownlinkStatus` | `streaming`, `bitrate_kbps`, `client_connected` | Per-stream downlink health |

Two messages (`IsrRole`, `VisualOdometry`) are marked in their own source
comments as **gap-fills**: the ICD's topic table cited them before a
message body existed anywhere in `ICD-MC-001.md`, so they were authored
directly in `mc_msgs` (2026-07-26) and still need ICD-owner
review/ratification.

## External message types in use

| Type | Used by | Purpose |
|---|---|---|
| `px4_msgs/*` | `fcc_bridge` (`XrceDdsBackend`) | PX4 uXRCE-DDS topics, vendored — see `scripts/05-build-workspace.sh` |
| `std_msgs/Header` | `mission_manager_node` (`/mc/manager/heartbeat`) | Liveness heartbeat; also embedded as `header` in every `mc_msgs` message |
| `nav_msgs/Path` | `search_planner_node` → `mission_manager_node` | Boustrophedon coverage waypoints |
| `geometry_msgs/PoseWithCovarianceStamped` | `visual_localisation_node` → `mission_manager_node` | Visual odometry pose estimate |
| `geometry_msgs/PoseStamped` | nested in `mc_msgs/MissionUpload.ingress_waypoints` | Ingress waypoint list |
| `geographic_msgs/GeoPoint` | nested in `mc_msgs/MissionUpload.search_polygon`, `mc_msgs/VoronoiCell.vertices` | Geodetic polygon/cell vertices |

## Where to go next

- New to the repo? Start at `docs/GETTING-STARTED.md` and `docs/ARCHITECTURE.md`.
- Need the MAVLink/uXRCE-DDS wire-level contracts these topics eventually
  reach? `ICD-MC-001.md`.
- Need the algorithms that produce/consume this data (Voronoi, boustrophedon,
  ISR election, VO)? `SDD-MC-001.md` §3.x.
