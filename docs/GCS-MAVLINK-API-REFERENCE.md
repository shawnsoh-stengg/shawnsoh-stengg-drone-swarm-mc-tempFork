# GCS↔MC MAVLink API Reference

External MAVLink interface reference for the two GCS-facing channels that terminate in `ws/src/gcs_comms/`: **Channel B** (GCS operator commands and telemetry, `gcs_mavlink_node`) and **Channel C** (Mission Orchestrator's mission-plan library, `mission_orchestrator_node`). Grounded in the current code, not just `ICD-MC-001.md`.

**Channel A** (GCS↔FCC, standard PX4/QGroundControl autopilot protocol) is out of scope — see `CLAUDE.md`'s "MAVLink channel separation" note and `ICD-MC-001.md` for that channel. For the internal ROS 2 graph these two nodes feed into/out of (`/mc/gcs/*`, `/mc/health`, `/mc/detections`, etc.), see `docs/API-REFERENCE.md`.

## Implementation status

- ✅ **implemented** — the message/command is actually sent, received, or acted on in code today.
- 🧭 **designed** — documented in `ICD-MC-001.md` as part of this interface, not yet wired up.

Channel B (`gcs_mavlink_node`) implements the full custom telemetry/status set (`MC_STATUS`/`MC_DETECTION`/`MC_HEALTH`), the mission-upload protocol, and three of four custom commands end-to-end; a handful of ICD-designed standard messages (`COMMAND_INT`, inbound `MISSION_ACK`, `PARAM_*`, `STATUSTEXT`) have no code behind them yet. Channel C
(`mission_orchestrator_node`) is fully implemented (GCS-004, done 2026-08-05/06) except MAVLink 2 packet signing, which the spec requires but the code doesn't yet apply.

---

## Channel B — GCS ↔ MC (`gcs_mavlink_node`, package `gcs_comms`)

### Transport

- MAVLink 2 over UDP, routed by its own `mavlink-router` instance/config (`mavlink-router-channel-b.conf`) — isolated from Channel A and Channel C.
- GCS laptop: `192.168.10.10`. Port `14551` is the GCS's own receive port (MC→GCS — `mavlink-router`'s `GCS_CHANNEL_B` endpoint, a fixed remote address:port). Port `14552` is the MC's own receive port (GCS→MC — `GCS_MC_RECV`, bound `0.0.0.0:14552` on the MC). `ICD-MC-001.md`'s §11.2/ §11.4 summary tables previously had this backwards; §4.1/§4.2's own `mavlink-router-channel-b.conf` snippet, the real `config/mavlink-router-channel-b.conf`, and `scripts/09b-smoke-test-routing.sh` all agree with the direction stated here. - `gcs_mavlink_node` itself listens on a loopback socket `GCS_MAVLINK_LOOPBACK_PORT` (env, default `14560`), bound to `INADDR_LOOPBACK` — `mavlink-router` is the only process with a real network-facing socket for this channel.
- MC `system_id` = the drone's numeric ID; `component_id = 191` (`MAV_COMP_ID_ONBOARD_COMPUTER`). GCS operator `system_id = 255`.

### Messages: GCS → MC

| Message | ID | Rate/Trigger | Status |
|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz | ✅ |
| `COMMAND_LONG` | 76 | on operator command | ✅ (generic passthrough — see below) |
| `MISSION_COUNT` | 44 | mission-upload start | ✅ |
| `MISSION_ITEM_INT` | 73 | one per mission item, on `MISSION_REQUEST_INT` | ✅ |
| `COMMAND_INT` | 75 | — | 🧭 |
| `MISSION_ACK` | 47 | inbound (GCS aborting an upload) | 🧭 |
| `PARAM_SET` | 23 | — | 🧭 |
| `PARAM_REQUEST_LIST` | 21 | — | 🧭 |

### Messages: MC → GCS

| Message | ID | Rate/Trigger | Status |
|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz, `type=18` (`MAV_TYPE_ONBOARD_CONTROLLER`) | ✅ |
| `COMMAND_ACK` | 77 | on every `COMMAND_LONG` (≤500 ms, FR-061/FR-003) | ✅ |
| `MISSION_REQUEST_INT` | 51 | mission-upload flow | ✅ |
| `MISSION_ACK` | 47 | outbound, ends a mission-upload sequence | ✅ |
| `MC_STATUS` | 50100 | ≥2 Hz (FR-062) | ✅ |
| `MC_DETECTION` | 50101 | event-driven, ≤1 s (FR-043) | ✅ |
| `MC_HEALTH` | 50103 | ≥1 Hz (FR-071), event-driven on `/mc/health` | ✅ |
| `PARAM_VALUE` | 22 | — | 🧭 |
| `STATUSTEXT` | 253 | severity-tagged text (8 levels, EMERGENCY→DEBUG) | 🧭 (no send site in code yet) |

### `COMMAND_LONG` handling

`gcs_mavlink_node` does **generic passthrough**, matching `fcc_bridge_node`'s "zero business logic" philosophy: any `COMMAND_LONG` addressed to this drone's `system_id`/`component_id=191` is decoded into `mc_msgs/GcsCommand`, published on `/mc/gcs/command`, and unconditionally ACKed `MAV_RESULT_ACCEPTED`. It does not special-case individual command codes — actual command semantics live downstream, split across two consumers:

**`mission_manager_node`** (`command_arbiter.{hpp,cpp}` classifies tier;
`mission_manager_node.cpp` implements the effect):

| Command | ID | Tier | Params | Effect | Status |
|---|---|---|---|---|---|
| `MC_CMD_PHASE_TRANSITION` | 50001 | PHASE | param1 = target phase (§ below) | requests a mission-phase transition | ✅ |
| `MC_CMD_ARM_AUTHORISE` | 50002 | PHASE | param1 = mission-upload ID | authorises arming for that upload | ✅ |
| `MC_CMD_END_MISSION` | 50003 | PHASE | — | ends the active mission | ✅ |
| `MC_CMD_GCS_OVERRIDE` | 50004 | PHASE | param1 = enable (1/0), param2 = override type | received/logged here; **effect implemented in `health_monitor_node`**, not here (see below) | ✅ (split) |
| `MAV_CMD_COMPONENT_ARM_DISARM` | 400 | EMERGENCY (disarm only) | param1 = 0 | forces FAULT phase, bypasses queue | ✅ |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | 20 | EMERGENCY | — | forces FAULT phase, bypasses queue | ✅ |
| `MAV_CMD_DO_SET_MODE` | 176 | EMERGENCY (for PX4 `custom_main` ∈ {MANUAL, ALTCTL, POSCTL, STABILIZED}) | param2 = PX4 custom_main | forces FAULT phase, bypasses queue | ✅ |

`MC_CMD_PHASE_TRANSITION`/`MC_CMD_ARM_AUTHORISE`/`MC_CMD_END_MISSION` are "PHASE" tier — queued through the normal transition table. The `EMERGENCY` tier (disarm, RTL, or a manual-flight-mode `DO_SET_MODE`) bypasses the queue entirely and forces `FAULT` directly, so GCS override authority is never blocked behind a pending phase transition (FR-003's 500 ms response bound).

**`health_monitor_node`** (per `docs/API-REFERENCE.md`'s own row for this node): consumes `/mc/gcs/command`'s `MC_CMD_GCS_OVERRIDE` to drive `OVERRIDE_LOW_BATTERY_RTL`/`OVERRIDE_GCS_LINK_HOLD` state (ICD §10.7) — the
one command whose *effect* lives outside `mission_manager_node`. This split is deliberate, not a gap: `mission_manager_node` only needs to know the override was requested, not fold battery/link-loss failsafe logic into its own state machine.

Custom command IDs (50001-50004) are plain `uint32` values carried in `COMMAND_LONG.command` — they are not separate MAVLink messages, so they have no `<message>` entry in `mc_mavlink_defs.xml`.

### Custom messages (`mc_mavlink_defs.xml`, IDs 50100-50105)

**`MC_STATUS`** (50100, MC→GCS):

| Field | Type | Notes |
|---|---|---|
| `time_usec` | uint64 | UTC, µs |
| `lat`, `lon` | int32 | degE7 |
| `alt_msl` | int32 | mm |
| `heading` | uint16 | cdeg, 0-36000 |
| `groundspeed` | uint16 | cm/s |
| `climb_rate` | int16 | cm/s, positive = up |
| `battery_remaining` | uint8 | percent, 0-100 |
| `mission_phase` | uint8 | enum `MC_MISSION_PHASE` |
| `localisation_mode` | uint8 | enum `MC_LOCALISATION_MODE` (GNSS / VISUAL_ODOMETRY) |
| `fault_mask` | uint16 | bitmask, see `docs/API-REFERENCE.md`'s `health_monitor_node` row |
| `peer_count` | uint8 | swarm peers currently ALIVE |
| `isr_role` | uint8 | 0=non-ISR, 1=ISR |

**`MC_DETECTION`** (50101, MC→GCS, one message per `mc_msgs/DetectionArray` element):

| Field | Type | Notes |
|---|---|---|
| `time_usec` | uint64 | UTC, µs, frame-capture time |
| `detection_id` | uint32 | monotonically increasing counter |
| `class_id` | uint16 | detection class label ID |
| `confidence` | uint16 | ×10000 (9500 = 0.95) |
| `lat`, `lon` | int32 | degE7, estimated geodetic position |
| `alt` | int32 | mm, MSL |
| `pos_uncertainty` | uint16 | cm, 1-sigma horizontal |
| `drone_id` | uint8 | reporting drone |
| `class_name` | char[24] | null-terminated label string |

**`MC_HEALTH`** (50103, MC→GCS, event-driven on `/mc/health`):

| Field | Type | Notes |
|---|---|---|
| `time_usec` | uint64 | UTC, µs |
| `cpu_load`, `mem_used`, `disk_used` | uint8 | percent |
| `gpu_temp`, `cpu_temp` | uint8 | °C |
| `fault_mask` | uint16 | bitmask (ICD §10.8) |
| `node_alive_mask` | uint16 | one bit per monitored node (ICD §10.9) |
| `uptime_s` | uint32 | MC uptime, seconds |

**`MC_MISSION_UPLOAD` (ID slot 50102) is not a real wire message.** It has no `<message>` entry in `mc_mavlink_defs.xml` — the ICD uses the name only to label the logical operation. The actual upload rides the **standard** mission protocol (`MISSION_COUNT` → `MISSION_REQUEST_INT` → one `MISSION_ITEM_INT` per item → `MISSION_ACK`, `MAV_MISSION_TYPE=3`), with
item semantics distinguished by the `command` field of each `MISSION_ITEM_INT` (item-command IDs, below).

### Mission-upload item encoding (`MISSION_ITEM_INT.command`)

| Command | ID | Payload | Notes |
|---|---|---|---|
| Ingress waypoint | 50200 | lat, lon, alt, speed | |
| Muster point | 50201 | lat, lon, alt, speed | folded into `ingress_waypoints`, no dedicated field |
| Formation offset | 50202 | param1 = drone_id, N/E offset (mm-scaled int32 ×1000), D offset (native float) | |
| Search-area vertex | 50203 | param1 = vertex_index, param2 = total_vertices, lat/lon | |
| Swarm peer entry | 50204 | drone_id, IP octets 3–4 (reconstructed as `192.168.<oct3>.<oct4>`) | |

Implemented (GCS-002, 2026-07-26) via `handle_mission_count()` / `handle_mission_item_int()`. Known gaps: `arm_authorised` is always `false` at upload time — set later via `MC_CMD_ARM_AUTHORISE` — and `low_battery_threshold`/`offboard_phase_mask` aren't overridable at upload (default to 0.20 / 0).

### Timing and failsafe behavior

| Condition | Bound | Action | Requirement |
|---|---|---|---|
| GCS link silent | > 10 s (`gcs_link_timeout_s`, default 10.0) | HOLD + `GCS_LINK_LOST` fault | FR-073 / SFR-007 |
| `COMMAND_LONG` → ACK | ≤ 500 ms | — | FR-061 / FR-003 |
| Mission-item silence | > 1 s (`mission_request_retry_s`, default 1.0) | resend `MISSION_REQUEST_INT` | — |
| Mission-upload total silence | > 5 s (`mission_upload_timeout_s`, default 5.0) | abort, `MAV_MISSION_ERROR` | — |
| Low battery | — | RTL unless `MC_CMD_GCS_OVERRIDE` active | FR-072 / SFR-008 |

Params confirmed in `ws/src/gcs_comms/params/default.yaml`.

---

## Channel C — Mission Orchestrator ↔ MC (`mission_orchestrator_node`, package `gcs_comms`)

**Status: implemented** (GCS-004, done 2026-08-05/06). This section summarizes the as-built interface; `MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md` is the full authoritative spec.

### Transport

- A third, fully isolated MAVLink channel — separate `mavlink-router` config from both Channel A and Channel B.
- Ports: `14571` UDP is the Orchestrator's own receive port (MC→Orchestrator — `ORCHESTRATOR_CHANNEL_C`, a fixed remote address:port); `14572` UDP is the MC's own receive port (Orchestrator→MC — `ORCHESTRATOR_MC_RECV`, bound `0.0.0.0:14572` on the MC); `14580` UDP is the loopback from `mavlink-router` to `mission_orchestrator_node` (`ORCHESTRATOR_MAVLINK_LOOPBACK_PORT`, default `14580`). Same direction convention as Channel B — see its transport note above.
- Orchestrator is assumed co-located with the GCS laptop, `192.168.10.10`.
- MC `component_id = 191` (same as Channel B). Orchestrator `system_id = 255`, `component_id = 190` (`MAV_COMP_ID_MISSIONPLANNER`).
- 🧭 **MAVLink 2 packet signing is required by the spec but not yet implemented** — open item.

### Custom dialect additions (`mc_mavlink_defs.xml`)

| Message | ID | Direction | Fields | Status |
|---|---|---|---|---|
| `MC_MISSION_PLAN_ACTIVATE` | 50104 | Orchestrator→MC | `client_request_id` (uint32), `filename` (char[64]) | ✅ |
| `MC_MISSION_PLAN_ACK` | 50105 | MC→Orchestrator | `client_request_id`, `filename`, `result` (uint8), `error_message` (char[50]) | ✅ |

`result` values: `0=ACCEPTED`, `1=REJECTED_INVALID_JSON`,
`2=REJECTED_NOT_FOUND`, `3=REJECTED_PHASE`.

### Hand-decoded framing — why this node doesn't use `mavlink_parse_char()`

`mission_orchestrator_node` decodes the raw MAVLink v2 header directly off each UDP datagram (`decode_raw_v2_frame()`) instead of calling into the compiled dialect's `mavlink_parse_char()`. The compiled dialect's generated CRC table has no entry for `MC_MISSION_PLAN_ACTIVATE`(50104), so `mavlink_parse_char()` would silently drop the frame as a checksum failure rather than fail to compile — this applies to `FILE_TRANSFER_PROTOCOL`(110) too. Each `recvfrom()` call returns exactly one complete frame, which is what makes hand-decoding safe here.

### MAVFTP mission-plan library (`FILE_TRANSFER_PROTOCOL`, ID 110)

Virtual filesystem root: `/var/drone-mc/mission_plans/` (param `plans_dir`), flat namespace, single session. Server implementation (`ftp_dispatch()`) is adapted from PX4's `mavlink_ftp.cpp` design, not a literal port. Max payload 239 bytes (`kFtpMaxData`).

| Opcode | ID | Operation | Result |
|---|---|---|---|
| `CreateFile` | 6 | new plan | `Nak(FileExists)` if the name is already taken |
| `OpenFileWO` / `WriteFile` | 11 / 7 | update existing plan | `Nak(FileNotFound)` if absent; full truncate-replace on `TerminateSession` |
| `ListDirectory` | 3 | enumerate plans | one entry per plan file, plus a synthetic read-only `/_active` |
| `OpenFileRO` / `ReadFile` | 4 / 5 | read a plan | `Nak(FileNotFound)` if absent |
| `RemoveFile` | 8 | delete a plan | `Nak(FileProtected)` if it's the currently active plan |
| `TerminateSession` / `ResetSessions` | 1 / 2 | session control | — |
| `Ack` / `Nak` | 128 / 129 | responses | — |

JSON is validated only after a full transfer completes (MAVFTP has no mid-transfer reject hook) — an invalid upload stays on disk with a `STATUSTEXT` warning; `activate` is what actually rejects it.

### Mission plan JSON format

One `<name>.json` file per plan, fields mirror `mc_msgs/MissionUpload` 1:1:

```
name, created_at, updated_at,
ingress_waypoints: [{lat, lon, alt_m, speed_mps}],
formation_offsets: [{drone_id, north_m, east_m, down_m}],
search_polygon:    [{lat, lon}],
swarm_manifest:    [{drone_id, ip}],
arm_authorised, low_battery_threshold, offboard_phase_mask
```

### Activate flow

`MC_MISSION_PLAN_ACTIVATE` → MC loads and validates the named plan's JSON → converts to `mc_msgs/MissionUpload` (fresh `upload_id`) → publishes `/mc/gcs/mission_upload` — **the same topic and message type `gcs_mavlink_node` already produces**, so `mission_manager_node` needs zero changes to consume it — → updates `_active` tracking → replies`MC_MISSION_PLAN_ACK`.

**Phase-gated to `PREFLIGHT` only** (resolved 2026-08-06). No `HOLD` phase exists in the implemented mission state machine, so `PREFLIGHT` is the only pre-arm window available to gate on. A request outside `PREFLIGHT` isrejected with `result = REJECTED_PHASE`.

### Open items

- MAVLink 2 signing (spec requirement, not yet implemented).
- No SRS-MC-001.md requirements exist yet for Channel C (spec's own open item — Channel B's commands/telemetry trace to FR-060–064/FR-071–073/IR-010–014; Channel C has no equivalent FR/IR tags to cite).

---

## Requirement traceability

| Requirement | Applies to |
|---|---|
| FR-003 | GCS override / command-response authority, ≤500 ms |
| FR-043 | Detection reporting to GCS, ≤1 s |
| FR-060–064 | Mission data storage, command ACK timing, ≥2 Hz status, event-driven detections, dedicated channel (no Channel A injection) |
| FR-071–073 | Health ≥1 Hz, RTL on low battery, HOLD on GCS-link-loss >10 s |
| SFR-007 | GCS-link-loss → HOLD failsafe |
| SFR-008 | Low-battery → RTL failsafe |
| SFR-009 | OFFBOARD-stream-staleness (Channel-B-adjacent, via `health_monitor_node`) |
| IR-007 | MC→FCC mission-protocol precedent that Channel B's mission upload and Channel C's MAVFTP channel both follow (dedicated port, not routed by `mavlink-router`'s cross-channel routing) |
| IR-010–014 | Dedicated channel, 1 Hz heartbeat w/ system_id/component_id, mavlink-router as sole ingress/egress, `COMMAND_LONG`/mission-item encoding |

Note on IR-014: it recommends MAVROS for GCS-facing services — in practice `gcs_mavlink_node` is raw MAVLink (not MAVROS); see `CLAUDE.md`'s "MAVROS was never adopted" callout. Channel C has no SRS requirements yet (open item, see above).
