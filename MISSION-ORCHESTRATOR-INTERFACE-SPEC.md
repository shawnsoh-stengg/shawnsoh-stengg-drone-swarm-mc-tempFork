# Mission Orchestrator ↔ MC Interface Specification (Proposal)

Interface contract for the Mission Orchestrator app (built by another team,
**out of scope for this project to implement**) to create, store, update,
activate, and delete a library of named mission plans on each drone's
Mission Computer. This project's deliverable is this specification — the
workflow and wire contract the other team builds against — plus the
MC-side implementation that serves it (tracked as `GCS-004` in
`PROJECT-PLAN-5-MONTH.md`, not yet implemented).

**Status: proposal, not yet applied.** Like `FCC-MISSION-CHANNEL-BACKLOG.md`
and `OFFBOARD-MODE-SWITCHING-BACKLOG.md` before their tickets landed, this
doc's ICD/SDD/SRS deltas (§7) describe what would need to change, not
changes already made. Nothing here has been implemented — no new `mc_msgs`
types, no new node, no `zenoh-bridge-dds.json5` change.

## 1. Why this doc exists

`GCS-ZENOH-DESIGN-REVIEW.md` established that "GCS" is two separate actors
— QGroundControl (vehicle telemetry/control) and a Mission Orchestrator app
(mission plan creation/upload/update) — and that the Orchestrator, unlike
QGC, has no MAVLink-compatibility constraint. Following that review's
conclusion, three decisions were made 2026-07-22 that this spec is built
on:

1. **Transport:** Zenoh, reusing `mc_msgs` schemas directly (not a second
   custom-dialect wire format like `mc_mavlink_defs.xml`), riding the same
   `zenoh-bridge-dds` mesh already built for cross-host MC↔MC traffic
   (`ZBD-001`).
2. **Network path:** the Orchestrator runs on the GCS laptop/site alongside
   QGC, reaching each MC over the existing `192.168.10.x` MANET link on its
   own dedicated channel — independent of QGC's MAVLink Channel B.
3. **Plan model:** a *library* of named, stored mission plans (create,
   list, update, activate, delete) — not today's single-active-mission
   model, which only supports "upload replaces the current mission."

## 2. Scope

**In scope (this project delivers):**
- This interface specification.
- MC-side implementation: a new node, new `mc_msgs` types, plan-library
  persistence, and the `zenoh-bridge-dds.json5` allow-list change needed to
  expose it.

**Out of scope (the other team delivers):**
- The Mission Orchestrator application itself — its UI, its plan-authoring
  logic, its own Zenoh client implementation.

**Unaffected by this doc:**
- QGC's MAVLink channel (vehicle telemetry/control) — untouched.
- The MC→FCC mission protocol (`ICD-MC-001.md` §3.2/§3.3) — still standard
  MAVLink, because stock PX4 has no DDS-writable mission topic. `activate`
  (§5.4 below) still ends by producing exactly the same
  `mc_msgs/MissionUpload` that `gcs_mavlink_node` produces today, which
  `fcc_bridge_node` forwards to the FCC exactly as it does today. **No
  change to the MC→FCC leg at all.**

## 3. Network and transport

The Orchestrator does not need its own `zenoh-bridge-dds` instance — that
component's job is bridging *local ROS 2/DDS* topics onto the Zenoh mesh
for processes that are themselves DDS participants. The Orchestrator is
not a ROS 2 node; it only needs a plain Zenoh session (any Zenoh client
library — Python, C, Rust, etc.) configured to `connect` directly to each
target drone's already-open listen endpoint:

```
Orchestrator's connect list: tcp/192.168.10.11:7447 (Drone 1)
                              tcp/192.168.10.12:7447 (Drone 2)
                              tcp/192.168.10.13:7447 (Drone 3)
                              ... one entry per drone in the swarm
```

No change to each drone's `zenoh-bridge-dds.json5` `listen` block is
needed — `tcp/0.0.0.0:7447` already accepts inbound connections (see
`config/zenoh-bridge-dds.json5`). The **only** required change is widening
that file's `allow` list (currently `["^$"]`, forwarding nothing) to
include the new topics introduced in §4 — see §7.

**Multi-drone fan-out (flagged, not fully validated):** today's single
`mc_msgs/MissionUpload` already carries a `swarm_manifest` covering every
drone, and each drone's own `mission_manager_node` is assumed to select its
own entry by `DRONE_ID` (ICD §9.2's `MissionUpload` fields). This spec
assumes the same model: the Orchestrator connects to *every* drone's Zenoh
endpoint and activates the *same* stored plan on each identically, mirroring
how a single GCS mission upload today reaches every drone with the same
payload. This is inferred from the existing `MissionUpload` design, not
independently confirmed against `swarm_coord`'s (still-unimplemented) peer
logic — flag for confirmation before implementation.

**Security (flagged, not solved here):** `zenoh-bridge-dds.json5` has no
authentication/access-control configuration today (SITL-appropriate for a
"forwards nothing" bridge). This interface changes that — it's now a
mission-modifying channel reachable from a separate team's application
across an organizational boundary. A security review (TLS, Zenoh ACLs, or
another mechanism) is required before this ships; not attempted in this
spec.

## 4. Message schema (new `mc_msgs` types)

All new types added to `ws/src/mc_msgs/msg/`, registered in
`mc_msgs/CMakeLists.txt`'s `rosidl_generate_interfaces()` call, following
the existing one-type-per-purpose convention (`FccCommand`,
`FccMissionAck`, `FccOffboardSetpoint` are each separate small types
today — no change to that convention).

Every request carries a `client_request_id` (`uint32`), mirroring the
existing `upload_id` correlation-ID idiom `mc_msgs/FccMission`/
`FccMissionAck` already use — the Orchestrator generates it, the ack echoes
it back, exactly like today's FCC mission-handshake pattern. Every
mutating request+ack pair is a **full-replace**, not a partial-field patch
— consistent with how `MissionUpload`/`FccMission` already work.

```
mc_msgs/MissionPlan
  string plan_id                       # server-assigned, empty on create
  string name
  uint64 created_at                    # UTC microseconds
  uint64 updated_at
  geometry_msgs/PoseStamped[] ingress_waypoints
  mc_msgs/FormationOffset[] formation_offsets
  mc_msgs/GeoPoint[] search_polygon
  mc_msgs/SwarmPeer[] swarm_manifest
  bool arm_authorised
  float32 low_battery_threshold
  uint16 offboard_phase_mask
  # identical payload fields to mc_msgs/MissionUpload (ICD §9.2) plus
  # plan_id/name/created_at/updated_at for library storage — deliberately
  # NOT a new schema, a superset of the existing one.

mc_msgs/MissionPlanSummary
  string plan_id
  string name
  uint64 created_at
  uint64 updated_at
  uint16 waypoint_count

mc_msgs/MissionPlanCreateRequest
  uint32 client_request_id
  mc_msgs/MissionPlan plan              # plan_id ignored, server assigns

mc_msgs/MissionPlanUpdateRequest
  uint32 client_request_id
  string plan_id
  mc_msgs/MissionPlan plan              # full replace of stored plan

mc_msgs/MissionPlanActivateRequest
  uint32 client_request_id
  string plan_id

mc_msgs/MissionPlanDeleteRequest
  uint32 client_request_id
  string plan_id

mc_msgs/MissionPlanListRequest
  uint32 client_request_id

mc_msgs/MissionPlanListResponse
  uint32 client_request_id
  mc_msgs/MissionPlanSummary[] plans

mc_msgs/MissionPlanAck
  uint32 client_request_id
  string plan_id
  bool success
  string error_message                  # empty on success
```

`MissionPlanAck` is reused across create/update/activate/delete — same
"one ack type, correlation ID ties it to the request" pattern
`FccMissionAck` already establishes for the MC→FCC leg.

## 5. Topics and workflow

New topics, namespaced under `/mc/orchestrator/` (parallel to the existing
`/mc/gcs/*`/`/mc/fcc/*` namespacing convention):

| Topic | Message | Direction |
|---|---|---|
| `/mc/orchestrator/plan_create_request` | `MissionPlanCreateRequest` | Orchestrator → MC |
| `/mc/orchestrator/plan_update_request` | `MissionPlanUpdateRequest` | Orchestrator → MC |
| `/mc/orchestrator/plan_activate_request` | `MissionPlanActivateRequest` | Orchestrator → MC |
| `/mc/orchestrator/plan_delete_request` | `MissionPlanDeleteRequest` | Orchestrator → MC |
| `/mc/orchestrator/plan_list_request` | `MissionPlanListRequest` | Orchestrator → MC |
| `/mc/orchestrator/plan_ack` | `MissionPlanAck` | MC → Orchestrator |
| `/mc/orchestrator/plan_list_response` | `MissionPlanListResponse` | MC → Orchestrator |

Owned by a **new node, `mission_orchestrator_node`**, in the `gcs_comms`
package (sibling to `gcs_mavlink_node` — same package charter, same
"GCS-facing translation" role, just a different transport/actor).

### 5.1 Create
Orchestrator publishes `MissionPlanCreateRequest`. `mission_orchestrator_node`
validates the payload (same field-presence checks `mission_manager_node`'s
`pre_arm_checks()` already implies are needed — waypoints, formation
offsets, search polygon, swarm manifest all present and consistent),
assigns a new `plan_id`, persists to
`/var/drone-mc/mission_plans/<plan_id>.yaml` (same YAML-file idiom
`mission_manager_node` already uses for its own single-mission
persistence, just one file per plan instead of one file total), and
replies with `MissionPlanAck`.

### 5.2 List
Orchestrator publishes `MissionPlanListRequest`. Node scans
`/var/drone-mc/mission_plans/`, replies with `MissionPlanListResponse`
(summaries only — full plan content requires activating or a future
`plan_get_request`, not specified here since it wasn't asked for).

### 5.3 Update
Orchestrator publishes `MissionPlanUpdateRequest`. Node validates,
overwrites the stored YAML file (full replace, no partial-item patching —
consistent with §4), replies with `MissionPlanAck`.

### 5.4 Activate
Orchestrator publishes `MissionPlanActivateRequest`. Node loads the stored
plan, converts it into `mc_msgs/MissionUpload` (assigning a fresh
`upload_id` per the existing convention), and publishes to
`/mc/gcs/mission_upload` — **the exact topic `gcs_mavlink_node` already
publishes to today.** `mission_manager_node` requires **zero changes**: it
doesn't know or care whether a `MissionUpload` came from the MAVLink path
or this new one. `mission_orchestrator_node` replies with `MissionPlanAck`
once published.

**Open item, needs a product decision:** today's `/mc/gcs/mission_upload`
consumption is event-driven with no documented phase guard (SDD §4.1/§4.1a)
— nothing currently stops an upload from arriving mid-mission. This spec
does not define whether `activate` should be restricted to
PREFLIGHT/HOLD-only, how it interacts with GCS override authority (FR-003,
500 ms response requirement), or what happens if `mission_manager_node`
is mid-phase when a new plan activates. Recommend restricting `activate`
to PREFLIGHT/HOLD in `mission_manager_node`, matching how a normal mission
upload naturally happens before ARMING today — but this needs confirmation
from whoever owns `mission_manager_node`'s state machine, not assumed here.

### 5.5 Delete
Orchestrator publishes `MissionPlanDeleteRequest`. Node removes the stored
YAML file and replies with `MissionPlanAck` — **fails** (`success=false`)
if `plan_id` matches the currently-active mission (exact "currently
active" check TBD — likely comparing against the last-activated `plan_id`
the node itself tracked, not re-deriving it from `mission_manager_node`,
to avoid a new cross-node dependency).

## 6. Persistence

New directory `/var/drone-mc/mission_plans/`, one YAML file per plan
(`<plan_id>.yaml`), owned exclusively by `mission_orchestrator_node` — no
other node reads or writes this directory. This is additive: it does not
touch `mission_manager_node`'s existing single-file
`/var/drone-mc/mission_upload.yaml` persistence (SDD §4.5), which continues
to work exactly as it does today regardless of which node produced the
`MissionUpload` that landed there.

## 7. Deltas to apply (not yet applied)

- **`ws/src/mc_msgs/`**: 8 new `.msg` files (§4), `CMakeLists.txt` entries.
- **`ws/src/gcs_comms/`**: new `mission_orchestrator_node.{hpp,cpp}`.
- **`config/zenoh-bridge-dds.json5`**: widen `allow` from `["^$"]` to
  include the new topics, e.g. `["rt/mc/orchestrator/.*"]` (DDS topic names
  carry rmw's `rt/` prefix — matches the pattern the file's own comment
  already anticipated: *"Extend this list once a real cross-host DDS topic
  exists"*).
- **`ICD-MC-001.md`**: new IFD section for this interface (next free IFD
  number), §9.2 message definitions for the 8 new types, env var/config
  reference for `/var/drone-mc/mission_plans/`.
- **`SDD-MC-001.md`**: `mission_orchestrator_node` design section
  (mirrors `gcs_mavlink_node`'s §11.2 structure), persistence section
  update (§4.5 area).
- **`SRS-MC-001.md`**: new requirement(s) for plan-library CRUD, referencing
  this spec.
- **`PROJECT-PLAN-5-MONTH.md`**: `GCS-004` ticket (added, this doc) tracks
  the MC-side implementation above. Not yet started.
- **Diagrams**: `08-icd-interface-topology`, `09-icd-message-map`,
  `02-software-architecture` would each need the new interface/node added,
  once implementation is scheduled — not done in this proposal pass.

## 8. Open items requiring confirmation before implementation

1. Multi-drone fan-out model (§3) — confirm against `swarm_coord`'s actual
   (currently unimplemented) peer/manifest logic once it lands.
2. Security/access control for the Orchestrator's Zenoh session (§3) —
   needs a dedicated review, not solved here.
3. `activate`'s phase-gating rules (§5.4) — needs a decision from whoever
   owns `mission_manager_node`.
4. Whether "delete the active plan" should be a hard reject or should
   trigger some other behavior (§5.5).
5. Whether a `plan_get_request` (fetch one full stored plan, not just
   summaries) is needed — not requested, not specified, easy to add later
   if needed.
