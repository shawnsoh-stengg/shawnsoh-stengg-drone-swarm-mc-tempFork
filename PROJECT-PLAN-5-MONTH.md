# Swarm Search Drone MC — 5-Month Implementation Plan

Ticket backlog for carrying `swarm-drones` from its current state (design
complete, `fcc_bridge` mostly implemented, all other packages scaffolded but
empty) to a SITL-testable, deployable system. Grounded in a fresh audit of
`ws/src/` as of 2026-07-12, not just the design docs — see "Current State"
below for what's actually built vs. documented.

**Story points:** 1 pt = 4 work hours. 1 day = 8 hours = 2 pts.

**Scope note:** this doc does not assign tickets to specific weeks/months —
sizing is left as story points only, sequenced by dependency order (grouped
into phases). Total point count is summarized at the end for reference.

**Ticket ID prefixes:** `FCC-*` (fcc_bridge), `ME-*` (mission_exec), `SC-*`
(swarm_coord), `VIS-*` (vision), `VU-*` (video_uplink), `GCS-*` (gcs_comms),
`MAVR-*` (mavlink-router), `ZBD-*` (zenoh-bridge-dds), `HM-*`
(health_monitor), `DEP-*` (deployment/Docker/config), `TEST-*` (test
infrastructure). Continues the informal `FCC-XXX` convention already used in
`fcc_bridge` code comments.

**Update (2026-07-13):** the gimbal camera + GCS video uplink feature
(HID-MC-001 HIF-007, ICD-MC-001 IFD-006/IFD-007, SRS-MC-001 IR-040–042/
FR-080–084, SDD-MC-001 §9a) landed as design docs plus a constructor-only
`video_uplink` package scaffold (`perception_front_node`, `video_mux_node`).
This revision adds the corresponding backlog (Track E, `VU-*`) and rolls the
`detection_node`→`perception_down_node` rename through the `vision` row
below. Everything else in this plan is unchanged from the 2026-07-12 audit.

**Update (2026-07-22):** CycloneDDS's cross-host unicast discovery (eth0 +
static `<Peers>` list) is replaced by a new standalone `zenoh-bridge-dds`
daemon — a static-peer Zenoh mesh better suited to the lossy,
bandwidth-constrained MANET link than DDS's own discovery protocol.
CycloneDDS itself is unchanged for same-host (`lo`) discovery, which is all
that's actually in use today (no cross-host DDS topic exists yet — see
`ZBD-001` below). This adds Track F (`ZBD-*`), structured the same way as
`MAVR-*`/`DEP-003` since `zenoh-bridge-dds` is architecturally the same kind
of thing as `mavlink-router`: a standalone non-ROS daemon with its own build
ticket plus a matching Dockerfile ticket. **Both `ZBD-001` and `DEP-006`
below are already implemented** (not just planned) as of this update.

**Update (2026-07-22, later same day):** clarified that "GCS" is two
separate actors — QGroundControl (vehicle telemetry/control, unchanged,
still MAVLink) and a **Mission Orchestrator** app (mission plan
create/upload/update, built by another team, out of scope to implement).
This project's deliverable for the latter is the interface spec plus the
MC-side implementation serving it — see
`MISSION-ORCHESTRATOR-INTERFACE-SPEC.md` and new ticket `GCS-004` below
(Zenoh-native, reuses `mc_msgs`, rides the `ZBD-001` mesh). Not yet
started — see that spec's §8 for open items blocking implementation.
`GCS-ZENOH-DESIGN-REVIEW.md` has the full analysis of why this traffic
moves to Zenoh while QGC's traffic stays MAVLink.

---

## Current State (as of this audit)

| Package | Status |
|---|---|
| `fcc_bridge` | **Mostly done.** `FccBackend`/`XrceDdsBackend`/`fcc_bridge_node` implemented (FCC-001–005: factory, backend, mission forwarding all wired and tested). Gaps: ack feedback (FCC-006), SITL backend (FCC-007), DDS-agent lifecycle (FCC-008) — see `FCC-MISSION-CHANNEL-BACKLOG.md`. |
| `mc_msgs` | **Partial.** 5 of ~19 required message types exist (`VehicleState`, `FaultEvent`, `FccMission`, `FccMissionItem`, `FccCommand`). |
| `mission_exec` | **Stub.** Both nodes (`mission_manager_node`, `search_planner_node`) are constructor-only. |
| `swarm_coord` | **Stub.** Both nodes (`swarm_coordinator_node`, `formation_controller_node`) are constructor-only. |
| `vision` | **Stub.** Both nodes (`visual_localisation_node`, `perception_down_node` — renamed from `detection_node`) are constructor-only. |
| `video_uplink` | **Stub, new 2026-07-13.** Both nodes (`perception_front_node`, `video_mux_node`) are constructor-only, scaffolded alongside the gimbal camera design docs. Not yet in the dependency graph before this revision. |
| `gcs_comms` | **Stub.** `gcs_mavlink_node` is constructor-only; no MAVROS dependency even declared. |
| `health_monitor` | **Stub.** `health_monitor_node` is constructor-only. |
| `mavlink-router` | **Not in-tree.** Only a systemd unit (`drone-mc-mavlink-router.service`) referencing an external prebuilt image; no source, build, or `.conf` file in the repo. |
| `zenoh-bridge-dds` | **Done, 2026-07-22.** `ZBD-001`/`DEP-006`: config, systemd unit, Dockerfile, build/smoke-test scripts, and `cyclonedds.xml` scoped down to same-host-only all landed. |
| Docker images | Only `docker/ros2-core`, `docker/fcc-bridge`, and `docker/zenoh-bridge` exist. 5 service containers still have no Dockerfile. |
| Test infra | Only C++ gtest stubs per package (name-check only). No `launch_testing`/pytest, no `.launch.py`, no SITL harness anywhere. |

---

## Dependency Graph

```
Phase 1 — Interfaces & FCC completion (unblocks almost everything else)
  FCC-006 ──> FCC-007, FCC-008 independent
  MSG-001 (bulk mc_msgs additions, bundled into each consuming ticket below)

Phase 2 — Core node logic (parallel tracks once Phase 1 lands)
  Track A (mission/swarm):
    SC-002 (peer discovery) ──> SC-003 (ISR election) ──> SC-004 (Voronoi)
                                                              │
    ME-001 (state machine, needs FCC-005/006) ───────────────┤
                                                              ▼
                                                     ME-002 (coverage path)
    SC-004 ──> SC-001 (formation PD control, also needs ME-001, FCC-005/006)

  Track B (vision):
    VIS-001 (VO core) ──> VIS-002 (GNSS/VO handover)
    VIS-003 (detection inference) ──> VIS-004 (geolocation, needs VIS-001/002 for pose)

  Track E (gimbal video — independent package, no dependency on Track B):
    VU-001 (perception_front_node: RTSP ingest + inference + tracking,
            reuses VIS-003's engine pattern but not blocked by it)
      ──> VU-002 (video_mux_node: GCS video re-encode + metadata uplink)

  Track C (GCS):
    GCS-001 (MAVROS + channel + link status) ──> GCS-002 (mission upload FSM)
                                              └──> GCS-003 (telemetry mux, needs VIS-004, HM-001)
    MAVR-001 (mavlink-router build/config) — independent, needed before any HIL test
    GCS-004 (mission_orchestrator_node, Zenoh plan-library CRUD) — needs
             ZBD-001, independent of GCS-001/002/003 (different transport,
             different GCS-side actor); 3 open items before implementation
             starts, see MISSION-ORCHESTRATOR-INTERFACE-SPEC.md §8

  Track F (cross-host DDS transport — independent, not GCS-specific,
           grouped here since it's the same kind of standalone daemon):
    ZBD-001 (zenoh-bridge-dds build/config, replaces CycloneDDS's cross-host
             discovery) — independent, done 2026-07-22

  Track D (health):
    HM-001 (resource monitoring) ──> HM-002 (liveness/watchdog, needs ME-001 heartbeat)
                                 ──> HM-003 (failsafe tree, needs FCC-004 fault signals, GCS-001 link status)

Phase 3 — Deployment
  DEP-001, DEP-002, DEP-003, DEP-005 (Dockerfiles) — each needs its node's package.xml/CMake stable
  DEP-006 (zenoh-bridge-dds Dockerfile, needs ZBD-001) — done 2026-07-22
  DEP-004 (params/config population) — needs every node's tunables finalized

Phase 4 — Test infrastructure
  TEST-001 (launch_testing + mock nodes) — needed before TEST-002
  TEST-002 (SITL harness, needs FCC-007, ME-001) ──> validates all of Phase 2/3
  TEST-003 (HIL smoke tests, needs Phase 3 Dockerfiles)
```

---

## Phase 1 — Interfaces & FCC Completion

### FCC-006, FCC-007, FCC-008

Already fully specified in `FCC-MISSION-CHANNEL-BACKLOG.md` — not
duplicated here. Story points added for planning purposes:

| Ticket | Points | Summary |
|---|---|---|
| FCC-006 | 5 pts | `FccBackend` bool-return ack propagation + `/mc/fcc/mission_ack` topic |
| FCC-007 | 10 pts | `MavlinkUdpBackend` — SITL fallback backend |
| FCC-008 | 5 pts | `MicroXRCEAgent` lifecycle — implement the `FCC_TRANSPORT` switch |

**Relationships:** FCC-006 has no blockers (FCC-005 is done). FCC-007 and
FCC-008 are independent of each other and of FCC-006. FCC-007 blocks
TEST-002 (SITL harness needs a working SITL backend). FCC-008 blocks any
real HIL use of FCC-002 but blocks nothing in this plan's software work.

---

## Phase 2 — Core Node Logic

### Track A — Mission & Swarm Coordination

#### SC-002: swarm_coordinator_node — peer discovery and lifecycle

**Points:** 8

**Description:** As a developer, I want `swarm_coordinator_node` to
discover, track, and time out swarm peers over the UDP swarm protocol
(ICD IFD-003), so that ISR election, Voronoi partitioning, and formation
control all have a live, deduplicated peer roster to work from.

**Acceptance Criteria:**
- Sends/receives the swarm protocol header + heartbeat message type (ICD
  §5.2/§5.3) on the peer UDP socket; maintains a `Peer`/`PeerState` map keyed
  by `drone_id`.
- Implements the peer management state machine (ICD §5.4:
  `UNKNOWN→ALIVE→LOST→DEAD` with documented timeouts).
- Publishes `/mc/swarm/peer_states` (`mc_msgs/SwarmPeerArray`) at 1 Hz + on
  change.
- Own vehicle state (from `/mc/fcc/vehicle_state`) and mission phase (from
  `/mc/mission_phase`) are folded into the outgoing heartbeat so peers see
  this drone's `SwarmPeer` fields correctly populated.

**Deliverables:**
- `swarm_coordinator_node.{hpp,cpp}` — peer socket, state machine, heartbeat
  publisher.
- `mc_msgs/msg/SwarmPeerArray.msg`, `SwarmPeer.msg` (per ICD §7.2, already
  specified there — just not yet created as files).
- Node integration tests: peer transitions through all 4 states correctly
  under simulated packet loss/timeout; `peer_states` reflects the roster
  accurately.

**Relationships:** No blockers. Blocks SC-003, SC-004, and SC-001
(formation controller needs `/mc/swarm/peer_states`).

---

#### SC-003: swarm_coordinator_node — ISR election algorithm

**Points:** 6

**Description:** As a developer, I want `swarm_coordinator_node` to run the
weighted ISR (Intelligence/Surveillance/Reconnaissance) election algorithm
during the `ISR_SELECTION` mission phase, so the swarm autonomously agrees
on which drones take the ISR role without a GCS decision.

**Acceptance Criteria:**
- Computes the weighted score `0.4×battery + 0.4×proximity + 0.2×sensor_ok`
  per peer (SDD §6.2) from `/mc/swarm/peer_states`.
- Elects the top ⌈N/2⌉ scoring drones as ISR; all drones compute this
  independently from the same peer roster (deterministic tie-breaking by
  `drone_id`).
- Runs an ACK consensus round (peers exchange election result, only commits
  once ACKs from all `ALIVE` peers are received or a timeout elapses) before
  publishing.
- Publishes `/mc/swarm/isr_role` (`mc_msgs/IsrRole` — **not yet defined in
  ICD-MC-001 §7.2**, despite being in the §7.1 topic table; this ticket adds
  the message definition to both `mc_msgs` and the ICD, matching the pattern
  FCC-005 set for `FccMission`/etc.).

**Deliverables:**
- `swarm_coordinator_node.{hpp,cpp}` extended with the election algorithm
  and ACK consensus state machine.
- `mc_msgs/msg/IsrRole.msg` + `ICD-MC-001.md` §7.2 update.
- Node integration tests: known peer roster → correct election result;
  consensus blocks on a missing ACK and proceeds once received/timed out.

**Relationships:** Depends on SC-002 (needs `peer_states`). Blocks nothing
downstream directly (SC-004/Voronoi doesn't require ISR result), but the
mission phase order (`ISR_SELECTION` before `VORONOI_PARTITION`) means
ME-001's transition table exercises this before SC-004 in practice.

---

#### SC-004: swarm_coordinator_node — Voronoi partitioning

**Points:** 10

**Description:** As a developer, I want `swarm_coordinator_node` to
partition the search area into per-drone Voronoi cells during
`VORONOI_PARTITION`, so `search_planner_node` (ME-002) has a bounded region
to generate each drone's coverage path from.

**Acceptance Criteria:**
- Computes Voronoi partitioning via Fortune's sweep-line algorithm (SDD
  §6.3), deterministic given the same peer position roster.
- All drones compute independently, then exchange `VORONOI_ACCEPT` messages
  (ICD §5.3) with a 1 cm tolerance check before committing.
- Re-partitions automatically on peer loss during `SEARCH` (peer transitions
  to `LOST`/`DEAD` mid-mission triggers a new partition round).
- Publishes `/mc/swarm/voronoi_cell` (`mc_msgs/VoronoiCell`, already
  specified in ICD §7.2) per drone, tagged with `round` (0 = initial, 1+ =
  re-partition).

**Deliverables:**
- `swarm_coordinator_node.{hpp,cpp}` extended with Fortune's sweep
  implementation, VORONOI_ACCEPT consensus, and re-partition trigger.
- `mc_msgs/msg/VoronoiCell.msg`.
- Node integration tests: known peer positions → matching cell boundaries
  across "simulated" peers within tolerance; peer-loss triggers a new round.

**Relationships:** Depends on SC-002 (peer roster). Blocks ME-002 (coverage
path needs a cell to plan within) and is a prerequisite for SC-001's
`SEARCH`-phase relevance (formation control is mainly an `INGRESS`-phase
concern, so this isn't a hard blocker for SC-001 itself).

---

#### ME-001: mission_manager_node — mission phase state machine

Already fully specified in `FCC-MISSION-CHANNEL-BACKLOG.md`.

**Points:** 13

**Relationships:** Depends on FCC-005 (done) and FCC-006. Blocks SC-001,
ME-002 (needs `/mc/mission_phase` to gate/sequence), HM-002 (heartbeat
consumer).

---

#### ME-002: search_planner_node — boustrophedon coverage path generation

**Points:** 8

**Description:** As a developer, I want `search_planner_node` to generate a
boustrophedon ("lawnmower") coverage path within each drone's assigned
Voronoi cell, so the drone has concrete waypoints to fly during `SEARCH`.

**Acceptance Criteria:**
- Computes the minimum-area bounding rectangle of the assigned Voronoi cell
  (from `/mc/swarm/voronoi_cell`) and generates parallel sweep lines within
  it (SDD §7.1), with +5 m altitude offset between rows to avoid inter-row
  collision risk.
- Back-projects the resulting Cartesian waypoints to WGS-84 for PX4 mission
  upload (SDD §7.3 coordinate projection).
- Publishes `/mc/search/coverage_path` (`nav_msgs/Path`) on receiving a new
  Voronoi cell assignment (including re-partition rounds from SC-004).
- Recomputes and republishes automatically when a new `round` is received on
  `voronoi_cell` (re-partition mid-`SEARCH`).

**Deliverables:**
- `search_planner_node.{hpp,cpp}` — bounding-rectangle + sweep-line
  algorithm, WGS-84 back-projection.
- Node integration tests: known cell polygon → expected sweep pattern
  (row count, spacing, altitude stagger); re-partition triggers
  recomputation.

**Relationships:** Depends on SC-004 (needs a cell to plan within) and
ME-001 (phase gating — only active `VORONOI_PARTITION`→`SEARCH`). Blocks
nothing further in this plan (its output feeds `mission_manager`'s
`VORONOI_PARTITION→SEARCH` transition, already scoped as ME-001's
responsibility to consume).

---

#### SC-001: formation_controller_node — INGRESS formation-slot PD tracking

Already fully specified in `FCC-MISSION-CHANNEL-BACKLOG.md`.

**Points:** 10

**Relationships:** Depends on FCC-005/006 (done/pending), ME-001 (phase
gating), and SC-002 (`/mc/swarm/peer_states` for leader state). No
downstream blockers in this plan.

---

### Track B — Vision

#### VIS-001: visual_localisation_node — visual odometry pipeline

**Points:** 13

**Description:** As a developer, I want `visual_localisation_node` to
produce a scale-correct 6-DoF pose estimate from the CSI camera stream, so
the drone can navigate during GNSS-denied conditions.

**Acceptance Criteria:**
- FAST corner detection + ORB descriptors → KLT optical-flow tracking
  between frames (SDD §8.1).
- 5-point essential matrix estimation + RANSAC for relative pose between
  keyframes.
- Scale recovered via IMU pre-integration between keyframes (SDD §8.2), not
  assumed/fixed.
- Maintains a 10-keyframe sliding window (no loop closure, per design) for
  local consistency.
- Publishes `/mc/local_pose` (`geometry_msgs/PoseWithCovarianceStamped`) at
  ≥10 Hz and `/mc/fcc/visual_odometry` (new `mc_msgs/VisualOdometry`,
  mirroring the existing internal `fcc_bridge::VisualOdometry` struct field
  layout) for `FccBackend::publish_visual_odometry()`.

**Deliverables:**
- `visual_localisation_node.{hpp,cpp}` with the VO pipeline (OpenCV
  feature/tracking, custom or Eigen-based essential-matrix/RANSAC, IMU
  pre-integration).
- `mc_msgs/msg/VisualOdometry.msg` + ICD-MC-001 §7.2 entry (currently
  referenced in the §7.1 topic table but undefined, same gap pattern as
  `IsrRole`).
- Node integration test: recorded frame+IMU sequence (bag or synthetic) →
  pose trajectory within tolerance of ground truth.

**Relationships:** No blockers. Blocks VIS-002 (handover logic needs a
working VO source) and VIS-004 (geolocation needs `/mc/local_pose`).

---

#### VIS-002: visual_localisation_node — GNSS↔VO handover and drift bound

**Points:** 5

**Description:** As a developer, I want `visual_localisation_node` to
automatically switch between GNSS and visual-odometry position sources
based on GNSS fix quality, so the drone degrades gracefully without manual
intervention.

**Acceptance Criteria:**
- Monitors `/mc/fcc/vehicle_state.gnss_ok`/`hdop`; switches
  `LocalisationMode` (`GNSS_PRIMARY` ↔ `VISUAL_ODOMETRY`) per SDD §8.3
  hysteresis thresholds — no rapid toggling at the boundary.
- Publishes `/mc/localisation_mode` (`mc_msgs/LocalisationMode`, ICD §7.2
  already specifies the fields) on every mode change.
- Enforces the documented drift bound (SDD §8.4, 5 m/min) — VO-only output
  degrades/flags rather than silently drifting unbounded past that point.

**Deliverables:**
- `visual_localisation_node.{hpp,cpp}` extended with the handover state
  machine and drift monitor.
- `mc_msgs/msg/LocalisationMode.msg`.
- Node integration test: simulated GNSS degradation/reacquisition sequence
  → correct mode transitions with no chatter at the threshold.

**Relationships:** Depends on VIS-001. No downstream blockers in this plan
beyond GCS-003 (telemetry mux relays `localisation_mode` to GCS, but that's
a simple relay, not a hard dependency on VIS-002's internal logic).

---

#### VIS-003: perception_down_node — YOLOv8n TensorRT inference pipeline

**Points:** 10

**Description:** As a developer, I want `perception_down_node` (renamed
from `detection_node` — SDD §9, ICD §9.2, HID/ICD gimbal-camera revision) to
run real-time object detection on the downward camera stream via a
TensorRT-optimized YOLOv8n model, so the swarm can surface visual
detections during `SEARCH`.

**Acceptance Criteria:**
- Renames the existing `detection_node.{hpp,cpp}` stub (class, file names,
  and the `Node("detection_node")` constructor argument) to
  `perception_down_node` throughout `ws/src/vision/` — CMakeLists.txt,
  package.xml, and the existing gtest stub (`test_detection_node.cpp` →
  `test_perception_down_node.cpp`) all follow.
- ONNX→TensorRT engine build/cache on first run (SDD §9.2) — subsequent
  starts load the cached engine rather than rebuilding.
- 640×640 input preprocessing; dedicated inference thread with a frame
  queue of max depth 2, dropping the oldest frame on backpressure (SDD §9.1)
  rather than blocking capture.
- Post-processing applies IoU 0.45 / confidence 0.5 NMS thresholds.
- GPU utilization throttled to ≤80% (SDD §9.4) to leave headroom for VO and
  for `perception_front_node` (VU-001, `video_uplink` package — a separate
  process that independently reuses this same TensorRT engine/model).

**Deliverables:**
- `perception_down_node.{hpp,cpp}` — TensorRT engine loading/inference,
  frame-queue management, NMS post-processing.
- CMake/package.xml updated with the TensorRT/CUDA dependency (currently
  absent).
- Node integration test: a fixture image/video with known objects → expected
  bounding boxes within IoU tolerance of ground truth.

**Relationships:** No blockers. Blocks VIS-004 (geolocation consumes raw
detections). VU-001 (`perception_front_node`) reuses this ticket's engine-
loading pattern but does not block on it landing first.

---

#### VIS-004: perception_down_node — target geolocation and GPU resource management

**Points:** 6

**Description:** As a developer, I want `perception_down_node` to convert
each raw 2D detection into a geolocated target position, so downstream
consumers (GCS, mission planning) get actionable lat/lon rather than pixel
coordinates.

**Acceptance Criteria:**
- Geolocation via camera intrinsics + drone pose (SDD §9.3): ray
  unprojection from the bounding-box center, flat-terrain intersection,
  with uncertainty propagation into `pos_uncertainty_m`.
- Consumes `/mc/local_pose` (VIS-001/002) for the drone-pose half of the
  geolocation calculation.
- Publishes `/mc/detections` (`mc_msgs/DetectionArray`, ICD §9.2 already
  specifies `Detection`/`DetectionArray` fields) event-driven, one message
  per processed frame with detections.
- GPU resource management: detection inference backs off if VO (VIS-001)
  reports GPU contention, per the ≤80% budget split between the two GPU
  consumers (SDD §9.4).

**Deliverables:**
- `perception_down_node.{hpp,cpp}` extended with geolocation math and GPU
  coordination with `visual_localisation_node`.
- `mc_msgs/msg/Detection.msg`, `DetectionArray.msg`.
- Node integration test: known detection + known pose/intrinsics → expected
  lat/lon within tolerance; GPU-contention backoff observable under load.

**Relationships:** Depends on VIS-003 (raw detections) and VIS-001/002
(`/mc/local_pose`). Blocks GCS-003 (telemetry mux relays `/mc/detections`).

---

### Track E — Gimbal Video Perception (`video_uplink` package)

#### VU-001: perception_front_node — RTSP ingest, inference, and tracking

**Points:** 10

**Description:** As a developer, I want `perception_front_node` to ingest
the front gimbal camera's RTSP/H.265 stream, run AI inference reusing the
same TensorRT engine as the downward pipeline, and track detections
frame-to-frame, so the swarm has continuous gimbal-mounted target tracking
independent of the downward camera (SDD §9a.1, FR-080–082, IR-040).

**Acceptance Criteria:**
- GStreamer `rtspsrc` pipeline connects to `rtsp://192.168.10.200:8554/stream1`
  (HIF-007, reached via the Jetson's existing `eth0` — the gimbal camera
  shares the baseboard's internal switch, not a second Jetson interface),
  decodes H.265, and feeds the same frame-queue/preprocessing/TensorRT/NMS
  pattern VIS-003/004 established, reusing the loaded engine rather than a
  second copy.
- IoU-based greedy tracker (SDD §9a.1: `track_iou_threshold=0.3`,
  `track_miss_limit=10`) assigns/maintains `track_id` across frames.
- Publishes `/mc/perception/front/detections` (event, ≤1 s) and
  `/mc/perception/front/tracks` (≥5 Hz) using the new `mc_msgs/Track`/
  `TrackArray` types.
- `rtspsrc` auto-reconnects on error; link silence > 3 s is observable via
  the node's liveness heartbeat, which `health_monitor_node` (HM-002/HM-003)
  uses to set `GIMBAL_LINK_LOST`/clear `GIMBAL_OK` — explicitly excluded
  from the flight failsafe tree (FR-084).
- Runs as a separate node/process from `perception_down_node` (different
  package, `video_uplink` not `vision`) — a stall in one does not affect the
  other, subject to the shared ≤80% GPU budget (PR-010, FR-082).

**Deliverables:**
- `perception_front_node.{hpp,cpp}` — rtspsrc pipeline, inference reuse, IoU
  tracker.
- `mc_msgs/msg/Track.msg`, `TrackArray.msg` (ICD §9.2 already specifies
  fields — not yet created as files).
- `video_uplink/CMakeLists.txt`/`package.xml` updated with GStreamer RTSP
  client plugins + TensorRT/CUDA dependencies (currently absent — the
  package is a constructor-only scaffold as of the diagram/docs commit).
- Node integration test: new `mock_gimbal_node` (RTSP test server looping a
  bag-derived clip, TP-MC-001 §2.4) → expected detections/tracks within
  tolerance; simulated link loss → heartbeat silence detected, no fault
  raised on the flight side.

**Relationships:** Reuses VIS-003's TensorRT engine-loading pattern but is
not blocked by it landing first — can build against a standalone engine
instance and de-duplicate later if desired. Blocks VU-002 (needs this
node's frame/track output).

---

#### VU-002: video_mux_node — GCS video re-encode and metadata uplink

**Points:** 8

**Description:** As a developer, I want `video_mux_node` to re-encode
`perception_front_node`'s frame stream to H.265, mux in a per-frame
metadata track, and serve it to the GCS over a dedicated RTSP session, so
the operator gets a live annotated video feed independent of the MAVLink
telemetry channel (SDD §9a.2, FR-083–084, IR-041–042).

**Acceptance Criteria:**
- GStreamer encode pipeline (`nvv4l2h265enc`) re-encodes frames from
  `perception_front_node`; muxes a KLV-style metadata track (ICD IFD-007
  §8.2: `time_usec`, `track_id`, `class_id`, `confidence`, `lat`/`lon`/`alt`)
  sourced from `/mc/perception/front/detections` + `/tracks`.
- Serves the muxed stream via its own RTSP server at
  `rtsp://192.168.10.11:8555/mc_video` — no dependency on
  `gcs_mavlink_node`/mavlink-router.
- Publishes `/mc/video/uplink_status` (`mc_msgs/VideoUplinkStatus`) at
  ≥1 Hz reflecting `streaming`, `client_connected`, `bitrate_kbps`.
- On `perception_front_node` frame/track silence > 3 s, stops serving new
  frames and sets `streaming=false`, tearing down the active RTSP session
  but keeping the encode pipeline warm for reconnect (ICD IFD-007 §8.4). A
  GCS client disconnect alone does not raise a fault.

**Deliverables:**
- `video_mux_node.{hpp,cpp}` — encode/mux pipeline, RTSP server, status
  publisher.
- `mc_msgs/msg/VideoUplinkStatus.msg`.
- Node integration test: synthetic frame+track input → mock RTSP client
  receives a valid stream with correctly-muxed metadata; source silence →
  `streaming=false` within 3 s, no fault escalation.

**Relationships:** Depends on VU-001 (`perception_front_node`'s output).
Blocks nothing further in this plan — GCS-side consumption of the uplink is
out of scope for this repo.

---

### Track C — GCS Communication

#### GCS-001: gcs_mavlink_node — MAVROS integration, channel setup, link status

**Points:** 8

**Description:** As a developer, I want `gcs_mavlink_node` to establish the
GCS↔MC MAVLink channel (separate from the GCS↔FCC channel mavlink-router
also carries), so the GCS can send commands to and receive status from the
mission computer directly.

**Acceptance Criteria:**
- Integrates MAVROS as a library dependency (currently not even declared in
  `package.xml`/`CMakeLists.txt`) with a custom plugin for
  `MC_STATUS`/`MC_DETECTION`/`MC_HEALTH` MAVLink messages (SDD §11.1, ICD
  §4.6).
- Decodes incoming GCS commands into `/mc/gcs/command`
  (`mc_msgs/GcsCommand`, ICD §7.2 already specifies fields).
- Monitors GCS heartbeat; publishes `/mc/gcs/link_status`
  (`mc_msgs/LinkStatus`, ICD §7.2 already specifies fields) at 1 Hz.
- Correctly binds only to the GCS↔MC channel ports (ICD §4.1/§4.2) — does
  not touch or interfere with the GCS↔FCC channel that mavlink-router
  routes separately.

**Deliverables:**
- `gcs_mavlink_node.{hpp,cpp}` — MAVROS integration, custom plugin, command
  decode, link-status publisher.
- `mc_msgs/msg/GcsCommand.msg`, `LinkStatus.msg`.
- `CMakeLists.txt`/`package.xml` updated with the MAVROS dependency.
- Node integration test: mock GCS MAVLink peer → correct `GcsCommand`
  decode and `link_status` heartbeat tracking (matches the `mock_gcs_node`
  strategy TP-MC-001 §2.4 already documents).

**Relationships:** No blockers (independent of the FCC-side work). Blocks
GCS-002, GCS-003.

---

#### GCS-002: gcs_mavlink_node — mission upload state machine

**Points:** 8

**Description:** As a developer, I want `gcs_mavlink_node` to receive a full
mission upload from the GCS (ingress waypoints, formation offsets, search
polygon, swarm manifest) and hand it to the rest of the MC, so a mission can
actually be initiated from the ground station.

**Acceptance Criteria:**
- Implements the mission-upload state machine (SDD §11.2): receives and
  assembles `MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK` (or the
  custom-message equivalent carrying `MissionUpload`'s richer fields) into a
  complete `mc_msgs/MissionUpload`.
- Publishes `/mc/gcs/mission_upload` (`mc_msgs/MissionUpload`) once
  fully assembled and validated.
- Defines the still-missing `mc_msgs/FormationOffset` (referenced in
  `MissionUpload.formation_offsets` per ICD §7.2 but never itself defined —
  same documentation gap as `IsrRole`) and adds it to the ICD.

**Deliverables:**
- `gcs_mavlink_node.{hpp,cpp}` extended with the upload state machine.
- `mc_msgs/msg/MissionUpload.msg`, `FormationOffset.msg` +
  `ICD-MC-001.md` §7.2 update for `FormationOffset`.
- Node integration test: simulated multi-item upload sequence (including a
  dropped/retried item) → correctly assembled `MissionUpload`.

**Relationships:** Depends on GCS-001. Feeds `mission_manager`/
`swarm_coordinator` (consumers per ICD §7.1) but doesn't block ME-001/SC-002
directly — those tickets can develop against a hand-built `MissionUpload`
fixture in their own tests and integrate once this lands.

---

#### GCS-003: gcs_mavlink_node — telemetry multiplexer

**Points:** 4

**Description:** As a developer, I want `gcs_mavlink_node` to republish key
internal MC state to the GCS at a steady rate, so the operator has
continuous situational awareness.

**Acceptance Criteria:**
- Multiplexes `/mc/fcc/vehicle_state`, `/mc/mission_phase`,
  `/mc/detections`, `/mc/health`, and `/mc/localisation_mode` into the
  custom `MC_STATUS`/`MC_DETECTION`/`MC_HEALTH` MAVLink messages at ≥2 Hz
  (SDD §11.3).
- Does not block or degrade the command-decode path (GCS-001) under
  detection-heavy traffic — runs on its own callback group/timer.

**Deliverables:**
- `gcs_mavlink_node.{hpp,cpp}` extended with the multiplexer.
- Node integration test: synthetic internal-topic traffic → correct,
  rate-bounded MAVLink relay to a mock GCS peer.

**Relationships:** Depends on GCS-001, VIS-004 (`/mc/detections`), HM-001
(`/mc/health`). Last of the GCS-side tickets — no downstream blockers.

---

#### GCS-004: mission_orchestrator_node — Zenoh-native mission plan library

**Points:** 8 — **Not started.** Full interface contract specified in
`MISSION-ORCHESTRATOR-INTERFACE-SPEC.md` (2026-07-22); this ticket tracks
the MC-side implementation of that spec, not the Mission Orchestrator app
itself (built by another team, out of scope).

**Description:** As a developer, I want a new node exposing mission-plan
CRUD (create/list/update/activate/delete) over Zenoh, reusing `mc_msgs`
schemas directly rather than a second MAVLink custom dialect, so the
Mission Orchestrator team has a concrete interface to build against —
separate from and independent of QGC's MAVLink channel (GCS-001/002/003).

**Acceptance Criteria:** Full detail in
`MISSION-ORCHESTRATOR-INTERFACE-SPEC.md` §4–§6. Summary:
- 8 new `mc_msgs` types (`MissionPlan`, `MissionPlanSummary`,
  `MissionPlanCreateRequest`, `MissionPlanUpdateRequest`,
  `MissionPlanActivateRequest`, `MissionPlanDeleteRequest`,
  `MissionPlanListRequest`, `MissionPlanListResponse`, `MissionPlanAck`).
- `activate` produces exactly the same `mc_msgs/MissionUpload` on
  `/mc/gcs/mission_upload` that `gcs_mavlink_node` produces today —
  `mission_manager_node` (ME-001) needs zero changes.
- Plan library persisted as one YAML file per plan under
  `/var/drone-mc/mission_plans/`, additive to (not replacing)
  `mission_manager_node`'s existing single-file mission persistence.
- `config/zenoh-bridge-dds.json5`'s `allow` list widened to forward the new
  `/mc/orchestrator/*` topics (currently `["^$"]`, forwards nothing).

**Deliverables:**
- `mc_msgs/msg/MissionPlan*.msg` × 8 + `CMakeLists.txt` entries.
- `ws/src/gcs_comms/src/mission_orchestrator_node.{hpp,cpp}`.
- `config/zenoh-bridge-dds.json5` allow-list update.
- Node integration test: mock Zenoh peer issuing create/list/update/
  activate/delete → correct persistence, correct `MissionPlanAck`
  correlation, correct `/mc/gcs/mission_upload` publish on activate.

**Relationships:** Depends on `ZBD-001` (the Zenoh mesh this rides).
Independent of GCS-001/002/003 (different transport, different actor).
Three open items block a confident estimate on the phase-gating/security
sub-items — see `MISSION-ORCHESTRATOR-INTERFACE-SPEC.md` §8 — resolve
those before implementation starts.

---

#### MAVR-001: mavlink-router — build, config, and channel isolation

**Points:** 4

**Description:** As a developer, I want `mavlink-router` actually built and
configured in this repo, so `drone-mc-mavlink-router.service` runs real,
version-controlled software instead of an external image nobody here has
built.

**Acceptance Criteria:**
- `mavlink-router/` source is vendored or fetched (per `CLAUDE.md`'s
  documented `meson setup build && ninja -C build` build step) with a
  pinned version/commit.
- `/etc/drone-mc/mavlink-router.conf` (referenced in SDD §12.5's
  configuration-files list but not present anywhere in the repo) is added,
  implementing the GCS→FCC / GCS→MC channel separation (ICD §4.2) —
  explicitly verified to **not** forward Channel B (GCS↔MC) traffic onto
  Channel A (GCS↔FCC) or vice versa.
- A Dockerfile builds the router via Meson/Ninja (paired with DEP-003).

**Deliverables:**
- `mavlink-router/` (vendored source or `.repos` fetch, matching the
  `px4_msgs.repos` pattern already used for `px4_msgs`).
- `config/mavlink-router.conf`.
- Smoke test: router starts, and a test message on Channel B does not
  appear on a Channel A listener (and vice versa).

**Relationships:** Independent of everything else in Phase 2. Blocks
DEP-003 and any HIL testing that needs both GCS channels live
simultaneously.

---

### Track F — Cross-Host DDS Transport

#### ZBD-001: zenoh-bridge-dds — build, config, and CycloneDDS scope-down

**Points:** 4 — **Done, 2026-07-22.**

**Description:** As a developer, I want cross-host DDS discovery to ride a
static-peer Zenoh mesh instead of CycloneDDS's own unicast `<Peers>` list
over `eth0`, so the MC↔peer-MC link uses a transport actually suited to a
bandwidth-constrained, lossy MANET, rather than DDS's own discovery
protocol. (Note: no cross-host DDS topic exists yet — `swarm_coord`'s peer
protocol is a separate custom UDP protocol, and GCS never joins the DDS
domain at all — so this is forward-looking infrastructure, not fixing a
live problem.)

**Acceptance Criteria:**
- `docker/ros2-core/cyclonedds.xml` scoped down to `lo` only — the `eth0`
  `NetworkInterface` entry and the cross-host `<Peer address="192.168.10.1x">`
  entries are removed; only `127.0.0.1` remains.
- New `zenoh-bridge-dds.json5` config: peer mode (mesh, no central router),
  multicast scouting disabled (same MANET-bandwidth rationale
  `cyclonedds.xml` used to document), static per-drone connect/listen
  endpoints, and a topic allow-list that forwards nothing by default until a
  real cross-host DDS topic needs it.
- `scripts/02-install-systemd-units.sh` provisions the new config file the
  same install-if-missing way it already provisions `cyclonedds.xml`.
- `scripts/04c-smoke-test-dds.sh` updated to assert `cyclonedds.xml` is
  `lo`-only (no `eth0`, no cross-host peer) instead of the old unicast-Peers
  assertions.

**Deliverables:**
- `config/zenoh-bridge-dds.json5`.
- `docker/ros2-core/cyclonedds.xml` (edited).
- `scripts/02-install-systemd-units.sh`, `scripts/04c-smoke-test-dds.sh`
  (edited).
- SRS (`SW-008`, `SW-039`), ICD (§11.4a), SDD (§1.4, §12.3–12.5), and
  `CLAUDE.md` updated to describe the same-host/cross-host split.
- `diagrams/03-container-deployment.puml` + re-rendered `.png` showing the
  new `zenoh-bridge.service` component.

**Relationships:** No blockers. Independent of GCS-*/MAVR-001 (does not
touch MAVLink). Blocks `DEP-006`.

---

### Track D — Health Monitoring

#### HM-001: health_monitor_node — resource monitoring

**Points:** 4

**Description:** As a developer, I want `health_monitor_node` to sample
system resource metrics and publish them, so operators and the failsafe
tree have visibility into CPU/memory/thermal state.

**Acceptance Criteria:**
- 1 Hz timer samples CPU%, memory%, disk%, GPU temp, CPU temp (SDD §10.1).
- Publishes `/mc/health` (`mc_msgs/SystemHealth`, ICD §7.2 already specifies
  fields — the message itself doesn't exist yet).

**Deliverables:**
- `health_monitor_node.{hpp,cpp}` — resource-sampling timer + publisher.
- `mc_msgs/msg/SystemHealth.msg`.
- Node integration test: mocked `/proc`/`nvidia-smi`-equivalent readings →
  correct published values.

**Relationships:** No blockers. Blocks HM-002, HM-003 (both build on the
same node), GCS-003 (`/mc/health` consumer).

---

#### HM-002: health_monitor_node — node liveness tracking and manager watchdog

**Points:** 5

**Description:** As a developer, I want `health_monitor_node` to track
whether each critical node is alive, and specifically watchdog
`mission_manager_node`'s heartbeat, so a hung/crashed node triggers a
restart rather than a silent stall (FR-074).

**Acceptance Criteria:**
- Tracks liveness of the 7 critical topics/nodes (SDD §10.2) into
  `node_alive_mask` (bit layout per ICD §8.9), folded into `SystemHealth`.
- Consumes `/mc/manager/heartbeat`; if no heartbeat for >5 s, triggers the
  FR-074 restart path (systemd restart trigger, see HM-003) and notifies GCS
  via a fault event.

**Deliverables:**
- `health_monitor_node.{hpp,cpp}` extended with per-topic liveness tracking
  and the heartbeat watchdog timer.
- Node integration test: a topic going silent past its expected rate flips
  the corresponding `node_alive_mask` bit; heartbeat silence >5 s triggers
  the restart path.

**Relationships:** Depends on HM-001 and ME-001 (needs
`/mc/manager/heartbeat` to exist). Blocks HM-003.

---

#### HM-003: health_monitor_node — failsafe decision tree

**Points:** 8

**Description:** As a developer, I want `health_monitor_node` to run the
full failsafe decision tree, so the drone responds correctly and
automatically to link loss, low battery, GCS loss, and thermal events
without operator intervention.

**Acceptance Criteria:**
- Implements the failsafe decision tree (SDD §10.3) exactly:
  `FCC_LINK_LOST`→HOLD/RTL, `MANAGER_DEAD`→restart-then-RTL,
  `LOW_BATTERY`→RTL, `GCS_LINK_LOST`→HOLD, `GPU_OVERTEMP`→throttle.
- Publishes `/mc/fault` (`mc_msgs/FaultEvent`, already exists) on each
  triggered condition, with the corresponding `failsafe_action`.
- Implements the systemd-restart interface (SDD §10.4) — a Unix socket or
  equivalent mechanism health_monitor uses to request a node restart from
  the host, matching the `MANAGER_DEAD` path's "restart-then-RTL" behavior.
- Edge-triggered: does not repeatedly re-fire the same fault/action every
  tick while the condition persists (matches the pattern
  `formation_controller_node`'s SC-001 ticket already establishes for its
  own self-HOLD logic).

**Deliverables:**
- `health_monitor_node.{hpp,cpp}` extended with the decision tree and
  restart-socket interface.
- Node integration test: one case per decision-tree branch (fault input →
  correct `FaultEvent` + `failsafe_action`, no re-fire on persistence).

**Relationships:** Depends on HM-001, HM-002, FCC-004 (already
done — fault signal source), GCS-001 (`link_status` for `GCS_LINK_LOST`).
No downstream blockers in this plan.

---

## Phase 3 — Deployment

#### DEP-001: Dockerfiles — gcs_comms, health_monitor, swarm_coord, mission_exec

**Points:** 4

**Description:** As a developer, I want the remaining 4 first-party ROS 2
packages containerized, following the pattern `docker/fcc-bridge/Dockerfile`
already establishes, so all services can actually run under
`drone-mc.target`.

**Acceptance Criteria:**
- Each Dockerfile follows the existing two-stage pattern (colcon build
  stage `FROM ros2-core`, runtime stage copying the merged install tree,
  `BUILD_TESTING=OFF`) — no new pattern invented.
- Each image's `CMD` sources the workspace and runs the correct
  `ros2 run <pkg> <node>` with its params file, matching
  `docker/fcc-bridge/Dockerfile`'s `CMD` structure.

**Deliverables:**
- `docker/gcs-comms/Dockerfile`, `docker/health-monitor/Dockerfile`,
  `docker/swarm-coord/Dockerfile` (builds both `swarm_coordinator_node` and
  `formation_controller_node`), `docker/mission-exec/Dockerfile` (builds
  both `mission_manager_node` and `search_planner_node`).
- A build script per image following the `08-build-fcc-bridge-image.sh`
  pattern, or a generalized version of that script parameterized by
  package (avoid 4 near-duplicate scripts if a shared one is
  straightforward).

**Relationships:** Depends on each package's node(s) being implemented
(practically: can be built and validated against the stub nodes earlier,
but is only truly "done" once Phase 2 tickets for that package land).
Blocks TEST-003 (HIL smoke tests need real containers to test against).

---

#### DEP-002: Dockerfiles — camera and vision

**Points:** 3

**Description:** As a developer, I want the camera capture path (`gscam2`,
per `drone-mc-camera.service`'s existing comment) and the `vision` package
(VO + detection) containerized separately, matching the systemd split
already defined (`drone-mc-camera.service` vs `drone-mc-vision.service`) so
the lightweight capture path can restart independently of the GPU-heavy
pipeline.

**Acceptance Criteria:**
- `docker/camera/Dockerfile` wraps `gscam2` (external package, not built
  from this repo's `ws/src`) — no CMake/colcon build needed for it.
- `docker/vision/Dockerfile` builds `visual_localisation_node` and
  `perception_down_node` (renamed from `detection_node`), including the
  TensorRT/CUDA runtime dependencies VIS-003 introduces.

**Deliverables:**
- `docker/camera/Dockerfile`, `docker/vision/Dockerfile`.
- Build script(s) for both.

**Relationships:** `docker/vision/Dockerfile` depends on VIS-001–004 being
buildable (TensorRT/CUDA deps declared). `docker/camera/Dockerfile` is
independent. Blocks TEST-003.

---

#### DEP-003: mavlink-router Dockerfile

**Points:** 3

**Description:** As a developer, I want a Dockerfile building
`mavlink-router` via its Meson/Ninja build, so
`drone-mc-mavlink-router.service`'s image reference
(`ghcr.io/drone-mc/mavlink-router:${MC_VERSION}`) corresponds to something
this repo actually builds.

**Acceptance Criteria:**
- Multi-stage Dockerfile: build stage runs `meson setup build && ninja -C
  build` against MAVR-001's vendored source; runtime stage copies the
  resulting binary and `mavlink-router.conf`.

**Deliverables:**
- `docker/mavlink-router/Dockerfile`.
- Build script matching the `08-build-fcc-bridge-image.sh` pattern.

**Relationships:** Depends on MAVR-001. Blocks TEST-003 and any HIL test
needing both GCS channels live.

---

#### DEP-006: zenoh-bridge-dds Dockerfile

**Points:** 3 — **Done, 2026-07-22.**

**Description:** As a developer, I want a Dockerfile building
`zenoh-bridge-dds`, so `drone-mc-zenoh-bridge.service`'s image reference
(`ghcr.io/drone-mc/zenoh-bridge:${MC_VERSION}`) corresponds to something
this repo actually builds.

**Acceptance Criteria:**
- Single-stage Dockerfile (no Rust/CMake/clang builder stage needed):
  downloads Eclipse Zenoh's prebuilt `aarch64-unknown-linux-gnu` `.deb` for a
  pinned tagged release and installs it — the vendor already ships an
  Ubuntu-22.04-targeted build, so there's nothing to compile in-image.
- `entrypoint.sh` execs `zenoh-bridge-dds -c /etc/drone-mc/zenoh-bridge-dds.json5`
  as the container's sole process (signals reach it directly).
- Build script mirrors `09-build-mavlink-router-image.sh`'s shape (pinned
  version build-arg, `:dev`/`:latest`/`$MC_VERSION` tags).
- Smoke test mirrors `04c-smoke-test-dds.sh`'s static-check style, plus a
  `LIVE_CHECK=1` path that starts the container and confirms it stays up —
  specifically to catch a possible glibc mismatch between the vendor's
  prebuilt build and JetPack 6.2.2, which has **not** been verified on real
  hardware yet (flagged, not silently assumed compatible).

**Deliverables:**
- `docker/zenoh-bridge/Dockerfile`, `docker/zenoh-bridge/entrypoint.sh`.
- `scripts/10-build-zenoh-bridge-image.sh`.
- `scripts/10b-smoke-test-zenoh-bridge.sh`.
- `config/systemd/drone-mc-zenoh-bridge.service` (`be.slice`, no service
  dependencies beyond `health-monitor`, same pattern as
  `drone-mc-mavlink-router.service`).

**Relationships:** Depends on ZBD-001. Blocks TEST-003 and any HIL
verification of the vendor `.deb`'s glibc compatibility on the Jetson.

---

#### DEP-005: video_uplink Dockerfile

**Points:** 3

**Description:** As a developer, I want the `video_uplink` package
(`perception_front_node` + `video_mux_node`) containerized, following the
`docker/vision/Dockerfile` pattern (DEP-002) but with GStreamer RTSP
server/client plugins added, so a `drone-mc-video-uplink.service` (SDD
§12.3/§12.4 describe it; the unit file itself doesn't exist yet in
`config/systemd/`) runs a real image.

**Acceptance Criteria:**
- `docker/video-uplink/Dockerfile` follows the existing two-stage colcon
  pattern, adding the GStreamer RTSP client/server plugins
  (`gst-rtsp-server` or equivalent) to the runtime stage alongside the
  TensorRT/CUDA deps VU-001 introduces.
- Image `CMD` runs both `perception_front_node` and `video_mux_node`
  (matching how DEP-001 bundles multiple nodes per container where SDD
  groups them into one service).

**Deliverables:**
- `docker/video-uplink/Dockerfile`.
- `config/systemd/drone-mc-video-uplink.service` + `be.slice` membership
  (SDD §12.3/§12.4 already describe this service).
- Build script matching the `08-build-fcc-bridge-image.sh` pattern.

**Relationships:** Depends on VU-001/VU-002 being buildable (GStreamer/
TensorRT deps declared). Blocks TEST-003 (HIL smoke test for this service).

---

#### DEP-004: params/config population across all nodes

**Points:** 4

**Description:** As a developer, I want every node's `params/default.yaml`
populated with real tunables (not the current empty placeholders), plus the
still-missing top-level config files SDD §12.5 lists, so a real deployment
doesn't start every node with undocumented default behavior.

**Acceptance Criteria:**
- Every package's `params/default.yaml` (and `params/sitl.yaml` where
  applicable, per `CLAUDE.md`'s tooling decisions) has real values for every
  tunable its Phase 2 ticket introduced (PD gains, election weights,
  failsafe timeouts, TensorRT engine path, `video_uplink`'s
  `track_iou_threshold`/`track_miss_limit`/`gimbal_link_timeout_s`, etc.) —
  not empty `ros__parameters: {}` blocks.
- `config/camera_calibration.yaml` (SDD §12.5, referenced but absent) is
  added with real or placeholder-but-documented camera intrinsics.

**Deliverables:**
- Updated `params/default.yaml` per package.
- `config/camera_calibration.yaml`.

**Relationships:** Depends on every Phase 2 ticket that introduces a
tunable (effectively a trailing cleanup ticket — can start early and be
amended incrementally as each Phase 2 ticket lands, rather than waiting for
all of them).

---

## Phase 4 — Test Infrastructure

#### TEST-001: launch_testing / pytest harness + mock nodes

**Points:** 10

**Description:** As a developer, I want the TP-NI (node integration) test
level actually built, with the mock nodes TP-MC-001 §2.4 already specifies,
so node-level integration tests can run against realistic hardware-facing
inputs without needing real hardware.

**Acceptance Criteria:**
- `launch_testing` + `pytest` infrastructure exists and runs via `colcon
  test` (currently only gtest runs).
- Implements the 4 documented mocks (TP-MC-001 §2.4): `mock_fcc_node`
  (publishes `VehicleState`/`IMU`/`Odometry` from a bag, accepts trajectory
  setpoints), `mock_camera_node` (`Image`+`CameraInfo` from a bag at ≥15
  FPS), `mock_peer_node` (swarm UDP on loopback), `mock_gcs_node` (MAVLink 2
  on UDP loopback).
- At least one example `.launch.py` test using each mock, demonstrating the
  pattern for future node-integration tests to follow.

**Deliverables:**
- `test/` (or a new `ws/src/mc_test_utils`-style support package) containing
  the 4 mock nodes and `launch_testing` fixtures.
- Sample bag recordings (or synthetic equivalents if no hardware capture is
  available yet) for `mock_fcc_node`/`mock_camera_node`.

**Relationships:** No hard blockers (can start independent of Phase 2
node work, though it's most useful once nodes exist to test). Blocks
TEST-002.

---

#### TEST-002: SITL system test harness

**Points:** 13

**Description:** As a developer, I want a PX4 SITL + Gazebo test harness
running full mission phases and failsafe scenarios (TP-ST), so the whole
system's behavior is validated end-to-end before hardware testing.

**Acceptance Criteria:**
- SITL launch files bring up PX4 v1.14 SITL (Gazebo Garden/Ignition),
  `SIMULATION_MODE=true` MC software (via FCC-007's `MavlinkUdpBackend`),
  and 2–3 swarm peer instances on loopback (TP-MC-001 §3.2's documented
  `127.0.0.x` addressing scheme).
- At least one pytest scenario per mission phase transition (full
  PREFLIGHT→...→LAND happy path) and one per major failsafe (FCC link loss,
  low battery, GCS loss).
- Runs nightly per TP-MC-001 §2.3's regression policy (CI wiring, if CI
  exists in this repo — flag as a follow-up if no CI infra exists yet to
  hook into).

**Deliverables:**
- SITL launch files + pytest scenario suite.
- `scripts/`-level launch script (`ros2 launch mission_exec
  sitl_mission_test.launch.py`, per `CLAUDE.md`'s already-documented but
  currently nonexistent command).

**Relationships:** Depends on FCC-007 (SITL backend), ME-001 (state
machine to exercise), TEST-001 (mock/launch_testing infra this builds on).
Effectively validates the entire Phase 2 backlog once complete.

---

#### TEST-003: HIL smoke test additions

**Points:** 4

**Description:** As a developer, I want smoke tests for the newly
containerized services (Phase 3), matching the existing
`0Xb-smoke-test-*.sh` convention, so a real deployment can be verified
after installation the same way `fcc_bridge`/`ros2-core`/colcon already are.

**Acceptance Criteria:**
- One smoke-test script per new service container (gcs-comms,
  health-monitor, swarm-coord, mission-exec, camera, vision, video-uplink,
  mavlink-router) verifying the container starts, the node(s) inside are
  alive, and (where applicable) the FCC-008-style transport/config
  selection took effect.

**Deliverables:**
- `scripts/0Xb-smoke-test-<service>.sh` per new service, following the
  numbering/naming convention already established.

**Relationships:** Depends on DEP-001/002/003/005 (containers must exist to
smoke-test). Last ticket in the plan — closes the loop back to
deployability.

---

## Point Summary

| Phase | Tickets | Points |
|---|---|---|
| 1 — FCC completion | FCC-006, 007, 008 | 20 |
| 2A — Mission/Swarm | SC-002, 003, 004, ME-001, 002, SC-001 | 55 |
| 2B — Vision | VIS-001, 002, 003, 004 | 34 |
| 2E — Gimbal Video | VU-001, 002 | 18 |
| 2C — GCS | GCS-001, 002, 003, 004, MAVR-001 | 32 |
| 2D — Health | HM-001, 002, 003 | 17 |
| 2F — Cross-Host DDS | ZBD-001 | 4 |
| 3 — Deployment | DEP-001, 002, 003, 004, 005, 006 | 20 |
| 4 — Test infra | TEST-001, 002, 003 | 27 |
| **Total** | **33 tickets** | **227 pts ≈ 908 hrs ≈ 113.5 person-days** |

At ~21 working days/month, 227 points ≈ 5.4 person-months for a single
developer working this backlog alone — roughly in line with the 5-month
horizon, with some slack for the unplanned work that always surfaces (SITL
environment quirks, hardware bring-up issues, ICD gaps discovered mid-ticket
the way FCC-006 discovered `IsrRole`/`FormationOffset`). `ZBD-001`/`DEP-006`
(7 of the 227 points) are already done as of 2026-07-22. `GCS-004` (8
points, added 2026-07-22) has 3 open items blocking a confident start — see
`MISSION-ORCHESTRATOR-INTERFACE-SPEC.md` §8.

Not sequenced into specific weeks/months per current scope — re-run this
sizing exercise once team size is known, or ask for a capacity-sequenced
version of this same backlog.
