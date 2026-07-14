# MC↔FCC Mission-Channel Backlog

Follow-up tickets identified while implementing FCC-004 (`fcc_bridge_node`
topic adapter). FCC-004 wired up `/mc/fcc/vehicle_state` and `/mc/fault`
only; it deliberately deferred the mission-protocol side of the MC↔FCC
channel (SDD §3.1–§3.4, ICD IR-007) — the dedicated MAVLink 2/UDP
`192.168.12.31:14555` link used for mission upload/update, since stock PX4
has no DDS-writable mission topic. This doc collects the gaps found while
tracing that channel end to end, from `fcc_bridge_node`'s missing topic
subscriptions down to the two consumer nodes (`mission_manager_node`,
`formation_controller_node`) that are still empty stubs with nothing
calling the channel at all.

No system, container, or network changes are needed for any of this —
`drone-mc-fcc-bridge.service` already runs `--network=host`, and
`config/environment` already declares `FCC_MISSION_PORT=14555` and the
ethernet target independent of `FCC_TRANSPORT`. Everything below is
software-only.

**Dependency order:**

```
FCC-005 (mission forwarding) ──> FCC-006 (ack feedback) ──┬──> ME-001 (mission_manager_node)
                                                            └──> SC-001 (formation_controller_node)

FCC-007 (MavlinkUdpBackend / SITL) — independent, no dependency on the above
FCC-008 (MicroXRCEAgent lifecycle) — independent, no dependency on the above;
                                       blocks nothing here but blocks any
                                       hardware-in-the-loop use of FCC-002 today
```

FCC-008 isn't a mission-channel gap like the rest of this doc (it's the DDS
vehicle-state/command link, not mission upload) — it's included here because
it was found the same way (tracing an FCC-00X seam end to end) and follows
the same informal ticket convention.

Ticket ID prefixes are placeholders — only `FCC-XXX` has an established
convention in this repo (informal, from code comments in `fcc_bridge`);
`ME-001`/`SC-001` should be renumbered to match whatever prefix
`mission_exec`/`swarm_coord` use in the actual tracker.

---

## FCC-005: fcc_bridge_node — mission upload, item-update, and command forwarding

**As a** developer, **I want** `fcc_bridge_node` to forward mission upload,
mission-item-update, and command requests from application nodes to the FCC
via `FccBackend`, **so that** `mission_manager_node` and
`formation_controller_node` can drive PX4 missions without touching
transport-specific code (SDD §3.4 item 2, ICD §7.1).

### Acceptance Criteria

- Subscribes `/mc/fcc/mission_upload` (`mc_msgs/FccMission`) and calls
  `FccBackend::upload_mission()` on receipt — full-replace, on-change only
  (not periodic).
- Subscribes `/mc/fcc/mission_item_update` (`mc_msgs/FccMissionItem`) and
  calls `FccBackend::update_mission_item()`; sustains ≥5 Hz during INGRESS
  (`FORMATION_SLOT` tracking, SDD §5.2) without dropping
  `/mc/fcc/vehicle_state` below 10 Hz — i.e., runs on its own callback
  group, not serialized behind the vehicle_state timer.
- Mission-item-update callbacks are serialized **against themselves**
  (single dedicated group) — the underlying MAVLink mission-protocol socket
  /handshake (`MissionProtocolClient`) is not safe for concurrent
  invocation, so back-to-back updates arriving faster than the ≤500 ms ACK
  handshake (IR-007) must queue, not race.
- Subscribes `/mc/fcc/send_command` (`mc_msgs/FccCommand`) and calls
  `FccBackend::send_command()` — event-driven, from `mission_manager` and
  `health_monitor`.
- `mc_msgs/FccMission`, `FccMissionItem`, `FccCommand` are added, mirroring
  the existing `FccBackend::upload_mission()`/`update_mission_item()`/
  `send_command()` signatures (`MissionItem` fields, `cmd` +
  `float32[7] params`) — and ICD-MC-001 §7.2 is updated to actually define
  them (currently referenced in the §7.1 topic table but missing from the
  message catalogue).
- Adapter only, zero business logic — no mission validation, frame
  conversion, or retry policy added in the node; that stays in
  `FccBackend`/the calling application node, matching FCC-004's constraint.
- No regression to FCC-004 behavior: `/mc/fcc/vehicle_state` publish rate
  and `FCC_LINK_LOST` fault detection are unaffected by mission-channel
  traffic volume.

### Deliverables

- `fcc_bridge_node.{hpp,cpp}` — reentrant callback group(s) + three new
  subscriptions (mission upload, item update, command), per the threading
  model already scaffolded in `fcc_bridge_node_main.cpp`.
- `mc_msgs/msg/FccMission.msg`, `FccMissionItem.msg`, `FccCommand.msg`.
- `ICD-MC-001.md` §7.2 updated with the three new message definitions.
- Node integration test additions to `test_fcc_bridge_node.cpp`: published
  `FccMission` reaches `upload_mission()` with correct item count/fields; a
  `FccMissionItem` stream at 5 Hz sustains dispatch while `vehicle_state`
  stays ≥10 Hz; a published `FccCommand` reaches `send_command()` with
  correct `cmd`/`params`.

### Explicitly out of scope (tracked separately)

- Mission upload/update **ACK or failure feedback** surfaced back to
  callers (see FCC-006).
- `MavlinkUdpBackend` (SITL) — mission forwarding through it stays
  unimplemented until that backend lands (see FCC-007).
- Actual mission-building logic in `mission_manager_node`/
  `formation_controller_node` — both are still stubs; this ticket only
  gives them somewhere to publish to (see ME-001, SC-001).

---

## FCC-006: FccBackend mission-handshake ack propagation + `/mc/fcc/mission_ack` feedback topic

**Depends on:** FCC-005 (needs the mission-forwarding subscriptions to
exist as a call site).

**As a** developer, **I want** `fcc_bridge_node` to report whether each
mission upload/item-update actually reached the FCC (`MISSION_ACK`
success/failure), **so that** `formation_controller_node` can implement its
FR-024 self-supervision (command HOLD after its own mission-item updates
stop landing) and `mission_manager_node` can detect a failed mission
upload before advancing its state machine.

### Background

The pass/fail signal already exists —
`MissionProtocolClient::upload_mission()`/`update_mission_item()` both
return `bool` today. It's just swallowed one layer up: `XrceDdsBackend`'s
wrappers are `void` and only log on failure (`RCLCPP_ERROR`/`RCLCPP_WARN`),
so nothing above the backend can see it. This ticket plumbs that existing
signal through, rather than inventing new failure-detection logic.

### Acceptance Criteria

- `FccBackend::upload_mission()` and `update_mission_item()` change from
  `void` to `bool` (true = `MISSION_ACK`/`MAV_MISSION_ACCEPTED` received
  before timeout; false = timeout/reject) — `XrceDdsBackend` returns
  exactly what `MissionProtocolClient` already computes, no new logic.
- `fcc_bridge_node` publishes a new `/mc/fcc/mission_ack`
  (`mc_msgs/FccMissionAck`) once per upload/update call it forwards
  (FCC-005's subscriptions), carrying: which request it acks (seq for item
  updates, an upload id for full uploads), success/failure, and a UTC
  timestamp. Event-driven — no periodic republish.
- `formation_controller_node` can derive "time since last successful
  mission-item update" purely from this topic, without its own visibility
  into the MAVLink socket — this is what unblocks FR-024 (HOLD after >2 s
  of failed updates, since PX4 won't exit MISSION mode on a stale item on
  its own).
- `fcc_bridge_node` stays zero-business-logic: it reports raw
  success/failure only. Deciding HOLD/RTL/mission-abort from repeated
  failures remains `mission_manager`'s/`formation_controller`'s job — same
  ownership split SDD §10.3 already establishes for `FCC_LINK_LOST`.
- Existing `FccBackend` callers/tests are updated for the new signature:
  `FakeBackend` (`test_fcc_backend_contract.cpp`) and `XrceDdsBackend`
  assertions (`test_xrce_dds_backend.cpp`) reflect the bool return; no
  behavioral change otherwise.
- Interface is designed so `MavlinkUdpBackend`, when it eventually lands
  (FCC-007), implements the same bool-returning signature — avoids a
  second interface break later.

### Deliverables

- `fcc_backend.hpp` signature change; `xrce_dds_backend.{hpp,cpp}` updated
  to return the client's result.
- `mc_msgs/msg/FccMissionAck.msg`.
- `fcc_bridge_node.{hpp,cpp}`: publish `/mc/fcc/mission_ack` from the
  FCC-005 forwarding callbacks.
- Updated `test_fcc_backend_contract.cpp` / `test_xrce_dds_backend.cpp` for
  the new return type; new `test_fcc_bridge_node.cpp` case asserting a
  forwarded `mission_item_update` produces a matching `mission_ack`.
- `ICD-MC-001.md` updated: new `/mc/fcc/mission_ack` row in §7.1 and
  `mc_msgs/FccMissionAck` definition in §7.2.

### Explicitly out of scope

- The actual HOLD-on-stale-update logic inside `formation_controller_node`
  (FR-024) — this ticket only makes the ack signal available; consuming it
  is that node's own ticket (SC-001).

---

## FCC-007: MavlinkUdpBackend — SITL fallback backend

**As a** developer, **I want** a `MavlinkUdpBackend` implementation of
`FccBackend`, **so that** `fcc_bridge_node` can run against PX4 SITL
(`SIMULATION_MODE=true`) over MAVLink UDP instead of uXRCE-DDS, unblocking
SITL testing of vehicle state, mission upload/update, and commands without
hardware (SDD §3.3, §13.1).

### Background

`factory.cpp` currently throws when `SIMULATION_MODE=true` — this backend
was explicitly out of scope for FCC-001/FCC-002 and has stayed
unimplemented since. Per SDD §3.3, it wraps a single MAVLink UDP socket
carrying vehicle state, commands, *and* the mission protocol together
(unlike `XrceDdsBackend`, which splits DDS state/commands from a dedicated
mission-protocol UDP port) — there is no separate DDS channel to fall back
to in SITL.

### Acceptance Criteria

- `SIMULATION_MODE=true` selects `MavlinkUdpBackend` via the factory
  (`factory.cpp`'s throw is replaced by real construction).
- `get_vehicle_state()`/`is_link_alive()` populate the same
  `fcc_bridge::VehicleState` struct from `GLOBAL_POSITION_INT`, `ATTITUDE`,
  and `BATTERY_STATUS`, matching the field mapping `XrceDdsBackend` already
  established — `fcc_bridge_node`'s adapter logic (FCC-004) needs zero
  changes to work with either backend; that interchangeability is the
  entire point of the `FccBackend` seam.
- `send_command()` emits `COMMAND_LONG` with the same `params[7]` mapping
  `XrceDdsBackend::send_command()` uses for `VehicleCommand`.
- `upload_mission()`/`update_mission_item()` reuse the existing
  `MissionProtocolClient` handshake logic (`MISSION_COUNT`/
  `MISSION_ITEM_INT`×N/`MISSION_ACK` and `MISSION_WRITE_PARTIAL_LIST`) over
  the single SITL socket instead of a separate port — same `bool`-return
  contract as FCC-006, so this backend needs no ad hoc success signal of
  its own.
- `is_link_alive()` reflects silence on the one multiplexed MAVLink socket
  (no DDS/mission-channel split, since SITL carries both on one
  connection) — `fcc_bridge_node`'s `FCC_LINK_LOST` detection (FCC-004)
  requires no changes to work against this backend.
- Node-level test analogous to `test_xrce_dds_backend.cpp`: a mock PX4 SITL
  peer (plain UDP socket in the test) sends `GLOBAL_POSITION_INT`/
  `ATTITUDE`/`BATTERY_STATUS` and this backend aggregates correctly; a mock
  `MISSION_REQUEST_INT`/`MISSION_ACK` exchange confirms upload/update
  round-trips.

### Deliverables

- `mavlink_udp_backend.{hpp,cpp}`.
- `factory.cpp` updated to construct `MavlinkUdpBackend` instead of
  throwing on `SIMULATION_MODE=true`.
- `test_mavlink_udp_backend.cpp`.

### Explicitly out of scope

- A real PX4 SITL/Gazebo integration test (TP-ST level) — this ticket only
  covers the backend's own correctness against a mock MAVLink peer,
  matching `test_xrce_dds_backend.cpp`'s existing scope (mock uORB, not
  live PX4).

---

## ME-001: mission_manager_node — mission phase state machine

**Depends on:** FCC-005 (mission upload/item-update/command forwarding),
FCC-006 (mission ack feedback).

**As a** developer, **I want** `mission_manager_node` to drive the drone
through its full mission lifecycle via the documented phase transition
table, **so that** GCS commands, FCC state, and swarm signals reliably
advance (or fault) the mission without ad hoc logic scattered across nodes.

### Acceptance Criteria

- Implements the 11-phase enum and transition table exactly as SDD §4.1
  (`PREFLIGHT…LAND`, `FAULT=255`), as data
  (`std::unordered_map<Phase, TransitionTable>` of guard/action pairs) —
  not a hardcoded `switch` — so the table is inspectable/testable
  independent of the dispatch mechanism.
- Each transition's entry action dispatches to the correct channel, per the
  transition table: `ARMING→TAKEOFF` and the `FAULT→RTL` escape call
  `FccCommand` on `/mc/fcc/send_command`; `TAKEOFF→INGRESS` uploads the
  reserved `FORMATION_SLOT` (seq 0) via `/mc/fcc/mission_item_update`;
  `VORONOI_PARTITION→SEARCH` uploads the coverage path via
  `/mc/fcc/mission_upload`.
- Mission-channel entry actions block phase advancement on the
  corresponding `/mc/fcc/mission_ack` (FCC-006), not on a fixed sleep — a
  failed upload/update keeps the state machine in place (or routes to
  `FAULT`) rather than optimistically advancing.
- GCS override authority: an override/abort command on `/mc/gcs/command` is
  acted on within 500 ms of receipt regardless of current phase (FR-003) —
  verified with a latency-bounded test, not just "eventually handled."
- Publishes `/mc/mission_phase` (`mc_msgs/MissionPhase`) on every phase
  change plus a 1 Hz keepalive while steady, and `/mc/manager/heartbeat`
  at ≥1 Hz for `health_monitor`'s watchdog (FR-074).
- Consumes `/mc/fault` (`FCC_LINK_LOST` at minimum) to trigger the
  `FAULT→RTL` escape — does not independently re-derive link health from
  `/mc/fcc/vehicle_state.fcc_link_ok`.
- `MUSTER→ISR_SELECTION`, `ISR_SELECTION→VORONOI_PARTITION`, and
  `VORONOI_PARTITION→SEARCH` guards/signals to `swarm_coordinator`/
  `search_planner` are implemented against those nodes' documented topic
  contracts (`/mc/swarm/isr_role`, `/mc/swarm/voronoi_cell`,
  `/mc/search/coverage_path`) — this ticket does **not** implement ISR
  election, Voronoi partitioning, or coverage-path generation themselves;
  those nodes are mocked per TP §13.3's mock strategy for this node's own
  tests.

### Deliverables

- `mission_manager_node.{hpp,cpp}` with the transition-table
  implementation.
- Node integration tests: one per documented transition (guard true/false,
  correct entry action dispatched, correct topic published), plus the
  500 ms GCS-override latency test and the mission-ack-blocks-advancement
  test.
- `params/default.yaml` populated with the currently-placeholder
  `mission_manager_node` tunables (phase timeouts, retry counts).

### Explicitly out of scope

- `RETURN`/`LAND` phase detail and any transitions not already fully
  specified in SDD §4.1's abridged table — flag as a follow-up if the full
  table has more rows than what's captured here.
- ISR election, Voronoi partitioning, coverage-path generation
  (`swarm_coordinator_node`, `search_planner_node` — separate tickets, not
  drafted here).
- FR-024 formation self-supervision (that's `formation_controller_node`'s,
  see SC-001).

---

## SC-001: formation_controller_node — INGRESS formation-slot PD tracking

**Depends on:** FCC-005, FCC-006, ME-001 (needs `/mc/mission_phase` to gate
activation).

**As a** developer, **I want** `formation_controller_node` to steer each
drone's `FORMATION_SLOT` mission item toward its assigned offset from the
swarm leader during INGRESS, **so that** the swarm converges into formation
without manual waypoint management, and self-protects if its updates stop
reaching the FCC.

### Acceptance Criteria

- Runs its PD control loop (SDD §5.2) only while `/mc/mission_phase`
  reports `INGRESS` — no separate enable/disable topic; the loop is a
  no-op (or fully stopped) in every other phase, matching the entry/exit
  actions already assigned to `mission_manager_node`'s
  `TAKEOFF→INGRESS`/`INGRESS→MUSTER` transitions.
- Computes `target_pos_ned` from the leader's `PEER_STATE`
  (`/mc/swarm/peer_states`) plus the formation offset (from the GCS mission
  upload, stored in NED), and the own vehicle state from
  `/mc/fcc/vehicle_state` — all PD arithmetic stays in NED; the only added
  frame transform is the final NED→geodetic (`LocalCartesian`) projection
  immediately before packaging the `MissionItem`.
- Publishes `/mc/fcc/mission_item_update` at ≥5 Hz while active (ICD §7.1
  rate requirement).
- FR-024 self-supervision: tracks time since the last **successful**
  update using `/mc/fcc/mission_ack` (FCC-006), not merely
  time-since-last-publish (a rejected/timed-out ack must count as a gap).
  If no successful ack lands within 2 s, commands HOLD itself via
  `/mc/fcc/send_command` — PX4 does not exit `MISSION` mode on a stale
  item on its own, so this node must self-detect (SDD §5.3, FR-024).
- Recovers from HOLD automatically once updates start succeeding again —
  does not require a manual GCS command to resume, and does not command
  HOLD repeatedly on every tick while already in it (edge-triggered,
  matching the pattern used for `FCC_LINK_LOST` in `fcc_bridge_node`).

### Deliverables

- `formation_controller_node.{hpp,cpp}` with the PD loop and phase-gated
  activation.
- Node integration tests: PD output correctness (known leader state + own
  state + offset → expected target and published item), phase-gating
  on/off, and the ack-timeout → self-HOLD → recovery sequence (using a
  mocked `/mc/fcc/mission_ack` stream, no live FCC needed).
- `params/default.yaml` populated with the PD gains and the 2 s
  self-supervision timeout (currently a placeholder).

### Explicitly out of scope

- Formation offset validation/geometry checks on the GCS-uploaded mission
  data — assumed valid by the time it reaches this node (upstream
  validation, if needed, is `gcs_mavlink_node`'s or `mission_manager_node`
  's concern).

---

## FCC-008: MicroXRCEAgent lifecycle — actually implement the `FCC_TRANSPORT` switch

**As a** developer, **I want** a `MicroXRCEAgent` process launched with the
transport (`serial`/`ethernet`) selected by `FCC_TRANSPORT`, **so that**
`XrceDdsBackend`'s `/fmu/in`/`/fmu/out` DDS topics are actually backed by a
live bridge to the Pixhawk, and FCC-002 works as HID-MC-001 §4.3 already
documents it.

### Background

HID-MC-001 §4.3 states: "`XrceDdsBackend` then reads `FCC_TRANSPORT` to
choose its DDS link (FCC-002) — no code changes either way." That's not what
happens. `xrce_dds_backend.cpp` reads `FCC_TRANSPORT` once
(`XrceDdsBackend`'s ctor) purely to put it in a log line — it never branches
on the value, and nothing in the repo (Dockerfile, systemd unit, or launch
script) ever runs `MicroXRCEAgent` at all, in either transport mode. HID
§3.3/§4.2 document the two launch commands
(`MicroXRCEAgent serial --dev /dev/ttyTHS1 --baudrate 921600 --rts-cts` and
`MicroXRCEAgent udp4 --port 8888`) but they exist only as reference shell
snippets in the doc, not as anything the system runs.

Concretely: `config/environment` declares `FCC_SERIAL_DEVICE`,
`FCC_SERIAL_BAUD`, and `FCC_ETH_PORT` (matching the env vars
`xrce_dds_backend.hpp`'s ctor comment says it reads), but
`xrce_dds_backend.cpp` never reads any of the three — it only reads
`FCC_ETH_IP`/`FCC_MISSION_PORT`, both of which belong to the *mission*
channel (always ethernet, independent of `FCC_TRANSPORT`), not the DDS
link. So today, with `SIMULATION_MODE=false`, `fcc_bridge_node` starts,
`XrceDdsBackend` subscribes/publishes on the `/fmu/*` DDS topics, and
nothing ever bridges those to the Pixhawk — the node runs but is silently
disconnected from real vehicle state until `FCC_LINK_LOST` eventually fires
from `is_link_alive()` timing out. This is a functional gap, not just a
missing log branch.

### Acceptance Criteria

- A `MicroXRCEAgent` process is running wherever `fcc_bridge` runs, started
  before (or supervised alongside) `fcc_bridge_node`, so the `/fmu/in`/
  `/fmu/out` DDS topics `XrceDdsBackend` uses are backed by a real bridge to
  the FCC — not just present because CycloneDDS discovery succeeded between
  two ROS nodes with nothing on the other end.
- Transport is selected from `FCC_TRANSPORT` exactly as HID-MC-001 §4.3
  documents: `serial` → `MicroXRCEAgent serial --dev $FCC_SERIAL_DEVICE
  --baudrate $FCC_SERIAL_BAUD --rts-cts`; `ethernet` → `MicroXRCEAgent udp4
  --port $FCC_ETH_PORT`. `FCC_SERIAL_DEVICE`, `FCC_SERIAL_BAUD`, and
  `FCC_ETH_PORT` — already declared in `config/environment` and documented
  in `xrce_dds_backend.hpp`'s ctor comment — are consumed for the first time
  here (`XrceDdsBackend` itself still doesn't need them; they're the
  agent's args, not the ROS node's).
- Agent lifecycle is supervised by systemd, not the application: if the
  agent dies, `Restart=on-failure` brings it back per the existing
  `drone-mc-fcc-bridge.service` policy (or its own unit's policy, per
  whichever deliverable option below is chosen) — `fcc_bridge_node` is not
  responsible for spawning or restarting it.
- `is_link_alive()`/`FCC_LINK_LOST` (FCC-004) behavior is unchanged — this
  ticket makes the link real, it doesn't touch link-health detection logic.
- `HID-MC-001.md` §4.3 (and `SDD-MC-001.md`'s deployment section, if it
  describes container contents) is updated to state where the agent process
  actually lives, replacing the current "no code changes either way"
  language, which is inaccurate.

### Deliverables

- One of:
  - **(a)** A new `drone-mc-xrce-agent.service` systemd unit (paralleling
    `drone-mc-mavlink-router.service`), with `drone-mc-fcc-bridge.service`
    gaining `After=`/`Requires=` on it — matches the existing pattern of
    giving an independent-lifecycle bridge process its own unit rather than
    embedding it in another container.
  - **(b)** `docker/fcc-bridge/Dockerfile`'s `CMD` changed to launch
    `MicroXRCEAgent` as a supervised background process (e.g. via a small
    entrypoint script) ahead of `exec ros2 run fcc_bridge fcc_bridge_node`.
  - (a) is preferred if the agent needs restart/cgroup accounting
    independent of the ROS node's own crashes; (b) is simpler if they're
    meant to live and die together. Pick one and note the reasoning in the
    PR — HID-MC-001 §4.3 currently doesn't say either way.
- Updated `HID-MC-001.md` §4.3 reflecting the actual deployment.
- Smoke test addition (`04b-smoke-test-ros2.sh` or a new
  `0Xb-smoke-test-xrce-agent.sh`, per the existing `NNb-smoke-test-*.sh`
  convention) verifying the agent process is running and the selected
  transport matches `FCC_TRANSPORT` after `drone-mc.target` starts.

### Explicitly out of scope

- Any change to `XrceDdsBackend`'s own env var contract or logic — it stays
  exactly as documented in its ctor comment.
- `MavlinkUdpBackend`/SITL (`SIMULATION_MODE=true`) — that path doesn't use
  `MicroXRCEAgent` at all and is FCC-007's concern.
- Deeper agent health monitoring beyond systemd's own restart policy (e.g.
  surfacing agent-specific faults to `/mc/fault`) — `FCC_LINK_LOST` via
  `is_link_alive()` already covers "no data arriving," which is the
  observable symptom `health_monitor` needs regardless of *why* the agent is
  down.

---

## Note on sizing

ME-001 and SC-001 are noticeably larger units of work than the FCC-00X
tickets — a full node implementation each, versus an adapter extension.
Both may be worth splitting further once picked up (e.g., ME-001 into
"transition table + FCC dispatch" and "GCS override + fault escape" as
separate passes). They're kept as single tickets here because their
responsibilities (the state machine; the PD loop) are each one coherent
unit that doesn't decompose cleanly along the mission-channel boundary this
doc is organized around.
