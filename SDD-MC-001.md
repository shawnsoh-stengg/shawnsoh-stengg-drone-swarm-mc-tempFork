# Software Design Document
## Swarm Search Drone — Mission Computer (AI MC)

| Field | Value |
|---|---|
| Document | MC-SDD-001 |
| Version | 0.1-DRAFT |
| Date | 2026-06-20 |
| Parent SRS | MC-SRS-001 v0.1-DRAFT |
| Parent ICD | MC-ICD-001 v0.1-DRAFT |
| Platform | Jetson Orin NX 16GB · JetPack 6.2.2 · Ubuntu 22.04 aarch64 |
| Language | C++17 (nodes) · Python 3 (utilities) |
| Build | colcon · Meson/Ninja (mavlink-router) · Docker Engine |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Architectural Design](#2-architectural-design)
3. [FccBackend — MC↔FCC Abstraction](#3-fccbackend--mcfcc-abstraction)
4. [mission_executor_node — Mission Phase State Machine](#4-mission_executor_node--mission-phase-state-machine)
5. [formation_controller_node — Formation Keeping](#5-formation_controller_node--formation-keeping)
6. [swarm_coordinator_node — Swarm Management](#6-swarm_coordinator_node--swarm-management)
7. [search_planner_node — Coverage Path Planning](#7-search_planner_node--coverage-path-planning)
8. [visual_localisation_node — GNSS-Denied Localisation](#8-visual_localisation_node--gnss-denied-localisation)
9. [perception_down_node — AI Inference Pipeline](#9-perception_down_node--ai-inference-pipeline)
9a. [perception_front_node and video_mux_node — Gimbal Video Perception and Uplink](#9a-perception_front_node-and-video_mux_node--gimbal-video-perception-and-uplink)
10. [health_monitor_node — System Health and Failsafe](#10-health_monitor_node--system-health-and-failsafe)
11. [gcs_mavlink_node — GCS Communication](#11-gcs_mavlink_node--gcs-communication)
12. [Deployment Design](#12-deployment-design)
13. [Testing and Validation Design](#13-testing-and-validation-design)

---

## 1. Introduction

### 1.1 Purpose

This document describes the software design of the Mission Computer (MC) for the Swarm Search Drone. It specifies how each software component is structured, the algorithms used, the internal data flows, and the design decisions taken to satisfy the requirements in MC-SRS-001 and the interface contracts in MC-ICD-001.

### 1.2 Design Philosophy

Four principles govern the design:

1. **Loose coupling via ROS 2 topics.** No node holds a direct reference to another. All communication is through typed topics on the CycloneDDS bus. This allows individual nodes to be replaced, mocked, or tested in isolation.

2. **Transport agnosticism at the FCC boundary.** The `FccBackend` abstraction ensures that no application node knows whether the FCC is reached via uXRCE-DDS or MAVLink UDP. This is the single most important seam in the architecture.

3. **Single source of truth per domain.** One node owns each domain: `mission_executor_node` owns mission phase; `swarm_coordinator_node` owns peer state and cell assignment; `health_monitor_node` owns system health. Others subscribe, never assert.

4. **Fail toward safety.** Every timeout, every lost link, every unrecoverable node failure drives the system toward a known-safe state (HOLD or RTL). The design never silently degrades.

### 1.3 Relationship to Other Documents

| Document | Relationship |
|---|---|
| MC-SRS-001 | Requirements this design satisfies |
| MC-ICD-001 | Interface contracts implemented by this design |
| Diagrams 01–09 | Architectural context and interface topology |

### 1.4 Technology Summary

| Concern | Choice | Rationale |
|---|---|---|
| Middleware | ROS 2 Humble / CycloneDDS | Deterministic discovery, DDS QoS, PX4 uXRCE-DDS compatibility |
| Language | C++17 | Performance for control loops; structured bindings, `std::optional`, `std::variant` |
| Concurrency | ROS 2 multi-threaded executor + `rclcpp::CallbackGroup` | Isolates timer callbacks from subscription callbacks per node |
| Build | colcon + CMake | Standard ROS 2 workspace tooling |
| Containers | Docker Engine (not Compose) | Systemd integration via `ExecStart=docker run ...` |
| Configuration | Environment variables + ROS 2 parameter YAML | Runtime tunable without rebuild |

---

## 2. Architectural Design

### 2.1 Layered Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  Layer 4 — Mission Intelligence                              │
│  swarm_coordinator · mission_executor · search_planner       │
│  visual_localisation · perception_down · perception_front    │
├──────────────────────────────────────────────────────────────┤
│  Layer 3 — GCS and Swarm Communication                       │
│  gcs_mavlink_node · MAVROS · mavlink-router · video_mux_node  │
├──────────────────────────────────────────────────────────────┤
│  Layer 2 — FCC Abstraction (FccBackend)                      │
│  fcc_bridge_node · Micro XRCE-DDS Agent                      │
├──────────────────────────────────────────────────────────────┤
│  Layer 1 — Hardware and Transport                            │
│  ttyTHS1 serial · 192.168.12.x ethernet · CSI camera         │
│  MANET mesh radio · PX4 uORB                                 │
└──────────────────────────────────────────────────────────────┘
```

Layers only communicate downward via well-defined interfaces. Layer 4 nodes never touch Layer 1 resources directly.

### 2.2 Node Dependency Graph

```
                    GCS
                     │ MAVLink 2/UDP
                     ▼
              gcs_mavlink_node
                 │         │
         /mc/gcs/command   │
                 │    /mc/gcs/mission_upload
                 ▼         ▼
          mission_executor_node ◄── /mc/executor/heartbeat (self)
           │       │       │
    /mc/   │  /mc/ │  /mc/ │
   mission │  fcc/ │  swarm│
   _phase  │  send_│  /isr_│
           │  comm │  role │
           │  and  │       │
           │ set_  │       ▼
           │ traj  │  swarm_coordinator_node
           │       │       │        │
           ▼       │  /mc/swarm/   /mc/swarm/
    fcc_bridge_node│  peer_states  voronoi_cell
           │       │       │        │
    uXRCE-DDS      │       ▼        ▼
           │       │  formation_  search_
           ▼       │  controller  planner_node
         FCC       │  _node            │
                   │       │      /mc/search/
                   │ /mc/  │      coverage_path
                   │ form/ │           │
                   │ setp  ▼           ▼
                   └──► fcc_bridge_node (aggregated)
                              │
                    health_monitor_node ◄── /mc/executor/heartbeat
                         │
                    /mc/health  /mc/fault
                         │
                  gcs_mavlink_node (→ GCS)
```

`perception_front_node` and `video_mux_node` (§9a) form a separate pipeline — gimbal camera → GCS video — with no dependency on the graph above; a stall or fault in either has no effect on flight control or the swarm/GCS telemetry paths shown here.

### 2.3 Threading and Concurrency Model

Each node uses `rclcpp::executors::MultiThreadedExecutor` with two `CallbackGroup` instances:

- **Reentrant group** — subscription callbacks (data processing). Multiple callbacks may run in parallel.
- **Mutually exclusive group** — timer callbacks (periodic publish) and service handlers. Serialised to protect shared state.

Nodes with tight control loops (`formation_controller_node`, `fcc_bridge_node`) use a dedicated 10 ms `rclcpp::WallTimer` pinned to the mutually exclusive group, with subscription callbacks feeding a shared atomic state struct protected by `std::atomic` or `std::mutex` depending on complexity.

```cpp
// Pattern used in formation_controller_node
struct LeaderState {
    std::array<float,3> pos_ned;
    std::array<float,3> vel_ned;
    rclcpp::Time        stamp;
    bool                valid{false};
};
std::mutex          leader_mtx_;
LeaderState         leader_state_;   // written by sub callback, read by timer
```

### 2.4 ROS 2 Parameter Management

All tunable values are declared as ROS 2 parameters with defaults. A `params/` directory in each package holds per-environment YAML files:

```
fcc_bridge/params/
  default.yaml          ← serial transport, production defaults
  sitl.yaml             ← simulation transport overrides
formation_controller/params/
  default.yaml          ← Kp/Kd gains, max setpoint velocity
```

Parameters are loaded at node startup via `--ros-args --params-file` in the container `CMD`. Dynamic parameter updates via `rclcpp::Node::add_on_set_parameters_callback` are supported for gains only; transport selection requires restart.

### 2.5 Simulation Mode

When `SIMULATION_MODE=true`:
- `fcc_bridge_node` uses `MavlinkUdpBackend` pointed at the SITL UDP address instead of uXRCE-DDS.
- `camera_node` publishes synthetic frames from a ROS 2 bag or a Gazebo camera topic.
- `perception_front_node` connects to a local test RTSP server (e.g. `rtsp-simple-server` looping a recorded clip) instead of the physical gimbal camera at 192.168.10.200.
- All other nodes are unchanged — the transport-agnostic design makes this transparent.

This is enforced by a factory function called once at node startup, not by conditional compilation.

---

## 3. FccBackend — MC↔FCC Abstraction

**SRS refs:** FR-016, IR-001–006  
**Package:** `fcc_bridge`

### 3.1 Class Design

```
FccBackend  (abstract base)
│
├── XrceDdsBackend       ← production: uXRCE-DDS over ttyTHS1 or ethernet
└── MavlinkUdpBackend    ← SITL / fallback: MAVLink UDP
```

```cpp
class FccBackend {
public:
    virtual ~FccBackend() = default;

    // Upload or fully replace the active FCC mission (search paths, ingress/muster legs)
    virtual void upload_mission(const std::vector<MissionItem> & items) = 0;

    // Replace a single mission item in place (used for the moving formation slot)
    virtual void update_mission_item(uint16_t seq, const MissionItem & item) = 0;

    // Send a vehicle_command
    virtual void send_command(uint32_t cmd, std::array<float,7> params) = 0;

    // Send visual odometry feedback
    virtual void publish_visual_odometry(const VisualOdometry & vo) = 0;

    // Latest aggregated vehicle state (thread-safe)
    virtual VehicleState get_vehicle_state() const = 0;

    // True if the FCC link has been alive within the last timeout_s seconds
    virtual bool is_link_alive(double timeout_s = 1.0) const = 0;
};

// Factory: reads FCC_TRANSPORT env var, returns appropriate implementation
std::unique_ptr<FccBackend> make_fcc_backend();
```

### 3.2 XrceDdsBackend

Owns subscriptions to all `/fmu/out/` topics and publishers to all `/fmu/in/` topics. Aggregates incoming uORB messages into a single `VehicleState` struct updated atomically.

**State aggregation** merges seven asynchronous topic streams into one coherent snapshot. A `std::mutex`-protected struct is updated on each incoming message. `get_vehicle_state()` returns a copy of the struct under the lock.

```cpp
// Called on each vehicle_local_position message
void XrceDdsBackend::on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    auto lock = std::scoped_lock(state_mtx_);
    state_.x  = msg->x;
    state_.y  = msg->y;
    state_.z  = msg->z;
    state_.vx = msg->vx;
    state_.vy = msg->vy;
    state_.vz = msg->vz;
    state_.xy_valid = msg->xy_valid;
    last_fcc_msg_time_ = node_->now();
}
```

**Link liveness** uses `last_fcc_msg_time_` compared against `node_->now()`. If the gap exceeds `timeout_s`, `is_link_alive()` returns false and the owning `fcc_bridge_node` publishes `FCC_LINK_LOST` to `/mc/fault`.

**Mission item transport.** Stock PX4 has no DDS-writable mission topic — mission items live in FCC-side dataman, reachable only through the standard MAVLink mission protocol. `XrceDdsBackend` therefore also owns a MAVLink 2/UDP socket to `192.168.12.31:14555`, riding the existing HIF-002 Ethernet link (already MAVLink-capable per HID-MC-001 §4 — no new hardware). This channel is independent of the `FCC_TRANSPORT` primary/secondary DDS selection (§3.1); it is always up over 192.168.12.x regardless of which link currently carries uXRCE-DDS. `vehicle_state` and `vehicle_command` are unaffected by this — they stay on uXRCE-DDS. This is the only MAVLink traffic on the MC↔FCC interface (IR-007).

- `upload_mission(items)` drives the full handshake — `MISSION_COUNT` → `MISSION_ITEM_INT` ×N → `MISSION_ACK` (the same state machine `gcs_mavlink_node` uses for GCS uploads, §11.2). Used for search paths and INGRESS/MUSTER legs — infrequent, on mission change only.
- `update_mission_item(seq, item)` drives the MAVLink **partial mission update** microservice instead of a full re-upload — `MISSION_WRITE_PARTIAL_LIST(seq, seq)` → FCC's `MISSION_REQUEST_INT(seq)` → `MISSION_ITEM_INT(seq)` → `MISSION_ACK` — replacing only the target slot. Used for the `FORMATION_SLOT` (`seq = 0`) at ≥ 5 Hz during INGRESS; a full-handshake re-upload at that rate would be far too heavyweight.

There is no OFFBOARD-style setpoint stream: the FCC always flies in `MISSION` mode, and neither mission-protocol exchange has a PX4-imposed minimum rate — a gap does not cause a mode change (contrast the old `trajectory_setpoint`/`offboard_control_mode` pair).

### 3.3 MavlinkUdpBackend

Used in simulation and as a fallback. Wraps a pymavlink or C++ MAVLink UDP socket carrying vehicle state, commands, *and* the mission protocol over the same connection — there is no separate DDS channel to fall back to. Translates `send_command()` calls into `COMMAND_LONG` MAVLink messages and parses incoming `GLOBAL_POSITION_INT`, `ATTITUDE`, and `BATTERY_STATUS` messages into the same `VehicleState` structure. `upload_mission()` drives the full `MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK` handshake; `update_mission_item()` drives the `MISSION_WRITE_PARTIAL_LIST` partial-update microservice — the identical protocol logic `XrceDdsBackend` uses for its dedicated mission channel (§3.2), just multiplexed over the one socket instead of a separate port.

### 3.4 fcc_bridge_node

A thin ROS 2 node that:
1. Instantiates the appropriate `FccBackend` via the factory.
2. Subscribes to `/mc/fcc/mission_upload`, `/mc/fcc/mission_item_update`, and `/mc/fcc/send_command` and forwards to `FccBackend`.
3. Publishes `/mc/fcc/vehicle_state` at 10 Hz from `FccBackend::get_vehicle_state()`.
4. Publishes `/mc/fcc/imu` and `/mc/fcc/odometry` from the corresponding uORB topics.
5. Monitors link liveness and publishes `/mc/fault` on loss.

The node does **no business logic** — it is purely an adapter.

---

## 4. mission_executor_node — Mission Phase State Machine

**SRS refs:** FR-001–005, FR-010–016, FR-060–064  
**Package:** `mission_exec`

### 4.1 State Machine Implementation

The state machine is implemented as a `std::unordered_map<Phase, TransitionTable>` where each `TransitionTable` lists valid next phases and their guard conditions. This avoids a large `switch` statement and makes the transition table inspectable at runtime.

```cpp
enum class Phase : uint8_t {
    PREFLIGHT = 0, ARMING = 1, TAKEOFF = 2, INGRESS = 3,
    MUSTER = 4, ISR_SELECTION = 5, VORONOI_PARTITION = 6,
    SEARCH = 7, RETURN = 8, LAND = 9, FAULT = 255
};

struct Transition {
    Phase           target;
    std::function<bool()> guard;   // must return true for transition to proceed
    std::function<void()> action;  // executed on entering target phase
};

using TransitionTable = std::vector<Transition>;
```

**Transition table (abridged):**

| From | To | Guard | Entry action |
|---|---|---|---|
| `PREFLIGHT` | `ARMING` | GCS ARM auth received AND pre-arm checks pass | Log entry; notify swarm |
| `ARMING` | `TAKEOFF` | FCC arming_state == ARMED | Send TAKEOFF command to FCC |
| `TAKEOFF` | `INGRESS` | FCC at target altitude AND GCS INGRESS command | Upload reserved formation slot (seq 0), switch FCC to MISSION mode targeting it, enable formation controller |
| `INGRESS` | `MUSTER` | FCC waypoint complete (muster point reached) | Disable formation ctrl; notify GCS |
| `MUSTER` | `ISR_SELECTION` | All peers in MUSTER AND quorum count met | Signal swarm_coordinator |
| `ISR_SELECTION` | `VORONOI_PARTITION` | ISR result ACKed by all peers | Signal swarm_coordinator |
| `VORONOI_PARTITION` | `SEARCH` | Voronoi cell accepted AND coverage path received | Upload path to FCC; enable detection |
| `SEARCH` | `RETURN` | FCC mission complete OR GCS END_MISSION | Disable detection pipeline |
| `RETURN` | `LAND` | FCC at home waypoint | Send LAND command |
| `LAND` | `[*]` | FCC disarmed AND on ground | Log mission complete |
| Any | `RETURN` | GCS ABORT command | Cancel active FCC mission |
| Any | `FAULT` | `health_monitor_node` CRITICAL_FAULT event | Command RTL |

### 4.2 Pre-Arm Checks

Called as a guard before `PREFLIGHT → ARMING`. Returns true only if **all** conditions hold:

```
pre_arm_checks():
  ✓  FCC link alive (fcc_backend.is_link_alive())
  ✓  FCC arming_state == STANDBY (not STANDBY_ERROR)
  ✓  GCS ARM authorisation flag set in current mission upload
  ✓  Battery remaining > LOW_BATTERY_THRESHOLD + 10% margin
  ✓  All swarm peers in PREFLIGHT or ARMING phase
  ✓  Mission upload complete (waypoints, formation_offsets, search_polygon, swarm_manifest)
  ✓  Visual localisation node alive (if GNSS quality below threshold)
```

Any failed check generates a `STATUSTEXT` with the specific reason. The node does not arm without all checks passing — it does not attempt partial arm.

### 4.3 GCS Authority Model

The GCS retains authority to command phase transitions at any time (FR-003). GCS commands arrive as `/mc/gcs/command` messages. The executor processes them before evaluating autonomous transitions. The priority order is:

1. **Emergency commands** (`DISARM`, `RTL`): bypass transition table, execute immediately.
2. **Phase commands** (`PHASE_TRANSITION`): validated against the transition table; rejected if guard fails.
3. **Autonomous transitions**: evaluated only if no pending GCS command.

### 4.4 Watchdog Heartbeat

The executor publishes `std_msgs/Header` to `/mc/executor/heartbeat` at ≥ 1 Hz inside its main timer callback. If the node hangs or crashes, this topic goes silent. The `health_monitor_node` detects the silence within 5 s and initiates restart (SFR-003).

The heartbeat is the last thing published in each timer cycle — if it is published, all preceding logic in that cycle completed successfully.

### 4.5 Mission Upload Storage

Mission data is written to disk as a YAML file at `/var/drone-mc/mission_upload.yaml` on receipt and re-loaded at node startup. This ensures persistence across container restarts without requiring a fresh GCS upload after a watchdog restart.

---

## 5. formation_controller_node — Formation Keeping

**SRS refs:** FR-020–025, IR-004–006  
**Package:** `swarm_coord`

### 5.1 Control Architecture

```
/mc/swarm/peer_states ──► Leader state extractor
                              │
                              ▼
/mc/fcc/vehicle_state ──► PD target computation ──► NED→geodetic ──► /mc/fcc/mission_item_update
                              │                       projection      (seq = FORMATION_SLOT)
                         Gain scheduler
                        (rclcpp params)
```

The controller runs at **5 Hz** on a `rclcpp::WallTimer`. The FCC stays in `MISSION` mode throughout INGRESS; the controller tracks the leader by repeatedly replacing a single reserved mission item (`FORMATION_SLOT = 0`, uploaded during the `TAKEOFF → INGRESS` transition, SDD §4.1) rather than streaming an OFFBOARD setpoint. PD math is still done in the NED local frame; only the final target is converted to geodetic before being packaged into the `MissionItem`.

### 5.2 Mission-Item Target Computation

The error is computed in NED relative to the desired formation position:

```
desired_pos_ned = leader.pos_ned + formation_offset_ned

e_pos = desired_pos_ned - own_pos_ned          (position error, m)
e_vel = leader.vel_ned  - own_vel_ned          (velocity error, m/s)

// MissionItem carries no velocity field, so the D-term becomes a position
// lead (over lead_time_s) instead of a velocity feedforward:
target_pos_ned = desired_pos_ned + Kp * e_pos + Kd * e_vel * lead_time_s
```

`target_pos_ned` is clamped so the implied correction never exceeds `max_vel_xy` / `max_vel_z` × the 5 Hz cycle period, then converted to geodetic (`lat`, `lon`, `alt_msl`) via `GeographicLib::LocalCartesian` — the same local-tangent origin used by `search_planner_node` (§7.3) — and written into a `MissionItem` (`command = MAV_CMD_NAV_WAYPOINT`, `frame = MAV_FRAME_GLOBAL_INT`). The item is published via `FccBackend::update_mission_item(FORMATION_SLOT, item)`.

`acceptance_radius` on the formation slot is set well above the expected tracking error (`formation.slot_acceptance_radius_m`, default 15 m). Because the slot is continuously replaced rather than sequenced, a small acceptance radius risks PX4 latching a stale "waypoint reached" state between replacements; the wide radius trades tight arrival detection (not needed — the slot is never "completed", only tracked) for stable continuous tracking.

**Gain parameters (ROS 2):**

| Parameter | Default | Description |
|---|---|---|
| `formation.kp_xy` | `0.5` | Proportional gain, horizontal axes |
| `formation.kp_z` | `0.4` | Proportional gain, vertical axis |
| `formation.kd_xy` | `0.2` | Derivative / lead gain, horizontal |
| `formation.kd_z` | `0.15` | Derivative / lead gain, vertical |
| `formation.lead_time_s` | `0.4` | Lookahead applied to the velocity-error lead term |
| `formation.max_vel_xy` | `5.0` | Implied target-motion saturation, horizontal (m/s) |
| `formation.max_vel_z` | `2.0` | Implied target-motion saturation, vertical (m/s) |
| `formation.slot_acceptance_radius_m` | `15.0` | Acceptance radius on the reserved formation mission item |

Target motion between consecutive cycles is clamped to `max_vel_xy` / `max_vel_z` before projection to prevent commanding an unreachable jump in the mission item.

### 5.3 Update Cadence and Mission-Mode Safety

Unlike OFFBOARD, PX4's `MISSION` mode does not exit autonomously if mission item updates stop arriving — the FCC simply continues toward the last-written item indefinitely. The 5 Hz cadence above is therefore a tracking-quality target, not a mode-retention requirement (contrast the old `trajectory_setpoint` ≥ 2 Hz floor). The staleness safety net moves entirely into `formation_controller_node`: if its own `WallTimer` stalls (no leader state, no FCC link, or the node itself hangs), the watchdog in §5.4 and the executor heartbeat (§4.4) are what command HOLD — PX4 will not do it for us.

### 5.4 Leader State Staleness

On every timer tick, the controller checks the age of the last received leader state:

```cpp
auto age = node_->now() - leader_state_.stamp;
if (age > rclcpp::Duration(2, 0)) {          // FR-024: 2 s threshold
    publish_fault(FaultId::LEADER_LOST);
    send_fcc_command(MAV_CMD_NAV_LOITER_UNLIM);  // HOLD
    return;
}
```

After recovering leader contact, the controller resumes normal operation on the next timer tick without requiring a phase transition.

### 5.5 Coordinate Frame Consistency

The leader's state is received in NED from the swarm heartbeat (`PEER_STATE` message). The own vehicle state comes from `/mc/fcc/vehicle_state` also in NED. The formation offset is stored in NED from the GCS mission upload. All PD arithmetic (§5.2) stays in NED — the single added frame transform versus the old OFFBOARD design is the final `LocalCartesian` projection of `target_pos_ned` to geodetic immediately before packaging the `MissionItem`.

---

## 6. swarm_coordinator_node — Swarm Management

**SRS refs:** FR-030–036, IR-020–023  
**Package:** `swarm_coord`

### 6.1 Peer Discovery and Lifecycle

```
PeerState enum: UNKNOWN → ALIVE → LOST → DEAD
```

At startup, the coordinator reads the swarm manifest from `/mc/gcs/mission_upload` and creates a `Peer` entry for each drone. A dedicated UDP socket (port 14900 for heartbeats, port 14901 for coordination) handles all peer traffic. Reception runs on a background `std::thread`; processing is posted to the ROS 2 callback queue via `rclcpp::Node::create_wall_timer` polling.

**Peer liveness tracking:**

```cpp
struct Peer {
    uint8_t         id;
    std::string     ip;
    PeerState       state{PeerState::UNKNOWN};
    rclcpp::Time    last_heartbeat;
    SwarmHeartbeat  last_msg;
};

void on_heartbeat_received(uint8_t sender_id, const SwarmHeartbeat & hb) {
    auto & peer = peers_.at(sender_id);
    peer.last_heartbeat = node_->now();
    peer.last_msg = hb;
    if (peer.state != PeerState::ALIVE) {
        peer.state = PeerState::ALIVE;
        publish_peer_recovery_event(sender_id);
    }
}

// Called at 1 Hz
void check_peer_liveness() {
    for (auto & [id, peer] : peers_) {
        auto age = (node_->now() - peer.last_heartbeat).seconds();
        if (peer.state == PeerState::ALIVE && age > 3.0) {
            peer.state = PeerState::LOST;
            publish_peer_lost_event(id);      // → mission_executor, GCS
        } else if (peer.state == PeerState::LOST && age > 30.0) {
            peer.state = PeerState::DEAD;
        }
    }
}
```

### 6.2 ISR Selection Algorithm

The coordinator (drone with the lowest ID in the swarm manifest) runs the election after collecting `ISR_CANDIDATE` messages from all ALIVE peers within a 5 s collection window.

**Scoring function:**

```
score(drone) = w_bat  × battery_pct / 100.0
             + w_prox × (1.0 / max(dist_to_centroid_m, 1.0))
             + w_sens × sensor_ok(sensor_flags)
```

Default weights: `w_bat = 0.4`, `w_prox = 0.4`, `w_sens = 0.2`

`sensor_ok()` returns 1.0 if bits `CAMERA_OK` and `AI_INFERENCE_OK` are both set in `sensor_flags`, else 0.0.

Drones are ranked by score. The top `ceil(N/2)` drones (where N = ALIVE peer count) are elected to the ISR subset. The coordinator broadcasts `ISR_RESULT` and collects `ISR_RESULT_ACK` from all peers within 5 s. If any ACK is missing, the result is re-broadcast once. After two failed re-broadcasts, the mission executor is notified of a coordination fault.

If the coordinator drone itself is LOST, the next-lowest-ID ALIVE drone assumes coordinator role. This is detected by each drone monitoring whether any `ISR_RESULT` is received within 8 s of entering `ISR_SELECTION`; if not, the next-lowest-ID drone initiates the election.

### 6.3 Voronoi Partition Algorithm

**Step 1 — Position exchange.** Each drone broadcasts its current geodetic position as a `VORONOI_PROPOSAL`. The coordinator waits for proposals from all ALIVE ISR-selected drones within 5 s.

**Step 2 — Voronoi computation.** Each drone independently computes the Voronoi diagram from the collected positions and clips it to the search polygon. The algorithm is deterministic given the same input set, so all drones produce identical results.

```
voronoi_partition(sites, polygon):
  1. Project geodetic sites to a local Cartesian plane (ENU, origin = polygon centroid)
  2. Compute Voronoi diagram using Fortune's sweep-line algorithm
     (O(n log n) for n drones — n ≤ 16, so latency is negligible)
  3. For each Voronoi cell:
     a. Clip cell polygon against the search area polygon using Sutherland-Hodgman
     b. Retain only the cell assigned to this drone (matched by site index)
  4. Back-project cell boundary vertices from Cartesian to geodetic
  5. Return cell as ordered vertex list (CCW)
```

**Step 3 — Consensus.** Each drone sends `VORONOI_ACCEPT` containing its computed cell boundary. To confirm convergence, each drone checks that the received `VORONOI_ACCEPT` from every peer matches what it computed locally (same vertex count and coordinates within 1 cm tolerance). If all match, consensus is declared. Mismatch triggers a re-proposal round (round counter incremented).

**Re-partition on peer loss** (FR-036): If a peer is declared LOST during SEARCH, the remaining drones re-enter `VORONOI_PARTITION` with the updated ALIVE set. The search planner on each drone generates a new coverage path for the new cell. The current FCC waypoint mission is cancelled and the new path uploaded.

### 6.4 Swarm Heartbeat Publisher

A `rclcpp::WallTimer` at 1 Hz builds and sends a `SWARM_HEARTBEAT` packet to each peer's IP at port 14900. The packet is constructed from the latest `/mc/fcc/vehicle_state` and `/mc/mission_phase` data.

The coordinator also publishes `/mc/swarm/peer_states` (a `SwarmPeerArray`) at 1 Hz for internal consumption by `formation_controller_node` and `mission_executor_node`.

---

## 7. search_planner_node — Coverage Path Planning

**SRS refs:** FR-040–046  
**Package:** `mission_exec`

### 7.1 Algorithm: Boustrophedon Coverage

```
boustrophedon_path(cell_polygon, footprint_width, overlap):

  overlap_fraction = clamp(overlap, 0.0, 0.9)
  row_spacing      = footprint_width * (1.0 - overlap_fraction)

  1. Find the minimum-area bounding rectangle of cell_polygon
     (rotate-calipers on convex hull — O(n))
  2. Align the sweep direction along the longer axis of the bounding rectangle
  3. Generate parallel sweep lines perpendicular to the sweep axis,
     spaced row_spacing apart, spanning the full bounding rectangle
  4. For each sweep line:
     a. Intersect with cell_polygon to get entry and exit points
     b. If intersection is non-empty, add as a row waypoint pair
  5. Order rows: alternate left→right and right→left (boustrophedon pattern)
     to minimise transition distance
  6. Insert a transition waypoint between consecutive rows at altitude + 5 m
     to allow FCC to turn without overshooting
  7. Project all Cartesian waypoints back to geodetic (lat, lon)
  8. Return ordered waypoint list with:
       - position (lat, lon, alt_msl)
       - acceptance_radius = row_spacing / 2
       - speed = configured search speed (default 5 m/s)
```

**Parameters (ROS 2):**

| Parameter | Default | Description |
|---|---|---|
| `search.footprint_width_m` | `20.0` | Camera ground footprint width at operating altitude (m) |
| `search.overlap_fraction` | `0.2` | Row overlap (0.0–0.9) |
| `search.search_altitude_m` | `50.0` | AGL search altitude; converted to MSL using terrain model |
| `search.search_speed_ms` | `5.0` | Waypoint-to-waypoint transit speed (m/s) |
| `search.transition_alt_offset_m` | `5.0` | Extra altitude gain during row transitions |

### 7.2 Path Upload and Execution

On receiving a new `/mc/swarm/voronoi_cell`, the planner:
1. Computes the path (≤ 5 s, PR-008).
2. Publishes the path to `/mc/search/coverage_path`.
3. The `mission_executor_node` receives the path, formats it as PX4 mission items, and uploads via `fcc_bridge_node`.

On cell update (re-partition), the planner re-runs immediately. The executor cancels the active FCC mission and uploads the new path. The starting waypoint is chosen as the path waypoint nearest to the drone's current position to minimise dead transit.

### 7.3 Coordinate Projection

All internal geometry operates in a local Cartesian ENU frame (origin = cell centroid). The `GeographicLib::LocalCartesian` class handles forward and inverse geodetic projections. This avoids the distortion of naïve lat/lon arithmetic over the cell area (typically < 1 km²).

---

## 8. visual_localisation_node — GNSS-Denied Localisation

**SRS refs:** FR-050–053, IR-032  
**Package:** `vision`

### 8.1 Visual Odometry Pipeline

```
Camera frame (/camera/image_raw)
      │
      ▼
 Feature extraction (FAST corners, ORB descriptors)
      │
      ▼
 Feature tracking (KLT optical flow, frame-to-frame)
      │
      ▼
 Essential matrix estimation (5-point algorithm + RANSAC)
      │
      ▼
 Relative pose recovery (R, t — up to scale)
      │
      ▼
 Scale recovery (from IMU pre-integration distance)
      │
      ▼
 Pose graph update (sliding window of 10 frames)
      │
      ▼
 /mc/local_pose  (PoseWithCovarianceStamped, ≥10 Hz)
```

The scale ambiguity inherent to monocular VO is resolved by comparing the VO-estimated translation magnitude with the IMU-integrated distance between the same two frames. The IMU pre-integration runs on `sensor_combined` data from `/mc/fcc/imu`.

### 8.2 IMU Pre-integration

Between consecutive camera frames (at timestamps t₀ and t₁), IMU measurements are integrated to yield a relative position and velocity increment:

```
Δp_imu = ∫∫ (a_body - a_bias) dt² over [t₀, t₁]
```

Bias estimates are maintained with a simple random-walk model, updated when a GNSS fix is available. When GNSS is lost, the last known biases are held constant.

The scale factor `s` is updated each frame as:

```
s = ‖Δp_imu‖ / ‖Δp_vo‖    (smoothed with α=0.1 low-pass filter)
```

### 8.3 GNSS Handover Logic

```
localisation_mode_update():
  if gnss_ok() and mode == VISUAL_ODOMETRY:
    re-initialise VO origin from current GNSS fix
    mode = GNSS_PRIMARY
    publish_mode_change(GNSS_REACQUIRED)
  elif not gnss_ok() and mode == GNSS_PRIMARY:
    initialise VO from last GNSS fix as origin
    mode = VISUAL_ODOMETRY
    publish_mode_change(GNSS_FIX_LOST)
```

`gnss_ok()` returns true when `fix_type ≥ 3` AND `hdop < hdop_threshold` (default 2.0). Both conditions must hold for 3 consecutive status messages before declaring GNSS as recovered (hysteresis prevents oscillation at the threshold).

### 8.4 Drift Bound

Position drift in pure VO mode is bounded by the sliding window pose graph. The window retains the last 10 keyframes. Loop closure is not implemented; drift accumulates monotonically. The 5 m/min drift bound (FR-053) is met in typical textured environments by maintaining ≥ 200 tracked features per frame and a frame rate of ≥ 15 FPS.

The covariance published in `/mc/local_pose` reflects estimated VO drift (grows with time since last GNSS fix). The detection geolocation uncertainty is propagated from this covariance.

---

## 9. perception_down_node — AI Inference Pipeline (renamed from detection_node)

**SRS refs:** FR-042–044, PR-007  
**Package:** `vision`

### 9.1 Inference Architecture

```
/camera/image_raw ──► Frame queue (max depth = 2, drop oldest on overflow)
                             │
                             ▼
                    Preprocessing (BGR8 → RGB, resize to model input)
                             │
                             ▼
                    TensorRT inference (Jetson GPU)
                    Model: YOLOv8n / custom detection head
                    Input: 640×640×3   Output: (N, 85) bounding boxes
                             │
                             ▼
                    NMS (IoU threshold 0.45, confidence threshold 0.5)
                             │
                             ▼
                    Geolocation (§9.2)
                             │
                             ▼
                    /mc/detections  (DetectionArray)
```

The frame queue depth of 2 prevents backpressure from blocking the camera pipeline. If inference runs slower than the incoming frame rate, frames are dropped rather than queued. This preserves real-time camera availability for `visual_localisation_node`.

**Inference runs on a dedicated thread** separate from the ROS 2 executor. Results are posted back to the main callback queue via `rclcpp::Node::create_wall_timer` polling a result queue, keeping ROS 2 thread safety intact.

### 9.2 TensorRT Model Management

At node startup:
1. Check for a cached `.engine` file at `/var/drone-mc/model.engine`.
2. If absent, build from the ONNX model at `/etc/drone-mc/model.onnx` using `trtexec` (first-run compilation, ≈ 60 s on Jetson Orin NX).
3. Load the engine and allocate CUDA buffers.

Model updates are applied by replacing the ONNX file and deleting the cached engine; the node rebuilds on next restart.

### 9.3 Target Geolocation Algorithm

Given a detection bounding box centre `(u, v)` in image pixels:

```
geolocation(u, v, camera_info, drone_pose):

  1. Unproject pixel to unit ray in camera optical frame:
       r_cam = K⁻¹ × [u, v, 1]ᵀ    (K = 3×3 intrinsic matrix)
       r_cam /= ‖r_cam‖

  2. Transform ray to NED world frame:
       T_cam_to_body = static TF from /tf_static
       T_body_to_ned  = rotation matrix from drone_pose.q
       r_ned = T_body_to_ned × T_cam_to_body × r_cam

  3. Intersect ray with flat terrain plane at known altitude:
       terrain_alt = drone_pose.alt_msl - search_altitude_m
       λ = (terrain_alt - drone_pose.z_ned) / r_ned.z
       target_pos_ned = drone_pos_ned + λ × r_ned

  4. Convert NED to geodetic (GeographicLib)

  5. Compute position uncertainty:
       range_m        = λ
       pixel_fov_rad  = atan(sensor_width / (2 × fx))    (per-pixel FOV)
       pos_uncertainty_m = range_m × tan(pixel_fov_rad)  (1-sigma horizontal)
       pos_uncertainty_m += visual_odom_covariance_m      (add VO uncertainty)
```

The flat-terrain assumption introduces error proportional to terrain relief. A DEM-based intersection may be added in a future revision.

### 9.4 GPU Resource Management

Inference is limited to consume ≤ 80% of GPU throughput (PR-010). This is enforced by:
1. A configurable `inference_rate_limit_fps` parameter (default 15 FPS).
2. Frame dropping in the queue when inference falls behind.
3. Jetson GPU clock policy: `nvpmodel` mode set to `MAXN` (all 8 CPUs + GPU) in the container environment.

---

## 9a. perception_front_node and video_mux_node — Gimbal Video Perception and Uplink

**SRS refs:** FR-080–084, IR-040–042
**Package:** `video_uplink`

### 9a.1 perception_front_node — RTSP Ingest, Inference, and Tracking

```
rtsp://192.168.10.200:8554/stream1 ──► rtspsrc (H.265 depay/decode, HIF-007/IFD-006)
                                              │
                                              ▼
                                     Frame queue (max depth = 2, drop oldest)
                                              │
                                              ▼
                                     Preprocessing (resize to model input)
                                              │
                                              ▼
                                     TensorRT inference — same engine as
                                     perception_down_node (§9.2)
                                              │
                                              ▼
                                     NMS (IoU 0.45 / conf 0.5)
                                              │
                                              ▼
                                     Geolocation (§9.3 algorithm; gimbal-mount TF)
                                              │
                                ┌─────────────┴──────────────┐
                                ▼                              ▼
                     /mc/perception/front/detections   IoU-based greedy tracker
                     (DetectionArray, event-driven,      (frame-to-frame association)
                      FR-081)                                    │
                                                                  ▼
                                                    /mc/perception/front/tracks
                                                    (TrackArray, ≥ 5 Hz, FR-081)
```

The node reuses the same TensorRT engine and geolocation math as `perception_down_node` (§9.2, §9.3) — only the frame source (`rtspsrc` vs. CSI) and the camera-to-body TF (gimbal mount vs. downward mount) differ. Running both pipelines concurrently is subject to the shared ≤ 80% GPU throughput budget (PR-010, FR-082); each has its own `inference_rate_limit_fps` parameter so one can be turned down independently of the other under load.

**Tracker:** a track is created for each unmatched detection. On each new frame, detections are greedily matched to existing tracks by bounding-box IoU (`track_iou_threshold`, default 0.3); a match increments `frames_tracked`, a miss increments a per-track miss counter. A track is dropped after `track_miss_limit` (default 10) consecutive misses.

**Link health:** `rtspsrc` is configured to auto-reconnect on error. If no frame arrives for `gimbal_link_timeout_s` (default 3 s, ICD IFD-006 §7.3 / FR-084), the node stops publishing tracks/detections and its liveness heartbeat goes silent; `health_monitor_node` (§10.2) detects this, sets `GIMBAL_LINK_LOST` in `fault_mask` and clears `GIMBAL_OK` in `sensor_flags`. Per FR-084 this is deliberately excluded from the failsafe decision tree (§10.3) — it is perception-only and never commands a flight-mode change.

### 9a.2 video_mux_node — GCS Video Re-encode and Metadata Mux

```
perception_front_node frame stream ──► GStreamer encode (H.265, nvv4l2h265enc)
  + /mc/perception/front/detections               │
  + /mc/perception/front/tracks                    ▼
                                          KLV-style metadata mux (ICD IFD-007 §8.2:
                                          time_usec, track_id, class_id, confidence,
                                          lat/lon/alt)
                                                     │
                                                     ▼
                                          RTSP server :8555 → GCS
                                          (rtsp://192.168.10.11:8555/mc_video)
                                                     │
                                                     ▼
                                          /mc/video/uplink_status (≥ 1 Hz)
```

`video_mux_node` is independent of `gcs_mavlink_node` — it owns its own RTSP server and never touches the MAVLink GCS channel (IR-041, IR-042). `streaming` in `/mc/video/uplink_status` is `false` until `perception_front_node` frames are flowing and a GCS RTSP client has connected; `client_connected` and `bitrate_kbps` are read from the underlying GStreamer RTSP server session state.

On `perception_front_node` frame/track silence > 3 s, `video_mux_node` stops serving new frames and sets `streaming=false`, tearing down the active RTSP session, but keeps its encode pipeline warm so it can resume immediately once frames return (ICD IFD-007 §8.4). A GCS client disconnecting does not raise a fault — the node keeps encoding, ready for reconnect.

---

## 10. health_monitor_node — System Health and Failsafe

**SRS refs:** FR-070–076, SFR-001–008  
**Package:** `health_monitor`

### 10.1 Resource Monitoring

A 1 Hz `rclcpp::WallTimer` reads system metrics:

| Metric | Source | Fault threshold |
|---|---|---|
| CPU utilisation | `/proc/stat` (delta over 1 s) | > 95% sustained 5 s |
| Memory used | `/proc/meminfo` MemAvailable | < 10% free |
| Disk used | `statvfs("/var/drone-mc")` | > 90% used |
| GPU temperature | `/sys/devices/gpu.0/load` and tegrastats (parsed via pipe) | > 85 °C |
| CPU temperature | `/sys/class/thermal/thermal_zone*/temp` | > 90 °C |

### 10.2 Node Liveness Tracking

The `node_alive_mask` is built by monitoring the last-received timestamp on a set of "heartbeat" topics. Each critical node publishes at ≥ 1 Hz on its respective topic.

```cpp
struct NodeWatch {
    std::string     topic;
    uint8_t         mask_bit;
    double          timeout_s;
    rclcpp::Time    last_received;
};
```

Critical nodes and timeouts:

| Node | Heartbeat topic | Timeout |
|---|---|---|
| `mission_executor_node` | `/mc/executor/heartbeat` | 5 s |
| `fcc_bridge_node` | `/mc/fcc/vehicle_state` | 2 s |
| `gcs_mavlink_node` | `/mc/gcs/link_status` | 2 s |
| `swarm_coordinator_node` | `/mc/swarm/peer_states` | 3 s |
| `camera_node` | `/camera/image_raw` | 3 s |
| `perception_down_node` | `/mc/detections` (or alive topic) | 10 s |
| `visual_localisation_node` | `/mc/local_pose` | 5 s |
| `perception_front_node` | `/mc/perception/front/tracks` (or alive topic) | 3 s (FR-084) |
| `video_mux_node` | `/mc/video/uplink_status` | 2 s |

### 10.3 Failsafe Decision Tree

```
every 1 s:

  if FCC_LINK_LOST in fault_mask:
    → command HOLD (if FCC responds); else declare FAULT, command RTL
    → abort: cannot RTL without FCC link

  if EXECUTOR_DEAD in fault_mask:
    restart_count[executor]++
    if restart_count[executor] <= 3:
      → systemd_restart("mission-exec.service")
    else:
      → command RTL via fcc_bridge_node directly
      → publish FAULT event

  if LOW_BATTERY in fault_mask:
    if gcs_override_active(OVERRIDE_LOW_BATTERY_RTL):
      → suppress (GCS has acknowledged)
    else:
      → command RTL

  if GCS_LINK_LOST in fault_mask:
    if mission_phase in {INGRESS, MUSTER, ISR_SELECTION, VORONOI_PARTITION, SEARCH}:
      if not gcs_override_active(OVERRIDE_GCS_LINK_HOLD):
        → command HOLD

  if GPU_OVERTEMP in fault_mask:
    → reduce inference_rate_limit_fps to 5 FPS (via dynamic param)
    → publish WARNING statustext

  # GIMBAL_LINK_LOST (§9a.1, FR-084) is intentionally NOT handled here —
  # it is perception-only and must never drive a flight-mode command.

  publish /mc/health (all metrics + fault_mask + node_alive_mask)
  publish MC_HEALTH (50103) via gcs_mavlink_node
```

### 10.4 Systemd Restart Interface

The health monitor triggers restarts via a Unix domain socket to a small privileged helper process (`drone-mc-watchdog`) that runs outside Docker and has permission to call `systemctl restart`. The helper validates the service name against an allowlist before executing.

```
/var/run/drone-mc/watchdog.sock
  Protocol: one-line JSON request: {"action": "restart", "service": "mission-exec.service"}
  Response: {"ok": true} or {"ok": false, "reason": "..."}
```

This avoids running Docker in privileged mode and keeps the restart mechanism simple and auditable.

---

## 11. gcs_mavlink_node — GCS Communication

**SRS refs:** FR-060–064, IR-010–014  
**Package:** `gcs_comms`

### 11.1 Architecture

`gcs_mavlink_node` uses MAVROS as a library (not as a standalone node) to handle standard MAVLink message encoding and decoding. Custom messages (`MC_STATUS`, `MC_DETECTION`, `MC_HEALTH`) are implemented as a MAVROS plugin.

The node runs a GCS heartbeat monitor:

```cpp
// Called at 1 Hz
void check_gcs_heartbeat() {
    auto age = (node_->now() - last_gcs_heartbeat_).seconds();
    LinkStatus msg;
    msg.gcs_link_ok = (age < gcs_link_timeout_s_);
    msg.last_heartbeat_us = last_gcs_heartbeat_.nanoseconds() / 1000;
    link_status_pub_->publish(msg);
}
```

### 11.2 Mission Upload State Machine

The mission upload uses the MAVLink mission protocol, which is a multi-message handshake. The node implements a coroutine-style state machine using `rclcpp::WallTimer` polling:

```
IDLE
  → on MISSION_COUNT received:
       expected_count = count; items_received = 0; items = {}
       send MISSION_REQUEST_INT(seq=0)
       state = RECEIVING

RECEIVING
  → on MISSION_ITEM_INT(seq):
       validate seq == items_received
       decode item into mc_msgs/MissionUpload field
       items_received++
       if items_received == expected_count:
         send MISSION_ACK(MAV_MISSION_ACCEPTED)
         publish /mc/gcs/mission_upload
         state = IDLE
       else:
         send MISSION_REQUEST_INT(seq=items_received)
  → on timeout (5 s without next item):
       send MISSION_ACK(MAV_MISSION_ERROR)
       state = IDLE
```

### 11.3 Telemetry Multiplexer

The node publishes `MC_STATUS` at ≥ 2 Hz by aggregating data from three topics: `/mc/fcc/vehicle_state`, `/mc/mission_phase`, and `/mc/health`. A 500 ms `rclcpp::WallTimer` pulls the latest values and constructs the outgoing message. Detections and health updates are event-driven (published immediately on receipt from their source topics).

---

## 12. Deployment Design

**SRS refs:** SW-001–043

### 12.1 Container Build Strategy

Each container is built from a multi-stage Dockerfile:

```dockerfile
# Stage 1: colcon build
FROM ros2-core AS builder
COPY src/ /ws/src/
RUN colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Stage 2: minimal runtime
FROM ros2-core AS runtime
COPY --from=builder /ws/install /opt/drone-mc/install
COPY params/ /etc/drone-mc/params/
ENV AMENT_PREFIX_PATH=/opt/drone-mc/install
CMD ["ros2", "run", "fcc_bridge", "fcc_bridge_node", ...]
```

The shared colcon workspace (`/opt/drone-mc/install`) is built once and bind-mounted into all containers. The `ros2-core` base image is built from `dustynv/ros:humble-ros-base-l4t-r36.3.0` (Jetson-native, JetPack 6.2.2) pinned by SHA256 digest.

### 12.2 Systemd Service Units

Each service unit follows this template:

```ini
# /etc/systemd/system/drone-mc-fcc-bridge.service
[Unit]
Description=Drone MC — FCC Bridge (uXRCE-DDS)
PartOf=drone-mc.target
After=drone-mc-mavlink-router.service
Requires=drone-mc-mavlink-router.service

[Service]
Slice=rt.slice
EnvironmentFile=/etc/drone-mc/environment
ExecStart=/usr/bin/docker run --rm --name mc-fcc-bridge \
  --device /dev/ttyTHS1 \
  --network=host \
  --mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install \
  --mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc \
  --env-file /etc/drone-mc/environment \
  ghcr.io/drone-mc/fcc-bridge:${MC_VERSION}
ExecStop=/usr/bin/docker stop mc-fcc-bridge
Restart=on-failure
RestartSec=5s
StartLimitIntervalSec=60
StartLimitBurst=3
MemoryMax=512M
CPUQuota=50%
DeviceAllow=/dev/ttyTHS1 rw
PrivateTmp=yes
ProtectSystem=strict

[Install]
WantedBy=drone-mc.target
```

### 12.3 cgroups v2 Slice Configuration

```
/etc/systemd/system/rt.slice:
  CPUWeight=800       # higher priority than be.slice
  MemoryHigh=4G

/etc/systemd/system/be.slice:
  CPUWeight=200
  MemoryHigh=8G
```

Services in `rt.slice`: `fcc-bridge`, `swarm-coord`  
Services in `be.slice`: `mavlink-router`, `gcs-comms`, `mission-exec`, `vision`, `health-monitor`, `video-uplink`

### 12.4 Startup Ordering

```
drone-mc.target
  └── health-monitor.service    (first — monitors everything else)
  └── mavlink-router.service    (no dependencies)
  └── fcc-bridge.service        (after mavlink-router)
  └── gcs-comms.service         (after fcc-bridge, mavlink-router)
  └── swarm-coord.service       (after gcs-comms)
  └── mission-exec.service      (after swarm-coord, fcc-bridge, gcs-comms)
  └── vision.service            (after fcc-bridge)
  └── video-uplink.service      (no dependencies — HIF-007 is a separate physical link)
```

### 12.5 Configuration Files

All configuration lives in `/etc/drone-mc/` (bind-mounted read-only into every container):

```
/etc/drone-mc/
  environment                  ← environment variables (§9.4 of ICD)
  cyclonedds.xml               ← CycloneDDS DDS domain and discovery config
  mavlink-router.conf          ← mavlink-router endpoint config
  camera_calibration.yaml      ← camera intrinsics and distortion
  params/                      ← ROS 2 parameter YAML files per node
    fcc_bridge.yaml
    formation_controller.yaml
    swarm_coordinator.yaml
    mission_executor.yaml
    search_planner.yaml
    visual_localisation.yaml
    perception_down.yaml       ← renamed from detection.yaml
    perception_front.yaml
    video_mux.yaml
    health_monitor.yaml
  model.onnx                   ← AI detection model (shared by perception_down_node and perception_front_node)
```

---

## 13. Testing and Validation Design

**SRS refs:** SW-050–052

### 13.1 SITL Integration

The SITL environment replaces hardware interfaces transparently:

| Production | SITL Equivalent |
|---|---|
| Pixhawk 6X on ttyTHS1 | PX4 SITL on UDP 14540/14541 (Gazebo) |
| IMX219 CSI camera | ROS 2 bag playback via `ros2 bag play` |
| Front gimbal IP camera | Local `rtsp-simple-server` looping a recorded clip |
| MANET radio peers | Loopback UDP to peer SITL instances |
| MANET radio GCS | QGroundControl on localhost |

`SIMULATION_MODE=true` selects `MavlinkUdpBackend` in `fcc_bridge_node`. All other nodes are identical to production. SITL validation must cover all mission phases with ≥ 2 drone instances (SW-051).

### 13.2 Smoke Test Framework

Each installation script (e.g., `06-install-ros2.sh`) has a companion `06b-smoke-test-ros2.sh`. The smoke test follows a fixed pattern:

```bash
#!/usr/bin/env bash
set -euo pipefail

PASS=0; FAIL=0

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        echo "PASS: $desc"; ((PASS++))
    else
        echo "FAIL: $desc"; ((FAIL++))
    fi
}

check "ROS 2 sourced"         bash -c "source /opt/ros/humble/setup.bash && ros2 --version"
check "CycloneDDS available"  ldconfig -p | grep -q cyclone
check "fcc_bridge package"    bash -c "source /opt/drone-mc/install/setup.bash && ros2 pkg list | grep -q fcc_bridge"

echo "Results: ${PASS} passed, ${FAIL} failed"
[[ $FAIL -eq 0 ]] || exit 1
```

### 13.3 Node-Level Test Strategy

Each node package includes a `test/` directory with:

| Test type | Tool | Coverage |
|---|---|---|
| Unit tests | `gtest` | Algorithm correctness (ISR scoring, Voronoi clipping, boustrophedon path, PD controller output) |
| Node integration tests | `launch_testing` + `rclcpp` | Topic publish/subscribe handshakes, service calls, parameter loading |
| SITL system tests | Python (`pytest` + `rclpy`) | End-to-end mission phase progression with mock FCC and GCS |

**Mock strategy:** Hardware-facing nodes (`fcc_bridge_node`, `camera_node`) are replaced by lightweight mock publishers in SITL tests. Mock nodes publish pre-recorded data from ROS 2 bags at the specified rates. All other nodes run unmodified — the test validates their logic against realistic data without requiring hardware.

### 13.4 Continuous Validation

Installation script + smoke test pairs serve as the CI validation gate. A GitHub Actions workflow (or equivalent) runs:

1. Build all Docker images (aarch64 via QEMU cross-compilation for CI, native for deployment).
2. Run all smoke tests inside the built images.
3. Run SITL multi-drone test (2 drone instances, all phases).
4. Report pass/fail per phase against the verification matrix in SRS §7.

---

*MC-SDD-001 · v0.1-DRAFT · 2026-06-20*
