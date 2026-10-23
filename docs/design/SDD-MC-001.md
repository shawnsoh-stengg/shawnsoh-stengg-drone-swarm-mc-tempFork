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
4. [mission_manager_node — Mission Phase State Machine](#4-mission_manager_node--mission-phase-state-machine)
5. [formation_controller_node — Formation Keeping](#5-formation_controller_node--formation-keeping)
6. [swarm_coordinator_node — Swarm Management](#6-swarm_coordinator_node--swarm-management)
7. [search_planner_node — Coverage Path Planning](#7-search_planner_node--coverage-path-planning)
8. [visual_localisation_node — GNSS-Denied Localisation](#8-visual_localisation_node--gnss-denied-localisation)
9. [perception_down_node — AI Inference Pipeline](#9-perception_down_node--ai-inference-pipeline)
9a. [perception_front_node and video_mux_node — Gimbal and Downward Video Perception and Downlink](#9a-perception_front_node-and-video_mux_node--gimbal-and-downward-video-perception-and-downlink)
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

3. **Single source of truth per domain.** One node owns each domain: `mission_manager_node` owns mission phase; `swarm_coordinator_node` owns peer state and cell assignment; `health_monitor_node` owns system health. Others subscribe, never assert.

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
| Middleware | ROS 2 Humble / CycloneDDS (same-host) + zenoh-bridge-dds (cross-host) | Deterministic discovery + DDS QoS locally; static-peer Zenoh mesh for the bandwidth-constrained, lossy MANET link instead of DDS's own discovery protocol; PX4 uXRCE-DDS compatibility unaffected |
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
│  swarm_coordinator · mission_manager · search_planner       │
│  visual_localisation · perception_down · perception_front    │
├──────────────────────────────────────────────────────────────┤
│  Layer 3 — GCS and Swarm Communication                       │
│  gcs_mavlink_node · MAVROS · mavlink-router · video_mux_node  │
├──────────────────────────────────────────────────────────────┤
│  Layer 2 — FCC Abstraction (FccBackend)                      │
│  fcc_bridge_node · Micro XRCE-DDS Agent                      │
├──────────────────────────────────────────────────────────────┤
│  Layer 1 — Hardware and Transport                            │
│  ttyTHS1 serial · 192.168.100.x ethernet · CSI camera        │
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
          mission_manager_node ◄── /mc/manager/heartbeat (self)
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
                    health_monitor_node ◄── /mc/manager/heartbeat
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
- `perception_front_node` connects to a local test RTSP server (e.g. `rtsp-simple-server` looping a recorded clip) instead of the physical gimbal camera at 192.168.100.200.
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

> **Subnet move (2026-10-06):** the drone network moved from `192.168.10.0/24` to `192.168.100.0/24` because `192.168.10.x` collided with the AI.R hangar Wi-Fi. Every host octet is unchanged (`.11` MC, `.31` FCC, `.10` GCS, `.1` MANET AU, `.200+` gimbal camera). Addresses throughout this document, including the dated correction notes and bench observations recorded before the move, have been renumbered to the new subnet; `test-results/` keeps the addresses as recorded. `scripts/01-install-os.sh`/`02-install-systemd-units.sh` migrate existing `/etc/drone-mc/environment` files; the FCC, radios, camera and GCS are moved by hand (`docs/DEPLOYMENT.md`, "Moving the drone subnet").

**FCC Ethernet addressing.** **Correction (2026-09-25):** earlier revisions of this document dual-homed the FCC — a private MC↔FCC subnet 192.168.12.x (FCC `192.168.12.31`, plus a second Jetson address `MC_ETH_INTERNAL_IP`) and a separate GCS-facing `192.168.100.31` (`FCC_GCS_IP`) for Channel A only. That was never physically possible: PX4's NuttX network stack gives the Pixhawk exactly one IPv4 address (single `IPADDR` in `/fs/microsd/net.cfg`; `netman show`). On the orin-nx bench (2026-09-25) the Pixhawk answered only on `192.168.100.19` and `192.168.12.19` was unreachable, so everything sent to 192.168.12.x had gone nowhere. The FCC now has one address, `FCC_ETH_IP` (default `192.168.100.31`, per-drone), on the same 192.168.100.x subnet as the GCS and peer MCs, and the Jetson has one eth0 address, `MC_ETH_IP` (default `192.168.100.11`). Every MC↔FCC Ethernet channel targets `FCC_ETH_IP`: the uXRCE-DDS Ethernet fallback (UDP `:8888`), the mission protocol (`:14555`), the optical-flow relay (`:14556`), IGS-004's HIL_GPS sender (`:14557`) and Channel A's mavlink-router pass-through (`:14550`). One consequence: MC↔FCC unicast now shares the 192.168.100.x segment with GCS and MANET traffic instead of a private subnet. `FCC_GCS_IP`/`MC_ETH_INTERNAL_IP` are retired; `01-install-os.sh`/`02-install-systemd-units.sh` migrate existing `/etc/drone-mc/environment` files automatically.

**Mission item transport.** Stock PX4 has no DDS-writable mission topic — mission items live in FCC-side dataman, reachable only through the standard MAVLink mission protocol. `XrceDdsBackend` therefore also owns a MAVLink 2/UDP socket to `FCC_ETH_IP:14555`, riding the existing HIF-002 Ethernet link (already MAVLink-capable per HID-MC-001 §4 — no new hardware). This channel is independent of the `FCC_TRANSPORT` primary/secondary DDS selection (§3.1); it is always up to `FCC_ETH_IP` regardless of which link currently carries uXRCE-DDS. `vehicle_state` and `vehicle_command` are unaffected by this — they stay on uXRCE-DDS. This is one of three MAVLink exceptions on the MC↔FCC interface (IR-007) — the other two are the optical-flow relay described next and IGS-004's HIL_GPS sender (owned by `gnss_sim_node`/`indoor_nav`, not this backend — ICD §3.3b).

- `upload_mission(items)` drives the full handshake — `MISSION_COUNT` → `MISSION_ITEM_INT` ×N → `MISSION_ACK` (the same state machine `gcs_mavlink_node` uses for GCS uploads, §11.2). Used for search paths and INGRESS/MUSTER legs — infrequent, on mission change only.
- `update_mission_item(seq, item)` re-runs that same full handshake against the last successfully uploaded mission, with only item `seq` replaced, rather than a partial write. **FCC-010 (2026-09-14):** the original design instead drove the MAVLink partial-update microservice — `MISSION_WRITE_PARTIAL_LIST(seq, seq)` → FCC's `MISSION_REQUEST_INT(seq)` → `MISSION_ITEM_INT(seq)` → `MISSION_ACK` — but PX4 has no handler for `MISSION_WRITE_PARTIAL_LIST` at any known firmware version (confirmed against real PX4 source, v1.14.4 through v1.17.0), so that microservice can never be ACKed by real firmware. `MissionProtocolClient` now caches the last mission `upload_mission()` accepted, and `update_mission_item()` patches the requested item into a copy of it and calls `upload_mission()` again — same wire mechanism, no new PX4-facing message. Used for the `FORMATION_SLOT` (`seq = 0`) at ≥ 2 Hz during INGRESS (lowered from the original ≥ 5 Hz design, since a full-handshake re-upload is heavier per cycle than a single-item partial write would have been) — requires a prior successful `upload_mission()` call to have something to patch.

**Optical-flow relay (HIF-008, IGS-002).** A second, independent MAVLink 2/UDP socket to `FCC_ETH_IP:14556` (`OpticalFlowRelay`, `fcc_bridge/optical_flow_relay.{hpp,cpp}`), decoding `OPTICAL_FLOW_RAD`/`DISTANCE_SENSOR` frames the FCC's flow MAVLink instance streams. Chosen over a custom PX4 `dds_topics.yaml` firmware addition (HID-MC-001 §10's other option) because it needs no firmware rebuild pipeline — the FCC-side change is a startup-script MAVLink stream config, not a compiled-in change. `XrceDdsBackend` runs this on its own receive thread (own socket, own `MAVLINK_COMM_2` parser state, distinct from the mission channel's `MAVLINK_COMM_0` and `MavlinkUdpBackend`'s `MAVLINK_COMM_1`); `MavlinkUdpBackend` feeds the same `OpticalFlowRelay` from its existing shared SITL socket. The body-frame velocity math (`vx`/`vy` from `integrated_x/y`, gyro-compensated, scaled by the paired rangefinder distance) is replicated exactly from PX4's own `VehicleOpticalFlow.cpp` (confirmed against the real v1.14.4 tag originally, re-confirmed byte-for-byte against v1.17.0 — see ICD §3.3a for the formula). `fcc_bridge_node` republishes the result as `mc_msgs/OpticalFlow` on `/mc/sensors/optical_flow`, consumed by `gnss_sim_node` (IGS-001).

By default the FCC flies in `MISSION` mode via the mission-protocol exchanges above, and neither has a PX4-imposed minimum rate — a gap does not cause a mode change. The one exception is a phase/leg the active mission plan flags via `offboard_phase_mask` (ICD-MC-001 §9.2, FR-026): `mission_manager_node` sequences entry into `OFFBOARD` and `formation_controller_node` streams `offboard_control_mode`/`trajectory_setpoint` for the duration (§4.1a, §5.3a) — reintroducing the ≥ 2 Hz floor that pair enforces (contrast the earlier, since-removed design that used it unconditionally for every leg), scoped only to opted-in legs so the gap-tolerant behaviour above stays the default everywhere else. `XrceDdsBackend::stream_offboard_setpoint()` (ICD §3.4) publishes this pair; it does not itself change the FCC's mode.

### 3.3 MavlinkUdpBackend

Used in simulation and as a fallback. Wraps a pymavlink or C++ MAVLink UDP socket carrying vehicle state, commands, *and* the mission protocol over the same connection — there is no separate DDS channel to fall back to. Translates `send_command()` calls into `COMMAND_LONG` MAVLink messages and parses incoming `GLOBAL_POSITION_INT`, `ATTITUDE`, and `BATTERY_STATUS` messages into the same `VehicleState` structure. `upload_mission()` drives the full `MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK` handshake; `update_mission_item()` re-runs that same handshake against the cached last-uploaded mission with one item replaced (FCC-010) — the identical protocol logic `XrceDdsBackend` uses for its dedicated mission channel (§3.2), just multiplexed over the one socket instead of a separate port. Same multiplexing for the optical-flow relay (§3.2's `OpticalFlowRelay`): `OPTICAL_FLOW_RAD`/`DISTANCE_SENSOR` frames on this one shared socket, if PX4 SITL's own simulated flow sensor sends them — no real H-Flow exists in simulation, so `get_optical_flow()` reports invalid in practice today.

### 3.4 fcc_bridge_node

A thin ROS 2 node that:
1. Instantiates the appropriate `FccBackend` via the factory.
2. Subscribes to `/mc/fcc/mission_upload`, `/mc/fcc/mission_item_update`, and `/mc/fcc/send_command` and forwards to `FccBackend`.
3. Publishes `/mc/fcc/vehicle_state` at 10 Hz from `FccBackend::get_vehicle_state()`.
4. Publishes `/mc/fcc/imu` and `/mc/fcc/odometry` from the corresponding uORB topics.
5. Monitors link liveness and publishes `/mc/fault` on loss.

The node does **no business logic** — it is purely an adapter.

---

## 4. mission_manager_node — Mission Phase State Machine

**SRS refs:** FR-001–008, FR-010–016, FR-060–064  
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
| `TAKEOFF` | `INGRESS` | FCC at target altitude AND GCS INGRESS command AND the plan has ≥1 `ingress_waypoints` entry | Upload the plan's `ingress_waypoints` as the FCC mission (`MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, alt above home); on `MISSION_ACK`, send `MAV_CMD_MISSION_START` (resent ≤3× at 1 s until `nav_state` = AUTO_MISSION, then a CRITICAL STATUSTEXT and hold — never a FAULT) |
| `INGRESS` | `MUSTER` | Vehicle within `waypoint_acceptance_radius_m` (horizontal, lat/lon) of the last `ingress_waypoints` entry (muster point) AND the plan doesn't skip the swarm phases | Disable formation ctrl; notify GCS |
| `INGRESS` | `RETURN` | Same route-complete check AND the plan is **transit only** (`SKIP_SWARM_PHASES\|SKIP_SEARCH`) | Send RTL |

> **Ingress fix (2026-10-15, PR #21):** TAKEOFF → INGRESS used to upload a one-item `FORMATION_SLOT` placeholder at the vehicle's own position, which `formation_controller_node` (SC-001) was meant to overwrite but never does, and nothing ever switched PX4 into MISSION mode — so the vehicle just held. Every drone now flies the plan's own ingress route; SC-001's follower behaviour (§5) will build on this once implemented. `send_ingress_route_upload()`/`send_mission_start()`/`check_mission_start()` in `mission_manager_node.cpp`.
| `MUSTER` | `ISR_SELECTION` | All ALIVE peers in the plan's `swarm_manifest` in MUSTER AND quorum count met | Signal swarm_coordinator |
| `ISR_SELECTION` | `VORONOI_PARTITION` | ISR result ACKed by all peers | Signal swarm_coordinator |
| `VORONOI_PARTITION` | `SEARCH` | Voronoi cell accepted AND coverage path received | Upload path to FCC; `MAV_CMD_MISSION_START` on ack (resent like INGRESS, but always from item 0, until the vehicle leaves its SEARCH-entry position); enable detection |
| `SEARCH` | `RETURN` | FCC mission complete (`nav_state` = AUTO_LOITER, or every pose of the coverage path uploaded on entry reached in order, each within `waypoint_acceptance_radius_m` horizontally and vertically) OR GCS END_MISSION | Disable detection pipeline; send RTL |
| `RETURN` | `LAND` | FCC at home waypoint | Send LAND command |
| `LAND` | `PREFLIGHT` | FCC disarmed AND on ground (\|vz\| ≤ `landed_max_vertical_speed_mps`) | Log mission complete; `reset_mission_state()` (EXEC-006) |
| Any | `RETURN` | GCS ABORT command | Cancel active FCC mission |
| Any | `FAULT` | `health_monitor_node` CRITICAL_FAULT event, mission-channel retries exhausted, or a GCS EMERGENCY-tier command (DISARM, RTL, MANUAL/ALTCTL/POSCTL/STABILIZED — §4.3) | Command RTL (GCS EMERGENCY: forward the GCS command instead) |
| `FAULT` | — | none: terminal until restart (FR-007) | Once FCC disarmed AND on ground: one-time GCS `StatusText` (CRITICAL) "FAULT: landed+disarmed, restart required" |

**Mission recycle and terminal FAULT (EXEC-006, FR-006/FR-007).** A normal end (mission complete, or GCS END_MISSION/ABORT) goes `LAND` → `PREFLIGHT` once the FCC is down and disarmed, so the next mission needs no container restart. `VehicleState` has no land-detector flag, so "on ground" is disarmed with \|vz\| under a small threshold: PX4 only auto-disarms after its own land detector fires, and the vz check stops an in-air disarm (RC kill) from counting as landed while the vehicle is still falling. `reset_mission_state()` clears the flown mission upload (in memory, and its §4.5 persisted file) and ARM authorisation, the RETURN/FAULT latches, any pending mission-channel ack, and the ISR role, Voronoi cell and coverage path, so FR-010's "ARM authorisation as part of the *current* mission upload" can't be satisfied by the mission just flown. An upload that arrived *during* the mission (Channel B accepts one in any phase) is a different message from the one flown, so it survives the reset as the next mission's upload, with its own `arm_authorised`. `FAULT` has no outgoing row and no GCS "clear fault" command, deliberately including an operator EMERGENCY takeover: a faulted drone needs inspection and a restart (power cycle, or at least `mission-exec`) before it flies again. `mission_orchestrator_node`'s `PREFLIGHT`-only activate gate keeps refusing plans meanwhile.

**Partial missions (EXEC-008).** `MissionUpload.skip_phase_mask` (ICD §9.2) lets a plan leave stages out instead of always flying all of them. Bit *N* skips phase ID *N*, but only two stage sets fly today: `0` = the full swarm mission above, and `SKIP_SWARM_PHASES | SKIP_SEARCH` = **transit only**, INGRESS → RETURN → LAND. Any other combination would leave a phase waiting on state that never comes (e.g. skipping MUSTER alone strands ISR_SELECTION), so it fails pre-arm (§4.2), and both GCS decoders reject it at upload. That includes `SKIP_SWARM_PHASES` alone (**solo**: INGRESS → SEARCH over the whole `search_polygon` → RETURN). Its mask value is reserved, but nothing publishes `/mc/search/coverage_path` until `search_planner_node` (EXEC-002) is built, so a solo drone would hold at the end of INGRESS forever; its INGRESS → SEARCH row returns with EXEC-002. INGRESS carries one row per stage set (see the table above). The guards read the *flown* upload's mask (the one latched at PREFLIGHT → ARMING), so a mid-mission upload can't change the shape of the mission in flight. Skipped phases are never entered, so nothing keyed off them runs: `swarm_coordinator_node`'s MUSTER-triggered ISR election never starts. A swarm drone's MUSTER quorum and pre-arm peer check only count peers in its own `swarm_manifest`, so a transit drone on the same mesh doesn't hold the swarm up. `MAV_CMD_MISSION_START` now follows **every** upload ack, coverage path included: before this, nothing started the coverage path, so the vehicle sat at the end of the ingress route with an unflown mission in dataman. SEARCH is entered with the vehicle already in AUTO_MISSION (the finished ingress route), so `nav_state` can't confirm the start there: SEARCH's resends stop once the vehicle has left its SEARCH-entry position by more than `waypoint_acceptance_radius_m`, and keep `param1 = 0`, because PX4 v1.17's navigator only moves a mission on for `param1 >= 0` (`navigator_main.cpp`), so `-1` would leave the finished ingress mission where it is. Every phase-table entry into RETURN also sends RTL, as the Any → RETURN row already did. The SEARCH → RETURN "mission complete" check gained a coverage-progress test because PX4 v1.17 stays in AUTO_MISSION when a mission ends: it logs "Mission finished, loitering", and Commander only switches to LOITER after a takeoff completes (checked against v1.17.0's `Commander.cpp`/`mission_base.cpp`). So AUTO_LOITER alone never fired after a real search. The test requires every waypoint of the path *as uploaded on entering SEARCH* to be reached in order, each within `waypoint_acceptance_radius_m` horizontally and vertically. Checking only the last waypoint ended SEARCH at once for a sweep ending near its start, or for rows stacked over the same footprint. A coverage path published mid-SEARCH (re-partition on peer loss) is logged but not re-uploaded yet, so the FCC and the completion check both stay on the uploaded path.

### 4.1a Hybrid Mode Arbitration (MISSION ⇄ OFFBOARD)

Reverses the blanket "FCC always flies MISSION" statement in §3.2/§5.3 for legs the mission plan opts in — see `OFFBOARD-MODE-SWITCHING-BACKLOG.md` for the full design history, including why the original MISSION-only decision was made (OFFBOARD's ≥ 2 Hz PX4-imposed floor vs. MISSION's gap tolerance) and why this reintroduces that tradeoff only where explicitly opted in (FR-020, FR-026).

On each phase entry, `mission_manager_node` checks the entering phase's bit in `offboard_phase_mask` (from the `MissionUpload` that produced the active plan, ICD §9.2):

- **Bit clear (default):** unchanged MISSION-mode upload/replace flow from §4.1's transition table.
- **Bit set:** sequence OFFBOARD entry before handing control to `formation_controller_node`:
  1. Command `formation_controller_node` into OFFBOARD-stream mode; wait for ≥ 1 s of confirmed `mc_msgs/FccOffboardSetpoint` publication at ≥ 2 Hz.
  2. Call `send_command(MAV_CMD_DO_SET_MODE, OFFBOARD)`.
  3. Confirm entry by observing `/mc/fcc/vehicle_state.nav_state == OFFBOARD` within 1 s. One retry of step 2 on timeout; if still unconfirmed, abort the leg, remain in the vehicle's current mode, and raise fault `OFFBOARD_ENTRY_FAILED` (FR-027) — closes the gap where PX4 silently rejects the mode change (e.g. a failed pre-arm/EKF check) and the MC would otherwise keep streaming setpoints PX4 never acts on. `formation_controller_node` is not told OFFBOARD is "active" for self-supervision purposes until this confirmation lands.

On phase exit from an OFFBOARD-flagged phase — including an `OFFBOARD_STREAM_STALE` fault from `health_monitor_node` (§10.3), which drives this *same* path, not a separate one — `mission_manager_node` confirms a valid mission is already loaded in FCC-side dataman for the upcoming leg, calls `send_command(MAV_CMD_DO_SET_MODE, MISSION)`, confirms the switch via `nav_state == MISSION` within 1 s (same retry-once-then-fault behaviour as entry, FR-029), and only then signals `formation_controller_node` to stop streaming. This avoids a window where the FCC has neither a setpoint stream nor a mission to fly, and avoids the MC's own bookkeeping saying "back in MISSION" while `nav_state` says otherwise.

`mission_manager_node` is the sole node in the system that calls `send_command()` for a flight-mode change arising from this arbitration logic. `health_monitor_node` never does so for `OFFBOARD_STREAM_STALE` — it only publishes the fault (§10.3) — which keeps the "is a mission loaded" check in the one place that owns it (this section) instead of duplicating it in `health_monitor_node`.

**PX4-side backstop.** PX4's own OFFBOARD-loss failsafe, `COM_OBL_RC_ACT` (action) with its timeout `COM_OF_LOSS_T` (v1.17 default 1.0 s; both names verified against the v1.17.0 tag's `commander_params.c` on 2026-10-20), stays **enabled**, configured with a timeout longer than `health_monitor_node`'s 500 ms `OFFBOARD_STREAM_STALE` threshold (SFR-009) — proposed 1.5-2 s. This makes it a silent backstop for an unresponsive MC software stack rather than a second primary path racing the MC-driven fallback above; disabling it outright would remove the FCC's last line of defense if the companion computer itself dies mid-OFFBOARD leg. The chosen value should be recorded as a PX4 parameter override in `HID-MC-001.md` alongside the airframe's other overrides (e.g. `BAT1_V_DIV`, §7.6) — not yet done as of this section landing.

### 4.2 Pre-Arm Checks

Called as a guard before `PREFLIGHT → ARMING`. Returns true only if **all** conditions hold:

```
pre_arm_checks():
  ✓  FCC link alive (fcc_backend.is_link_alive())
  ✓  FCC arming_state == STANDBY (not STANDBY_ERROR)
  ✓  GCS ARM authorisation flag set in current mission upload
  ✓  Battery remaining > LOW_BATTERY_THRESHOLD + 10% margin
  ✓  All swarm peers in the plan's swarm_manifest in PREFLIGHT or ARMING phase (skipped for a transit plan)
  ✓  Mission upload complete: a flyable skip_phase_mask (§4.1), ingress waypoints, plus
       formation_offsets + swarm_manifest unless the swarm phases are skipped, and
       search_polygon unless SEARCH is skipped
  ✓  Visual localisation node alive (if GNSS quality below threshold)
```

Any failed check generates a `STATUSTEXT` with the specific reason. The node does not arm without all checks passing — it does not attempt partial arm.

### 4.3 GCS Authority Model

The GCS retains authority to command phase transitions at any time (FR-003). GCS commands arrive as `/mc/gcs/command` messages. The manager processes them before evaluating autonomous transitions. The priority order is:

1. **Emergency commands** (`DISARM`, `RTL`): bypass transition table, execute immediately.
2. **Phase commands** (`PHASE_TRANSITION`): validated against the transition table; rejected if guard fails.
3. **Autonomous transitions**: evaluated only if no pending GCS command.

### 4.4 Watchdog Heartbeat

The manager publishes `std_msgs/Header` to `/mc/manager/heartbeat` at ≥ 1 Hz inside its main timer callback. If the node hangs or crashes, this topic goes silent. The `health_monitor_node` detects the silence within 5 s and initiates restart (SFR-003).

The heartbeat is the last thing published in each timer cycle — if it is published, all preceding logic in that cycle completed successfully.

### 4.5 Mission Upload Storage

Mission data is written to disk as a YAML file at `/var/drone-mc/mission_upload.yaml` on receipt and re-loaded at node startup. This ensures persistence across container restarts without requiring a fresh GCS upload after a watchdog restart.

This is unrelated to, and unaffected by, `mission_orchestrator_node`'s separate JSON plan library at `/var/drone-mc/mission_plans/` (§11.4, ICD §12.4, GCS-004) — that directory holds a whole library of named plans for the Orchestrator to CRUD; this file holds only whichever single `mc_msgs/MissionUpload` (from either source) was most recently accepted onto `/mc/gcs/mission_upload`. `mission_manager_node` reads/writes only this YAML file and has no knowledge of the JSON library's existence.

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

The controller runs at **2 Hz** on a `rclcpp::WallTimer` (lowered from an original 5 Hz design — FCC-010: replacing `FORMATION_SLOT` is a full mission re-upload, not a lightweight partial write, so 2 Hz keeps each cycle's handshake within its own period). By default the FCC stays in `MISSION` mode throughout INGRESS; the controller tracks the leader by repeatedly replacing a single reserved mission item (`FORMATION_SLOT = 0`, uploaded during the `TAKEOFF → INGRESS` transition, SDD §4.1) rather than streaming an OFFBOARD setpoint. PD math is still done in the NED local frame; only the final target is converted to geodetic before being packaged into the `MissionItem`. A leg the mission plan flags for OFFBOARD (§4.1a) instead streams that same NED target directly, with no geodetic conversion — see §5.3a.

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

`target_pos_ned` is clamped so the implied correction never exceeds `max_vel_xy` / `max_vel_z` × the 2 Hz cycle period, then converted to geodetic (`lat`, `lon`, `alt_msl`) via `GeographicLib::LocalCartesian` — the same local-tangent origin used by `search_planner_node` (§7.3) — and written into a `MissionItem` (`command = MAV_CMD_NAV_WAYPOINT`, `frame = MAV_FRAME_GLOBAL_INT`). The item is published via `FccBackend::update_mission_item(FORMATION_SLOT, item)`.

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

Unlike OFFBOARD, PX4's `MISSION` mode does not exit autonomously if mission item updates stop arriving — the FCC simply continues toward the last-written item indefinitely. The 2 Hz cadence above is therefore a tracking-quality target, not a mode-retention requirement — it is coincidentally the same number as, but a fundamentally different kind of bound from, the ≥ 2 Hz floor OFFBOARD imposes: that floor is a PX4-enforced mode-exit trigger (contrast §5.3a), while this 2 Hz is a MC-chosen budget so a full mission re-upload fits one cycle (FCC-010) — nothing in PX4 checks or enforces it. The staleness safety net moves entirely into `formation_controller_node`: if its own `WallTimer` stalls (no leader state, no FCC link, or the node itself hangs), the watchdog in §5.4 and the manager heartbeat (§4.4) are what command HOLD — PX4 will not do it for us. This section describes the MISSION-mode default; §5.3a covers the OFFBOARD path, where the ≥ 2 Hz floor is real and enforced by `health_monitor_node`'s `OFFBOARD_STREAM_STALE` watchdog (§10.3) rather than left to PX4's own `COM_OBL_RC_ACT`/`COM_OF_LOSS_T` failsafe timing.

### 5.3a Dual-Mode Control (OFFBOARD Path)

`formation_controller_node` becomes dual-mode for legs `mission_manager_node` (§4.1a) has flagged for OFFBOARD. It selects its output path by whichever mode `mission_manager_node` last *confirmed* — not merely commanded — for the current phase; the node only begins streaming (and self-supervising as such) once §4.1a's entry confirmation has landed.

- **MISSION mode (default):** unchanged from §5.2 — replace `FORMATION_SLOT` via `update_mission_item()`.
- **OFFBOARD mode:** the same PD math (§5.2) feeds a different output path — publish `mc_msgs/FccOffboardSetpoint` at ≥ 5 Hz via `FccBackend::stream_offboard_setpoint()` instead of packaging a `MissionItem`. No duplication of the leader-tracking/NED-frame logic between the two modes; only the final output step differs, and — unlike the MISSION-mode path — no `LocalCartesian` projection to geodetic is needed, since `FccOffboardSetpoint` stays NED-native end to end (§5.5).

Self-supervision extends symmetrically: a stalled `WallTimer` while streaming OFFBOARD setpoints triggers the same self-HOLD behaviour §5.4 already defines for a stalled MISSION-mode replace (FR-024).

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

The leader's state is received in NED from the swarm heartbeat (`PEER_STATE` message). The own vehicle state comes from `/mc/fcc/vehicle_state` also in NED. The formation offset is stored in NED from the GCS mission upload. All PD arithmetic (§5.2) stays in NED. On the MISSION-mode path (default), the single added frame transform is the final `LocalCartesian` projection of `target_pos_ned` to geodetic immediately before packaging the `MissionItem`. On the OFFBOARD path (§5.3a), that projection is skipped entirely — `mc_msgs/FccOffboardSetpoint` is NED-native, so `target_pos_ned` (and the yaw term) go straight into the setpoint with no frame transform at all.

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
            publish_peer_lost_event(id);      // → mission_manager, GCS
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

Drones are ranked by score. The top `ceil(N/2)` drones (where N = ALIVE peer count) are elected to the ISR subset. The coordinator broadcasts `ISR_RESULT` and collects `ISR_RESULT_ACK` from all peers within 5 s. If any ACK is missing, the result is re-broadcast once. After two failed re-broadcasts, the mission manager is notified of a coordination fault.

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

The coordinator also publishes `/mc/swarm/peer_states` (a `SwarmPeerArray`) at 1 Hz for internal consumption by `formation_controller_node` and `mission_manager_node`.

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
3. The `mission_manager_node` receives the path, formats it as PX4 mission items, and uploads via `fcc_bridge_node`.

On cell update (re-partition), the planner re-runs immediately. The manager cancels the active FCC mission and uploads the new path. The starting waypoint is chosen as the path waypoint nearest to the drone's current position to minimise dead transit.

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

## 9a. perception_front_node and video_mux_node — Gimbal and Downward Video Perception and Downlink

**SRS refs:** FR-080–086, IR-040–043
**Package:** `video_downlink` (renamed from `video_uplink`, 2026-07-22 —
video flows drone→GCS, which is a downlink, not an uplink)

Deployed as **two separate containers/systemd units**
(`drone-mc-video-ingest.service` for `perception_front_node`,
`drone-mc-video-downlink.service` for `video_mux_node`, §12.3/§12.4) despite
being one ROS 2 package — the same independent-restart rationale that
already splits `camera_node` out of `vision` (§12.1): `perception_front_node`
depends on an external RTSP link that can flap (FR-084) and does GPU-heavy
TensorRT inference, while `video_mux_node` is comparatively lightweight
encode/mux/serve. Bundling them would mean a `perception_front_node` hang
requiring a container restart also takes down `video_mux_node` — and since
`video_mux_node` now serves **both** Stream A (gimbal) and Stream B
(downward, §9a.3), that would undermine the independent-failure-domain
property FR-086 establishes between the two streams. Splitting containers
removes that coupling entirely.

### 9a.1 perception_front_node — RTSP Ingest, Inference, and Tracking

```
rtsp://192.168.100.200:8554/stream1 ──► rtspsrc (H.265 depay/decode, HIF-007/IFD-006)
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
                                          (rtsp://192.168.100.11:8555/mc_video)
                                                     │
                                                     ▼
                                          /mc/video/downlink_status (≥ 1 Hz)
```

`video_mux_node` is independent of `gcs_mavlink_node` — it owns its own RTSP server and never touches the MAVLink GCS channel (IR-041, IR-042). `streaming` in `/mc/video/downlink_status` is `false` until `perception_front_node` frames are flowing and a GCS RTSP client has connected; `client_connected` and `bitrate_kbps` are read from the underlying GStreamer RTSP server session state.

On `perception_front_node` frame/track silence > 3 s, `video_mux_node` stops serving new frames and sets `streaming=false`, tearing down the active RTSP session, but keeps its encode pipeline warm so it can resume immediately once frames return (ICD IFD-007 §8.4). A GCS client disconnecting does not raise a fault — the node keeps encoding, ready for reconnect.

### 9a.3 video_mux_node — Downward Camera Stream (Stream B)

```
/camera/image_raw (camera_node, IFD-004) ──► GStreamer encode (H.265, nvv4l2h265enc)
  + /mc/detections (perception_down_node)          │
                                                     ▼
                                          Metadata mux (ICD IFD-007 §8.2a:
                                          time_usec, class_id, confidence,
                                          lat/lon/alt — no track_id;
                                          perception_down_node has no tracker)
                                                     │
                                                     ▼
                                          RTSP server :8556 → GCS (Stream B)
                                          (rtsp://192.168.100.11:8556/mc_video_down)
                                                     │
                                                     ▼
                                          /mc/video/down_downlink_status (≥ 1 Hz)
```

Additive to §9a.2, not a replacement — `perception_down_node`'s existing
detection/geolocation role (§9.2, §9.3) is unchanged; this only adds a
consumer of its already-published `/mc/detections`. `video_mux_node` now
owns two independent GStreamer encode/RTSP pipelines within one process —
Stream A (§9a.2, unchanged) and Stream B (this section) — each with its
own encode pipeline, RTSP session, and silence-timeout/streaming-status
state. This mirrors the isolation intent `mavlink-router`'s entrypoint
already establishes for its two independent Channel A/B processes (SDD
§12.2): one component can serve two transports without either one's
failure affecting the other, even though here it's two pipelines in one
process rather than two processes.

On `camera_node`/`perception_down_node` frame/detection silence > 3 s,
`video_mux_node` stops serving new frames on Stream B and sets
`streaming=false` in `/mc/video/down_downlink_status`, tearing down Stream
B's RTSP session — **Stream A's session, encode state, and
`/mc/video/downlink_status` are unaffected** (ICD IFD-007 §8.4, FR-086). The
reverse also holds: Stream A silence does not affect Stream B. A GCS
client disconnecting from either stream does not raise a fault — that
stream's encode pipeline keeps running, ready for reconnect.

**Resource note (flagged, not fully budgeted):** running two concurrent
`nvv4l2h265enc` encode pipelines alongside the existing inference
pipelines (§9.2, §9a.1, ≤80% GPU throughput budget per PR-010) adds load,
though H.265 encode on Jetson typically uses the dedicated NVENC hardware
block rather than the Tensor cores/SM compute the TensorRT inference
pipelines contend for — likely non-competing, but not independently
verified against real hardware here. Worth confirming during TP-HW-007
rather than assumed.

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
| `mission_manager_node` | `/mc/manager/heartbeat` | 5 s |
| `fcc_bridge_node` | `/mc/fcc/vehicle_state` | 2 s |
| `gcs_mavlink_node` | `/mc/gcs/link_status` | 2 s |
| `swarm_coordinator_node` | `/mc/swarm/peer_states` | 3 s |
| `camera_node` | `/camera/image_raw` | 3 s |
| `perception_down_node` | `/mc/detections` (or alive topic) | 10 s |
| `visual_localisation_node` | `/mc/local_pose` | 5 s |
| `perception_front_node` | `/mc/perception/front/tracks` (or alive topic) | 3 s (FR-084) |
| `video_mux_node` | `/mc/video/downlink_status` | 2 s |

### 10.3 Failsafe Decision Tree

```
every 1 s:

  if FCC_LINK_LOST in fault_mask:
    → command HOLD (if FCC responds); else declare FAULT, command RTL
    → abort: cannot RTL without FCC link

  if MANAGER_DEAD in fault_mask:
    restart_count[manager]++
    if restart_count[manager] <= 3:
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

  if OFFBOARD_STREAM_STALE in fault_mask:
    → publish FAULT event only — does NOT command the FCC directly
    → mission_manager_node consumes this fault and executes the fallback
      through its §4.1a exit-sequencing path (MISSION if a valid mission
      is loaded, else HOLD; confirmed via nav_state)

  # GIMBAL_LINK_LOST (§9a.1, FR-084) is intentionally NOT handled here —
  # it is perception-only and must never drive a flight-mode command.

  publish /mc/health (all metrics + fault_mask + node_alive_mask)
  publish MC_HEALTH (50103) via gcs_mavlink_node
```

`OFFBOARD_STREAM_STALE`'s branch is deliberately **not** shaped like the branches above: `FCC_LINK_LOST`, `MANAGER_DEAD` (after 3 restarts), and `LOW_BATTERY`/`GCS_LINK_LOST` all have `health_monitor_node` command the FCC directly. `OFFBOARD_STREAM_STALE` instead routes execution through `mission_manager_node` (§4.1a), because falling back out of OFFBOARD needs the same "is a mission already loaded in dataman" check §4.1a's exit sequencing owns — duplicating that check here would create two independent sources of truth for it. This is a known inconsistency with the existing branches above, accepted for this one fault rather than retrofitted across the others, which is a larger change out of scope for this addition.

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

> **Stale as of GCS-001 (2026-07-25) — MAVROS was never adopted this way.** `docker/ros2-core/Dockerfile` only provisions `ros-humble-mavlink` (generated common-dialect C headers), not MAVROS — see `ws/src/gcs_comms/CMakeLists.txt`'s own comment on this. `gcs_mavlink_node` instead talks raw MAVLink v2 over a loopback UDP socket using those headers directly, the same pattern `fcc_bridge`'s mission-protocol client already established for the MC↔FCC seam. The custom `MC_STATUS`/`MC_DETECTION`/`MC_HEALTH` messages (§11.3) aren't a MAVROS plugin either — as of GCS-003 (2026-08-02) they're hand-packed in `gcs_mavlink_node.cpp` against the wire layout documented in `ws/src/gcs_comms/mc_mavlink_defs.xml`, which is otherwise unchanged from ICD-MC-001.md §4.6.
>
> **Update, GCS-005 (2026-08-06) — MAVROS adopted, but not this way either.** MAVROS now actually runs, but as a second, independent OS process (the `ros-humble-mavros` apt package's `mavros_node` executable) inside the `gcs-comms` container alongside `gcs_mavlink_node` — not linked into it, not a library dependency of it, and `gcs_mavlink_node.cpp`/`CMakeLists.txt` are unchanged by this. Its purpose is also different from what this section originally proposed: not standard-MAVLink encode/decode for `gcs_mavlink_node`'s own Channel B, but a live ROS 2 topic mirror of Channel A's GCS↔FCC vehicle telemetry (`fcu_url=udp://:14553@`, `config/mavlink-router-channel-a.conf`'s new `MAVROS_CHANNEL_A` endpoint, ICD §4.2), scoped for a still-unbuilt future need — pushing/pulling flight, mission, and system info to/from the Mission Orchestrator App. Nothing today consumes MAVROS's topics off-box; wiring that up is unscoped follow-on work, same status as `mission_orchestrator_node`'s own deployment gap below (§12.4).

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

**Implemented (GCS-003, 2026-08-02).** `/mc/localisation_mode` is also multiplexed into `MC_STATUS.localisation_mode` on the same 500 ms timer (not called out above, but required by ICD §4.6's field list). `MC_STATUS.peer_count`/`isr_role` have no source topic in this ticket's scope and are left at 0 pending a future ticket wiring `/mc/swarm/peer_states`/`/mc/swarm/isr_role` in as well. The subscriptions and the 500 ms timer each run on their own callback group (reentrant / mutually exclusive respectively, per §2.1), distinct from `recv_timer_`'s group, so detection-heavy traffic can't delay GCS-001's command-decode path. See `gcs_mavlink_node.cpp` for the send-path synchronization this introduces (a mutex guarding the shared UDP socket state and MAVLink's per-channel sequence counter, both previously safe by virtue of every send running on one callback group).

### 11.4 mission_orchestrator_node — Mission Plan Library (GCS-004, Channel C)

**Implemented (GCS-004, 2026-08-05).** Sibling node in package `gcs_comms` — same "raw MAVLink v2 over a loopback UDP socket" approach §11.1 describes for `gcs_mavlink_node`, but a different actor (the Mission Orchestrator, not QGC), a different dialect subset, and its own loopback port (`ORCHESTRATOR_MAVLINK_LOOPBACK_PORT`, default `14580`, fed by `mavlink-router-channel-c.conf`). Full interface contract: `MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md`, summarized in `ICD-MC-001.md` §12 (IFD-008).

**MAVFTP server.** A single-session, flat-namespace MAVFTP server (`FILE_TRANSFER_PROTOCOL`, standard `common.xml` message 110) over `/var/drone-mc/mission_plans/`, handling `CreateFile`/`OpenFileWO`/`WriteFile`/`ReadFile`/`BurstReadFile`/`ListDirectory`/`RemoveFile`/`TerminateSession`/`ResetSessions`. Design follows PX4's own `mavlink_ftp.cpp` (session/opcode handling, error-code translation, entry-count-paginated `ListDirectory`) rather than inventing MAVFTP server logic from scratch, but is an independent implementation scoped to this interface's narrower needs (one session at a time, no subdirectories, no rename/truncate/CRC32 opcodes) — not a code port. `BurstReadFile` streams the whole remaining file as consecutive `Ack` packets within one `recv_timer_` tick (small JSON plan files, unlike PX4's own tick-driven `send()` loop sized for arbitrary-size log/param files over a bandwidth-limited radio), terminated by an explicit `Nak(EOF)`.

**Raw MAVLink v2 frame decoding.** `mission_orchestrator_node` cannot use `mavlink_parse_char()` to receive `MC_MISSION_PLAN_ACTIVATE` (50104): that function looks up `CRC_EXTRA` by msgid from whichever dialect headers are compiled into the translation unit, and — per the same "no mavgen build step" decision `gcs_mavlink_node` already made (§11.1's stale-note) — only the plain `common` dialect is compiled in here, which has no entry for 50104. `mavlink_parse_char()` then treats every such frame as a checksum failure and drops it rather than decoding it (the same finding `gcs_comms/test/test_gcs_mavlink_node.cpp`'s `RawMavlinkFrame` documents for the send side of the equivalent problem, GCS-003). The receive loop instead decodes the fixed MAVLink v2 header directly — safe because `mavlink-router-channel-c.conf`'s loopback UDP endpoint delivers exactly one complete frame per datagram — replicating `mavlink_finalize_message()`'s own X25 CRC scheme by hand for the two msgids this node ever receives (`FILE_TRANSFER_PROTOCOL`'s `CRC_EXTRA` is a known dialect-independent constant; `MC_MISSION_PLAN_ACTIVATE`'s was derived the same way GCS-003's hand-packed constants were — an actual `mavgen` run against `mc_mavlink_defs.xml`, not hand math).

**JSON validation timing.** MAVFTP has no hook to reject a `WriteFile`/`TerminateSession` sequence mid-transfer for a domain-specific reason — a transfer either lands on disk or it doesn't. JSON parsing happens *after* the transfer completes (`TerminateSession` on a write session): the file stays on disk regardless of validity, but an invalid parse publishes a `STATUSTEXT` warning immediately (ICD §4.7 severity table) so the Orchestrator doesn't have to wait for a later `activate` attempt to find out. `activate` is what actually rejects a bad plan (`REJECTED_INVALID_JSON`).

**Activate.** `MC_MISSION_PLAN_ACTIVATE` loads and validates the named plan's JSON, converts it to `mc_msgs/MissionUpload` (fresh `upload_id`), and publishes `/mc/gcs/mission_upload` — the same topic and message type `gcs_mavlink_node`'s mission-upload state machine (§11.2) already produces; `mission_manager_node` needs zero changes regardless of which node produced the upload. `activate` does **not** currently enforce any mission-phase restriction — the interface spec's §9 open item on phase-gating (recommended: restrict to PREFLIGHT) is unresolved, so `REJECTED_PHASE` is a defined ack result the node never actually returns yet.

**Plan library persistence.** One JSON file per plan (`<name>.json`, filename is the plan's identity) under `/var/drone-mc/mission_plans/`, additive to §4.5's YAML mission-upload persistence, not a replacement for it (different node, different directory, different format, no shared code).

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

The fcc-bridge image is the one exception to the plain two-stage template above: it adds a third stage that builds the Micro XRCE-DDS Agent from source (no ROS 2 dependency, built FROM plain Ubuntu rather than `ros2-core` — FCC-008) and copies the resulting binary into the runtime stage alongside the colcon install tree. `docker/fcc-bridge/entrypoint.sh` launches the Agent (transport chosen by `FCC_TRANSPORT`, HID-MC-001 §4.3) before `fcc_bridge_node`, and both processes share the container's lifecycle — see §12.2's `drone-mc-fcc-bridge.service` template, unchanged by this addition since the Agent is supervised as part of the same container rather than a separate unit.

The gcs-comms image (GCS-005) follows the same two-process-one-container shape as fcc-bridge, but needs no extra build stage — `ros-humble-mavros` is a prebuilt arm64 apt package (unlike the Agent), so it's installed directly in the runtime stage alongside the colcon-built `gcs_comms` install tree. `docker/gcs-comms/entrypoint.sh` launches `gcs_mavlink_node` first, waits for it to register, then launches `mavros_node` (§11.1) — either process dying takes the container down, same "supervised together" rationale as fcc-bridge's Agent+node pair.

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
Services in `be.slice`: `mavlink-router`, `zenoh-bridge`, `gcs-comms`, `mission-exec`, `vision`, `health-monitor`, `video-ingest`, `video-downlink`, `indoor-nav` (start-on-demand, see §12.4)

### 12.4 Startup Ordering

```
drone-mc.target
  └── health-monitor.service    (first — monitors everything else)
  └── mavlink-router.service    (no dependencies)
  └── zenoh-bridge.service      (no dependencies — cross-host DDS bridge, independent of mavlink-router)
  └── fcc-bridge.service        (after mavlink-router)
  └── gcs-comms.service         (after fcc-bridge, mavlink-router)
  └── swarm-coord.service       (after gcs-comms)
  └── mission-exec.service      (after swarm-coord, fcc-bridge, gcs-comms)
  └── vision.service            (after fcc-bridge)
  └── video-ingest.service       (no dependencies — HIF-007 is a separate physical link)
  └── video-downlink.service     (no dependencies — independent of video-ingest, FR-086)

on demand only (not WantedBy= drone-mc.target, never enabled):
  indoor-nav.service            (requires fcc-bridge, after gcs-comms)
```

`indoor-nav.service` (`gnss_sim_node`, `INDOOR-GNSS-SIMULATION-BACKLOG.md`) is deliberately **not** in the boot set. It injects a synthetic GPS fix into PX4 as `HIL_GPS` (ICD §3.3b), so it runs only when an operator starts it for an indoor flight or real-FCC bench (`FT-006`) session (`sudo systemctl start drone-mc-indoor-nav`). Its unit has no `[Install]` section, and `scripts/02-install-systemd-units.sh` installs but never enables it. `PartOf=drone-mc.target` still stops it with the target. It `Requires=` fcc-bridge because all of its inputs come from there (flow, baro, `vehicle_state`). gcs-comms is `After=` only, since the anchor command can also be published straight onto `/mc/gcs/command`. See `docs/DEPLOYMENT.md`.

`mission_orchestrator_node` (§11.4, GCS-004) has its own systemd unit and container as of `GCS-006` (2026-08-26, `PROJECT-PLAN-5-MONTH.md` Phase 3) — `config/systemd/drone-mc-mission-orchestrator.service` runs `ghcr.io/drone-mc/mission-orchestrator`, separate from `gcs-comms.service` per GCS-005's (2026-08-06) explicit exclusion (§11.1, §12.1). `mavlink-router-channel-c.conf` (§12.5) now has a corresponding running `mavlink-routerd` process too — `docker/mavlink-router/entrypoint.sh` launches it as a third independent process alongside Channel A/B's own two, same isolation pattern. The plan library at `/var/drone-mc/mission_plans/` is bind-mounted read-write from the host for persistence across container restarts. See `GCS-006`'s own ticket entry for the full deliverable list.

### 12.5 Configuration Files

All configuration lives in `/etc/drone-mc/` (bind-mounted read-only into every container):

```
/etc/drone-mc/
  environment                  ← environment variables (§9.4 of ICD)
  cyclonedds.xml               ← CycloneDDS same-host (lo-only) DDS domain config
  zenoh-bridge-dds.json5       ← zenoh-bridge-dds cross-host DDS bridge config (peer list, mode, topic allow-list)
  mavlink-router-channel-a.conf ← Channel A (GCS ↔ FCC) endpoint config
  mavlink-router-channel-b.conf ← Channel B (GCS ↔ MC) endpoint config
  mavlink-router-channel-c.conf ← Channel C (Mission Orchestrator ↔ MC) endpoint config (§11.4)
  camera_calibration.yaml      ← camera intrinsics and distortion
  params/                      ← ROS 2 parameter YAML files per node
    fcc_bridge.yaml
    gcs_comms.yaml              ← gcs_mavlink_node + mission_orchestrator_node (§11)
    mavros.yaml                 ← MAVROS override: fcu_url, target_component_id (GCS-005)
    formation_controller.yaml
    swarm_coordinator.yaml
    mission_manager.yaml
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
