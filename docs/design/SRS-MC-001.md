# System and Software Requirements Specification
## Swarm Search Drone — Mission Computer (AI MC)

| Field | Value |
|---|---|
| Document | MC-SRS-001 |
| Version | 0.1-DRAFT |
| Date | 2026-06-19 |
| Platform | Jetson Orin NX 16GB · JetPack 6.2.2 · Ubuntu 22.04 aarch64 |
| FCC | Pixhawk 6X · PX4 v1.17 |
| Stack | ROS 2 Humble · CycloneDDS (same-host) · zenoh-bridge-dds (cross-host) · uXRCE-DDS · MAVLink 2 |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [System Overview](#2-system-overview)
3. [Stakeholders and User Needs](#3-stakeholders-and-user-needs)
4. [System Requirements](#4-system-requirements)
   - 4.1 [Functional Requirements](#41-functional-requirements)
   - 4.2 [Interface Requirements](#42-interface-requirements)
   - 4.3 [Performance Requirements](#43-performance-requirements)
   - 4.4 [Safety Requirements](#44-safety-requirements)
5. [Software Requirements](#5-software-requirements)
   - 5.1 [Architecture](#51-architecture)
   - 5.2 [ROS 2 Node Specifications](#52-ros-2-node-specifications)
   - 5.3 [Container Specifications](#53-container-specifications)
   - 5.4 [Systemd Integration](#54-systemd-integration)
   - 5.5 [SITL Validation](#55-sitl-validation)
6. [Constraints and Assumptions](#6-constraints-and-assumptions)
7. [Verification Matrix](#7-verification-matrix)

---

## 1. Introduction

### 1.1 Purpose

This document specifies the system and software requirements for the Mission Computer (MC) of the Swarm Search Drone. It defines the functional behaviour, external interfaces, performance bounds, safety constraints, and software architecture that the MC must satisfy to operate as a cooperative member of an autonomous search swarm.

This specification is the primary contractual baseline between the system architecture, software development, and integration teams. All MC software releases shall be traceable to requirements herein.

### 1.2 Scope

Requirements apply to:

- Onboard functional behaviour of a single drone within a swarm during all mission phases
- Interfaces between the MC and: the Flight Control Computer (FCC), Ground Control Station (GCS), swarm peer MCs, the onboard downward-facing IP camera, and the front gimbal IP camera / GCS video downlink
- Software architecture, deployment packaging, and validation approach

**Out of scope:** GCS software, FCC firmware (PX4), MANET radio firmware, airframe design, battery management electronics.

### 1.3 Definitions and Acronyms

| Term | Definition |
|---|---|
| MC | Mission Computer — Jetson Orin NX 16GB running the drone AI software stack |
| FCC | Flight Control Computer — Pixhawk 6X running PX4 v1.17 |
| GCS | Ground Control Station — QGroundControl laptop at 192.168.10.10 |
| AU | Air Unit — MANET mesh datalink radio |
| MANET | Mobile Ad-hoc Network — mesh radio linking GCS and all drones |
| uXRCE-DDS | Micro XRCE-DDS bridge between PX4 uORB and ROS 2 |
| MAVROS | MAVLink ROS 2 node providing a ROS 2 interface to MAVLink autopilots |
| ISR | Intelligence, Surveillance and Reconnaissance |
| Voronoi cell | Bounded search area assigned to a drone via distributed Voronoi decomposition |
| FccBackend | Transport-agnostic MC interface to the FCC (decouples MAVLink UDP from uXRCE-DDS) |
| SITL | Software-In-The-Loop — PX4 simulation for pre-hardware validation |
| GNSS | Global Navigation Satellite System |
| RTL | Return-To-Launch — PX4 flight mode |
| NED | North-East-Down local coordinate frame |
| MUST | Mandatory. Failure to satisfy constitutes a system deficiency. |
| SHOULD | Strongly recommended. Non-compliance requires documented rationale. |
| MAY | Optional. Implementation at developer discretion. |

### 1.4 Applicable Documents

- PX4 Autopilot v1.17 documentation and uORB topic reference
- Micro XRCE-DDS Agent configuration reference
- MAVLink v2 message definition (mavlink.io)
- ROS 2 Humble API documentation
- CycloneDDS configuration guide
- Eclipse Zenoh / zenoh-plugin-dds (zenoh-bridge-dds) documentation
- Holybro Pixhawk Jetson Baseboard hardware manual
- NVIDIA Jetson Orin NX System Module datasheet (JetPack 6.2.2)

---

## 2. System Overview

### 2.1 System Description

The MC is an onboard compute node that bridges the FCC (which controls the airframe) and the swarm coordination, mission execution, and AI perception tasks requiring a general-purpose compute platform. It runs on a Jetson Orin NX 16GB paired with a Pixhawk 6X on a Holybro Pixhawk Jetson Baseboard.

The overarching goal is a GNSS-denied-capable autonomous drone that operates as a cooperative member of an autonomous search swarm, transitioning from GCS-directed setup during ingress to fully autonomous, onboard mission execution over the search area.

### 2.2 System Context

```
┌─────────────────────────────────────────────────────────────┐
│                        AIRBORNE DRONE                        │
│                                                              │
│  ┌──────────────────────────────┐   ┌────────────────────┐  │
│  │  Mission Computer (MC)       │   │  Pixhawk 6X (FCC)  │  │
│  │  Jetson Orin NX 16GB         │◄──►  PX4 v1.17 / NuttX │  │
│  │  JetPack 6.2.2 / Ubuntu 22   │   │  uORB topics       │  │
│  │  192.168.10.11               │   │  192.168.10.31     │  │
│  │                              │   └────────────────────┘  │
│  │  ROS 2 Humble / CycloneDDS   │      ▲ uXRCE-DDS          │
│  │  Docker / drone-mc.target    │      │ ttyTHS1 / 12.x eth  │
│  └──────────────────────────────┘                            │
│           │  MANET mesh radio (AU)                           │
└───────────┼─────────────────────────────────────────────────┘
            │
   ┌─────────┼─────────────────────────────────────────────┐
   │         │          MANET MESH  192.168.10.x            │
   │  ┌──────▼──────┐   ┌─────────────┐  ┌─────────────┐  │
   │  │  GCS / QGC   │   │  Peer MC #2 │  │  Peer MC #3 │  │
   │  │192.168.10.10 │   │192.168.10.12│  │192.168.10.13│  │
   │  └─────────────┘   └─────────────┘  └─────────────┘  │
   └────────────────────────────────────────────────────────┘
```

### 2.3 Network Topology (Fixed)

| Node | IP Address | Role |
|---|---|---|
| Jetson MC | 192.168.10.11 | Mission Computer — this system |
| Pixhawk FCC | 192.168.10.31 | Flight Control Computer |
| QGC Laptop | 192.168.10.10 | Ground Control Station |
| MC↔FCC internal | 192.168.12.x | Internal ethernet (also on MANET mesh) |
| Peer MCs | 192.168.10.12+ | Other drone MCs in swarm |

### 2.4 MAVLink Channel Separation

> **Important:** The MC operates three independent MAVLink channels. Commingling traffic on these channels is a defect.

| Channel | Transport | Purpose |
|---|---|---|
| GCS → FCC | MAVLink 2/UDP | Standard PX4 autopilot channel. MC does not inject traffic. |
| GCS → MC | MAVLink 2/UDP | Dedicated channel for mission upload and MC telemetry. Routed via mavlink-router on the MC. |
| MC ↔ FCC | MAVLink 2/UDP | 192.168.12.x ethernet. Mission protocol channel (stock PX4 has no uXRCE-DDS-writable mission topic). |
| MC ↔ FCC | uXRCE-DDS | Over ttyTHS1 serial. Not MAVLink. |

### 2.5 Mission Phases

| Phase | Description |
|---|---|
| `PREFLIGHT` | GCS uploads mission data (waypoints, formation offsets, search polygon, swarm manifest). MC performs self-test, peer discovery, and readiness verification. |
| `ARMING` / `TAKEOFF` | GCS authorises arm. MC arms FCC, commands takeoff to assigned altitude, confirms position hold. |
| `INGRESS` | Drone transits to operation area in straight-and-level cruise, maintaining assigned formation offset relative to the formation leader. GCS retains authority throughout. |
| `MUSTER` | All drones converge on the muster point. MC reports arrival and waits for swarm quorum. |
| `ISR_SELECTION` | Distributed election of which drones perform ISR duty, based on sensor capability, battery state, and position. Fully onboard. |
| `VORONOI_PARTITION` | Drones exchange positions and run consensus-based Voronoi decomposition of the search polygon. Each drone accepts its assigned cell. |
| `SEARCH` | Each drone executes a boustrophedon coverage path within its Voronoi cell. AI inference runs on camera feed. Detections reported to GCS in real-time. |
| `RETURN` / `LAND` | On search completion or GCS command. MC disarms FCC after landing confirmation. |
| `FAULT` | Critical fault declared. RTL commanded. |

---

## 3. Stakeholders and User Needs

### 3.1 Mission Operator (GCS)

Plans and uploads missions, monitors all drones, and can reassert authority at any time. Needs: reliable telemetry, timely detection reports, simple command/acknowledge interface, and confidence that autonomous operations remain within the planned search boundary.

### 3.2 Swarm System

The collective behaviour of all MCs. Needs: consistent message formats, deterministic consensus outcomes, and graceful handling of peer loss without mission collapse.

### 3.3 System Integrators and Developers

Build, integrate, and validate the MC software stack. Needs: containerised deployment for reproducibility, idempotent installation scripts, SITL support for test without hardware, and clear interface contracts between nodes.

---

## 4. System Requirements

**Requirement notation:** Each requirement has an ID, priority (MUST / SHOULD / MAY), and verification method (Test / Inspection / Analysis / Demo).

---

### 4.1 Functional Requirements

#### 4.1.1 Mission Phase Management

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-001 | MUST | Analysis | The MC shall implement a persistent mission phase state machine with the following named states: PREFLIGHT, ARMING, TAKEOFF, INGRESS, MUSTER, ISR_SELECTION, VORONOI_PARTITION, SEARCH, RETURN, LAND, FAULT. |
| FR-002 | MUST | Test | The MC shall transition between mission phases only upon receipt of a valid GCS phase-transition command or upon satisfaction of defined autonomous transition conditions. Undefined transitions shall be rejected and reported to the GCS. |
| FR-003 | MUST | Test | The GCS shall be able to command a mission phase transition at any point during flight. The MC shall execute the command within 500 ms of receipt. |
| FR-004 | MUST | Test | The MC shall broadcast its current mission phase to all swarm peers at ≥ 1 Hz as part of the swarm heartbeat message. |
| FR-005 | SHOULD | Inspection | The MC shall log all mission phase transitions with timestamp, source (GCS or autonomous), and the triggering condition. Logs shall survive container restart. |

#### 4.1.2 FCC Control and Monitoring

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-010 | MUST | Test | The MC shall arm the FCC only after receiving explicit GCS ARM authorisation as part of the current mission upload, and only after all MC pre-arm checks pass. |
| FR-011 | MUST | Test | The MC shall command FCC disarm immediately upon receipt of a GCS DISARM command or upon triggering of a safety failsafe condition defined in §4.4. |
| FR-012 | MUST | Test | The MC shall command FCC flight mode changes (HOLD, RTL, MISSION, TAKEOFF, OFFBOARD) via uXRCE-DDS `vehicle_command` topic using the PX4 v1.17 command set. `mission_manager_node` is the sole node authorised to issue a flight-mode command (§4.1.3, FR-026); OFFBOARD is commanded only for a phase/leg the active mission plan explicitly flags for it. |
| FR-013 | MUST | Test | The MC shall upload waypoints to the FCC for autonomous execution during INGRESS and MUSTER phases via the PX4 mission upload mechanism. |
| FR-014 | MUST | Test | The MC shall subscribe to and process FCC telemetry at ≥ 10 Hz: vehicle local position, vehicle global position, vehicle velocity, vehicle attitude, armed state, flight mode, battery voltage, and battery remaining percentage. |
| FR-015 | MUST | Test | The MC shall detect FCC communication loss (silence > 1 s on the uXRCE-DDS bridge), declare a critical fault, attempt to command HOLD if link recovers within 3 s, and notify the GCS. |
| FR-016 | MUST | Inspection | The MC shall use the FccBackend abstraction layer for all FCC interactions. The underlying transport (uXRCE-DDS or MAVLink UDP) shall be configurable at deployment time without modification to application code. |

#### 4.1.3 Formation Keeping

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-020 | MUST | Test | During INGRESS phase, the MC shall maintain its assigned formation offset (North, East, Down in metres) relative to the formation leader, commanding the FCC in MISSION mode via a continuously-updated mission item, unless the active mission plan flags the leg for OFFBOARD (`offboard_phase_mask`, ICD-MC-001 §9.2), in which case `formation_controller_node` streams OFFBOARD setpoints instead (FR-026–029). |
| FR-021 | MUST | Demo | Formation offsets for each drone shall be uploaded by the GCS as part of the pre-flight mission upload and stored persistently on the MC across restarts. |
| FR-022 | MUST | Test | The MC shall receive the formation leader's position and velocity from swarm peer messages at ≥ 5 Hz and compute NED position targets accordingly. |
| FR-023 | MUST | Test | Formation-keeping position targets shall be converted to geodetic coordinates and pushed to the FCC as a mission item replacement (PX4 mission upload mechanism) at ≥ 5 Hz while in MISSION mode. |
| FR-024 | MUST | Test | If the formation leader's state message is not received for > 2 s, the MC shall command FCC HOLD mode and alert the GCS with the reason. |
| FR-025 | SHOULD | Test | The formation controller shall implement a PD position controller with configurable gains for NED position error, tunable via a ROS 2 parameter file without code recompilation. |

#### 4.1.3a OFFBOARD Formation Keeping (Hybrid Mode Arbitration)

Reverses the blanket MISSION-only constraint FR-020 previously stated, scoped narrowly to legs the active mission plan explicitly flags — see `OFFBOARD-MODE-SWITCHING-BACKLOG.md` for the full design rationale, including why the original MISSION-only decision was made and why reintroducing OFFBOARD does not reopen that reliability tradeoff for phases that don't opt in.

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-026 | MUST | Test | `mission_manager_node` shall read the per-phase `offboard_phase_mask` from the active mission plan and command the corresponding FCC flight mode (MISSION or OFFBOARD) at each phase entry. It shall be the sole node in the system that commands a flight-mode change; `health_monitor_node` shall never call `send_command()` for a flight-mode change (FR-012, SFR-009). |
| FR-027 | MUST | Test | Before commanding `DO_SET_MODE OFFBOARD`, the MC shall establish an OFFBOARD setpoint stream at ≥ 2 Hz for at least 1 s, issue the mode command, then confirm entry by observing `/mc/fcc/vehicle_state.nav_state == OFFBOARD` within 1 s. If unconfirmed, the MC shall retry the mode command once; if still unconfirmed, it shall abort the OFFBOARD leg, remain in the vehicle's current mode, and raise fault `OFFBOARD_ENTRY_FAILED`. `formation_controller_node` shall not treat OFFBOARD as active until entry is confirmed. |
| FR-028 | MUST | Test | While OFFBOARD is the FCC's confirmed active mode, `formation_controller_node` shall publish OFFBOARD position setpoints at ≥ 5 Hz (tracking-quality target, mirrors FR-023's cadence), never below the PX4-imposed 2 Hz floor. |
| FR-029 | MUST | Test | Before commanding `DO_SET_MODE MISSION` to leave OFFBOARD, the MC shall confirm a valid mission is already loaded in FCC-side dataman for the upcoming leg, issue the mode command, then confirm exit by observing `/mc/fcc/vehicle_state.nav_state == MISSION` within 1 s (same retry-once-then-fault behaviour as FR-027) before `formation_controller_node` stops streaming. This is the same exit path used both for a normal phase exit and for the `OFFBOARD_STREAM_STALE` fallback (SFR-009). |

#### 4.1.4 Swarm Coordination

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-030 | MUST | Test | The MC shall discover and establish communication with all swarm peers listed in the uploaded swarm manifest within 30 s of entering PREFLIGHT state. |
| FR-031 | MUST | Test | The MC shall broadcast a swarm heartbeat at 1 Hz containing: drone ID, IP address, current mission phase, geodetic position, battery remaining (%), and sensor health flags. |
| FR-032 | MUST | Test | A swarm peer shall be declared LOST after 3 consecutive missed heartbeats (3 s at 1 Hz). The GCS shall be notified immediately upon peer loss declaration. |
| FR-033 | MUST | Test | The MC shall participate in a distributed ISR subnet selection algorithm during ISR_SELECTION phase. The algorithm shall elect a subset of drones for ISR duty based on sensor capability flags, battery remaining, and proximity to the search area centroid. |
| FR-034 | MUST | Test | The MC shall participate in consensus-based Voronoi decomposition during VORONOI_PARTITION phase. All participating drones shall converge on a consistent, non-overlapping partition of the search polygon within 10 s of phase entry. |
| FR-035 | MUST | Test | Upon consensus convergence, the MC shall store the boundaries of its assigned Voronoi cell and confirm acceptance to all peers before the SEARCH phase begins. |
| FR-036 | SHOULD | Test | If a peer drone is declared LOST during SEARCH phase, the remaining drones shall re-run Voronoi partitioning to redistribute the lost drone's uncovered cell among active drones. |

#### 4.1.5 Search Execution

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-040 | MUST | Test | The MC shall compute a boustrophedon (lawnmower) coverage path within its assigned Voronoi cell. Row spacing shall be parameterised by sensor footprint width and configurable overlap percentage. |
| FR-041 | MUST | Test | The MC shall upload the computed coverage path as a waypoint mission to the FCC and command autonomous mission execution for SEARCH phase transit. |
| FR-042 | MUST | Test | During SEARCH phase, the MC shall continuously process camera frames through the onboard AI inference model for target detection. |
| FR-043 | MUST | Test | All detections shall be reported to the GCS within 1 s of detection, including: detection class label, geodetic position estimate (lat, lon, alt), UTC timestamp, and confidence score (0.0–1.0). |
| FR-044 | MUST | Demo | Detection processing and GCS reporting shall not interrupt or pause coverage path execution. Both shall run concurrently. |
| FR-045 | SHOULD | Test | If the assigned cell boundary is updated (e.g., following peer loss re-partitioning), the MC shall recompute and re-upload the coverage path, resuming from the nearest waypoint to the drone's current position. |
| FR-046 | MUST | Test | The MC shall transition to RETURN phase upon completion of all coverage path waypoints or upon receipt of a GCS end-mission command. |

#### 4.1.6 Visual Localisation

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-050 | MUST | Test | The MC shall provide continuous position estimates via visual odometry when GNSS fix quality falls below a configurable threshold (default: fix type < 3D fix, or HDOP > 2.0). |
| FR-051 | MUST | Test | Visual odometry pose estimates shall be fused with FCC IMU data to produce a 6-DoF pose estimate, published on `/mc/local_pose` at ≥ 10 Hz, and fed back to the FCC for navigation. |
| FR-052 | MUST | Test | The MC shall notify the GCS whenever localisation mode transitions between GNSS-primary and visual-odometry-primary, with the reason for the transition. |
| FR-053 | SHOULD | Test | Position drift during purely visual-odometry-based operation shall not exceed 5 m/min under nominal flight conditions (adequate texture, no aggressive manoeuvres). |

#### 4.1.7 GCS Communication

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-060 | MUST | Test | The MC shall receive and persistently store mission data uploaded by the GCS: ingress waypoints, formation offset table, search area polygon (geodetic vertices), and swarm manifest (drone IDs and IPs). |
| FR-061 | MUST | Test | The MC shall acknowledge every GCS command with a success or failure response within 500 ms, including a reason code on failure. |
| FR-062 | MUST | Test | The MC shall stream a status telemetry message to the GCS at ≥ 2 Hz containing: geodetic position, heading, groundspeed, altitude, mission phase, localisation mode, battery remaining (%), and a bitmask of active faults. |
| FR-063 | MUST | Test | Target detection reports shall be streamed to the GCS as they are generated (event-driven), independently of the periodic telemetry stream. |
| FR-064 | MUST | Inspection | All GCS communication shall use the dedicated MC↔GCS MAVLink channel. MC software shall not inject traffic onto the FCC autopilot MAVLink channel. |

#### 4.1.8 Health Monitoring and Failsafe

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-070 | MUST | Test | The MC shall continuously monitor: CPU utilisation, memory usage, disk space remaining, GPU temperature, and the liveness of all critical ROS 2 nodes. |
| FR-071 | MUST | Test | The health monitor shall publish a `/mc/health` message at ≥ 1 Hz containing resource metrics and a bitmask of active health flags, and stream it to the GCS. |
| FR-072 | MUST | Test | The MC shall command FCC RTL when battery remaining falls below the configurable `LOW_BATTERY_THRESHOLD` (default: 20%), unless an active GCS override is in effect. |
| FR-073 | MUST | Test | The MC shall command FCC HOLD if the GCS link is lost for > 10 s while the drone is in any autonomous mission phase (INGRESS, MUSTER, ISR_SELECTION, VORONOI_PARTITION, SEARCH). |
| FR-074 | MUST | Test | A watchdog shall monitor the `mission_manager_node` heartbeat. If no heartbeat is received for > 5 s, the watchdog shall trigger a node restart via systemd and notify the GCS. |
| FR-075 | MUST | Test | After 3 consecutive failed restart attempts on any critical node, the health monitor shall declare a critical fault and command FCC RTL via the FccBackend. |
| FR-076 | MUST | Inspection | All commands issued to the FCC shall be logged with: UTC timestamp, command type, parameters, commanding source (GCS / autonomous manager / watchdog / health monitor), and FCC acknowledgement result. Logs shall persist across container restart. |

#### 4.1.9 Gimbal Perception and Dual-Camera Video Downlink

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| FR-080 | MUST | Test | The MC shall continuously ingest RTSP/H.265 video from the front gimbal IP camera via `perception_front_node` and run AI inference for target detection and tracking, independently of the downward-camera detection pipeline (§4.1.5, §4.2.4). |
| FR-081 | MUST | Test | `perception_front_node` shall report detections within 1 s of detection and publish continuously-updated tracks (track ID, class, confidence, geodetic position) at ≥ 5 Hz while the gimbal video pipeline is active. |
| FR-082 | SHOULD | Analysis | Gimbal-camera detection processing shall run concurrently with downward-camera detection processing without degrading either pipeline's frame rate below its configured minimum. |
| FR-083 | MUST | Test | The MC shall re-stream the front gimbal video, with embedded per-frame detection/track metadata, to the GCS via a dedicated RTSP/H.265 video downlink (`video_mux_node`), independent of the MC↔GCS MAVLink telemetry channel (§4.1.7). |
| FR-084 | MUST | Test | Loss of the gimbal camera RTSP link for > 3 s shall be treated as perception-only degradation: the health monitor shall declare `GIMBAL_LINK_LOST` and clear the `GIMBAL_OK` sensor flag. This condition shall not trigger any flight failsafe action. |
| FR-085 | MUST | Test | The MC shall re-stream the downward CSI camera video, with embedded per-frame detection metadata, to the GCS via a second, independent RTSP/H.265 video downlink (`video_mux_node`, Stream B), in addition to and independent of the existing front-gimbal downlink (Stream A, FR-083). This is additive — `perception_down_node`'s existing detection/geolocation role (§4.1.5) is unchanged. |
| FR-086 | MUST | Test | The downward-camera video downlink (Stream B) shall be an independent failure domain from the front-gimbal downlink (Stream A): loss of `camera_node`/`perception_down_node` frames or detections for > 3 s shall stop Stream B and set its `streaming` status false without affecting Stream A, and vice versa. |

---

### 4.2 Interface Requirements

#### 4.2.1 MC ↔ FCC

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| IR-001 | MUST | Test | The MC shall communicate with the FCC via uXRCE-DDS over ttyTHS1 serial (primary) and via MAVLink over 192.168.12.x ethernet link (for the mission protocol channel). The active transport shall be indicated in `/mc/health`. |
| IR-002 | MUST | Inspection | Transport selection (serial vs. ethernet) shall be controlled by a deployment-time configuration parameter without code changes. |
| IR-003 | MUST | Inspection | The FccBackend interface shall expose transport-agnostic ROS 2 topics: subscribed topics for vehicle state (position, velocity, attitude, status, battery) and published topics for commands (mission upload, mission item update, vehicle_command). Application code shall interact with the FccBackend interface only. |
| IR-004 | MUST | Inspection | The MC shall subscribe to the following PX4 uORB topics via uXRCE-DDS: `vehicle_local_position`, `vehicle_global_position`, `vehicle_status`, `battery_status`, `vehicle_attitude`, `vehicle_odometry`, `sensor_combined`. |
| IR-005 | MUST | Inspection | The MC shall publish to the following PX4 uORB topics via uXRCE-DDS: `vehicle_command`. |
| IR-006 | MUST | Test | End-to-end command latency from MC publishing `vehicle_command` to PX4 actuation confirmation shall be ≤ 100 ms. |
| IR-007 | MUST | Test | The MC shall upload and replace FCC mission items using the standard MAVLink mission protocol (full upload: `MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK`; single-item replace: `MISSION_WRITE_PARTIAL_LIST` partial update) over a dedicated UDP port on the 192.168.12.x link, independent of the `FCC_TRANSPORT` DDS transport selection (IR-002). This is the only MAVLink traffic on the MC↔FCC interface — uXRCE-DDS is otherwise used exclusively (stock PX4 has no DDS-writable mission topic). |

#### 4.2.2 MC ↔ GCS

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| IR-010 | MUST | Inspection | The MC shall maintain a dedicated MAVLink 2/UDP channel to the GCS at 192.168.10.10, separate from the FCC autopilot channel. |
| IR-011 | MUST | Test | The MC shall transmit a MAVLink `HEARTBEAT` at 1 Hz on the GCS channel, with `system_id` set to the drone's assigned ID and `component_id = MAV_COMP_ID_ONBOARD_COMPUTER`. |
| IR-012 | MUST | Inspection | All MAVLink traffic on the GCS channel shall be routed through mavlink-router v4. mavlink-router shall be the sole ingress and egress point for this channel. |
| IR-013 | MUST | Inspection | Swarm mission upload messages (waypoints, formation offsets, search polygon, swarm manifest) shall use the MAVLink `COMMAND_LONG` or mission item protocol. Custom payload encoding shall be documented in the ICD. |
| IR-014 | SHOULD | Inspection | MAVROS shall be used for GCS-facing MAVLink services where a native MAVROS interface exists (e.g., `SET_MODE`, `COMMAND_LONG`). |

#### 4.2.3 MC ↔ Swarm Peers

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| IR-020 | MUST | Test | The MC shall communicate with peer drones over the MANET mesh datalink using UDP on the 192.168.10.x subnet. Peer IPs shall be sourced from the GCS-uploaded swarm manifest. |
| IR-021 | MUST | Inspection | All swarm coordination messages shall use a versioned binary protocol with a fixed header containing: protocol version (uint8), sender drone ID (uint8), message type (uint8), sequence number (uint16), and UTC timestamp (uint64, µs). |
| IR-022 | MUST | Test | The swarm coordination layer shall tolerate peer message latency up to 500 ms without treating the peer as LOST. |
| IR-023 | MUST | Test | Peer LOST shall be declared after 3 consecutive missed heartbeats at the nominal 1 Hz rate (i.e., > 3 s silence). |

#### 4.2.4 MC ↔ Camera

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| IR-030 | MUST | Test | The MC shall ingest frames from the IMX219 CSI camera via the gscam2 ROS 2 node. |
| IR-031 | MUST | Test | The camera node shall publish `/camera/image_raw` and `/camera/camera_info` at ≥ 15 FPS during SEARCH and INGRESS phases. |
| IR-032 | MUST | Test | The `visual_localisation_node` shall subscribe to `/camera/image_raw` and the IMU topic bridged from the FCC via uXRCE-DDS. |
| IR-033 | SHOULD | Inspection | Camera calibration parameters (intrinsics, distortion model) shall be stored in a calibration file on the MC and loaded at node startup. The `camera_info` topic shall reflect the loaded calibration. |

#### 4.2.5 MC ↔ Gimbal Camera

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| IR-040 | MUST | Test | The MC shall ingest RTSP/H.265 video from the front gimbal IP camera over the HIF-007 Ethernet path — the baseboard's existing internal Ethernet switch, reached via Jetson's single `eth0` interface (192.168.10.11) to the camera at 192.168.10.200 — via `perception_front_node`, independent of the downward CSI camera interface (§4.2.4). |

#### 4.2.6 MC ↔ GCS Video Downlink

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| IR-041 | MUST | Test | The MC shall serve the front gimbal video (H.265) with embedded per-frame metadata (UTC timestamp, track ID, class, confidence, geodetic position) to the GCS via a dedicated RTSP session (`video_mux_node`), independent of the MC↔GCS MAVLink channel (§4.2.2). |
| IR-042 | MUST | Inspection | The GCS video downlink RTSP session shall be independently restartable, with health reported via `/mc/video/downlink_status`, without affecting MAVLink telemetry or command traffic on the MC↔GCS channel. |
| IR-043 | MUST | Test | The MC shall serve the downward CSI camera video (H.265) with embedded per-frame detection metadata (UTC timestamp, class, confidence, geodetic position) to the GCS via a second, independent RTSP session (`video_mux_node`, Stream B), health reported via `/mc/video/down_downlink_status`, without affecting Stream A (IR-041/042), the MC↔GCS MAVLink channel, or `perception_down_node`'s existing detection role. |

---

### 4.3 Performance Requirements

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| PR-001 | MUST | Test | Mission state machine update rate: ≥ 10 Hz. |
| PR-002 | MUST | Test | Formation position setpoints published to FCC: ≥ 5 Hz during INGRESS. |
| PR-003 | MUST | Test | GCS status telemetry stream rate: ≥ 2 Hz. |
| PR-004 | MUST | Test | FCC telemetry receive and process rate: ≥ 10 Hz. |
| PR-005 | MUST | Test | Swarm heartbeat transmit rate: 1 Hz. Peer loss declared after 3 s silence. |
| PR-006 | MUST | Test | Camera frame delivery to detection pipeline: ≥ 15 FPS during SEARCH. |
| PR-007 | MUST | Test | Detection pipeline end-to-end latency (frame ingestion to `/mc/detections` publish): ≤ 500 ms. |
| PR-008 | MUST | Test | Coverage path computation for a Voronoi cell of up to 1 km²: ≤ 5 s. |
| PR-009 | MUST | Test | All MC services shall reach operational state within 120 s of `drone-mc.target` activation. |
| PR-010 | SHOULD | Test | AI inference shall consume ≤ 80% of available GPU resources, preserving ≥ 20% for visual odometry and system overhead. |
| PR-011 | SHOULD | Test | Visual odometry pose estimates shall be published at ≥ 10 Hz. |

---

### 4.4 Safety Requirements

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| SFR-001 | MUST | Test | The MC shall not command FCC ARM without explicit GCS ARM authorisation present in the current uploaded mission. Unauthorised arm attempts shall be rejected and logged. |
| SFR-002 | MUST | Test | The MC shall not disable, suppress, or override any PX4 failsafe configuration. All MC-commanded actions shall remain within the safety envelope defined by the PX4 v1.17 failsafe configuration. |
| SFR-003 | MUST | Test | The `mission_manager_node` watchdog shall trigger FCC HOLD and notify the GCS if the manager's heartbeat is silent for > 5 s. |
| SFR-004 | MUST | Test | After 3 consecutive failed systemd restart attempts on any node declared critical, the health monitor shall command FCC RTL via the FccBackend and set mission phase to FAULT. |
| SFR-005 | MUST | Inspection | The MC shall validate the schema and parameter ranges of all incoming MAVLink messages and swarm protocol messages before acting on their content. Malformed messages shall be discarded and logged. |
| SFR-006 | MUST | Inspection | All commands issued to the FCC shall be logged with: UTC timestamp, command type, parameters, commanding source (GCS / autonomous manager / watchdog / health monitor), and FCC acknowledgement result. Logs shall persist across container restart. |
| SFR-007 | MUST | Test | The MC shall command FCC HOLD if the GCS datalink is lost for > 10 s while in any autonomous phase (INGRESS, MUSTER, ISR_SELECTION, VORONOI_PARTITION, SEARCH). |
| SFR-008 | MUST | Test | The MC shall command FCC RTL when battery remaining falls below `LOW_BATTERY_THRESHOLD` (default: 20%). This shall not be suppressible unless a GCS override command is active and the GCS link is healthy. |
| SFR-009 | MUST | Test | If the OFFBOARD setpoint stream age exceeds 500 ms while OFFBOARD is the FCC's confirmed active mode (mirrors the FR-003 500 ms GCS-override response bound), `health_monitor_node` shall publish `/mc/fault` with `OFFBOARD_STREAM_STALE` and a recommended fallback action; it shall not command the FCC directly. `mission_manager_node` shall treat this fault as a phase-exit trigger and execute the fallback through FR-029's exit-sequencing path (MISSION if a valid mission is loaded, else HOLD). PX4's own `COM_OBL_ACT` failsafe remains enabled as a slower backstop, configured with a longer timeout than this 500 ms threshold (HID-MC-001, PX4 parameter reference). |

---

## 5. Software Requirements

### 5.1 Architecture

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| SW-001 | MUST | Inspection | All MC software components shall be packaged as Docker containers managed by Docker Engine on the Jetson Orin NX. |
| SW-002 | MUST | Inspection | All containers shall be orchestrated under the `drone-mc.target` systemd target. Starting the target shall bring up all MC services in correct dependency order. |
| SW-003 | MUST | Inspection | All ROS 2 nodes shall use CycloneDDS as the DDS middleware for same-host discovery. CycloneDDS configuration shall be managed via a shared `cyclonedds.xml` mounted into each container, scoped to the loopback interface only — it does not perform discovery across hosts (see SW-008). |
| SW-008 | MUST | Inspection | Cross-host DDS transport, where needed, shall ride a `zenoh-bridge-dds` container per drone, bridging selected DDS topics over a Zenoh mesh in peer mode with a static per-drone connect/listen endpoint list (`zenoh-bridge-dds.json5`), not CycloneDDS's own multicast/unicast discovery protocol — better suited to the bandwidth-constrained, lossy MANET link (IR-020). The bridge's topic allow-list shall default to forwarding nothing until a specific cross-host DDS topic requires it. |
| SW-004 | MUST | Inspection | All ROS 2 packages shall be built in a shared colcon workspace bind-mounted into all containers that require it, avoiding redundant builds per container. |
| SW-005 | MUST | Inspection | All installation and configuration scripts shall be numbered sequentially (e.g., `06-`, `07-`, `08-`), idempotent, and guarded with `set -euo pipefail`. |
| SW-006 | MUST | Inspection | A companion smoke-test script (e.g., `06b-smoke-test-*.sh`) shall exist for each installation script. Smoke tests shall emit explicit PASS / FAIL for each check, with non-zero exit on any failure. |
| SW-007 | MUST | Inspection | Application-layer nodes shall be written in C++17. GCS-side utilities (test, monitor, log, simulation) shall be written in Python 3. |

### 5.2 ROS 2 Node Specifications

#### fcc_bridge_node

Bridges PX4 uXRCE-DDS topics to ROS 2. Implements the FccBackend interface. Provides transport-agnostic vehicle state and command topics to all other nodes.

| Direction | Topics |
|---|---|
| Subscribes | `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_global_position`, `/fmu/out/vehicle_status`, `/fmu/out/battery_status`, `/fmu/out/vehicle_attitude`, `/fmu/out/vehicle_odometry` |
| Publishes | `/mc/fcc/vehicle_state` |
| Publishes (to FCC) | `/fmu/in/mission`, `/fmu/in/vehicle_command` |

**Req:** SW-010 · MUST · Test

#### gcs_mavlink_node

Manages the MC↔GCS dedicated MAVLink 2/UDP channel. Translates GCS commands to ROS 2 services and MC telemetry to MAVLink for the GCS.

| Direction | Topics / Services |
|---|---|
| Subscribes | `/mc/fcc/vehicle_state`, `/mc/mission_phase`, `/mc/health`, `/mc/detections` |
| Publishes | `/mc/gcs/mission_upload`, `/mc/gcs/command`, `/mc/gcs/link_status` |

**Req:** SW-011 · MUST · Test

#### swarm_coordinator_node

Manages peer discovery, swarm heartbeat exchange, ISR subnet selection algorithm, and consensus-based Voronoi decomposition. The authoritative source of swarm state on each drone.

| Direction | Topics |
|---|---|
| Publishes | `/mc/swarm/peer_states`, `/mc/swarm/voronoi_cell`, `/mc/swarm/isr_role` |

**Req:** SW-012 · MUST · Test

#### formation_controller_node

Receives formation leader state from swarm coordinator. Computes NED position targets via PD controller to hold assigned offset, converts to geodetic, and pushes as a mission item replacement. Publishes at ≥ 5 Hz during INGRESS.

| Direction | Topics |
|---|---|
| Subscribes | `/mc/swarm/peer_states`, `/mc/fcc/vehicle_state` |
| Publishes | `/mc/fcc/mission_item_update` |

**Req:** SW-013 · MUST · Test

#### mission_manager_node

Top-level mission phase state machine. Arbitrates between GCS commands and autonomous triggers. Emits watchdog heartbeat at ≥ 1 Hz. Single point of truth for current mission phase.

| Direction | Topics |
|---|---|
| Publishes | `/mc/mission_phase`, `/mc/manager/heartbeat` |

**Req:** SW-014 · MUST · Test

#### search_planner_node

Computes boustrophedon coverage path from Voronoi cell boundary and sensor footprint parameters. Outputs waypoint list for FCC upload.

| Direction | Topics |
|---|---|
| Subscribes | `/mc/swarm/voronoi_cell` |
| Publishes | `/mc/search/coverage_path` |

**Req:** SW-015 · MUST · Test

#### visual_localisation_node

Runs visual odometry on camera frames. Fuses with IMU data. Publishes 6-DoF pose at ≥ 10 Hz. Feeds pose back to FCC via `vehicle_visual_odometry` uORB topic when GNSS is degraded.

| Direction | Topics |
|---|---|
| Subscribes | `/camera/image_raw`, `/mc/fcc/vehicle_state` (IMU) |
| Publishes | `/mc/local_pose`, `/mc/localisation_mode` |

**Req:** SW-016 · MUST · Test

#### camera_node (gscam2)

IMX219 CSI camera driver. Publishes raw image and camera info at ≥ 15 FPS. Loads calibration from file at startup.

| Direction | Topics |
|---|---|
| Publishes | `/camera/image_raw`, `/camera/camera_info` |

**Req:** SW-017 · MUST · Test

#### perception_down_node (renamed from detection_node)

Subscribes to `/camera/image_raw`. Runs AI inference on Jetson GPU. Publishes detection reports with class, geodetic position estimate, confidence, and timestamp.

| Direction | Topics |
|---|---|
| Subscribes | `/camera/image_raw`, `/mc/local_pose` |
| Publishes | `/mc/detections` |

**Req:** SW-018 · MUST · Test

#### perception_front_node

RTSP/H.265 client for the front gimbal IP camera (HIF-007, IFD-006). Runs AI inference and multi-frame tracking on Jetson GPU, independent of `perception_down_node`. Publishes detections and continuously-updated tracks.

| Direction | Topics |
|---|---|
| Publishes | `/mc/perception/front/detections`, `/mc/perception/front/tracks` |

**Req:** SW-020 · MUST · Test

#### video_mux_node

Subscribes to `perception_front_node`'s frame stream and track/detection topics (Stream A) and, additively, to `camera_node`'s frame stream and `perception_down_node`'s detections (Stream B). Re-encodes each to H.265, muxes a per-frame metadata track, and serves each as its own independent RTSP session to the GCS (IFD-007), independent of `gcs_mavlink_node` and of each other.

| Direction | Topics |
|---|---|
| Subscribes | `/mc/perception/front/detections`, `/mc/perception/front/tracks` (Stream A); `/camera/image_raw`, `/mc/detections` (Stream B) |
| Publishes | `/mc/video/downlink_status` (Stream A), `/mc/video/down_downlink_status` (Stream B) |

**Req:** SW-021, FR-085, FR-086, IR-043 · MUST · Test

#### health_monitor_node

Monitors CPU, memory, disk, GPU temperature, and node liveness via ROS 2 alive checks. Publishes `/mc/health` at ≥ 1 Hz. Triggers failsafe actions per §4.1.8.

| Direction | Topics |
|---|---|
| Publishes | `/mc/health`, `/mc/fault` |

**Req:** SW-019 · MUST · Test

### 5.3 Container Specifications

| Req ID | Container | Contents | Special Access |
|---|---|---|---|
| SW-030 | `ros2-core` | ROS 2 Humble base, CycloneDDS (same-host only), shared colcon workspace | Base layer for all containers |
| SW-031 | `fcc-bridge` | Micro XRCE-DDS Agent, fcc_bridge_node | `DeviceAllow ttyTHS1` |
| SW-032 | `mavlink-router` | mavlink-router v4 daemon, routing config | `network_mode=host` |
| SW-033 | `gcs-comms` | gcs_mavlink_node, MAVROS | `network_mode=host` |
| SW-034 | `swarm-coord` | swarm_coordinator_node, formation_controller_node | `network_mode=host` |
| SW-035 | `mission-exec` | mission_manager_node, search_planner_node, watchdog supervisor | — |
| SW-036 | `vision` | camera_node (gscam2), visual_localisation_node, perception_down_node (renamed from detection_node) | GPU device, `/dev/video*` |
| SW-037 | `health-monitor` | health_monitor_node | Read-only `/sys`, `/proc` |
| SW-038 | `video-ingest` | perception_front_node | GPU device, `network_mode=host` (RTSP :8554 ingest) |
| SW-038a | `video-downlink` | video_mux_node | `network_mode=host` (RTSP ports 8555/8556) |
| SW-039 | `zenoh-bridge` | zenoh-bridge-dds daemon, cross-host DDS bridge config | `network_mode=host` |

### 5.4 Systemd Integration

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| SW-040 | MUST | Inspection | Each Docker container shall have a corresponding systemd service unit. All units shall declare `Requires=` and `After=` dependencies to enforce startup order. |
| SW-041 | MUST | Inspection | All MC service units shall be grouped under `drone-mc.target`. Starting the target shall bring up all MC services; stopping it shall cleanly shut down all containers. |
| SW-042 | MUST | Inspection | Service units shall declare resource limits: `DeviceAllow` (where required), `MemoryMax`, `CPUQuota`, `PrivateTmp=yes`, `ProtectSystem=strict`. |
| SW-043 | MUST | Inspection | cgroups v2 slices shall partition resources: `rt.slice` for real-time services (fcc-bridge, swarm-coord); `be.slice` for best-effort services (logging, telemetry archival). |

### 5.5 SITL Validation

| ID | Priority | Verification | Requirement |
|---|---|---|---|
| SW-050 | MUST | Test | All ROS 2 nodes shall be validated against PX4 SITL (Gazebo or jMAVSim) before hardware-in-the-loop testing commences on physical drones. |
| SW-051 | MUST | Test | SITL validation shall exercise all mission phases end-to-end with a minimum two-drone simulated swarm, covering peer discovery, formation keeping, Voronoi partition, and search execution. |
| SW-052 | MUST | Inspection | All MC software components shall support a `SIMULATION_MODE` deployment flag. When set, hardware interfaces (uXRCE-DDS serial, CSI camera) shall be replaced with SITL-compatible equivalents without code branching in application logic. |

---

## 6. Constraints and Assumptions

### 6.1 Platform Constraints

- Target platform is Jetson Orin NX 16GB, JetPack 6.2.2, Ubuntu 22.04 LTS aarch64. Requirements are validated on this platform only.
- FCC is Pixhawk 6X running PX4 v1.17. uORB topic names, types, and MAVLink message IDs are fixed to this version. Note: the vendored `px4_msgs` ROS 2 package (the DDS-side mirror of these uORB types) is currently still pinned to PX4 `release/1.14` pending `PROJECT-PLAN-5-MONTH.md`'s `DEP-009` — this is a tracked gap, not a second target version.
- Network topology (IP addresses) is fixed as defined in §2.3 and is not configurable at runtime.
- The MANET mesh radio is assumed to provide UDP transport with maximum link latency ≤ 500 ms and loss rate < 5% within the operating area.

### 6.2 Operational Assumptions

- The swarm consists of a fixed, known set of drones defined at mission upload time. Dynamic entry of new drones mid-mission is not required.
- GNSS-denied operation is bounded to environments with adequate visual texture for feature-based odometry. Featureless environments (open water, uniform terrain) are out of scope for visual localisation.
- The search area polygon is convex or simply-connected (no holes). Non-convex polygons with holes are out of scope for the Voronoi decomposition algorithm.
- The formation leader is designated at mission upload time and does not change during INGRESS phase.
- One drone in the swarm is designated the ISR selection and Voronoi partition coordinator. Coordinator designation is specified in the swarm manifest uploaded by the GCS.

### 6.3 Build and Deployment Assumptions

- PlantUML (v1.2026 compatible) is used for all architecture block diagrams.
- Linear is used to track architectural decisions and phase completions.
- Docker images are built natively on the Jetson (aarch64) or cross-compiled and pushed to a local registry accessible from the Jetson.

---

## 7. Verification Matrix

All requirements shall be verified prior to hardware-in-the-loop acceptance.

**Methods:** T = Test · I = Inspection · A = Analysis · D = Demo  
**Phases:** SITL = Software-in-the-loop · HIL = Hardware-in-the-loop · Int = Integration

| ID | Summary | Method | Phase |
|---|---|---|---|
| FR-001 | Mission phase state machine | A | SITL |
| FR-002 | Phase transition validity | T | SITL |
| FR-003 | GCS phase command ≤ 500 ms | T | SITL |
| FR-004 | Phase broadcast ≥ 1 Hz | T | SITL |
| FR-005 | Transition logging persists | I | SITL |
| FR-010 | ARM requires GCS authorisation | T | SITL |
| FR-011 | DISARM on command / failsafe | T | SITL |
| FR-012 | Flight mode commands | T | SITL |
| FR-013 | Waypoint upload to FCC | T | SITL |
| FR-014 | FCC telemetry ≥ 10 Hz | T | SITL |
| FR-015 | FCC comms loss detection | T | SITL |
| FR-016 | FccBackend abstraction | I | Code review |
| FR-020–029 | Formation keeping (INGRESS), incl. hybrid MISSION/OFFBOARD arbitration | T / D | SITL → HIL |
| FR-030–036 | Swarm coordination | T | SITL (≥ 2 drones) |
| FR-040–046 | Search execution | T / D | SITL → HIL |
| FR-050–053 | Visual localisation | T | HIL |
| FR-060–064 | GCS communication | T / I | SITL |
| FR-070–076 | Health and failsafe | T / I | SITL |
| FR-080–086 | Gimbal + downward perception and video downlink | T / A | SITL → HIL |
| IR-001–007 | MC ↔ FCC interface | T / I | SITL → HIL |
| IR-010–014 | MC ↔ GCS MAVLink | T / I | SITL |
| IR-020–023 | Swarm peer protocol | T / I | SITL |
| IR-030–033 | Camera interface | T / I | HIL |
| IR-040 | Gimbal camera interface | T | HIL |
| IR-041–043 | GCS video downlink (both streams) | T / I | HIL |
| PR-001–011 | Performance rates and latencies | T | SITL → HIL |
| SFR-001–009 | Safety and failsafe | T / I | SITL |
| SW-001–008 | Architecture and build conventions | I | Int |
| SW-010–021 | ROS 2 node interface contracts | T | SITL |
| SW-030–039, SW-038a | Container packaging | I / T | Int |
| SW-040–043 | Systemd integration | I / T | Int |
| SW-050–052 | SITL validation coverage | T | SITL |

---

*MC-SRS-001 · v0.1-DRAFT · 2026-06-19*
