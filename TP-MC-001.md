# Test Plan
## Swarm Search Drone — Mission Computer (AI MC)

| Field | Value |
|---|---|
| Document | MC-TP-001 |
| Version | 0.1-DRAFT |
| Date | 2026-06-20 |
| Parent SRS | MC-SRS-001 v0.1-DRAFT |
| Parent ICD | MC-ICD-001 v0.1-DRAFT |
| Parent SDD | MC-SDD-001 v0.1-DRAFT |
| Platform | Jetson Orin NX 16GB · JetPack 6.2.2 · Ubuntu 22.04 aarch64 |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Test Strategy](#2-test-strategy)
3. [Test Environment](#3-test-environment)
4. [Unit Tests — TP-UT](#4-unit-tests--tp-ut)
5. [Node Integration Tests — TP-NI](#5-node-integration-tests--tp-ni)
6. [SITL System Tests — TP-ST](#6-sitl-system-tests--tp-st)
7. [Hardware-in-the-Loop Tests — TP-HW](#7-hardware-in-the-loop-tests--tp-hw)
8. [Deployment Smoke Tests — TP-SM](#8-deployment-smoke-tests--tp-sm)
9. [Requirements Traceability Matrix](#9-requirements-traceability-matrix)
10. [Test Schedule](#10-test-schedule)
11. [Defect Management](#11-defect-management)

---

## 1. Introduction

### 1.1 Purpose

This document defines the test approach, test cases, and acceptance criteria for the Mission Computer (MC) software. It establishes what is tested at each level, how tests are executed, and what constitutes a pass for each requirement in MC-SRS-001.

### 1.2 Scope

This plan covers:

- **Software under test:** All ROS 2 nodes and supporting libraries in the MC colcon workspace.
- **System under test (SITL):** The full MC software stack running against PX4 SITL (Gazebo) with simulated swarm peers.
- **System under test (HIL):** The full MC software stack on physical Jetson Orin NX + Pixhawk 6X hardware.
- **Out of scope:** PX4 firmware validation, QGroundControl functionality, MANET radio RF performance, airframe mechanics.

### 1.3 Definitions

| Term | Meaning |
|---|---|
| DUT | Device Under Test — the Jetson Orin NX running MC software |
| SITL | Software-In-The-Loop — PX4 simulated in Gazebo on a desktop PC |
| HIL | Hardware-In-The-Loop — physical Jetson + Pixhawk, no airframe |
| Mock node | A ROS 2 node that publishes pre-recorded or synthetic data at a specified rate, replacing a hardware-facing node in tests |
| Pass | Test produces the expected result and all measured values are within tolerance |
| Fail | Any expected result is not met, or any measured value is outside tolerance |
| Blocked | Test cannot be executed due to a prerequisite failure |

### 1.4 Risk Areas

| Risk | Mitigation |
|---|---|
| Transport mismatch between SITL and production (uXRCE-DDS vs MAVLink UDP) | `FccBackend` abstraction validated at unit level; SITL validated with `MavlinkUdpBackend`; HIL validated with `XrceDdsBackend` — same application nodes both ways |
| Voronoi consensus divergence across drone instances | Deterministic algorithm inputs are tested in unit tests; SITL multi-drone test validates agreement |
| VO drift causing incorrect geolocation | Drift bound tested in HIL; unit test validates geolocation geometry against ground-truth pose |
| Watchdog restart causing mission state loss | Mission upload persistence to disk tested; recovery sequence tested in SITL |
| GPU thermal throttling degrading inference latency | HIL endurance test measures latency under sustained load at operating temperature |
| Gimbal RTSP link flapping incorrectly triggering a flight failsafe | `GIMBAL_LINK_LOST` is explicitly excluded from the failsafe decision tree (SDD §10.3, FR-084); TP-ST-013 verifies no HOLD/RTL occurs on gimbal link loss |

---

## 2. Test Strategy

### 2.1 Test Levels

```
TP-UT  Unit Tests          gtest      Algorithms, guards, state logic in isolation
TP-NI  Node Integration    launch_testing + rclpy    ROS 2 node pub/sub/service behaviour
TP-ST  SITL System Tests   pytest + rclpy + PX4 SITL Full mission phases, failsafe scenarios
TP-HW  Hardware-in-the-Loop           Jetson + Pixhawk    Performance, latency, resource bounds
TP-SM  Smoke Tests         Bash scripts               Deployment validation after installation
```

Tests run in this order during CI: TP-SM → TP-UT → TP-NI → TP-ST. TP-HW is run manually on physical hardware before each milestone.

### 2.2 Pass/Fail Criteria

A test level passes when **all** test cases in that level pass. A single failing test case fails the level. The software is not promoted to the next level until the current level passes.

### 2.3 Regression Policy

All TP-UT and TP-NI tests run on every pull request. TP-ST runs nightly. TP-HW runs on milestone branches (M2, M4, M6).

### 2.4 Mock Strategy

Hardware-facing nodes (`fcc_bridge_node`, `camera_node`) are replaced by mock nodes in TP-NI tests. Mock nodes publish data from ROS 2 bag recordings captured on physical hardware. All other nodes run unmodified.

| Hardware node | Mock replacement |
|---|---|
| `fcc_bridge_node` (uXRCE-DDS) | `mock_fcc_node` — publishes `VehicleState`, `IMU`, `Odometry` from bag; accepts trajectory setpoints |
| `camera_node` (CSI) | `mock_camera_node` — publishes `Image` + `CameraInfo` from bag at ≥15 FPS |
| Gimbal IP camera (RTSP/H.265) | `mock_gimbal_node` — local RTSP test server (e.g. `rtsp-simple-server`) looping a bag-derived clip at `rtsp://127.0.0.1:8554/stream1`, replacing the physical 192.168.10.200 camera for `perception_front_node` tests |
| Swarm UDP peers | `mock_peer_node` — sends/receives swarm protocol UDP packets on loopback |
| GCS MAVLink | `mock_gcs_node` — sends/receives MAVLink 2 on UDP loopback |

---

## 3. Test Environment

### 3.1 Unit and Node Integration Test Host

| Component | Specification |
|---|---|
| Host OS | Ubuntu 22.04 aarch64 (native on DUT) or x86_64 via cross-build CI |
| ROS 2 | Humble |
| Test framework | `gtest` (TP-UT) · `launch_testing` + `pytest` (TP-NI) |
| Build | `colcon test` (invokes both gtest and launch tests) |

### 3.2 SITL Environment

| Component | Specification |
|---|---|
| SITL host | Ubuntu 22.04 x86_64 desktop (separate from DUT) |
| PX4 version | v1.14 (matching production firmware) |
| Simulator | Gazebo Garden or Ignition |
| MC software | Runs on DUT or SITL host via Docker; `SIMULATION_MODE=true` |
| Swarm peers | 2–3 additional MC Docker instances on same host, UDP loopback or LAN |
| GCS | QGroundControl on SITL host |

Drone instances communicate over `127.0.0.x` (instance 1 = `.11`, instance 2 = `.12`, instance 3 = `.13`) with the fixed IP scheme replicated in loopback.

### 3.3 HIL Environment

| Component | Specification |
|---|---|
| DUT | Jetson Orin NX 16GB on Holybro Pixhawk Jetson Baseboard |
| FCC | Pixhawk 6X, PX4 v1.14, connected via ttyTHS1 |
| Camera | IMX219 CSI-2 at sensor-id 0 |
| Network | GCS laptop at 192.168.10.10, FCC at 192.168.10.31, Jetson at 192.168.10.11 |
| GCS | QGroundControl on GCS laptop |
| Power | Bench power supply (simulate battery states by injecting `battery_status` mock) |

### 3.4 Test Data

| Dataset | Contents | Used in |
|---|---|---|
| `bags/fcc_nominal.bag` | 5-min flight: all `/fmu/out/*` topics at nominal rates | TP-NI mock_fcc_node |
| `bags/camera_outdoor.bag` | 3-min outdoor flight: `/camera/image_raw` at 15 FPS | TP-NI mock_camera_node, TP-NI detection tests |
| `bags/gnss_lost.bag` | 2-min sequence: GNSS fix → loss → recovery | TP-NI VO handover test |
| `test_data/search_polygon_500m.json` | 500 m × 500 m convex search polygon | TP-UT boustrophedon, TP-ST SEARCH phase |
| `test_data/mission_upload_3drone.yaml` | Full mission upload: ingress wpt, muster, formation offsets, search polygon, 3 peers | TP-ST full mission test |
| `test_data/detection_ground_truth.json` | Known target positions and expected geolocation output (within tolerance) | TP-NI detection geolocation |
| `bags/gimbal_outdoor.mp4` | 3-min RTSP/H.265 clip, front gimbal view: 1920×1080 @ 30 FPS, looped by `mock_gimbal_node` | TP-NI perception_front_node, TP-NI video_mux_node |

---

## 4. Unit Tests — TP-UT

Unit tests are implemented in `test/` within each package, using `gtest`. Run with `colcon test --packages-select <pkg>`.

---

### TP-UT-001: ISR Election Scoring — Correct Winner

**SRS ref:** FR-031  
**SDD ref:** §6.2  
**Package:** `swarm_coord`

**Description:** Verify that the ISR scoring function selects the correct drones given known inputs.

**Test inputs:**

| Drone | battery_pct | dist_to_centroid_m | sensor_flags |
|---|---|---|---|
| 1 | 80 | 45 | CAMERA_OK \| AI_OK |
| 2 | 65 | 30 | CAMERA_OK \| AI_OK |
| 3 | 90 | 60 | CAMERA_OK only |

**Expected scores** (w_bat=0.4, w_prox=0.4, w_sens=0.2):

| Drone | Expected score |
|---|---|
| 1 | 0.4×0.80 + 0.4×(1/45) + 0.2×1.0 = 0.529 |
| 2 | 0.4×0.65 + 0.4×(1/30) + 0.2×1.0 = 0.473 |
| 3 | 0.4×0.90 + 0.4×(1/60) + 0.2×0.0 = 0.367 |

**Expected result:** With `isr_count = ceil(3/2) = 2`, elected set = {Drone 1, Drone 2}.

**Pass criteria:**
- `score(d1) > score(d2) > score(d3)` (within float tolerance 1e-4)
- `elect(candidates, n=3) == {1, 2}`

---

### TP-UT-002: ISR Tie-Breaking — Lowest ID Wins

**SRS ref:** FR-031  
**Package:** `swarm_coord`

**Description:** When two drones have equal scores, the one with the lower `drone_id` is elected.

**Test inputs:** Drone A (id=3) and Drone B (id=1) with identical battery, distance, and sensor values.

**Expected result:** Elected set includes Drone B (id=1) before Drone A (id=3).

**Pass criteria:** `elect(candidates)[0].id == 1`

---

### TP-UT-003: Boustrophedon Path — Row Count

**SRS ref:** FR-041, PR-008  
**SDD ref:** §7.1  
**Package:** `mission_exec`

**Description:** For a 500 m × 500 m square cell with `footprint_width=20 m`, `overlap=0.2`, verify the row count is correct.

**Calculation:** `row_spacing = 20 × 0.8 = 16 m`. Rows spanning 500 m: `ceil(500 / 16) = 32` rows.

**Pass criteria:**
- `path.rows.size() == 32` (± 1 for boundary clipping)
- Rows are ordered in alternating direction (row[0] left→right, row[1] right→left, …)
- All waypoint coordinates lie within the input cell polygon

---

### TP-UT-004: Boustrophedon Path — Nearest Start Waypoint

**SRS ref:** FR-041  
**Package:** `mission_exec`

**Description:** Starting waypoint selected is the path waypoint nearest to a given current position.

**Test inputs:** 32-row path over a 500 m × 500 m cell; drone current position at the midpoint of the cell.

**Pass criteria:** `path.waypoints[0]` is the waypoint nearest to the current position (Euclidean distance in ENU < any other waypoint in the path).

---

### TP-UT-005: Boustrophedon Path — Computation Time

**SRS ref:** PR-008  
**Package:** `mission_exec`

**Description:** Coverage path generation completes within 5 s for a cell with up to 20 vertices.

**Pass criteria:** `elapsed_ms < 5000` across 10 repeated runs on the target platform.

---

### TP-UT-006: Voronoi Partition — 3-Site Cell Boundary

**SRS ref:** FR-035  
**SDD ref:** §6.3  
**Package:** `swarm_coord`

**Description:** For 3 known drone positions within a convex polygon, verify that the Voronoi cells partition the polygon without gaps or overlaps.

**Test inputs:** 3 sites at known ENU coordinates; convex 6-vertex polygon (500 m × 500 m rectangle with chamfered corners).

**Pass criteria:**
- `sum(cell_areas) ≈ polygon_area` (within 0.1% tolerance)
- No cell has zero area
- All cell vertex coordinates lie within or on the input polygon boundary

---

### TP-UT-007: Voronoi Consensus — Mismatch Detection

**SRS ref:** FR-035  
**Package:** `swarm_coord`

**Description:** The consensus check rejects a `VORONOI_ACCEPT` message whose cell boundary vertices do not match the locally computed result.

**Test inputs:** Locally computed cell with 8 vertices; incoming `VORONOI_ACCEPT` with one vertex displaced by 2 m (> 1 cm tolerance).

**Pass criteria:** `consensus_check(local_cell, received_cell) == MISMATCH`.

---

### TP-UT-008: Formation Controller — PD Output

**SRS ref:** FR-021  
**SDD ref:** §5.2  
**Package:** `swarm_coord`

**Description:** Verify the PD controller produces correct velocity setpoints for a known position and velocity error.

**Test inputs:**
- Leader position: (100, 50, -30) NED, velocity: (2, 0, 0)
- Own position: (95, 50, -30), velocity: (1.5, 0, 0)
- Formation offset: (−10, 0, 0) (desired: 10 m behind leader)
- Kp_xy=0.5, Kd_xy=0.2

**Expected:**
- `e_pos = (100-10, 50, -30) - (95, 50, -30) = (-5, 0, 0)`
- `e_vel = (2, 0, 0) - (1.5, 0, 0) = (0.5, 0, 0)`
- `vx = 0.5×(-5) + 0.2×0.5 = -2.4` m/s

**Pass criteria:** `|setpoint.velocity[0] - (-2.4)| < 0.001`

---

### TP-UT-009: Formation Controller — Velocity Saturation

**SRS ref:** FR-021  
**Package:** `swarm_coord`

**Description:** Output velocity is clamped to `max_vel_xy` when the error is large.

**Test inputs:** Position error of 100 m, Kp_xy=0.5 → unclamped output 50 m/s. `max_vel_xy=5.0 m/s`.

**Pass criteria:** `setpoint.velocity[0] ≤ 5.0`

---

### TP-UT-010: Mission Manager — Invalid Transition Rejected

**SRS ref:** FR-001  
**SDD ref:** §4.1  
**Package:** `mission_exec`

**Description:** Attempting a phase transition not in the transition table returns an error and does not change the current phase.

**Test inputs:** Current phase = `PREFLIGHT`; attempt transition to `SEARCH`.

**Pass criteria:**
- `transition_result == TRANSITION_REJECTED`
- `current_phase == PREFLIGHT` (unchanged)

---

### TP-UT-011: Mission Manager — Pre-Arm Check Fails on Low Battery

**SRS ref:** FR-001, SFR-005  
**Package:** `mission_exec`

**Description:** Pre-arm checks fail when battery remaining is below `LOW_BATTERY_THRESHOLD + 10%` margin (threshold=20%, so fail if < 30%).

**Test inputs:** `battery_remaining = 0.28` (28%).

**Pass criteria:** `pre_arm_checks() == false`; reason includes "battery".

---

### TP-UT-012: Mission Manager — GCS Emergency Command Bypasses Guards

**SRS ref:** FR-003  
**SDD ref:** §4.3  
**Package:** `mission_exec`

**Description:** A GCS `DISARM` command is executed immediately regardless of the current phase or transition table.

**Test inputs:** Current phase = `SEARCH`; GCS command = `DISARM`.

**Pass criteria:**
- FCC `vehicle_command` DISARM is issued
- Phase transitions to `FAULT`

---

### TP-UT-013: Detection Geolocation — Flat-Terrain Ray Cast

**SRS ref:** FR-043  
**SDD ref:** §9.3  
**Package:** `vision`

**Description:** Verify the geolocation algorithm produces the correct world position for a known camera pose and pixel coordinate.

**Test inputs (synthetic, all in ENU frame):**
- Camera at NED (0, 0, −50) (50 m altitude), pointing nadir, no roll/pitch
- Camera intrinsics: fx=fy=800, cx=640, cy=360 (1280×720)
- Detection bounding box centre: (640, 360) (image centre)
- Terrain altitude: 0 m MSL

**Expected:** Target at NED (0, 0, 0) — directly below the camera.

**Pass criteria:** `|target_ned[0]| < 0.01` and `|target_ned[1]| < 0.01`

---

### TP-UT-014: Detection Geolocation — Off-Axis Target

**SRS ref:** FR-043  
**Package:** `vision`

**Description:** Target at known angular offset from nadir produces correct horizontal displacement.

**Test inputs:** Camera at altitude 50 m, pixel (800, 360) — 160 px right of centre.
- Angular offset: `atan((800-640)/800)` ≈ 11.3°
- Expected ground offset: `50 × tan(11.3°)` ≈ 10.0 m East

**Pass criteria:** `|target_east_m - 10.0| < 0.5 m`

---

### TP-UT-015: Health Monitor — FCC Link Timeout

**SRS ref:** FR-015, SFR-001  
**Package:** `health_monitor`

**Description:** `is_link_alive()` returns false when the last FCC message is older than the timeout.

**Test inputs:** `last_fcc_msg_time` set to `now() - 1.5 s`; `timeout_s = 1.0`.

**Pass criteria:** `backend.is_link_alive(1.0) == false`

---

### TP-UT-016: Health Monitor — Battery RTL Threshold

**SRS ref:** FR-072, SFR-008  
**Package:** `health_monitor`

**Description:** The failsafe decision tree triggers RTL when `battery_remaining < 0.20`.

**Test inputs:** `vehicle_state.battery_remaining = 0.19`; no GCS override active.

**Pass criteria:**
- `fault_mask & LOW_BATTERY != 0`
- RTL command issued to `fcc_bridge_node`

---

### TP-UT-017: Health Monitor — Battery RTL Suppressed by Override

**SRS ref:** SFR-008  
**Package:** `health_monitor`

**Description:** Low-battery RTL is suppressed when the GCS override is active **and** GCS link is healthy.

**Test inputs:** `battery_remaining = 0.19`; GCS override `OVERRIDE_LOW_BATTERY_RTL` active; `gcs_link_ok = true`.

**Pass criteria:** RTL command is **not** issued.

---

### TP-UT-018: VO Scale Recovery Computation

**SRS ref:** FR-051  
**SDD ref:** §8.2  
**Package:** `vision`

**Description:** Scale factor computed from IMU and VO displacement magnitudes is correct.

**Test inputs:** `‖Δp_imu‖ = 2.0 m`; `‖Δp_vo‖ = 0.5` (unit VO output); `previous_scale = 1.0`.

**Expected:** `raw_scale = 2.0 / 0.5 = 4.0`; smoothed: `scale = 0.9×1.0 + 0.1×4.0 = 1.3`.

**Pass criteria:** `|scale_factor - 1.3| < 0.001`

---

### TP-UT-019: perception_front_node — IoU Tracker Continuity

**SRS ref:** FR-081  
**SDD ref:** §9a.1  
**Package:** `video_uplink`

**Description:** Verify the IoU-based greedy tracker maintains a stable `track_id` across frames for a slowly-moving detection, and drops a track after `track_miss_limit` consecutive misses.

**Test inputs:** A synthetic 25-frame detection sequence: frames 1–10 contain a bounding box drifting 2 px/frame (IoU vs. previous frame ≥ 0.3 throughout); frames 11–25 contain no detection in that region (15 consecutive misses, exceeding `track_miss_limit=10`).

**Pass criteria:**
- `track_id` is identical across frames 1–10; `frames_tracked == 10` at frame 10.
- The track is still present (not yet dropped) at frame 20 (10th miss).
- The track is dropped (absent from output) by frame 21 (11th consecutive miss).

---

## 5. Node Integration Tests — TP-NI

Node integration tests use `launch_testing` and `rclpy`. Each test launches the node under test plus the relevant mock nodes, then asserts on topic traffic and message content. Run with `colcon test --packages-select <pkg>`.

Default timeout per test: 30 s unless specified.

---

### TP-NI-001: fcc_bridge_node — VehicleState Publication Rate

**SRS ref:** IR-003  
**Package:** `fcc_bridge`

**Preconditions:** `mock_fcc_node` publishing all `/fmu/out/*` topics from `bags/fcc_nominal.bag`.

**Procedure:**
1. Launch `fcc_bridge_node` + `mock_fcc_node`.
2. Subscribe to `/mc/fcc/vehicle_state` for 5 s.
3. Count messages received.

**Pass criteria:** `msg_count / 5.0 ≥ 10.0 Hz`

---

### TP-NI-002: fcc_bridge_node — VehicleState Content Accuracy

**SRS ref:** IR-003  
**Package:** `fcc_bridge`

**Preconditions:** `mock_fcc_node` publishing a known `vehicle_local_position` message with `x=10.0, y=20.0, z=-30.0`.

**Procedure:**
1. Receive one `/mc/fcc/vehicle_state` message.
2. Compare fields.

**Pass criteria:**
- `|vehicle_state.x - 10.0| < 0.001`
- `|vehicle_state.y - 20.0| < 0.001`
- `|vehicle_state.z - (-30.0)| < 0.001`

---

### TP-NI-003: fcc_bridge_node — FCC Link Loss Fault

**SRS ref:** FR-015, SFR-001  
**Package:** `fcc_bridge`

**Preconditions:** `mock_fcc_node` publishing normally; `/mc/fault` subscriber active.

**Procedure:**
1. Stop `mock_fcc_node` (halt topic publication).
2. Wait 1.5 s.
3. Check `/mc/fault` for `FCC_LINK_LOST`.

**Pass criteria:** `FCC_LINK_LOST` fault event published on `/mc/fault` within 1.5 s of link silence.

---

### TP-NI-004: formation_controller_node — Mission Item Update Rate

**SRS ref:** FR-021, IR-004  
**Package:** `swarm_coord`

**Preconditions:** `mock_fcc_node` publishing VehicleState; `mock_peer_node` publishing a valid leader `PEER_STATE` at 5 Hz.

**Procedure:**
1. Launch `formation_controller_node`.
2. Set mission phase to `INGRESS`.
3. Count `/mc/fcc/mission_item_update` messages over 5 s.

**Pass criteria:** `msg_count / 5.0 ≥ 5.0 Hz`

---

### TP-NI-005: formation_controller_node — Leader Loss → HOLD

**SRS ref:** FR-024  
**Package:** `swarm_coord`

**Preconditions:** `formation_controller_node` running in INGRESS with a valid leader state.

**Procedure:**
1. Stop `mock_peer_node` (halt `PEER_STATE` publication).
2. Wait 2.5 s.
3. Check `/mc/fault` for `LEADER_LOST`.
4. Check that a `vehicle_command HOLD` was sent to `fcc_bridge_node`.

**Pass criteria:**
- `LEADER_LOST` fault published within 2.5 s
- `HOLD` command forwarded to mock FCC

---

### TP-NI-006: swarm_coordinator_node — Peer LOST Declaration

**SRS ref:** IR-023  
**Package:** `swarm_coord`

**Preconditions:** `swarm_coordinator_node` with a 3-drone manifest; `mock_peer_node` sending heartbeats for Drone 2 and Drone 3.

**Procedure:**
1. Stop Drone 2 heartbeats.
2. Wait 3.5 s.
3. Check `/mc/swarm/peer_states`.

**Pass criteria:**
- `peer_states.peers[1].state == LOST` (Drone 2 declared LOST)
- `peer_states.peers[2].state == ALIVE` (Drone 3 unaffected)

---

### TP-NI-007: swarm_coordinator_node — Peer Recovery

**SRS ref:** IR-023  
**Package:** `swarm_coord`

**Preconditions:** Drone 2 already in LOST state (from TP-NI-006 or equivalent setup).

**Procedure:**
1. Resume Drone 2 heartbeats.
2. Wait 1.5 s.
3. Check `/mc/swarm/peer_states`.

**Pass criteria:** `peer_states.peers[1].state == ALIVE`

---

### TP-NI-008: search_planner_node — Path Generated Within 5 s

**SRS ref:** PR-008  
**Package:** `mission_exec`

**Preconditions:** `search_planner_node` running; mission parameters loaded.

**Procedure:**
1. Publish a `VoronoiCell` message on `/mc/swarm/voronoi_cell` (500 m × 500 m polygon).
2. Record time until first `/mc/search/coverage_path` message received.

**Pass criteria:** `elapsed_s < 5.0`

---

### TP-NI-009: search_planner_node — Path Coverage

**SRS ref:** FR-041  
**Package:** `mission_exec`

**Preconditions:** Same as TP-NI-008.

**Procedure:**
1. Receive coverage path from `/mc/search/coverage_path`.
2. Project all waypoints onto the cell polygon.
3. Compute covered area using sweep band simulation.

**Pass criteria:**
- All waypoints lie within or on cell boundary
- Covered area ≥ 95% of cell area (accounting for edge effects)
- Adjacent row lateral separation ≤ `footprint_width_m` (no uncovered gap)

---

### TP-NI-010: mission_manager_node — GCS Mission Upload Processing

**SRS ref:** IR-010, IR-011  
**Package:** `mission_exec`

**Preconditions:** `gcs_mavlink_node` running; `mock_gcs_node` ready.

**Procedure:**
1. `mock_gcs_node` sends a complete mission upload sequence (MISSION_COUNT → MISSION_ITEM_INT × N → expects MISSION_ACK).
2. Subscribe to `/mc/gcs/mission_upload`.

**Pass criteria:**
- `MISSION_ACK(MAV_MISSION_ACCEPTED)` received by mock GCS
- `/mc/gcs/mission_upload` published with correct waypoint count, formation offsets, and peer manifest

---

### TP-NI-011: mission_manager_node — Heartbeat Continuity

**SRS ref:** FR-074  
**Package:** `mission_exec`

**Preconditions:** `mission_manager_node` running in PREFLIGHT.

**Procedure:**
1. Subscribe to `/mc/manager/heartbeat` for 10 s.
2. Record all timestamps.
3. Compute maximum inter-message gap.

**Pass criteria:** `max_gap_s ≤ 1.0` (heartbeat rate ≥ 1 Hz continuously)

---

### TP-NI-012: health_monitor_node — Manager Heartbeat Loss Detection

**SRS ref:** FR-074, SFR-003  
**Package:** `health_monitor`

**Preconditions:** `health_monitor_node` running; `mock_manager_node` publishing `/mc/manager/heartbeat` at 1 Hz.

**Procedure:**
1. Stop `mock_manager_node`.
2. Wait 5.5 s.
3. Check `/mc/health` for `MANAGER_UNRESPONSIVE` in `fault_mask`.

**Pass criteria:**
- `fault_mask & MANAGER_UNRESPONSIVE != 0` within 5.5 s
- Systemd restart request sent to watchdog socket (mock socket records the request)

---

### TP-NI-013: perception_down_node — Inference Latency

**SRS ref:** PR-007  
**Package:** `vision`

**Preconditions:** `mock_camera_node` publishing `/camera/image_raw` at 15 FPS from bag. TensorRT engine pre-built.

**Procedure:**
1. Record timestamp on each incoming frame.
2. Record timestamp on each `/mc/detections` message.
3. Compute per-frame latency: `t_detection - t_frame`.
4. Run for 60 s (≥900 frames).

**Pass criteria:** `p99(latency_ms) ≤ 500` (99th-percentile detection latency ≤ 500 ms)

---

### TP-NI-014: gcs_mavlink_node — MC_STATUS Encoding

**SRS ref:** IR-011, IR-014  
**Package:** `gcs_comms`

**Preconditions:** `gcs_mavlink_node` running; `mock_gcs_node` listening on UDP 14552.

**Procedure:**
1. Publish a known `/mc/fcc/vehicle_state` (lat=1.2345678°, lon=103.8765432°, alt=50.0 m, battery=75%).
2. Receive `MC_STATUS (50100)` at `mock_gcs_node`.
3. Decode fields.

**Pass criteria:**
- `|decoded_lat_deg - 1.2345678| < 1e-5°`
- `|decoded_lon_deg - 103.8765432| < 1e-5°`
- `|decoded_alt_m - 50.0| < 0.1 m`
- `|decoded_battery_pct - 75| < 1`

---

### TP-NI-015: gcs_mavlink_node — COMMAND_ACK Timeliness

**SRS ref:** IR-013  
**Package:** `gcs_comms`

**Preconditions:** `gcs_mavlink_node` + `mock_gcs_node`.

**Procedure:**
1. `mock_gcs_node` sends `COMMAND_LONG MC_CMD_PHASE_TRANSITION`.
2. Record time until `COMMAND_ACK` received.

**Pass criteria:** `ack_latency_ms ≤ 500`

---

### TP-NI-016: visual_localisation_node — GNSS Handover

**SRS ref:** FR-051, FR-052  
**Package:** `vision`

**Preconditions:** `mock_fcc_node` playing `bags/gnss_lost.bag` (contains GNSS fix → loss → recovery sequence).

**Procedure:**
1. Monitor `/mc/localisation_mode` messages.
2. Correlate mode changes with GNSS quality events in the bag.

**Pass criteria:**
- Mode changes `GNSS_PRIMARY → VISUAL_ODOMETRY` within 3 bag messages of GNSS quality drop
- Mode changes `VISUAL_ODOMETRY → GNSS_PRIMARY` within 3 bag messages of GNSS recovery (with 3-reading hysteresis)
- `/mc/local_pose` is published continuously through both transitions (no gap > 200 ms)

---

### TP-NI-017: perception_front_node — Detection/Track Publication Rate

**SRS ref:** FR-081, IR-040  
**Package:** `video_uplink`

**Preconditions:** `mock_gimbal_node` serving `bags/gimbal_outdoor.mp4` over RTSP at `rtsp://127.0.0.1:8554/stream1`. TensorRT engine pre-built.

**Procedure:**
1. Launch `perception_front_node` pointed at the mock RTSP URL.
2. Subscribe to `/mc/perception/front/tracks` for 10 s.
3. Count messages.

**Pass criteria:** `msg_count / 10.0 ≥ 5.0 Hz`

---

### TP-NI-018: perception_front_node — Link Loss Detected, No Flight Fault

**SRS ref:** FR-084, IR-040  
**Package:** `video_uplink`

**Preconditions:** `perception_front_node` running against `mock_gimbal_node`; `/mc/fault` subscriber active.

**Procedure:**
1. Stop `mock_gimbal_node` (halt the RTSP stream).
2. Wait 3.5 s.
3. Check the node's liveness heartbeat/alive topic for silence.
4. Check `/mc/fault` for the absence of any flight-affecting fault (`FCC_LINK_LOST`, `GCS_LINK_LOST`, etc.).

**Pass criteria:**
- `perception_front_node` liveness silence detected within 3.5 s (feeds `health_monitor_node`'s `GIMBAL_LINK_LOST`/`GIMBAL_OK` bits, HM-002/HM-003, not tested here)
- `/mc/fault` contains **no** flight-failsafe event as a result of this link loss (FR-084)

---

### TP-NI-019: video_mux_node — Metadata Mux Accuracy

**SRS ref:** IR-041  
**Package:** `video_uplink`

**Preconditions:** `perception_front_node` publishing known synthetic detections/tracks (fixed `track_id`, `class_id`, `confidence`, `lat`/`lon`/`alt`); `video_mux_node` running; a mock RTSP client connected.

**Procedure:**
1. Publish one synthetic `Track` with known field values.
2. Capture the corresponding muxed frame's metadata track at the mock RTSP client.
3. Decode and compare fields.

**Pass criteria:**
- Decoded `track_id`, `class_id` match exactly
- `|decoded_confidence - sent_confidence| < 1e-4` (×10000 fixed-point)
- `|decoded_lat_deg - sent_lat_deg| < 1e-5°`, same for `lon`

---

### TP-NI-020: video_mux_node — Source Silence → streaming=false

**SRS ref:** FR-083, IR-042  
**Package:** `video_uplink`

**Preconditions:** `video_mux_node` running with `perception_front_node` publishing normally; a mock RTSP client connected (`streaming=true`).

**Procedure:**
1. Stop `perception_front_node` (halt frame/track publication).
2. Wait 3.5 s.
3. Subscribe to `/mc/video/uplink_status`.

**Pass criteria:**
- `streaming == false` within 3.5 s of source silence
- No fault raised on `/mc/fault` (perception-only degradation, not a flight concern)
- Resuming `perception_front_node` publication restores `streaming == true` within 1 s, without restarting `video_mux_node`

---

### TP-NI-021: video_mux_node — Downward Stream (Stream B) Metadata Correctness

**SRS ref:** FR-085, IR-043  
**Package:** `video_uplink`

**Preconditions:** `camera_node` publishing synthetic `/camera/image_raw` frames; `perception_down_node` publishing known synthetic `Detection`s (fixed `class_id`, `confidence`, `lat`/`lon`/`alt`, no `track_id` — this pipeline has no tracker); `video_mux_node` running both streams; a mock RTSP client connected to `:8556`.

**Procedure:**
1. Publish one synthetic `Detection` with known field values on `/mc/detections`.
2. Capture the corresponding muxed frame's metadata track at the `:8556` mock RTSP client.
3. Decode and compare fields.

**Pass criteria:**
- Decoded `class_id` matches exactly, no `track_id` field present (ICD §8.2a)
- `|decoded_confidence - sent_confidence| < 1e-4`
- `|decoded_lat_deg - sent_lat_deg| < 1e-5°`, same for `lon`
- Stream A (`:8555`) metadata is unaffected by Stream B traffic (independent pipelines, FR-086)

---

### TP-NI-022: video_mux_node — Downward Stream (Stream B) Source Silence → Independent of Stream A

**SRS ref:** FR-086, IR-043  
**Package:** `video_uplink`

**Preconditions:** `video_mux_node` running both streams, both `streaming=true`; a mock RTSP client connected to each.

**Procedure:**
1. Stop `camera_node`/`perception_down_node` (halt Stream B's frame/detection source) while Stream A keeps publishing normally.
2. Wait 3.5 s.
3. Subscribe to `/mc/video/down_uplink_status` and `/mc/video/uplink_status`.

**Pass criteria:**
- `/mc/video/down_uplink_status.streaming == false` within 3.5 s of source silence
- `/mc/video/uplink_status.streaming` (Stream A) remains `true`, unaffected — confirms the two pipelines are independent failure domains (FR-086)
- No fault raised on `/mc/fault` (perception-only degradation)
- Resuming Stream B's source restores `streaming == true` within 1 s, without restarting `video_mux_node` or affecting Stream A

---

## 6. SITL System Tests — TP-ST

SITL tests run a full MC stack against PX4 SITL (Gazebo). Each test is a `pytest` script that controls the GCS via a MAVLink connection and asserts on ROS 2 topic traffic.

**Setup for all TP-ST tests:**
1. Start PX4 SITL instance(s) with the Gazebo quadrotor model.
2. Start Micro XRCE-DDS Agent connected to SITL UDP port.
3. Start `drone-mc.target` with `SIMULATION_MODE=true`.
4. Wait for all services to reach `active (running)` state.

---

### TP-ST-001: Full Nominal Mission — 2 Drones, All Phases

**SRS ref:** FR-001–005, FR-020–036, FR-040–043  
**Timeout:** 20 min

**Preconditions:** 2 MC instances + 2 PX4 SITL instances; GCS `mock_gcs_node` with a complete `test_data/mission_upload_3drone.yaml` (trimmed to 2 drones).

**Procedure:**
1. Send mission upload to both MC instances via GCS.
2. Send `MC_CMD_ARM_AUTHORISE` → monitor ARMING phase.
3. Both FCC arm → TAKEOFF initiated → verify FCC at target altitude.
4. Send `INGRESS` command → verify both drones move toward muster point in formation.
5. Both reach muster → MUSTER phase → verify peer states show MUSTER.
6. ISR_SELECTION proceeds autonomously → verify ISR_RESULT broadcast.
7. VORONOI_PARTITION → verify `/mc/swarm/voronoi_cell` published on both instances.
8. SEARCH → verify coverage path uploaded to both FCCs → both enter AUTO:MISSION.
9. Send `MC_CMD_END_MISSION` → RETURN → LAND → verify disarmed.

**Pass criteria:**
- All phase transitions complete in sequence without manual intervention (after initial GCS commands in steps 2–4)
- ISR election declares `isr_count = 1` of 2 drones (top scorer)
- Voronoi cells are complementary (together cover the search polygon without overlap)
- Both drones complete LAND within 20 min total

---

### TP-ST-002: GCS Link Loss During SEARCH → HOLD → Recovery

**SRS ref:** FR-073, SFR-007  
**Timeout:** 5 min

**Preconditions:** Both MC instances in SEARCH phase (reached from TP-ST-001 or equivalent setup shortcut).

**Procedure:**
1. Block GCS MAVLink traffic on both MC instances (`iptables` drop on UDP 14551/14552).
2. Wait 12 s.
3. Check that both FCC instances enter HOLD (via FCC nav_state).
4. Restore GCS MAVLink traffic.
5. Send `MC_CMD_PHASE_TRANSITION SEARCH` via GCS.
6. Verify both drones resume AUTO:MISSION.

**Pass criteria:**
- Both drones enter HOLD within 12 s of GCS link drop (≤ 10 s threshold + network latency)
- `fault_mask & GCS_TIMEOUT != 0` on both instances during hold
- Drones resume SEARCH after GCS command (not automatically)

---

### TP-ST-003: Low Battery RTL During SEARCH

**SRS ref:** FR-072, SFR-008  
**Timeout:** 3 min

**Preconditions:** One MC instance in SEARCH phase.

**Procedure:**
1. Inject a `battery_status` mock message with `remaining = 0.18` into the FCC stream.
2. Monitor FCC nav_state and MC `/mc/health`.

**Pass criteria:**
- `fault_mask & LOW_BATTERY != 0` within 2 s of injection
- FCC enters RTL within 3 s of fault detection
- `STATUSTEXT` severity WARN received by GCS with "RTL" in text

---

### TP-ST-004: Manager Crash and Restart During SEARCH

**SRS ref:** FR-074, FR-075, SFR-003, SFR-004  
**Timeout:** 3 min

**Preconditions:** MC instance in SEARCH phase.

**Procedure:**
1. Kill `mission-exec.service` container: `docker kill mc-mission-exec`.
2. Monitor `/mc/manager/heartbeat` (should stop).
3. Wait 5.5 s for watchdog to detect silence.
4. Verify systemd restarts `mission-exec.service`.
5. Monitor `/mc/manager/heartbeat` resumes.
6. Verify FCC held in HOLD during restart.
7. Verify GCS receives `STATUSTEXT WARN "manager restarting"`.

**Pass criteria:**
- Heartbeat resumes within 15 s of kill (5 s watchdog + restart time)
- FCC does not leave HOLD during restart window
- `STATUSTEXT` received by GCS

**Extension — 3× failure (manual):** Kill manager 3 times without allowing recovery. Verify:
- After 3rd failed restart, RTL commanded via `fcc_bridge_node`
- `fault_mask & MANAGER_DEAD != 0`
- `STATUSTEXT CRITICAL` received

---

### TP-ST-005: Peer Drone Loss During SEARCH → Re-partition

**SRS ref:** FR-036  
**Timeout:** 5 min

**Preconditions:** 2 MC instances in SEARCH phase; each covering a Voronoi cell.

**Procedure:**
1. Terminate all services on Drone 2 MC instance (`systemctl stop drone-mc.target`).
2. Wait 3.5 s for Drone 1 to declare Drone 2 LOST.
3. Monitor Drone 1's `/mc/mission_phase` (should transition to VORONOI_PARTITION then back to SEARCH).
4. Monitor `/mc/swarm/voronoi_cell` — new cell should cover the full polygon.
5. Monitor `/mc/search/coverage_path` — new path uploaded to FCC.

**Pass criteria:**
- Drone 2 declared LOST within 3.5 s
- Drone 1 re-partitions and generates a new path within 10 s of LOST declaration
- New Voronoi cell area ≈ full polygon area (within 5%)
- Drone 1 resumes AUTO:MISSION with new path

---

### TP-ST-006: FCC Communication Loss — FAULT Declaration

**SRS ref:** FR-015, SFR-001  
**Timeout:** 2 min

**Preconditions:** MC instance in SEARCH phase.

**Procedure:**
1. Stop Micro XRCE-DDS Agent: `docker kill mc-fcc-bridge`.
2. Wait 1.5 s.
3. Check `/mc/fault` for `FCC_LINK_LOST`.
4. Check GCS for `STATUSTEXT CRITICAL`.

**Pass criteria:**
- `FCC_LINK_LOST` fault published within 1.5 s
- Phase transitions to `FAULT`
- GCS notified via `STATUSTEXT`

---

### TP-ST-007: Mission Upload via GCS — All Item Types Decoded

**SRS ref:** FR-062, IR-010, IR-011  
**Timeout:** 2 min

**Preconditions:** MC instance in PREFLIGHT; GCS connected.

**Procedure:**
1. Send a complete mission upload including: 1 ingress waypoint, 1 muster point, 2 formation offset entries, 5 search polygon vertices, 2 peer entries.
2. Subscribe to `/mc/gcs/mission_upload`.

**Pass criteria:**
- `MISSION_ACK(ACCEPTED)` sent by MC
- `/mc/gcs/mission_upload` contains: ingress waypoint matching sent coordinates (±1 m), formation offsets matching sent values (±0.01 m), polygon with 5 vertices, peer manifest with 2 entries including correct IPs

---

### TP-ST-008: Detection Reporting to GCS

**SRS ref:** FR-042, FR-043, IR-012  
**Timeout:** 5 min

**Preconditions:** MC instance in SEARCH phase; `mock_camera_node` publishing `bags/camera_outdoor.bag` which contains a frame with a known target at a known position.

**Procedure:**
1. Monitor GCS (mock_gcs_node) for `MC_DETECTION (50101)` messages.
2. Compare reported `lat_e7`, `lon_e7` against ground-truth position from `test_data/detection_ground_truth.json`.

**Pass criteria:**
- At least one `MC_DETECTION` received within 1 s of the target frame in the bag
- `position_error_m ≤ 15 m` (accounting for 50 m altitude, pixel FOV, and VO drift)

---

### TP-ST-009: Visual Odometry Mode — Position Continuity

**SRS ref:** FR-051, FR-053  
**Timeout:** 5 min

**Preconditions:** MC instance in SEARCH phase; `mock_fcc_node` playing `bags/gnss_lost.bag`.

**Procedure:**
1. Monitor `/mc/localisation_mode` — wait for transition to `VISUAL_ODOMETRY`.
2. Record `/mc/local_pose` for 60 s in VO mode.
3. At bag end (GNSS recovery), record final VO position vs reported GNSS position.

**Pass criteria:**
- `/mc/local_pose` published continuously during VO mode (no gap > 200 ms)
- Drift at end of 60 s VO segment ≤ 5 m from GNSS reference position

---

### TP-ST-010: ISR Election — 3-Drone Coordinator Failover

**SRS ref:** FR-031, FR-032  
**Timeout:** 3 min

**Preconditions:** 3 MC instances in MUSTER → entering ISR_SELECTION. Drone 1 is coordinator.

**Procedure:**
1. Kill Drone 1 MC immediately after ISR_SELECTION phase is entered.
2. Wait 8 s (coordinator failover timeout).
3. Observe which drone broadcasts `ISR_RESULT`.

**Pass criteria:**
- Drone 2 (next-lowest ID) broadcasts `ISR_RESULT` within 8 s of Drone 1 going LOST
- ISR election completes correctly with 2-drone ALIVE set
- Phase proceeds to VORONOI_PARTITION

---

### TP-ST-011: MC_STATUS Telemetry Rate to GCS

**SRS ref:** FR-063, IR-014  
**Timeout:** 2 min

**Preconditions:** MC instance in any active phase; GCS `mock_gcs_node` listening.

**Procedure:**
1. Receive `MC_STATUS (50100)` messages at mock GCS for 30 s.
2. Count messages.

**Pass criteria:** `msg_count / 30.0 ≥ 2.0 Hz`

---

### TP-ST-012: System Resource Usage — Steady State

**SRS ref:** PR-009, PR-010  
**Timeout:** 10 min

**Preconditions:** MC instance in SEARCH phase; all nodes running; inference active at 15 FPS.

**Procedure:**
1. Sample CPU usage (`/proc/stat`), RAM (`/proc/meminfo`), and GPU load (tegrastats) every 10 s for 5 min.
2. Record peak and mean values.

**Pass criteria:**
- Peak CPU < 80% (PR-009 headroom)
- Peak RAM < 12 GB (of 16 GB)
- GPU load < 80% (PR-010)
- No OOM-killer events in `dmesg`

---

### TP-ST-013: Gimbal Camera Link Loss During SEARCH — No Flight Impact

**SRS ref:** FR-084  
**Timeout:** 3 min

**Preconditions:** MC instance in SEARCH phase; `perception_front_node`/`video_mux_node` running against `mock_gimbal_node` (RTSP loopback source, substituting for the physical gimbal camera in this SITL-level test).

**Procedure:**
1. Stop `mock_gimbal_node` (simulates gimbal RTSP link loss).
2. Wait 5 s.
3. Monitor `/mc/health` (`fault_mask & GIMBAL_LINK_LOST`, `sensor_flags & GIMBAL_OK` cleared).
4. Monitor FCC `nav_state` and `/mc/mission_phase` throughout.
5. Restore `mock_gimbal_node`.

**Pass criteria:**
- `GIMBAL_LINK_LOST` set in `fault_mask` within 3.5 s of link loss (FR-084)
- FCC `nav_state` and `/mc/mission_phase` are **unaffected** — no HOLD, no RTL, mission phase remains `SEARCH` throughout (this is the negative assertion the ticket exists to verify)
- `/mc/detections` (downward camera pipeline) continues publishing normally, unaffected by the gimbal link loss
- On restore, `GIMBAL_LINK_LOST` clears and `GIMBAL_OK` sets within 5 s

---

## 7. Hardware-in-the-Loop Tests — TP-HW

HIL tests run on physical Jetson Orin NX + Pixhawk 6X. The FCC is connected via ttyTHS1. No airframe is attached; the propellers are removed. All tests are conducted with the system powered by bench supply.

HIL tests require one engineer operating the GCS and one engineer observing DUT logs. Results are manually recorded on the HIL Test Record Sheet.

---

### TP-HW-001: uXRCE-DDS Link Latency

**SRS ref:** IR-006  
**Preconditions:** FCC running PX4 v1.14; Micro XRCE-DDS Agent connected on ttyTHS1 at 921600 baud.

**Procedure:**
1. Record timestamp when `vehicle_command` is published by `fcc_bridge_node`.
2. Record timestamp when FCC acknowledges mode change (observed via `vehicle_status.nav_state`).
3. Repeat 100 times.

**Pass criteria:** `p99(actuation_latency_ms) ≤ 100`

---

### TP-HW-002: Camera Pipeline — Frame Rate

**SRS ref:** IR-030, PR-006  
**Preconditions:** IMX219 CSI camera connected at sensor-id 0; `camera_node` running.

**Procedure:**
1. Subscribe to `/camera/image_raw` for 30 s.
2. Count messages; record timestamps.
3. Compute mean and minimum frame rate.

**Pass criteria:**
- `mean_fps ≥ 15.0`
- `min_fps ≥ 12.0` (transient dip allowed)
- `image.encoding == "bgr8"`
- `image.width == 1280 and image.height == 720`

---

### TP-HW-003: TensorRT Inference Latency — Sustained Load

**SRS ref:** PR-007  
**Preconditions:** `perception_down_node` (renamed from `detection_node`) running with engine pre-built; `camera_node` running at 15 FPS.

**Procedure:**
1. Run inference continuously for 10 min.
2. Record per-frame latency (frame timestamp to `/mc/detections` timestamp).
3. Record GPU temperature every 30 s.

**Pass criteria:**
- `p99(latency_ms) ≤ 500` for the full 10-min run
- No thermal throttling events (GPU temp ≤ 85 °C throughout, or if throttled, latency still ≤ 500 ms)

---

### TP-HW-004: ttyTHS1 Serial Reliability

**SRS ref:** IR-001  
**Preconditions:** FCC connected on ttyTHS1; `fcc_bridge_node` running for 30 min.

**Procedure:**
1. Count `/mc/fcc/vehicle_state` messages published over 30 min.
2. Count any `FCC_LINK_LOST` fault events.

**Pass criteria:**
- Zero `FCC_LINK_LOST` events during 30-min run
- `message_rate ≥ 10 Hz` throughout (no sustained drops)

---

### TP-HW-005: Full System Boot Time

**SRS ref:** SW-010  
**Preconditions:** DUT fully powered off; `drone-mc.target` configured for autostart.

**Procedure:**
1. Apply power to DUT.
2. Record time from power-on to `mission_manager_node` heartbeat first received on ROS 2.
3. Repeat 5 times.

**Pass criteria:** `mean_boot_time_s ≤ 60`

---

### TP-HW-006: Smoke Test Suite on Physical DUT

**SRS ref:** SW-001–043  
**Preconditions:** Fresh OS + MC installation following the numbered installation scripts.

**Procedure:**
1. Run each `NN b-smoke-test-*.sh` script in sequence.
2. Record pass/fail for each check.

**Pass criteria:** All smoke test checks pass (zero failures).

---

### TP-HW-007: Gimbal RTSP Pipeline — Sustained Round-Trip Latency

**SRS ref:** FR-081, FR-083  
**Preconditions:** Physical gimbal IP camera connected and powered (HIF-007, 192.168.10.200); `perception_front_node` and `video_mux_node` running; a GCS-side RTSP client connected to `rtsp://192.168.10.11:8555/mc_video`.

**Procedure:**
1. Run the gimbal ingest → inference/tracking → re-encode → GCS uplink pipeline continuously for 10 min.
2. Record per-frame round-trip latency: gimbal camera frame timestamp to muxed-frame arrival at the GCS client.
3. Record GPU temperature every 30 s alongside `perception_down_node`'s concurrent load (both pipelines active).

**Pass criteria:**
- `p99(round_trip_latency_ms) ≤ 1000` for the full 10-min run
- No thermal throttling causes `perception_down_node`'s latency (TP-HW-003) to exceed its own 500 ms bound while both pipelines run concurrently (PR-010, FR-082)
- Zero `GIMBAL_LINK_LOST` events during the run (stable RTSP link)

---

## 8. Deployment Smoke Tests — TP-SM

Smoke tests are companion scripts to each installation script. They verify the installation without requiring a full system boot. Run after each script and again as a full suite before TP-NI.

| Test ID | Script | Key checks |
|---|---|---|
| TP-SM-001 | `01b-smoke-test-os.sh` | Ubuntu 22.04 · aarch64 · kernel ≥ 5.10 · `/dev/ttyTHS1` exists |
| TP-SM-002 | `02b-smoke-test-jetpack.sh` | JetPack 6.2.2 · nvcc present · tegrastats runs · CUDA 12.x |
| TP-SM-003 | `03b-smoke-test-docker.sh` | Docker Engine running · cgroups v2 enabled · `docker info` no errors |
| TP-SM-004 | `04b-smoke-test-ros2.sh` | ROS 2 Humble sourced · `ros2 --version` · CycloneDDS shared library present |
| TP-SM-005 | `05b-smoke-test-colcon.sh` | `colcon` in PATH · workspace installed at `/opt/drone-mc/install` · `ros2 pkg list` shows all MC packages |
| TP-SM-006 | `06b-smoke-test-fcc-links.sh` | HIF-001 (ttyTHS1) hardware verification: `config/environment` pins device/baud · `/dev/ttyTHS1` present, `dialout` group · fcc-bridge image has `MicroXRCEAgent` · `LIVE_CHECK=1`: `/fmu/out/vehicle_status` ≥ 5 Hz, `/fmu/out/vehicle_local_position` ≥ 10 Hz against a real Pixhawk 6X |
| TP-SM-007 | `07b-smoke-test-xrce-dds.sh` | `MicroXRCEAgent` binary in PATH · can bind to test port without error |
| TP-SM-008 | `08b-smoke-test-camera.sh` | `/dev/video0` exists · `gst-launch-1.0 nvarguscamerasrc ... appsink` produces frames (5 s run) |
| TP-SM-009 | `09b-smoke-test-services.sh` | All `drone-mc-*.service` units loaded · `drone-mc.target` enabled · systemd slice configs valid |
| TP-SM-010 | `10b-smoke-test-networking.sh` | Static IP 192.168.10.11 on Ethernet interface · `ping 192.168.10.31` success · UDP port 14900 bindable |
| TP-SM-011 | `10b-smoke-test-zenoh-bridge.sh` | `zenoh-bridge-dds.json5` declares peer mode, disables multicast scouting, has ≥1 connect endpoint · image contains the `zenoh-bridge-dds` binary · `drone-mc-zenoh-bridge.service` bind-mounts `/etc/drone-mc` read-only · `LIVE_CHECK=1`: container stays running (catches a glibc mismatch between the vendor's prebuilt aarch64 build and JetPack 6.2.2) |

**Pass criteria for each:** Exit code 0; zero FAIL lines in output.

**Note:** this table's script names reflect an earlier planned numbering scheme; the actual `scripts/` directory (see `CLAUDE.md`'s script listing) has since diverged for several entries (e.g. the real companion to `04-install-ros2-core.sh` is `04b-smoke-test-ros2.sh` + `04c-smoke-test-dds.sh`, not a single `04b`). TP-SM-011 above uses the real, current script name.

---

## 9. Requirements Traceability Matrix

The table below maps each SRS functional requirement to at least one test case. Requirements not listed here are either structural (no direct test) or verified by design review.

| Requirement | Description (short) | Unit | Node Integration | SITL | HIL |
|---|---|---|---|---|---|
| FR-001 | Mission phase FSM | TP-UT-010, TP-UT-011 | TP-NI-011 | TP-ST-001 | — |
| FR-002 | Pre-arm checks | TP-UT-011 | TP-NI-010 | TP-ST-001 | — |
| FR-003 | GCS overrides phase | TP-UT-012 | TP-NI-010 | TP-ST-001 | — |
| FR-004 | GCS ARM authorisation | — | TP-NI-010 | TP-ST-001 | — |
| FR-005 | Mission phase published | — | TP-NI-011 | TP-ST-001 | — |
| FR-015 | FCC link watchdog | TP-UT-015 | TP-NI-003 | TP-ST-006 | TP-HW-004 |
| FR-016 | FccBackend transport agnostic | — | TP-NI-001 | TP-ST-001 | TP-HW-001 |
| FR-021 | Formation setpoint ≥5 Hz | TP-UT-008, TP-UT-009 | TP-NI-004 | TP-ST-001 | — |
| FR-024 | Leader staleness → HOLD | — | TP-NI-005 | — | — |
| FR-031 | ISR selection algorithm | TP-UT-001, TP-UT-002 | — | TP-ST-001, TP-ST-010 | — |
| FR-035 | Voronoi partition | TP-UT-006, TP-UT-007 | — | TP-ST-001 | — |
| FR-036 | Re-partition on peer loss | — | TP-NI-006 | TP-ST-005 | — |
| FR-041 | Boustrophedon path | TP-UT-003, TP-UT-004, TP-UT-005 | TP-NI-008, TP-NI-009 | TP-ST-001 | — |
| FR-042 | Detection inference | — | TP-NI-013 | TP-ST-008 | TP-HW-003 |
| FR-043 | Detection geolocation | TP-UT-013, TP-UT-014 | TP-NI-013 | TP-ST-008 | — |
| FR-051 | VO localisation | — | TP-NI-016 | TP-ST-009 | — |
| FR-052 | VO feedback to FCC | — | TP-NI-016 | — | — |
| FR-053 | VO drift ≤ 5 m/min | TP-UT-018 | — | TP-ST-009 | — |
| FR-062 | Mission upload decoding | — | TP-NI-010 | TP-ST-007 | — |
| FR-063 | GCS telemetry | — | TP-NI-014, TP-NI-015 | TP-ST-011 | — |
| FR-072 | Low battery RTL | TP-UT-016, TP-UT-017 | — | TP-ST-003 | — |
| FR-073 | GCS link loss → HOLD | — | — | TP-ST-002 | — |
| FR-074 | Manager heartbeat watchdog | — | TP-NI-012 | TP-ST-004 | — |
| FR-075 | 3× restart → RTL | — | — | TP-ST-004 (ext.) | — |
| FR-080 | Gimbal ingest + inference | — | TP-NI-017 | — | TP-HW-007 |
| FR-081 | Gimbal detection/track publish | TP-UT-019 | TP-NI-017, TP-NI-018 | — | TP-HW-007 |
| FR-082 | Concurrent GPU budget (both pipelines) | — | — | — | TP-HW-007 |
| FR-083 | GCS video uplink | — | TP-NI-019, TP-NI-020 | — | TP-HW-007 |
| FR-084 | Gimbal link loss — no flight failsafe | — | TP-NI-018 | TP-ST-013 | — |
| FR-085 | Downward-camera GCS video uplink (Stream B) | — | TP-NI-021 | — | TP-HW-007 |
| FR-086 | Stream B independent failure domain | — | TP-NI-022 | — | TP-HW-007 |
| IR-001 | ttyTHS1 serial | — | — | — | TP-HW-004 |
| IR-003 | VehicleState ≥10 Hz | — | TP-NI-001, TP-NI-002 | — | — |
| IR-004 | Trajectory setpoint ≥5 Hz | — | TP-NI-004 | — | — |
| IR-006 | FCC actuation ≤100 ms | — | — | — | TP-HW-001 |
| IR-010 | Mission upload protocol | — | TP-NI-010 | TP-ST-007 | — |
| IR-013 | COMMAND_ACK ≤500 ms | — | TP-NI-015 | — | — |
| IR-014 | MC_STATUS ≥2 Hz | — | TP-NI-014 | TP-ST-011 | — |
| IR-023 | Peer LOST after 3 s | — | TP-NI-006, TP-NI-007 | TP-ST-005 | — |
| IR-030 | Camera ≥15 FPS | — | — | — | TP-HW-002 |
| IR-040 | Gimbal camera RTSP ingest | — | TP-NI-017, TP-NI-018 | — | TP-HW-007 |
| IR-041 | GCS video uplink RTSP + metadata | — | TP-NI-019 | — | TP-HW-007 |
| IR-042 | Uplink status, independent of MAVLink | — | TP-NI-020 | — | — |
| IR-043 | Downward-camera GCS video uplink (Stream B) | — | TP-NI-021, TP-NI-022 | — | TP-HW-007 |
| PR-007 | Detection latency ≤500 ms | — | TP-NI-013 | — | TP-HW-003 |
| PR-008 | Path generation ≤5 s | TP-UT-005 | TP-NI-008 | — | — |
| PR-009 | CPU usage budget | — | — | TP-ST-012 | — |
| PR-010 | GPU usage budget | — | — | TP-ST-012 | TP-HW-003 |
| SFR-001 | FCC link watchdog | TP-UT-015 | TP-NI-003 | TP-ST-006 | — |
| SFR-003 | Manager restart | — | TP-NI-012 | TP-ST-004 | — |
| SFR-004 | 3× restart → RTL | — | — | TP-ST-004 (ext.) | — |
| SFR-007 | GCS loss → HOLD | — | — | TP-ST-002 | — |
| SFR-008 | Low battery RTL (non-suppressible) | TP-UT-016, TP-UT-017 | — | TP-ST-003 | — |
| SW-001–043 | Software requirements | — | — | — | TP-HW-006, TP-SM-001–011 |

---

## 10. Test Schedule

Tests are aligned with the 6-month project milestones.

| Milestone | Month | Test activities |
|---|---|---|
| M1 — Architecture complete | 1 | TP-SM-001–005 (OS, JetPack, Docker, ROS 2, colcon) |
| M2 — Core nodes integrated | 2 | TP-UT-001–019 · TP-NI-001–007 · TP-SM-006–011 · TP-HW-001–002 |
| M3 — SITL single-drone | 3 | TP-ST-001 (single drone) · TP-ST-003 · TP-ST-006 · TP-ST-007 |
| M4 — SITL multi-drone | 4 | TP-NI-008–022 · TP-ST-001–013 (full suite) · TP-HW-003–004 |
| M5 — HIL integration | 5 | TP-HW-001–007 (all HIL tests on physical hardware) |
| M6 — System acceptance | 6 | Full regression: TP-SM + TP-UT + TP-NI + TP-ST · TP-HW repeat |

**CI gates (automated, every PR):**
- TP-SM-001–011
- TP-UT-001–018
- TP-NI-001–007

**Nightly CI:**
- All TP-NI
- TP-ST-001, TP-ST-002, TP-ST-003, TP-ST-006, TP-ST-007, TP-ST-013

---

## 11. Defect Management

### 11.1 Severity Classification

| Severity | Definition | Resolution SLA |
|---|---|---|
| S1 — Critical | Safety-critical test fails (failsafe, RTL, FCC link) | Fix before any HIL testing |
| S2 — High | Core mission function fails (ISR, Voronoi, SEARCH path) | Fix before next milestone |
| S3 — Medium | Non-safety functional defect (telemetry, logging, GCS display) | Fix within milestone |
| S4 — Low | Cosmetic, documentation, or non-blocking defect | Fix at discretion |

### 11.2 Defect Record Fields

Each defect is recorded with: Test ID · Severity · Date found · Description · Steps to reproduce · Expected result · Actual result · Root cause (when known) · Fix commit · Retest result.

### 11.3 Blocking Criteria

A milestone is blocked from closure if any S1 or S2 defect is open against that milestone's required test cases. S3/S4 defects may be deferred with documented justification.

---

*MC-TP-001 · v0.1-DRAFT · 2026-06-20*
