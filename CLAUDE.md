# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Status

Design is complete; implementation is underway. The 8 packages described in `SDD-MC-001.md` are being built out in `ws/src/` (`fcc_bridge`, `gcs_comms`, `health_monitor`, `mc_msgs`, `mission_exec`, `swarm_coord`, `vision`, `video_downlink`), alongside the vendored `px4_msgs` dependency fetched by `scripts/05-build-workspace.sh`.

## Document Map

| File | Purpose |
|------|---------|
| `SRS-MC-001.md` | Requirements (FR/IR/PR/SFR/SW tags — 80+ requirements) |
| `ICD-MC-001.md` | Interface specifications for all 5 interface domains |
| `SDD-MC-001.md` | Software design: node designs, algorithms, deployment architecture |
| `HID-MC-001.md` | Hardware interface specs (UART, CSI, Ethernet, power) |
| `TP-MC-001.md` | Test plan: 4-level strategy (UT → NI → ST → HW) |
| `diagrams/` | 28 PlantUML source files + rendered PNGs (01–28) |

Key diagrams: `02` (ROS 2 node topology), `03` (Docker/systemd deployment), `04` (mission state machine), `10` (FccBackend class hierarchy), `28` (system use cases — GCS Operator/QGC vs. Mission Planner/Orchestrator).

## Build System

```bash
# 0. One-time Jetson host setup (runs directly on the Jetson, not in Docker)
# Prerequisite: Jetson already flashed with JetPack 6.2.2 via NVIDIA SDK
# Manager (HID-MC-001 §8.1) and booted to a login prompt.
sudo ./scripts/01-install-os.sh          # Netplan dual-IP (eth0) + ttyTHS1 udev rule
sudo ./scripts/01b-smoke-test-os.sh      # TP-SM-001: OS + netplan + udev verification

# 1. Build the ros2-core base Docker image (once, or after Dockerfile changes)
sudo ./scripts/04-install-ros2-core.sh
sudo ./scripts/04b-smoke-test-ros2.sh        # verify ROS 2 Humble + CycloneDDS

# 2. Build the shared colcon workspace (ws/src/) inside the container
sudo ./scripts/05-build-workspace.sh         # writes to /opt/drone-mc/install
sudo ./scripts/05b-smoke-test-colcon.sh      # verify workspace

# 3. Build the fcc-bridge service image (ghcr.io/drone-mc/fcc-bridge)
sudo ./scripts/08-build-fcc-bridge-image.sh
sudo ./scripts/08b-smoke-test-fcc-bridge.sh

# 4. Install the drone-mc.target systemd orchestration (on the Jetson)
sudo ./scripts/02-install-systemd-units.sh    # installs + enables all 8 service units
sudo ./scripts/02b-smoke-test-units.sh        # verify units/target/slices
sudo systemctl start drone-mc.target          # brings up every MC service container

# MC_VERSION overrides the Docker image tag (default: dev)
MC_VERSION=1.0.0 ./scripts/04-install-ros2-core.sh

# Debug build
CMAKE_BUILD_TYPE=Debug ./scripts/05-build-workspace.sh

# Alternate install dir (useful on developer workstations)
DRONE_MC_INSTALL_DIR=/tmp/drone-mc-install ./scripts/05-build-workspace.sh
```

ROS 2 packages live in `ws/src/`. Once packages exist:

```bash
# Build single package (native, with workspace sourced)
source /opt/ros/humble/install/setup.bash
colcon build --merge-install --packages-select fcc_bridge

# Unit tests (gtest)
colcon test --packages-select fcc_bridge
colcon test-result --verbose

# Node-integration tests (launch_testing)
ros2 launch fcc_bridge fcc_bridge_test.launch.py

# System tests against PX4 SITL
ros2 launch mission_exec sitl_mission_test.launch.py

# mavlink-router (Meson/Ninja, separate from colcon)
cd mavlink-router && meson setup build && ninja -C build
```

## Architecture Overview

**Platform:** Jetson Orin NX 16GB (JetPack 6.2.2, Ubuntu 22.04 aarch64) + Pixhawk 6X (PX4 v1.14). Connected via ttyTHS1 (921600 baud, uXRCE-DDS) or 192.168.12.x ethernet fallback.

**4-Layer design** (SDD §2.1):

```
Layer 4 — Mission Intelligence
  swarm_coordinator_node, mission_manager_node, search_planner_node,
  visual_localisation_node, perception_down_node (renamed from detection_node),
  perception_front_node

Layer 3 — GCS & Swarm Communication
  gcs_mavlink_node (MAVROS bridge), mavlink-router v4, video_mux_node

Layer 2 — FCC Abstraction
  fcc_bridge_node  ←→  FccBackend (XrceDdsBackend | MavlinkUdpBackend)

Layer 1 — Hardware & Transport
  ttyTHS1 serial, 192.168.12.x ethernet, CSI camera, MANET radio
```

**7 ROS 2 packages** (`fcc_bridge`, `swarm_coord`, `mission_exec`, `vision`, `video_downlink`, `gcs_comms`, `health_monitor`) **plus 2 external daemons**: `mavlink_router`, `zenoh_bridge_dds`.

## Key Design Patterns

**FccBackend abstraction** — `fcc_bridge_node` uses a factory pattern to swap uXRCE-DDS (production) for MAVLink UDP (SITL) without changing any higher-layer code. This is the central seam for hardware-in-the-loop vs. simulation.

**Threading model** — Every node uses `MultiThreadedExecutor` with two `CallbackGroup`s: reentrant for subscription callbacks (parallel data processing), mutually exclusive for timer callbacks (serialized periodic publish). Control loops use `WallTimer` at 10 ms + atomic state.

**MAVLink channel separation** — Three distinct channels: GCS→FCC (standard PX4 autopilot), GCS→MC (separate MAVROS channel), MC↔FCC (uXRCE-DDS for vehicle state/commands — mavlink-router routes between the first two). MC↔FCC also carries one MAVLink exception: the standard mission protocol (`MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK`, plus `MISSION_WRITE_PARTIAL_LIST` for single-item replace) on a dedicated UDP port (`192.168.12.31:14555`, always ethernet, not routed by mavlink-router) — stock PX4 has no DDS-writable mission topic, so this is the only way to get waypoints into FCC-side dataman (ICD IR-007).

**Mission state machine** — 11 states in `mission_manager_node`: PREFLIGHT → ARMING → TAKEOFF → INGRESS → MUSTER → ISR_SELECTION → VORONOI_PARTITION → SEARCH → RETURN → LAND, with FAULT → RTL escape. GCS retains override authority at all times (500 ms response requirement, FR-003).

**Deployment** — Not Docker Compose. Each Docker container is managed as a systemd unit under `drone-mc.target`. RT services (`fcc_bridge`, `swarm_coord`) in `rt.slice`; best-effort (including `zenoh-bridge` and the new `video-ingest`/`video-downlink` containers — `perception_front_node` and `video_mux_node` split into separate containers, 2026-07-22, so one node's restart can't take the other down) in `be.slice`. Shared colcon workspace bind-mounted to all containers except `mavlink-router` and `zenoh-bridge`, which are standalone daemons with no ROS 2/colcon dependency.

## Key Algorithms (SDD §3.x)

- **ISR election:** weighted score (`0.4×battery + 0.4×proximity + 0.2×sensor_ok`), top ⌈N/2⌉ elected, ACK consensus.
- **Voronoi partitioning:** Fortune's sweep line (deterministic), all drones compute independently then exchange VORONOI_ACCEPT with 1 cm tolerance check. Re-partitions on peer loss during SEARCH.
- **Boustrophedon coverage:** min-area bounding rectangle + parallel sweep lines, +5 m altitude between rows. Cartesian waypoints back-projected to WGS-84 for PX4 mission upload.
- **Visual odometry (GNSS-denied):** FAST+ORB → KLT → 5-point essential + RANSAC; scale from IMU pre-integration; 10-keyframe sliding window, no loop closure; auto-handover GNSS↔VO on fix quality.
- **Detection:** YOLOv8n ONNX → TensorRT, 640×640, IoU 0.45 / conf 0.5, geolocation via camera intrinsics + drone pose. Max queue depth 2 (drop oldest on backpressure).

## Internal ROS 2 Topic Naming

All internal topics are namespaced under `/mc/`. Key topics that cross multiple packages:
- `/mc/fcc/vehicle_state` (10 Hz) — authoritative flight state from `fcc_bridge`
- `/mc/mission_phase` — current state machine phase
- `/mc/swarm/peer_states`, `/mc/swarm/voronoi_cell`, `/mc/swarm/isr_role`
- `/mc/local_pose` (10+ Hz) — visual odometry position estimate
- `/mc/detections` — AI detection results with geolocated positions (from `perception_down_node`, renamed from `detection_node`)
- `/mc/perception/front/detections`, `/mc/perception/front/tracks` — front gimbal camera detections/tracks from `perception_front_node`
- `/mc/video/downlink_status`, `/mc/video/down_downlink_status` — GCS video downlink health from `video_mux_node` (gimbal Stream A, downward Stream B)
- `/mc/health`, `/mc/fault` — system health and fault signals

## Network / IP Reference

| Host | Address |
|------|---------|
| Jetson MC | 192.168.10.11 |
| Pixhawk FCC | 192.168.10.31 |
| GCS laptop | 192.168.10.10 |
| MC↔FCC internal | 192.168.12.x |
| Peer MCs | 192.168.10.12+ |
| Gimbal IP camera (HIF-007, shared switch with Jetson MC) | 192.168.10.200+ |

## Repository Structure (Implemented)

```
docker/ros2-core/
  Dockerfile          # ros2-core base image (FROM for all service containers)
  cyclonedds.xml      # same-host (lo-only) DDS config, overridden by /etc/drone-mc bind-mount
config/
  environment         # default env vars for all containers (ICD §9.4)
  zenoh-bridge-dds.json5  # cross-host DDS bridge config (ICD §9.4), overridden by /etc/drone-mc bind-mount
  params/             # per-node ROS 2 YAML params (populated as packages land)
  netplan/
    01-drone-mc.yaml          # Jetson eth0 dual-IP (HIF-002/HIF-004), installed by 01-install-os.sh
  udev/
    99-drone-mc.rules         # /dev/ttyTHS1 dialout permissions (HIF-001), installed by 01-install-os.sh
  systemd/
    drone-mc.target                 # top-level target, all 9 services WantedBy= this (SDD §12.4)
    rt.slice / be.slice             # cgroup v2 slices (SDD §12.3)
    drone-mc-health-monitor.service # + mavlink-router, zenoh-bridge, fcc-bridge, gcs-comms,
                                     #   swarm-coord, mission-exec, camera, vision — installed by
                                     #   02-install-systemd-units.sh (video-ingest/video-downlink
                                     #   units not yet created — SDD-MC-001 §12.4 design only)
ws/
  src/                # colcon workspace — ROS 2 packages go here
scripts/
  01-install-os.sh            # Jetson host: install Netplan + udev rule (post-JetPack-flash)
  01b-smoke-test-os.sh        # TP-SM-001: OS + netplan + udev verification
  02-install-systemd-units.sh  # install + enable drone-mc.target and all 9 service units
  02b-smoke-test-units.sh      # units/target/slices loaded, enabled, valid (LIVE_CHECK=1 for a real boot test)
  04-install-ros2-core.sh     # build ros2-core Docker image
  04b-smoke-test-ros2.sh      # TP-SM-004: ROS 2 + CycloneDDS
  05-build-workspace.sh       # colcon build → /opt/drone-mc/install
  05b-smoke-test-colcon.sh    # TP-SM-005: workspace + pkg-list
  06b-smoke-test-fcc-links.sh  # HIF-001 (ttyTHS1) hardware-link verification
  06c-smoke-test-xrce-agent.sh # FCC-008: FCC_TRANSPORT selection + agent/node liveness
                                #   (no plain "06-*.sh": unlike every other numbered pair,
                                #   06b/06c verify config+behavior other scripts already set
                                #   up — HIF-001 wiring from 01-install-os.sh, the
                                #   MicroXRCEAgent-bearing image from 08-build-fcc-bridge-image.sh
                                #   — rather than installing/building anything of their own,
                                #   so there's no "06" step for them to pair with)
  08-build-fcc-bridge-image.sh  # build ghcr.io/drone-mc/fcc-bridge image
  08b-smoke-test-fcc-bridge.sh  # package/interface/executable registration + loopback publish
  09-build-mavlink-router-image.sh  # build ghcr.io/drone-mc/mavlink-router image (standalone, not FROM ros2-core)
  09b-smoke-test-routing.sh         # GCS-001/MAVR-001 channel isolation
  10-build-zenoh-bridge-image.sh    # build ghcr.io/drone-mc/zenoh-bridge image (standalone, not FROM ros2-core)
  10b-smoke-test-zenoh-bridge.sh    # config/image checks; LIVE_CHECK=1 verifies the container stays up
```

## Tooling Decisions

- **Diagrams:** PlantUML (v1.2026 compatible syntax). Source in `diagrams/*.puml`, rendered to `diagrams/*.png`.
- **Documents:** Markdown only (not HTML/artifacts).
- **Languages:** C++17 (all ROS 2 nodes), Python 3 (test utilities, GCS-side scripts).
- **DDS:** CycloneDDS with `cyclonedds.xml` in container base image, scoped to same-host (`lo`) discovery only. Cross-host DDS, if/when it's needed, rides `zenoh-bridge-dds` over a static-peer Zenoh mesh instead of CycloneDDS's own discovery protocol — a better fit for the bandwidth-constrained MANET link (`config/zenoh-bridge-dds.json5`).
- **Params:** YAML files in `params/default.yaml` and `params/sitl.yaml` per package.
