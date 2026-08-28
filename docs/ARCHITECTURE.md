# Architecture Overview

This is a synthesized, developer-facing entry point into the system design. For the authoritative, exhaustive design (node internals, algorithm proofs, deployment rationale) see `SDD-MC-001.md` at the repo root. For requirements and interface wire formats see `SRS-MC-001.md` and `ICD-MC-001.md`.

## What this software does

Each drone in the swarm runs an identical **mission computer (MC)** stack on a Jetson Orin NX, paired with a Pixhawk 6X flight controller (FCC). MCs coordinate over a MANET radio to run GCS-supervised, autonomous ISR search missions: elect a search-and-rescue (ISR) role among peers, partition the search area into Voronoi cells, fly boustrophedon coverage per cell, and
detect + geolocate targets — all while the ground control station (GCS) retains override authority at all times.

## The 4-layer design

```
Layer 4 — Mission Intelligence
  swarm_coordinator_node, mission_manager_node, search_planner_node,
  visual_localisation_node, perception_down_node, perception_front_node

Layer 3 — GCS & Swarm Communication
  gcs_mavlink_node (+ MAVROS), mavlink-router v4, video_mux_node

Layer 2 — FCC Abstraction
  fcc_bridge_node  <-->  FccBackend (XrceDdsBackend | MavlinkUdpBackend)

Layer 1 — Hardware & Transport
  ttyTHS1 serial (uXRCE-DDS), 192.168.12.x ethernet, CSI camera, MANET radio
```

Layers communicate strictly downward-to-upward through ROS 2 topics under the `/mc/` namespace (see below) — a Layer 4 node never talks to Layer 1 hardware directly, it goes through `fcc_bridge_node`. See diagram `02` (`docs/diagrams/02-software-architecture.puml`) for the full node topology.

## Packages

| Package | Layer | Responsibility |
|---|---|---|
| `fcc_bridge` | 2 | Abstracts the flight controller behind a stable ROS 2 interface; swaps transport (DDS vs. MAVLink UDP) via the `FccBackend` factory pattern |
| `gcs_comms` | 3 | `gcs_mavlink_node` (raw MAVLink v2 Channel B) + MAVROS (mirrors Channel A telemetry) |
| `health_monitor` | cross-cutting | Aggregates node/link health into `/mc/health`, `/mc/fault` |
| `mc_msgs` | cross-cutting | Custom ROS 2 message/service definitions shared across packages |
| `mission_exec` | 4 | `mission_manager_node` (11-state mission state machine) + `search_planner_node` (boustrophedon coverage) |
| `swarm_coord` | 4 | ISR role election, Voronoi partitioning, peer state exchange |
| `vision` | 4 | `perception_down_node` (YOLOv8n detection/geolocation), `perception_front_node`, `visual_localisation_node` (GNSS-denied VO) |
| `video_downlink` | 3 | `video_mux_node` — gimbal (Stream A) and downward (Stream B) video to GCS |
| `px4_msgs` (vendored) | 1 | PX4 uXRCE-DDS message definitions, fetched by `scripts/05-build-workspace.sh` |

Plus two external daemons that are **not** colcon packages: `mavlink-router`
(Meson/Ninja build) and `zenoh-bridge_dds` (cross-host DDS bridge over the
MANET link).

## Key design patterns

- **FccBackend abstraction** (`fcc_bridge`) — the seam between hardware-in-the-loop (uXRCE-DDS to a real Pixhawk) and SITL (MAVLink UDP to a simulator). Nothing above Layer 2 changes when the backend swaps.
- **MAVLink channel separation** — three distinct channels share the wire:
  - **Channel A**: GCS→FCC, standard PX4 autopilot traffic, routed by `mavlink-router`.
  - **Channel B**: GCS→MC, raw MAVLink v2 over a loopback socket in `gcs_mavlink_node` (MAVROS is *not* this channel's transport — see `SDD-MC-001.md` §11.1).
  - **Channel C / MC↔FCC mission protocol**: standard MAVLink mission protocol (`MISSION_COUNT`/`MISSION_ITEM_INT`/`MISSION_ACK`) on a dedicated UDP port (`192.168.12.31:14555`), since stock PX4 has no DDS-writable mission topic. Not routed by `mavlink-router`.
- **Threading model** — every node runs a `MultiThreadedExecutor` with a reentrant callback group for subscriptions and a mutually-exclusive group for timers; control loops use a 10 ms `WallTimer` plus atomic state.
- **Mission state machine** — 11 states in `mission_manager_node`: `PREFLIGHT → ARMING → TAKEOFF → INGRESS → MUSTER → ISR_SELECTION → VORONOI_PARTITION → SEARCH → RETURN → LAND`, with a `FAULT → RTL` escape path from any state. GCS override must be honored within 500 ms (FR-003). See diagram `04`.

## Key algorithms (SDD §3.x)

| Algorithm | Where | Summary |
|---|---|---|
| ISR election | `swarm_coord` | Weighted score `0.4×battery + 0.4×proximity + 0.2×sensor_ok`; top ⌈N/2⌉ elected, confirmed via ACK consensus |
| Voronoi partitioning | `swarm_coord` | Deterministic Fortune's sweep line, computed independently per drone, cross-checked via `VORONOI_ACCEPT` (1 cm tolerance); re-partitions on peer loss during SEARCH |
| Boustrophedon coverage | `mission_exec` | Min-area bounding rectangle + parallel sweep lines, +5 m altitude offset between rows; Cartesian waypoints back-projected to WGS-84 for PX4 mission upload |
| Visual odometry | `vision` | FAST+ORB → KLT → 5-point essential + RANSAC, IMU-preintegration scale, 10-keyframe sliding window (no loop closure); auto-handover GNSS↔VO on fix quality |
| Detection | `vision` | YOLOv8n ONNX → TensorRT, 640×640, IoU 0.45 / conf 0.5; geolocation via camera intrinsics + drone pose; max queue depth 2 (drop oldest) |

## Topic map (selected)

All internal topics are namespaced under `/mc/`:

| Topic | Publisher | Notes |
|---|---|---|
| `/mc/fcc/vehicle_state` | `fcc_bridge` | Authoritative flight state, 10 Hz |
| `/mc/mission_phase` | `mission_exec` | Current state machine phase |
| `/mc/swarm/peer_states`, `/mc/swarm/voronoi_cell`, `/mc/swarm/isr_role` | `swarm_coord` | Swarm coordination state |
| `/mc/local_pose` | `vision` | Visual odometry position estimate, 10+ Hz |
| `/mc/detections` | `perception_down_node` | Geolocated AI detections |
| `/mc/perception/front/detections`, `/mc/perception/front/tracks` | `perception_front_node` | Front gimbal camera |
| `/mc/video/downlink_status`, `/mc/video/down_downlink_status` | `video_mux_node` | GCS video downlink health (Stream A / Stream B) |
| `/mc/health`, `/mc/fault` | `health_monitor` | System health / fault signals |

## Deployment

Each Docker container is a `systemd` unit under `drone-mc.target` — not Docker Compose. `fcc_bridge` and `swarm_coord` run in `rt.slice` (real-time); everything else (including `zenoh-bridge`) runs in `be.slice` (best-effort). The shared colcon workspace is bind-mounted into every container except `mavlink-router` and `zenoh-bridge`, which are standalone daemons with no ROS 2/colcon dependency. See diagram `03`.

## Where to go next

- New to the repo and want to build it? See `docs/GETTING-STARTED.md`.
- Need the wire-level interface contract? `ICD-MC-001.md`.
- Need requirement traceability? `SRS-MC-001.md`.
- Need the full design rationale (this doc is a summary of it)?
  `SDD-MC-001.md`.
