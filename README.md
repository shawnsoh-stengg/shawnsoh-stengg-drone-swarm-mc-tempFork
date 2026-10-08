# swarm-drones

Multi-drone swarm mission computer (MC) software for a Jetson Orin NX +
Pixhawk 6X platform, built on ROS 2 Humble. A companion MC runs on each
drone and coordinates with peers over a MANET radio for GCS-supervised,
autonomous search abd Rescue missions (elect a role, partition the search area via
Voronoi cells, fly boustrophedon coverage, detect and geolocate targets).

## Status

Design phase is complete; ROS 2 package implementation is underway in
`ws/src/`. Packages currently present: `fcc_bridge`, `gcs_comms`,
`health_monitor`, `indoor_nav`, `mc_msgs`, `mc_test_utils`, `mission_exec`,
`px4_msgs` (vendored `release/1.17`, fetched by
`scripts/05-build-workspace.sh`), `swarm_coord`, `video_downlink`, `vision`.

Bench status: FT-006 (MC + indoor synthetic GPS against a real Pixhawk 6X /
PX4 v1.17) passes PX4 preflight on simulated GPS only, as of 2026-10-02
(`test-results/FT-006-mc-igs-bench-integration.md`). For indoor `HIL_GPS`
sessions, see the PX4 parameter set in `HID-MC-001.md` §10.7.

## Documentation

All design artifacts live at the repo root and in `docs/diagrams/`:

| File | Purpose |
|------|---------|
| `docs/INTRODUCTION.md` | New-teammate front door: mission narrative, jargon glossary, reading order, first-week checklist — start here |
| `docs/ARCHITECTURE.md` | Concise architecture overview (4-layer design, packages, patterns, algorithms) |
| `docs/GETTING-STARTED.md` | Step-by-step build/dev walkthrough for a new developer |
| `docs/API-REFERENCE.md` | Internal ROS 2 topic/message reference, grounded in current code |
| `docs/GCS-MAVLINK-API-REFERENCE.md` | External GCS↔MC MAVLink interface (Channel B commands/telemetry, Channel C MAVFTP mission-plan library) |
| `docs/TESTING.md` | How to run and write tests; current gtest conventions vs. the full TP-MC-001 test-level strategy |
| `docs/DEPLOYMENT.md` | systemd/Docker deployment: unit anatomy, image build pattern, config, known gaps |
| `docs/CONTRIBUTING.md` | Code style and commit/PR conventions, pulled from the existing source and git history |
| `SRS-MC-001.md` | Requirements (FR/IR/PR/SFR/SW tags — 80+ requirements) |
| `ICD-MC-001.md` | Interface specifications for all 5 interface domains |
| `SDD-MC-001.md` | Software design: node designs, algorithms, deployment architecture |
| `HID-MC-001.md` | Hardware interface specs (UART, CSI, Ethernet, power) |
| `TP-MC-001.md` | Test plan: 4-level strategy (UT → NI → ST → HW) |
| `PROJECT-PLAN-5-MONTH.md` | Ticket backlog and delivery plan |
| `FCC-MISSION-CHANNEL-BACKLOG.md` | Follow-up tickets for the MC↔FCC mission protocol channel |
| `OFFBOARD-MODE-SWITCHING-BACKLOG.md` | Proposal to add MISSION ⇄ OFFBOARD flight-mode switching |
| `GCS-ZENOH-DESIGN-REVIEW.md` | MAVLink vs. Zenoh design review for GCS↔MC traffic (partially superseded) |
| `MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md` | Mission Orchestrator ↔ MC interface (MAVLink Channel C mission plan library) |
| `INTEGRATION-AND-FLIGHT-TEST-PLAN.md` | Drone↔GCS/drone↔drone integration staging, CI/CD rollout, desk→indoor→outdoor flight test progression (`FT-*` tickets, with dated bench findings) |
| `INDOOR-GNSS-SIMULATION-BACKLOG.md` | `indoor_nav` package (`gnss_sim_node`): synthetic `HIL_GPS` fix so PX4 flies MISSION mode indoors (`IGS-*` tickets) |
| `HITL-SIMULATION-TEST-PLAN.md` | Networked bench rig: real Jetson MC ↔ PX4 SITL+Gazebo on a dev laptop (`HSR-*` tickets) |
| `test-results/` | Bench/hardware test procedures and results logs (FT-006, IGS-002, FCC-006) |
| `docs/diagrams/*.puml` | 29 PlantUML source diagrams (rendered to matching `.png`) |

Key diagrams: `02` (ROS 2 node topology), `03` (Docker/systemd deployment),
`04` (mission state machine), `10` (FccBackend class hierarchy), `28` (system
use cases).

## Platform

Jetson Orin NX 16GB (JetPack 6.2.2, Ubuntu 22.04 aarch64) + Pixhawk 6X
(PX4 v1.14), connected via `ttyTHS1` (921600 baud, uXRCE-DDS) with a
192.168.12.x Ethernet fallback.

## Building

```bash
# 0. One-time Jetson host setup (runs directly on the Jetson, not in Docker)
# Prerequisite: Jetson already flashed with JetPack 6.2.2 via NVIDIA SDK Manager
sudo ./scripts/01-install-os.sh
sudo ./scripts/01b-smoke-test-os.sh

# 1. Build the ros2-core base Docker image
sudo ./scripts/04-install-ros2-core.sh
sudo ./scripts/04b-smoke-test-ros2.sh

# 2. Build the shared colcon workspace (ws/src/) inside the container
sudo ./scripts/05-build-workspace.sh          # writes to /opt/drone-mc/install
sudo ./scripts/05b-smoke-test-colcon.sh

# 3. Build the fcc-bridge service image (ghcr.io/drone-mc/fcc-bridge)
sudo ./scripts/08-build-fcc-bridge-image.sh
sudo ./scripts/08b-smoke-test-fcc-bridge.sh

# 4. Build the mavlink-router service image (ghcr.io/drone-mc/mavlink-router)
# Standalone image, not FROM ros2-core — independent of steps 1-3 above.
sudo ./scripts/09-build-mavlink-router-image.sh
sudo ./scripts/09b-smoke-test-routing.sh

# 5. Build the zenoh-bridge service image (ghcr.io/drone-mc/zenoh-bridge)
# Standalone image, not FROM ros2-core.
sudo ./scripts/10-build-zenoh-bridge-image.sh
sudo ./scripts/10b-smoke-test-zenoh-bridge.sh

# 6. Build the gcs-comms service image (ghcr.io/drone-mc/gcs-comms)
sudo ./scripts/11-build-gcs-comms-image.sh
sudo ./scripts/11b-smoke-test-gcs-comms.sh

# 7. Build the mission-exec service image (ghcr.io/drone-mc/mission-exec)
sudo ./scripts/12-build-mission-exec-image.sh
sudo ./scripts/12b-smoke-test-mission-exec.sh

# 8. Build the swarm-coord service image (ghcr.io/drone-mc/swarm-coord)
sudo ./scripts/13-build-swarm-coord-image.sh
sudo ./scripts/13b-smoke-test-swarm-coord.sh

# 9. Build the health-monitor service image (ghcr.io/drone-mc/health-monitor)
sudo ./scripts/14-build-health-monitor-image.sh
sudo ./scripts/14b-smoke-test-health.sh

# 10. Build the mission-orchestrator service image (ghcr.io/drone-mc/mission-orchestrator)
# Same gcs_comms package as step 6, separate container (mission_orchestrator_node, GCS-006)
sudo ./scripts/15-build-mission-orchestrator-image.sh
sudo ./scripts/15b-smoke-test-mission-orchestrator.sh

# 11. Install the drone-mc.target systemd orchestration
sudo ./scripts/02-install-systemd-units.sh
sudo ./scripts/02b-smoke-test-units.sh
sudo systemctl start drone-mc.target

# 12. Indoor / bench sessions only: synthetic GPS (gnss_sim_node -> HIL_GPS)
# Start-on-demand, never in the boot set. Anchor it before PX4 will see a fix
# (tools/ft006_bench/ft006_bench.py anchor LAT LON ALT_MSL).
sudo ./scripts/16-build-indoor-nav-image.sh
sudo ./scripts/16b-smoke-test-indoor-nav.sh
sudo systemctl start drone-mc-indoor-nav
```

Per-drone values (`DRONE_ID`, `MC_ETH_MAC`, `MC_ETH_IP`, `FCC_ETH_IP`, and the
optional bench-only `MC_ETH_GATEWAY`/`MC_ETH_DNS`) live in
`/etc/drone-mc/environment` on each Jetson, not in the repo. Edit that file
and re-run `01-install-os.sh` and `02-install-systemd-units.sh`. Don't edit
`config/environment`, which is only the default for new units. `DRONE_ID`
must equal the FCC's `MAV_SYS_ID`. Changing any of `DRONE_ID`/`MAV_SYS_ID`, the FCC IP,
the MC IP or `GCS_IP` on a deployed drone also requires changes on the FCC
and the GCS. See the checklist in `docs/DEPLOYMENT.md`, "Changing an ID or
address on a deployed drone".

Each numbered script has a `<n>b-smoke-test-*.sh` companion that verifies
the previous step. See `CLAUDE.md` for build variants (debug builds,
alternate install directories, versioned image tags) and native
per-package `colcon build`/`colcon test` usage.


## Repository layout

```
docker/ros2-core/            ros2-core base Docker image
docker/fcc-bridge/           fcc-bridge service Docker image (FROM ros2-core)
docker/gcs-comms/            gcs-comms service Docker image (FROM ros2-core; gcs_mavlink_node + MAVROS)
docker/mission-exec/         mission-exec service Docker image (FROM ros2-core; mission_manager_node + search_planner_node)
docker/swarm-coord/          swarm-coord service Docker image (FROM ros2-core; swarm_coordinator_node + formation_controller_node)
docker/health-monitor/       health-monitor service Docker image (FROM ros2-core; health_monitor_node)
docker/mission-orchestrator/ mission-orchestrator service Docker image (FROM ros2-core; same gcs_comms package as gcs-comms, runs mission_orchestrator_node instead)
docker/mavlink-router/       mavlink-router service Docker image (standalone, not FROM ros2-core)
docker/zenoh-bridge/         zenoh-bridge service Docker image (standalone, not FROM ros2-core)
docker/indoor-nav/           indoor-nav service Docker image (FROM ros2-core; gnss_sim_node, start-on-demand)
config/                      environment, per-node params, netplan, udev, sysctl, systemd units
ws/src/                      ROS 2 colcon workspace (10 first-party packages + vendored px4_msgs)
scripts/                     numbered setup/build/install scripts + smoke tests
tools/                       bench/test helpers (ft006_bench, ft001_ac2_channel_b, igs002-flow-bench-validate, mission-orchestrator-sim)
test-results/                bench/hardware test procedures + results logs
docs/diagrams/               PlantUML architecture diagrams
```
