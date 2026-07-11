# swarm-drones

Multi-drone swarm mission computer (MC) software for a Jetson Orin NX +
Pixhawk 6X platform, built on ROS 2 Humble. A companion MC runs on each
drone and coordinates with peers over a MANET radio for GCS-supervised,
autonomous ISR search missions (elect a role, partition the search area via
Voronoi cells, fly boustrophedon coverage, detect and geolocate targets).

## Status

Design phase is complete; ROS 2 package implementation is underway in
`ws/src/`. Packages currently present: `fcc_bridge`, `gcs_comms`,
`health_monitor`, `mc_msgs`, `mission_exec`, `px4_msgs` (vendored, fetched by
`scripts/05-build-workspace.sh`), `swarm_coord`, `vision`.

## Documentation

All design artifacts live at the repo root and in `diagrams/`:

| File | Purpose |
|------|---------|
| `SRS-MC-001.md` | Requirements (FR/IR/PR/SFR/SW tags — 80+ requirements) |
| `ICD-MC-001.md` | Interface specifications for all 5 interface domains |
| `SDD-MC-001.md` | Software design: node designs, algorithms, deployment architecture |
| `HID-MC-001.md` | Hardware interface specs (UART, CSI, Ethernet, power) |
| `TP-MC-001.md` | Test plan: 4-level strategy (UT → NI → ST → HW) |
| `FCC-MISSION-CHANNEL-BACKLOG.md` | Follow-up tickets for the MC↔FCC mission protocol channel |
| `diagrams/*.puml` | 27 PlantUML source diagrams (rendered to matching `.png`) |

Key diagrams: `02` (ROS 2 node topology), `03` (Docker/systemd deployment),
`04` (mission state machine), `10` (FccBackend class hierarchy).

See `CLAUDE.md` for detailed architecture notes, design patterns, and the
full repository layout.

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
./scripts/04-install-ros2-core.sh
./scripts/04b-smoke-test-ros2.sh

# 2. Build the shared colcon workspace (ws/src/) inside the container
./scripts/05-build-workspace.sh          # writes to /opt/drone-mc/install
./scripts/05b-smoke-test-colcon.sh

# 3. Install the drone-mc.target systemd orchestration
sudo ./scripts/02-install-systemd-units.sh
sudo ./scripts/02b-smoke-test-units.sh
sudo systemctl start drone-mc.target
```

Each numbered script has a `<n>b-smoke-test-*.sh` companion that verifies
the previous step. See `CLAUDE.md` for build variants (debug builds,
alternate install directories, versioned image tags) and native
per-package `colcon build`/`colcon test` usage.

## Repository layout

```
docker/ros2-core/     ros2-core base Docker image
config/                environment, per-node params, netplan, udev, systemd units
ws/src/                ROS 2 colcon workspace (7 first-party packages + vendored px4_msgs)
scripts/               numbered setup/build/install scripts + smoke tests
diagrams/              PlantUML architecture diagrams
```
