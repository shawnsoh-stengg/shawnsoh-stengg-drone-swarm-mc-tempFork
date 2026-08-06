# swarm-drones

Multi-drone swarm mission computer (MC) software for a Jetson Orin NX +
Pixhawk 6X platform, built on ROS 2 Humble. A companion MC runs on each
drone and coordinates with peers over a MANET radio for GCS-supervised,
autonomous search abd Rescue missions (elect a role, partition the search area via
Voronoi cells, fly boustrophedon coverage, detect and geolocate targets).

## Status

Design phase is complete; ROS 2 package implementation is underway in
`ws/src/`. Packages currently present: `fcc_bridge`, `gcs_comms`,
`health_monitor`, `mc_msgs`, `mission_exec`, `px4_msgs` (vendored, fetched by
`scripts/05-build-workspace.sh`), `swarm_coord`, `video_downlink`, `vision`.

## Documentation

All design documents at /docs directory

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

# 5. Install the drone-mc.target systemd orchestration
sudo ./scripts/02-install-systemd-units.sh
sudo ./scripts/02b-smoke-test-units.sh
sudo systemctl start drone-mc.target
```

Each numbered script has a `<n>b-smoke-test-*.sh` companion that verifies
the previous step. See /docs for build variants (debug builds,
alternate install directories, versioned image tags) and native
per-package `colcon build`/`colcon test` usage.

## Repository layout

```
docker/ros2-core/       ros2-core base Docker image
docker/fcc-bridge/      fcc-bridge service Docker image (FROM ros2-core)
docker/mavlink-router/  mavlink-router service Docker image (standalone, not FROM ros2-core)
config/                 environment, per-node params, netplan, udev, systemd units
ws/src/                 ROS 2 colcon workspace (8 first-party packages + vendored px4_msgs)
scripts/                numbered setup/build/install scripts + smoke tests
docs/                   architecture design documents
```
