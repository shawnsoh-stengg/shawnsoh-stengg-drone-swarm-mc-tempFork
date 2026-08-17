# Getting Started

A step-by-step path for a developer new to this repo, from a fresh
checkout to a running `drone-mc.target` on a Jetson, plus the native
workflow for iterating on a single ROS 2 package. For architecture context
first, read `docs/ARCHITECTURE.md`.

## Prerequisites

- Jetson Orin NX 16GB already flashed with **JetPack 6.2.2** via NVIDIA SDK
  Manager (HID-MC-001 §8.1), booted to a login prompt. Steps 0+ below run
  directly on that Jetson.
- A Pixhawk 6X (PX4 v1.14) wired per `HID-MC-001.md`, reachable over
  `ttyTHS1` (921600 baud) or the `192.168.12.x` ethernet fallback.
- `sudo` access on the Jetson (the numbered scripts install udev rules,
  netplan config, sysctl floors, and systemd units).

If you're on a developer workstation without a Jetson/Pixhawk attached, skip
to [Native package development](#native-package-development-no-jetson-required)
below — you don't need real hardware to build and unit-test a package.

## Full build, in order

Each numbered script has a `<n>b-smoke-test-*.sh` (or `<n>c-*`) companion —
run it right after the matching build step to catch problems early rather
than at the end of the chain.

```bash
# 0. One-time OS setup: Netplan dual-IP (eth0), ttyTHS1 udev rule, CycloneDDS sysctl floor
sudo ./scripts/01-install-os.sh
sudo ./scripts/01b-smoke-test-os.sh

# 1. Build the ros2-core base Docker image (all service images FROM this)
sudo ./scripts/04-install-ros2-core.sh
sudo ./scripts/04b-smoke-test-ros2.sh
sudo ./scripts/04c-smoke-test-dds.sh

# 2. Build the shared colcon workspace inside the container -> /opt/drone-mc/install
sudo ./scripts/05-build-workspace.sh
sudo ./scripts/05b-smoke-test-colcon.sh

# 3. fcc-bridge service image (ghcr.io/drone-mc/fcc-bridge)
sudo ./scripts/08-build-fcc-bridge-image.sh
sudo ./scripts/08b-smoke-test-fcc-bridge.sh
#    (06b/06c verify HIF-001 wiring + the XRCE agent that 08 built — no standalone "06" step)

# 4. mavlink-router service image (standalone, Meson/Ninja build, not FROM ros2-core)
sudo ./scripts/09-build-mavlink-router-image.sh
sudo ./scripts/09b-smoke-test-routing.sh

# 5. zenoh-bridge service image (standalone, not FROM ros2-core)
sudo ./scripts/10-build-zenoh-bridge-image.sh
sudo ./scripts/10b-smoke-test-zenoh-bridge.sh

# 6. gcs-comms service image (gcs_mavlink_node + MAVROS)
sudo ./scripts/11-build-gcs-comms-image.sh
sudo ./scripts/11b-smoke-test-gcs-comms.sh

# 7. mission-exec service image (mission_manager_node + search_planner_node)
sudo ./scripts/12-build-mission-exec-image.sh
sudo ./scripts/12b-smoke-test-mission-exec.sh

# 8. Install + start the systemd orchestration (all services under drone-mc.target)
sudo ./scripts/02-install-systemd-units.sh
sudo ./scripts/02b-smoke-test-units.sh
sudo systemctl start drone-mc.target
```

Useful overrides while running any of the above:

```bash
MC_VERSION=1.0.0 ./scripts/04-install-ros2-core.sh   # tag images instead of "dev"
CMAKE_BUILD_TYPE=Debug ./scripts/05-build-workspace.sh
DRONE_MC_INSTALL_DIR=/tmp/drone-mc-install ./scripts/05-build-workspace.sh
```

### Verifying it came up

```bash
systemctl status drone-mc.target
systemctl list-units 'drone-mc-*'
journalctl -u drone-mc-fcc-bridge -f
```

## Native package development (no Jetson required)

Once a package exists in `ws/src/`, you can build and test it natively on
any machine with ROS 2 Humble installed (or in the `ros2-core` container),
without going through the Docker/systemd chain above:

```bash
source /opt/ros/humble/install/setup.bash

# Build one package
colcon build --merge-install --packages-select fcc_bridge

# Unit tests (gtest)
colcon test --packages-select fcc_bridge
colcon test-result --verbose

# Node-integration tests (launch_testing)
ros2 launch fcc_bridge fcc_bridge_test.launch.py

# System tests against PX4 SITL
ros2 launch mission_exec sitl_mission_test.launch.py
```

`mavlink-router` is a separate Meson/Ninja project, not part of the colcon
workspace:

```bash
cd mavlink-router && meson setup build && ninja -C build
```

Per-package YAML params live in `params/default.yaml` and `params/sitl.yaml`
— use the `sitl.yaml` variant when pointing a node at PX4 SITL instead of a
real Pixhawk.

## Repository map

```
docker/            Dockerfiles: ros2-core base + one dir per service image
config/            environment, per-node params, netplan, udev, sysctl, systemd units
ws/src/            colcon workspace: 8 first-party packages + vendored px4_msgs
scripts/           numbered setup/build/install scripts + <n>b/<n>c smoke tests
docs/diagrams/     28 PlantUML architecture diagrams (.puml + rendered .png)
docs/              this guide + ARCHITECTURE.md, API-REFERENCE.md, TESTING.md, DEPLOYMENT.md, CONTRIBUTING.md
```

The formal project documents (requirements, interfaces, design, hardware,
test plan, delivery backlog) live at the repo root — see the table in
`README.md` or `CLAUDE.md`'s Document Map.

## Where to go next

- Picking up a ticket? Check `PROJECT-PLAN-5-MONTH.md` for the backlog and
  `FCC-MISSION-CHANNEL-BACKLOG.md` / `OFFBOARD-MODE-SWITCHING-BACKLOG.md`
  for follow-on work.
- Touching the MC↔FCC or GCS↔MC wire format? Read `ICD-MC-001.md` first.
- Rendering or editing a diagram? The `.puml` sources under
  `docs/diagrams/` use PlantUML v1.2026-compatible syntax.
