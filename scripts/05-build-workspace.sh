#!/usr/bin/env bash
# 05-build-workspace.sh — Build the shared colcon workspace inside ros2-core.
# Companion smoke test: 05b-smoke-test-colcon.sh
#
# Runs colcon build inside the ros2-core container, writing the merged install
# tree to DRONE_MC_INSTALL_DIR on the host (default: /opt/drone-mc/install).
# The resulting directory is bind-mounted read-only into all service containers
# at runtime (SDD §12.1).
#
# Usage:
#   ./05-build-workspace.sh                        # Release build → /opt/drone-mc/install
#   CMAKE_BUILD_TYPE=Debug ./05-build-workspace.sh
#   DRONE_MC_INSTALL_DIR=/tmp/install ./05-build-workspace.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
INSTALL_DIR="${DRONE_MC_INSTALL_DIR:-/opt/drone-mc/install}"
IMAGE="${MC_VERSION:+ros2-core:${MC_VERSION}}"
IMAGE="${IMAGE:-ros2-core:latest}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

# Ensure host install dir exists and is writable by the current user.
if [[ ! -d "${INSTALL_DIR}" ]]; then
    echo "==> Creating ${INSTALL_DIR}"
    sudo mkdir -p "${INSTALL_DIR}"
    sudo chown "$(id -u):$(id -g)" "${INSTALL_DIR}"
fi

SRC_DIR="${REPO_ROOT}/ws/src"

# fcc_bridge (FCC-002) depends on px4_msgs, which is not one of the 7
# first-party packages (CLAUDE.md) and is not vendored into the repo —
# fetch/update it on the host before the read-only bind-mount below.
"${SCRIPT_DIR}/sync-px4-msgs.sh"

echo "==> Building workspace"
echo "    src:     ${SRC_DIR}"
echo "    install: ${INSTALL_DIR}"
echo "    image:   ${IMAGE}"
echo "    type:    ${BUILD_TYPE}"

# --network none: colcon build here is a pure local compile (no rosdep
# install, no FetchContent/ExternalProject in any current package's
# CMakeLists.txt), so it doesn't need connectivity — and some hosts'
# kernels lack the bridge driver's veth setup (e.g. a Jetson whose
# /lib/modules/<kernel>/ has no veth.ko), which --network=bridge (the
# implicit default) would otherwise require.
docker run --rm \
    --network none \
    --volume "${SRC_DIR}:/ws/src:ro" \
    --volume "${INSTALL_DIR}:/opt/drone-mc/install" \
    --workdir /ws \
    "${IMAGE}" \
    bash -c "
        set -eo pipefail
        source /opt/ros/humble/install/setup.bash
        set -u
        colcon build \
            --merge-install \
            --install-base /opt/drone-mc/install \
            --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
            --event-handlers console_cohesion+
    "

echo "==> Workspace installed to ${INSTALL_DIR}"
