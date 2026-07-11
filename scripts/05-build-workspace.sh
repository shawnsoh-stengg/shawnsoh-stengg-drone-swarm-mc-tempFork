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
# fetch it on the host before the read-only bind-mount below. Requires
# `vcs` (python3-vcstool). Idempotent: skips repos already checked out.
if ! command -v vcs >/dev/null 2>&1; then
    echo "==> vcstool not found — installing python3-vcstool"
    # python3-vcstool ships from packages.ros.org, not stock Ubuntu universe —
    # the bare Jetson host (unlike the ros2-core container) has no ROS apt
    # source configured, so apt-get here routinely 404s. Fall back to pip3
    # rather than treating an apt miss as fatal.
    if command -v apt-get >/dev/null 2>&1 && sudo apt-get update -qq && sudo apt-get install -y python3-vcstool; then
        :
    elif command -v pip3 >/dev/null 2>&1; then
        pip3 install --user vcstool
    else
        echo "==> ERROR: could not install vcstool automatically (no apt-get or pip3 found); install manually: pip install vcstool / apt install python3-vcstool" >&2
        exit 1
    fi
fi
if ! command -v vcs >/dev/null 2>&1; then
    echo "==> ERROR: vcstool install attempted but 'vcs' still not on PATH (if installed via pip3 --user, ensure ~/.local/bin is in PATH)" >&2
    exit 1
fi
echo "==> Fetching external ROS 2 message dependencies (ws/px4_msgs.repos)"
vcs import --skip-existing "${SRC_DIR}" < "${REPO_ROOT}/ws/px4_msgs.repos"

echo "==> Building workspace"
echo "    src:     ${SRC_DIR}"
echo "    install: ${INSTALL_DIR}"
echo "    image:   ${IMAGE}"
echo "    type:    ${BUILD_TYPE}"

docker run --rm \
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
