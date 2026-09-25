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
# `vcs` (python3-vcstool). Idempotent: skips repos already checked out,
# but re-points an existing checkout at the pinned branch (below).
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

# --skip-existing never moves an existing checkout, so a pin bump in
# ws/px4_msgs.repos silently leaves already-provisioned hosts on the old
# branch: DEP-009's release/1.14 -> release/1.17 bump never reached the
# orin-nx bench Jetson, whose fcc_bridge_node then decoded PX4 v1.17
# traffic with v1.14 message layouts (DDS matches on type name only, so
# nothing errors -- fields just misdecode). Enforce the pin here. The
# checkout is root-owned when this runs under sudo, hence safe.directory.
PX4_MSGS_DIR="${SRC_DIR}/px4_msgs"
PX4_MSGS_PIN="$(awk '$1 == "version:" {print $2}' "${REPO_ROOT}/ws/px4_msgs.repos")"
px4_git() { git -c safe.directory="${PX4_MSGS_DIR}" -C "${PX4_MSGS_DIR}" "$@"; }
PX4_MSGS_HEAD="$(px4_git rev-parse --abbrev-ref HEAD)"
if [[ "${PX4_MSGS_HEAD}" != "${PX4_MSGS_PIN}" ]]; then
    if [[ -n "$(px4_git status --porcelain)" ]]; then
        echo "==> ERROR: ${PX4_MSGS_DIR} is on '${PX4_MSGS_HEAD}' but ws/px4_msgs.repos pins '${PX4_MSGS_PIN}', and it has local changes — resolve by hand" >&2
        exit 1
    fi
    echo "==> px4_msgs is on '${PX4_MSGS_HEAD}', ws/px4_msgs.repos pins '${PX4_MSGS_PIN}' — switching"
    px4_git fetch origin "+refs/heads/${PX4_MSGS_PIN}:refs/remotes/origin/${PX4_MSGS_PIN}"
    px4_git checkout -B "${PX4_MSGS_PIN}" --track "origin/${PX4_MSGS_PIN}"
fi

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
