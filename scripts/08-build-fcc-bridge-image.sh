#!/usr/bin/env bash
# 08-build-fcc-bridge-image.sh — Build the ghcr.io/drone-mc/fcc-bridge
# service image consumed by config/systemd/drone-mc-fcc-bridge.service.
# Companion smoke test: 08b-smoke-test-fcc-bridge.sh
#
# Requires scripts/04-install-ros2-core.sh to have been run first (builds
# the ros2-core base image this Dockerfile builds FROM).
#
# Usage:
#   ./08-build-fcc-bridge-image.sh              # tags fcc-bridge:dev and :latest
#   MC_VERSION=1.0.0 ./08-build-fcc-bridge-image.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
IMAGE_TAG="${MC_VERSION:-dev}"
ROS2_CORE_IMAGE="ros2-core:${IMAGE_TAG}"
IMAGE="ghcr.io/drone-mc/fcc-bridge"

if ! docker image inspect "${ROS2_CORE_IMAGE}" &>/dev/null; then
    echo "==> ERROR: ${ROS2_CORE_IMAGE} not found — run scripts/04-install-ros2-core.sh first" >&2
    exit 1
fi

# fcc_bridge depends on px4_msgs (SDD §3, not one of the 7 first-party
# packages, not vendored into the repo) — fetch it into the build context
# on the host, same as scripts/05-build-workspace.sh. Idempotent.
if ! command -v vcs >/dev/null 2>&1; then
    echo "==> vcstool not found — installing python3-vcstool"
    if command -v apt-get >/dev/null 2>&1; then
        sudo apt-get update -qq && sudo apt-get install -y python3-vcstool
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
vcs import --skip-existing "${REPO_ROOT}/ws/src" < "${REPO_ROOT}/ws/px4_msgs.repos"

# --skip-existing never moves an existing checkout, so a pin bump in
# ws/px4_msgs.repos silently leaves already-provisioned hosts on the old
# branch: DEP-009's release/1.14 -> release/1.17 bump never reached the
# orin-nx bench Jetson, whose fcc_bridge_node then decoded PX4 v1.17
# traffic with v1.14 message layouts (DDS matches on type name only, so
# nothing errors -- fields just misdecode). Enforce the pin here. The
# checkout is root-owned when this runs under sudo, hence safe.directory.
PX4_MSGS_DIR="${REPO_ROOT}/ws/src/px4_msgs"
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

# INFRA-006: fail in seconds with a clear diagnosis instead of burning
# minutes inside a doomed `apt-get update` (the xrce_agent_builder stage
# below needs real internet access). More apt retries wouldn't help a
# genuinely unreachable DNS path -- only a fast, explicit check does.
echo "==> Checking build-time network reachability"
if ! docker run --rm --network=host ubuntu:22.04 \
    bash -c 'getent hosts ports.ubuntu.com || getent hosts archive.ubuntu.com' &>/dev/null
then
    cat >&2 <<'NETERR'
==> ERROR: a "docker run --network=host ubuntu:22.04" container can't
    resolve ports.ubuntu.com/archive.ubuntu.com. This build needs real
    internet access for the xrce_agent_builder stage's apt-get install.
    Diagnose with:
      getent hosts ports.ubuntu.com                                    # host DNS
      sudo docker run --rm --network=host ubuntu:22.04 \
          getent hosts ports.ubuntu.com                                # container DNS via host networking
    If the host command also fails, this is a host-level network/DNS
    problem (bad /etc/resolv.conf, DHCP, firewall) -- not something any
    docker build flag can work around.
NETERR
    exit 1
fi

echo "==> Building ${IMAGE}:${IMAGE_TAG} (base: ${ROS2_CORE_IMAGE})"
# --network=host: RUN steps inside a BuildKit build otherwise use Docker's
# own bridge network + embedded DNS resolver, a different network path
# than the one the daemon itself uses to pull images — this can fail to
# resolve apt mirrors (e.g. the xrce_agent_builder stage's `apt-get
# update`) even when the host's own DNS works fine (systemd-resolved
# stubs, VPN split-tunneling, restrictive network policy). Same
# --network=host reasoning every docker run in this repo already uses,
# just applied to the build itself.
docker build \
    --network=host \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/fcc-bridge/Dockerfile" \
    --build-arg "ROS2_CORE_IMAGE=${ROS2_CORE_IMAGE}" \
    "${REPO_ROOT}"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
