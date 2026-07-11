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

echo "==> Building ${IMAGE}:${IMAGE_TAG} (base: ${ROS2_CORE_IMAGE})"
docker build \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/fcc-bridge/Dockerfile" \
    --build-arg "ROS2_CORE_IMAGE=${ROS2_CORE_IMAGE}" \
    "${REPO_ROOT}"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
