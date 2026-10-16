#!/usr/bin/env bash
# 14-build-health-monitor-image.sh — Build the ghcr.io/drone-mc/health-monitor
# service image consumed by config/systemd/drone-mc-health-monitor.service.
# Companion smoke test: 14b-smoke-test-health.sh
#
# Requires scripts/04-install-ros2-core.sh to have been run first (builds
# the ros2-core base image this Dockerfile builds FROM).
#
# Usage:
#   ./14-build-health-monitor-image.sh              # tags health-monitor:dev and :latest
#   MC_VERSION=1.0.0 ./14-build-health-monitor-image.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
IMAGE_TAG="${MC_VERSION:-dev}"
ROS2_CORE_IMAGE="ros2-core:${IMAGE_TAG}"
IMAGE="ghcr.io/drone-mc/health-monitor"

"${SCRIPT_DIR}/require-docker.sh"

if ! docker image inspect "${ROS2_CORE_IMAGE}" &>/dev/null; then
    echo "==> ERROR: ${ROS2_CORE_IMAGE} not found — run scripts/04-install-ros2-core.sh first" >&2
    exit 1
fi

echo "==> Building ${IMAGE}:${IMAGE_TAG} (base: ${ROS2_CORE_IMAGE})"
# --network=host: see 08-build-fcc-bridge-image.sh's own comment on this —
# BuildKit RUN steps otherwise use a different network path (Docker's
# bridge + embedded DNS) than the daemon's own image pulls, which can
# fail to resolve apt mirrors even when the host's DNS works fine.
docker build \
    --network=host \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/health-monitor/Dockerfile" \
    --build-arg "ROS2_CORE_IMAGE=${ROS2_CORE_IMAGE}" \
    "${REPO_ROOT}"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
