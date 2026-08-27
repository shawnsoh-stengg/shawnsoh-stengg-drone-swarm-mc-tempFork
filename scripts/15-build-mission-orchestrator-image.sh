#!/usr/bin/env bash
# 15-build-mission-orchestrator-image.sh — Build the
# ghcr.io/drone-mc/mission-orchestrator service image consumed by
# config/systemd/drone-mc-mission-orchestrator.service (GCS-006).
# Companion smoke test: 15b-smoke-test-mission-orchestrator.sh
#
# Requires scripts/04-install-ros2-core.sh to have been run first (builds
# the ros2-core base image this Dockerfile builds FROM).
#
# Usage:
#   ./15-build-mission-orchestrator-image.sh              # tags mission-orchestrator:dev and :latest
#   MC_VERSION=1.0.0 ./15-build-mission-orchestrator-image.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
IMAGE_TAG="${MC_VERSION:-dev}"
ROS2_CORE_IMAGE="ros2-core:${IMAGE_TAG}"
IMAGE="ghcr.io/drone-mc/mission-orchestrator"

if ! docker image inspect "${ROS2_CORE_IMAGE}" &>/dev/null; then
    echo "==> ERROR: ${ROS2_CORE_IMAGE} not found — run scripts/04-install-ros2-core.sh first" >&2
    exit 1
fi

echo "==> Building ${IMAGE}:${IMAGE_TAG} (base: ${ROS2_CORE_IMAGE})"
# --network=host: see 08-build-fcc-bridge-image.sh's own comment on this —
# BuildKit RUN steps otherwise use a different network path (Docker's
# bridge + embedded DNS) than the daemon's own image pulls, which can
# fail to resolve apt mirrors even when the host's DNS works fine. No
# apt-get install happens in this particular Dockerfile (nlohmann-json3-dev
# already lives in ros2-core), but colcon's own build-time dependency
# resolution can still reach out, so the flag stays for consistency with
# every other build script here.
docker build \
    --network=host \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/mission-orchestrator/Dockerfile" \
    --build-arg "ROS2_CORE_IMAGE=${ROS2_CORE_IMAGE}" \
    "${REPO_ROOT}"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
