#!/usr/bin/env bash
# 11-build-gcs-comms-image.sh — Build the ghcr.io/drone-mc/gcs-comms
# service image consumed by config/systemd/drone-mc-gcs-comms.service.
# Companion smoke test: 11b-smoke-test-gcs-comms.sh
#
# Requires scripts/04-install-ros2-core.sh to have been run first (builds
# the ros2-core base image this Dockerfile builds FROM).
#
# Usage:
#   ./11-build-gcs-comms-image.sh              # tags gcs-comms:dev and :latest
#   MC_VERSION=1.0.0 ./11-build-gcs-comms-image.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
IMAGE_TAG="${MC_VERSION:-dev}"
ROS2_CORE_IMAGE="ros2-core:${IMAGE_TAG}"
IMAGE="ghcr.io/drone-mc/gcs-comms"

if ! docker image inspect "${ROS2_CORE_IMAGE}" &>/dev/null; then
    echo "==> ERROR: ${ROS2_CORE_IMAGE} not found — run scripts/04-install-ros2-core.sh first" >&2
    exit 1
fi

echo "==> Building ${IMAGE}:${IMAGE_TAG} (base: ${ROS2_CORE_IMAGE})"
docker build \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/gcs-comms/Dockerfile" \
    --build-arg "ROS2_CORE_IMAGE=${ROS2_CORE_IMAGE}" \
    "${REPO_ROOT}"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
