#!/usr/bin/env bash
# 09-build-mavlink-router-image.sh — Build the ghcr.io/drone-mc/mavlink-router
# service image consumed by config/systemd/drone-mc-mavlink-router.service.
# Companion smoke test: 09b-smoke-test-routing.sh
#
# Standalone image, not FROM ros2-core — does not require
# scripts/04-install-ros2-core.sh to have run first.
#
# Usage:
#   ./09-build-mavlink-router-image.sh              # tags mavlink-router:dev and :latest
#   MC_VERSION=1.0.0 ./09-build-mavlink-router-image.sh
#   MAVLINK_ROUTER_COMMIT=<sha> ./09-build-mavlink-router-image.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
IMAGE_TAG="${MC_VERSION:-dev}"
IMAGE="ghcr.io/drone-mc/mavlink-router"

echo "==> Building ${IMAGE}:${IMAGE_TAG}"
docker build \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/mavlink-router/Dockerfile" \
    --build-arg "MAVLINK_ROUTER_COMMIT=${MAVLINK_ROUTER_COMMIT:-42529d55b665e0a9a29e424e186f514c56c2e5b5}" \
    "${REPO_ROOT}/docker/mavlink-router"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
