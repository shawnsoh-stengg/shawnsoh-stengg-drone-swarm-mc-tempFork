#!/usr/bin/env bash
# 10-build-zenoh-bridge-image.sh — Build the ghcr.io/drone-mc/zenoh-bridge
# service image consumed by config/systemd/drone-mc-zenoh-bridge.service.
# Companion smoke test: 10b-smoke-test-zenoh-bridge.sh
#
# Standalone image, not FROM ros2-core — does not require
# scripts/04-install-ros2-core.sh to have run first.
#
# Usage:
#   ./10-build-zenoh-bridge-image.sh                    # tags zenoh-bridge:dev and :latest
#   MC_VERSION=1.0.0 ./10-build-zenoh-bridge-image.sh
#   ZENOH_BRIDGE_DDS_VERSION=1.9.0 ./10-build-zenoh-bridge-image.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
IMAGE_TAG="${MC_VERSION:-dev}"
IMAGE="ghcr.io/drone-mc/zenoh-bridge"

echo "==> Building ${IMAGE}:${IMAGE_TAG}"
# --platform linux/arm64: the Dockerfile's FROM ubuntu:22.04 is multi-arch, so
# without this it silently builds for the host's native arch on an x86 dev/CI
# box instead of the Jetson (aarch64) target the vendor .deb below is for. On
# an x86 host, install QEMU first (see docker/ros2-core/Dockerfile's header
# comment).
docker build \
    --platform linux/arm64 \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/zenoh-bridge/Dockerfile" \
    --build-arg "ZENOH_BRIDGE_DDS_VERSION=${ZENOH_BRIDGE_DDS_VERSION:-1.9.0}" \
    "${REPO_ROOT}/docker/zenoh-bridge"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
