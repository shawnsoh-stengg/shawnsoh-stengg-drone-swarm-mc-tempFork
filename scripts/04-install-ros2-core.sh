#!/usr/bin/env bash
# 04-install-ros2-core.sh — Build the ros2-core Docker base image.
# Companion smoke test: 04b-smoke-test-ros2.sh
#
# Usage:
#   ./04-install-ros2-core.sh              # tags ros2-core:dev and ros2-core:latest
#   MC_VERSION=1.0.0 ./04-install-ros2-core.sh
#
# On an x86 CI host, install QEMU first so the aarch64 layers execute:
#   sudo apt-get install -y qemu-user-static binfmt-support
#   docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
IMAGE_TAG="${MC_VERSION:-dev}"

echo "==> Building ros2-core:${IMAGE_TAG}"
docker build \
    --tag "ros2-core:${IMAGE_TAG}" \
    --tag "ros2-core:latest" \
    --file "${REPO_ROOT}/docker/ros2-core/Dockerfile" \
    "${REPO_ROOT}/docker/ros2-core"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "ros2-core:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "ros2-core:${IMAGE_TAG}"
