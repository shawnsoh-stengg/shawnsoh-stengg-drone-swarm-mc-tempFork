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

# INFRA-006: see 08-build-fcc-bridge-image.sh's own comment on this —
# fail in seconds with a clear diagnosis instead of burning minutes inside
# a doomed apt-get update/rosdep update/vcs import (DEP-009 follow-up,
# 2026-09-15: mavros/libmavconn are now a from-source build stage, needing
# github.com as well as the apt mirrors — ros-humble-mavros's own apt
# binary was pulled upstream, see docker/gcs-comms/Dockerfile's comment).
echo "==> Checking build-time network reachability"
if ! docker run --rm --network=host ubuntu:22.04 \
    bash -c 'getent hosts ports.ubuntu.com || getent hosts archive.ubuntu.com' &>/dev/null
then
    cat >&2 <<'NETERR'
==> ERROR: a "docker run --network=host ubuntu:22.04" container can't
    resolve ports.ubuntu.com/archive.ubuntu.com. This build needs real
    internet access for its apt-get installs and rosdep-resolved
    dependencies. Diagnose with:
      getent hosts ports.ubuntu.com                                    # host DNS
      sudo docker run --rm --network=host ubuntu:22.04 \
          getent hosts ports.ubuntu.com                                # container DNS via host networking
    If the host command also fails, this is a host-level network/DNS
    problem (bad /etc/resolv.conf, DHCP, firewall) -- not something any
    docker build flag can work around.
NETERR
    exit 1
fi
if ! docker run --rm --network=host ubuntu:22.04 \
    bash -c 'getent hosts github.com' &>/dev/null
then
    cat >&2 <<'NETERR'
==> ERROR: a "docker run --network=host ubuntu:22.04" container can't
    resolve github.com. This build needs it for the mavros/libmavconn
    from-source build stage's `vcs import` (docker/gcs-comms/mavros.repos)
    — the apt mirrors alone aren't enough. Diagnose the same way as the
    apt-mirror check above, substituting github.com.
NETERR
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
    --file "${REPO_ROOT}/docker/gcs-comms/Dockerfile" \
    --build-arg "ROS2_CORE_IMAGE=${ROS2_CORE_IMAGE}" \
    "${REPO_ROOT}"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
