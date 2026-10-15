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

"${SCRIPT_DIR}/require-docker.sh"

# INFRA-006: see 08-build-fcc-bridge-image.sh's own comment on this —
# fail in seconds with a clear diagnosis instead of burning minutes
# inside a doomed `apt-get update`.
echo "==> Checking build-time network reachability"
if ! docker run --rm --network=host ubuntu:22.04 \
    bash -c 'getent hosts ports.ubuntu.com || getent hosts archive.ubuntu.com' &>/dev/null
then
    cat >&2 <<'NETERR'
==> ERROR: a "docker run --network=host ubuntu:22.04" container can't
    resolve ports.ubuntu.com/archive.ubuntu.com. This build needs real
    internet access for its apt-get install steps. Diagnose with:
      getent hosts ports.ubuntu.com                                    # host DNS
      sudo docker run --rm --network=host ubuntu:22.04 \
          getent hosts ports.ubuntu.com                                # container DNS via host networking
    If the host command also fails, this is a host-level network/DNS
    problem (bad /etc/resolv.conf, DHCP, firewall) -- not something any
    docker build flag can work around.
NETERR
    exit 1
fi

echo "==> Building ${IMAGE}:${IMAGE_TAG}"
# --network=host: see 08-build-fcc-bridge-image.sh's own comment on this —
# BuildKit RUN steps otherwise use a different network path (Docker's
# bridge + embedded DNS) than the daemon's own image pulls, which can
# fail to resolve apt mirrors even when the host's DNS works fine.
docker build \
    --network=host \
    --tag "${IMAGE}:${IMAGE_TAG}" \
    --tag "${IMAGE}:latest" \
    --file "${REPO_ROOT}/docker/zenoh-bridge/Dockerfile" \
    --build-arg "ZENOH_BRIDGE_DDS_VERSION=${ZENOH_BRIDGE_DDS_VERSION:-1.9.0}" \
    "${REPO_ROOT}/docker/zenoh-bridge"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "${IMAGE}:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "${IMAGE}:${IMAGE_TAG}"
echo "    Run 'docker push ${IMAGE}:${IMAGE_TAG}' to publish, or leave local for a single-Jetson deploy."
