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

# INFRA-006: see 08-build-fcc-bridge-image.sh's own comment on this —
# fail in seconds with a clear diagnosis instead of burning minutes
# inside a doomed `apt-get update` (this Dockerfile's ros2-apt-source
# curl fetch and its ros-humble-* apt-get install both need real
# internet access).
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

echo "==> Building ros2-core:${IMAGE_TAG}"
# --network=host: see 08-build-fcc-bridge-image.sh's own comment on this —
# BuildKit RUN steps otherwise use a different network path (Docker's
# bridge + embedded DNS) than the daemon's own image pulls, which can
# fail to resolve apt mirrors even when the host's DNS works fine.
docker build \
    --network=host \
    --tag "ros2-core:${IMAGE_TAG}" \
    --tag "ros2-core:latest" \
    --file "${REPO_ROOT}/docker/ros2-core/Dockerfile" \
    "${REPO_ROOT}/docker/ros2-core"

echo "==> Build complete"
docker image inspect --format "  ID:      {{.Id}}" "ros2-core:${IMAGE_TAG}"
docker image inspect --format "  Created: {{.Created}}" "ros2-core:${IMAGE_TAG}"
