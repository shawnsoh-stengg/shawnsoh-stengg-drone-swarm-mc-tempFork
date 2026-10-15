#!/usr/bin/env bash
# require-docker.sh — Fail fast, with the real reason, when Docker isn't usable.
# Called first by 04-install-ros2-core.sh, 05-build-workspace.sh and every
# NN-build-*-image.sh; also safe to run by hand (sudo ./scripts/require-docker.sh).
#
# Those scripts' first docker calls (INFRA-006's network pre-flight, the
# "ros2-core image present?" check) discard docker's own output, so a missing
# docker binary or a stopped daemon used to surface as a misleading "can't
# resolve ports.ubuntu.com" or "ros2-core:dev not found" error. Seen on
# orin-nx 2026-10-15: no Docker at all, reported as a DNS failure.
set -euo pipefail

if ! command -v docker &>/dev/null; then
    cat >&2 <<'NODOCKER'
==> ERROR: docker is not installed (or not on this shell's PATH).
    None of this repo's scripts install Docker; on a Jetson it normally
    comes with SDK Manager's JetPack install. To install it on JetPack 6
    (NVIDIA's documented steps, needs internet access):
      sudo apt update
      sudo apt install -y nvidia-container curl
      curl https://get.docker.com | sh
      sudo systemctl --now enable docker
      sudo nvidia-ctk runtime configure --runtime=docker
      sudo systemctl restart docker
NODOCKER
    if [[ -x /snap/bin/docker ]]; then
        echo "    NOTE: /snap/bin/docker exists but sudo's PATH doesn't include" \
             "/snap/bin -- a snap Docker also lacks the NVIDIA runtime; prefer the" \
             "install above." >&2
    fi
    exit 1
fi

if ! docker_err="$(docker info 2>&1 >/dev/null)"; then
    echo "==> ERROR: docker is installed but not usable: \`docker info\` failed with:" >&2
    sed 's/^/      /' <<<"${docker_err}" | head -5 >&2
    cat >&2 <<'NODAEMON'
    Daemon not running:  sudo systemctl --now enable docker
    Permission denied:   run this script with sudo (or add the user to the
                         docker group and log in again)
NODAEMON
    exit 1
fi
