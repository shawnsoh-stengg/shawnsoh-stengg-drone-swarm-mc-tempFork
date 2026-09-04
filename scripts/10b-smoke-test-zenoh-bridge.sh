#!/usr/bin/env bash
# 10b-smoke-test-zenoh-bridge.sh — Verify the zenoh-bridge-dds image and
# config (cross-host DDS transport, replacing CycloneDDS's old eth0 unicast
# discovery — see docker/ros2-core/cyclonedds.xml, now lo-only).
# Ref: SDD §13.2 smoke test pattern.
#
# Static checks (always run): zenoh-bridge-dds.json5 declares peer mode,
# disables multicast scouting, and has at least one connect endpoint ·
# zenoh-bridge-dds binary is installed in the image · the systemd unit
# bind-mounts /etc/drone-mc read-only and sources /etc/drone-mc/environment ·
# scripts/02-install-systemd-units.sh provisions the config file.
#
# Set LIVE_CHECK=1 to additionally start the built image and confirm the
# container stays up for a few seconds without crashing — this is the
# specific check that would catch a glibc-compatibility mismatch between
# the vendor's prebuilt aarch64-gnu .deb and JetPack 6.2.2's glibc 2.35;
# confirmed working on real orin-nx hardware 2026-09-03, but still worth
# rerunning as a regression guard on new hardware/version bumps.
#
# If drone-mc-zenoh-bridge.service is already running, this stops it first
# and restarts it on exit — both it and this check's own container bind
# tcp/0.0.0.0:7447 with --network=host, so leaving the real service up
# produces a false "container not running" failure (address already in
# use), not a real crash. Needs root to stop/start the systemd unit.
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${REPO_ROOT}/config/zenoh-bridge-dds.json5"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-zenoh-bridge.service"
INSTALL_SCRIPT="${REPO_ROOT}/scripts/02-install-systemd-units.sh"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/zenoh-bridge:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/zenoh-bridge:latest}"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS + 1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL + 1))
    fi
}

check "zenoh-bridge-dds.json5 present"     test -f "${CONFIG}"
check "peer mode declared"                 grep -qF 'mode: "peer"' "${CONFIG}"
check "multicast scouting disabled"        grep -qF 'scouting: { multicast: { enabled: false } }' "${CONFIG}"
check "at least one connect endpoint"      grep -qE 'connect: \{ endpoints: \[.*"tcp/[0-9]' "${CONFIG}"
check "listen endpoint declared"           grep -qF 'listen: { endpoints:' "${CONFIG}"
check "forward-nothing default (allow [\"^\$\"])" grep -qF 'allow: ["^$"],' "${CONFIG}"

check "drone-mc-zenoh-bridge.service bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
check "drone-mc-zenoh-bridge.service sources /etc/drone-mc/environment" \
    grep -qE '^EnvironmentFile=/etc/drone-mc/environment$' "${UNIT}"
check "drone-mc-zenoh-bridge.service runs in be.slice" \
    grep -qE '^Slice=be\.slice$' "${UNIT}"

check "install script provisions zenoh-bridge-dds.json5" \
    grep -q 'zenoh-bridge-dds.json5' "${INSTALL_SCRIPT}"

check "image exists" docker image inspect "${IMAGE}"
check "zenoh-bridge-dds binary present in image" \
    docker run --rm --entrypoint /usr/bin/which "${IMAGE}" zenoh-bridge-dds

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: verifying the container starts and stays up"

    WORKDIR="$(mktemp -d)"
    SERVICE_WAS_ACTIVE=0
    if command -v systemctl &>/dev/null \
        && systemctl is-active --quiet drone-mc-zenoh-bridge.service 2>/dev/null
    then
        echo "==> drone-mc-zenoh-bridge.service is active — stopping it for" \
             "the duration of this check to avoid a tcp/7447 port collision"
        SERVICE_WAS_ACTIVE=1
        systemctl stop drone-mc-zenoh-bridge.service
    fi

    cleanup() {
        rm -rf "${WORKDIR}"
        [[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true
        if [[ "${SERVICE_WAS_ACTIVE}" -eq 1 ]]; then
            echo "==> restarting drone-mc-zenoh-bridge.service"
            systemctl start drone-mc-zenoh-bridge.service
        fi
    }
    trap cleanup EXIT

    # Idle config: no reachable peers, matching the SITL/dev guidance in
    # zenoh-bridge-dds.json5's header comment.
    sed -e 's#connect: { endpoints: \[.*\] }#connect: { endpoints: [] }#' \
        "${CONFIG}" > "${WORKDIR}/zenoh-bridge-dds.json5"

    cid="$(docker run -d --rm --network=host \
        --mount "type=bind,src=${WORKDIR},dst=/etc/drone-mc,readonly" \
        "${IMAGE}")"
    sleep 3
    check "zenoh-bridge-dds container is still running after 3s" \
        docker inspect -f '{{.State.Running}}' "${cid}"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
