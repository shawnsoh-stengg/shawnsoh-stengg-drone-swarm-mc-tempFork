#!/usr/bin/env bash
# 06b-smoke-test-fcc-links.sh — Verify HIF-001 (MC<->FCC serial link) on
# hardware (external tracker ID: FCC-006 "Verify HIF-001 (ttyTHS1) on
# hardware" — unrelated to this repo's own FCC-006, the mission-ack
# propagation ticket in FCC-MISSION-CHANNEL-BACKLOG.md).
# Ref: HID-MC-001 §3.4 (Link Verification), ICD-MC-001 §3 (IFD-001).
#
# Static checks (always run, no Pixhawk required): config/environment
# pins HIF-001's documented device/baud · /dev/ttyTHS1 present with
# dialout permissions on this host (mirrors 01b-smoke-test-os.sh so this
# script is self-contained when run standalone on the DUT) · the
# fcc-bridge image actually contains a MicroXRCEAgent binary (built from
# source in docker/fcc-bridge/Dockerfile's xrce_agent_builder stage,
# FCC-008).
#
# See scripts/06c-smoke-test-xrce-agent.sh for FCC-008's own transport-
# selection checks (does the container's entrypoint pick the right
# MicroXRCEAgent invocation for FCC_TRANSPORT, do both processes run
# together) — this script stays focused on HIF-001 link verification.
#
# Set LIVE_CHECK=1 to additionally start the Micro XRCE-DDS Agent against
# a real Pixhawk 6X wired via HIF-001 and assert /fmu/out/vehicle_status
# publishes at >=5 Hz (HIF-001 acceptance criterion) and
# /fmu/out/vehicle_local_position at >=10 Hz (HID §3.4). Requires: this
# script running on the Jetson DUT with a powered Pixhawk 6X on ttyTHS1,
# UXRCE_DDS_CFG=102 and UXRCE_DDS_BAUD=921600 already set on the FCC
# (HID §3.3), and the fcc-bridge image built locally
# (scripts/08-build-fcc-bridge-image.sh) with a MicroXRCEAgent binary.
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${REPO_ROOT}/config/environment"
DEVICE="${FCC_SERIAL_DEVICE:-/dev/ttyTHS1}"
BAUD="${FCC_SERIAL_BAUD:-921600}"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/fcc-bridge:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/fcc-bridge:latest}"
# Both the base ROS 2 Humble underlay and the /opt/drone-mc/install
# workspace overlay are needed for the LIVE_CHECK rate checks below —
# px4_msgs' typesupport lives in the latter (COPY --from=builder
# /opt/drone-mc/install, docker/fcc-bridge/Dockerfile), not in stock
# Humble. Sourcing only the underlay makes `ros2 topic hz` on
# /fmu/out/vehicle_status etc. fail to decode the message type — confirmed
# on real hardware for the analogous mc_msgs case in
# 06c-smoke-test-xrce-agent.sh.
SOURCE="source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS+1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL+1))
    fi
}

check "config/environment: FCC_SERIAL_DEVICE is /dev/ttyTHS1 (HIF-001)" \
    grep -q '^FCC_SERIAL_DEVICE=/dev/ttyTHS1$' "${ENV_FILE}"
check "config/environment: FCC_SERIAL_BAUD is 921600 (HIF-001)" \
    grep -q '^FCC_SERIAL_BAUD=921600$' "${ENV_FILE}"
check "${DEVICE} exists on this host" test -e "${DEVICE}"
check "${DEVICE} group is dialout" \
    bash -c "[[ \"\$(stat -c %G '${DEVICE}' 2>/dev/null)\" == dialout ]]"
# --network none: this check never needs connectivity, only the bridge
# driver's veth setup, which some hosts' kernels lack (e.g. a Jetson
# whose /lib/modules/<kernel>/ has no veth.ko) — real drone-mc containers
# never hit this since they all run with --network=host, as the
# LIVE_CHECK section below already does.
# --entrypoint bash: the image's own ENTRYPOINT is entrypoint.sh, which
# under set -u reads FCC_SERIAL_DEVICE immediately (FCC_TRANSPORT defaults
# to "serial") — without overriding it, "bash -c ..." is appended as
# *arguments* to entrypoint.sh rather than replacing it, so it dies on an
# unbound variable before this check's own command ever runs.
check "fcc-bridge image has MicroXRCEAgent binary" \
    docker run --rm --network none --entrypoint bash "${IMAGE}" -c "${SOURCE} && command -v MicroXRCEAgent"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: verifying live HIF-001 link against attached Pixhawk 6X"

    command -v docker >/dev/null || { echo "==> ERROR: docker required for LIVE_CHECK" >&2; exit 1; }

    # If drone-mc-fcc-bridge.service is already running, this smoke-test
    # container would fight it over the same real /dev/ttyTHS1 device *and*
    # collide on the DDS domain (both use --network=host; CycloneDDS's
    # same-host discovery merges them, producing a duplicate fcc_bridge_node
    # and cross-talk between the two — see 10b-smoke-test-zenoh-bridge.sh's
    # own precedent for this exact class of collision). Stop it for the
    # duration of this check, restart on exit. Needs root to stop/start the
    # systemd unit.
    SERVICE_WAS_ACTIVE=0
    if command -v systemctl &>/dev/null \
        && systemctl is-active --quiet drone-mc-fcc-bridge.service 2>/dev/null
    then
        echo "==> drone-mc-fcc-bridge.service is active — stopping it for" \
             "the duration of this check to avoid a /dev/ttyTHS1 + DDS collision"
        SERVICE_WAS_ACTIVE=1
        systemctl stop drone-mc-fcc-bridge.service
    fi

    cid=""
    cleanup() {
        [[ -n "${cid}" ]] && docker rm -f "${cid}" &>/dev/null || true
        if [[ "${SERVICE_WAS_ACTIVE}" -eq 1 ]]; then
            echo "==> restarting drone-mc-fcc-bridge.service"
            systemctl start drone-mc-fcc-bridge.service
        fi
    }
    trap cleanup EXIT

    # --entrypoint bash: same reasoning as the static check above — without
    # it, "-c ..." is appended as arguments to the image's own entrypoint.sh,
    # which crashes on an unbound FCC_SERIAL_DEVICE/FCC_SERIAL_BAUD before
    # MicroXRCEAgent ever launches (neither is set as a container env var
    # here — ${DEVICE}/${BAUD} are this script's own shell variables, not
    # exported into the container).
    cid="$(docker run -d --rm --name mc-fcc-links-smoke \
        --device "${DEVICE}" \
        --network=host \
        --entrypoint bash \
        "${IMAGE}" \
        -c "${SOURCE} && exec MicroXRCEAgent serial --dev ${DEVICE} --baudrate ${BAUD} --rts-cts")"

    check "MicroXRCEAgent container is running" \
        docker inspect -f '{{.State.Running}}' "${cid}"

    # Allow the Agent to establish the DDS session with the FCC's embedded
    # uXRCE-DDS client before sampling topic rates.
    sleep 2

    rate_status="$(docker exec "${cid}" bash -c \
        "${SOURCE} && timeout 6 ros2 topic hz /fmu/out/vehicle_status 2>/dev/null" \
        | grep -oP 'average rate: \K[0-9.]+' | tail -1 || true)"
    rate_localpos="$(docker exec "${cid}" bash -c \
        "${SOURCE} && timeout 6 ros2 topic hz /fmu/out/vehicle_local_position 2>/dev/null" \
        | grep -oP 'average rate: \K[0-9.]+' | tail -1 || true)"

    check "/fmu/out/vehicle_status >= 5 Hz (HIF-001 AC)" \
        bash -c "[[ -n '${rate_status}' ]] && awk -v r='${rate_status}' 'BEGIN{exit !(r >= 5.0)}'"
    check "/fmu/out/vehicle_local_position >= 10 Hz (HID §3.4)" \
        bash -c "[[ -n '${rate_localpos}' ]] && awk -v r='${rate_localpos}' 'BEGIN{exit !(r >= 10.0)}'"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
