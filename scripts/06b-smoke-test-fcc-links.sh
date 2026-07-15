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
# fcc-bridge image actually contains a MicroXRCEAgent binary.
#
# NOTE: that last check is expected to FAIL until FCC-008 (MicroXRCEAgent
# lifecycle) lands — see FCC-MISSION-CHANNEL-BACKLOG.md, which documents
# that FCC-008 "blocks any real HIL use of FCC-002 today". Neither
# docker/fcc-bridge/Dockerfile nor docker/ros2-core/Dockerfile installs
# the Agent binary as of this writing.
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
SOURCE="source /opt/ros/humble/install/setup.bash"

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
check "fcc-bridge image has MicroXRCEAgent binary" \
    docker run --rm "${IMAGE}" bash -c "${SOURCE} && command -v MicroXRCEAgent"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: verifying live HIF-001 link against attached Pixhawk 6X"

    command -v docker >/dev/null || { echo "==> ERROR: docker required for LIVE_CHECK" >&2; exit 1; }

    cid=""
    cleanup() { [[ -n "${cid}" ]] && docker rm -f "${cid}" &>/dev/null || true; }
    trap cleanup EXIT

    cid="$(docker run -d --rm --name mc-fcc-links-smoke \
        --device "${DEVICE}" \
        --network=host \
        "${IMAGE}" \
        bash -c "${SOURCE} && exec MicroXRCEAgent serial --dev ${DEVICE} --baudrate ${BAUD} --rts-cts")"

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
