#!/usr/bin/env bash
# 06c-smoke-test-xrce-agent.sh — Verify FCC-008: the fcc-bridge container's
# entrypoint launches MicroXRCEAgent with the transport selected by
# FCC_TRANSPORT, alongside fcc_bridge_node, and both share the container's
# shutdown lifecycle.
# Ref: HID-MC-001 §4.3, FCC-MISSION-CHANNEL-BACKLOG.md FCC-008.
#
# Runs entirely against the ethernet transport with loopback overrides — no
# real Pixhawk required (mirrors
# 08b-smoke-test-fcc-bridge.sh's FCC_ETH_IP=127.0.0.1 pattern). The
# serial-transport branch is exercised on real hardware by
# 06b-smoke-test-fcc-links.sh's LIVE_CHECK path instead, since that needs
# an actual /dev/ttyTHS1.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
PASS=0; FAIL=0
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/fcc-bridge:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/fcc-bridge:latest}"
# Both the base ROS 2 Humble underlay and the /opt/drone-mc/install
# workspace overlay are needed — mc_msgs' typesupport lives in the latter
# (COPY --from=builder /opt/drone-mc/install, docker/fcc-bridge/Dockerfile),
# not in stock Humble. Sourcing only the underlay makes any ros2 CLI tool
# that needs to decode an mc_msgs/px4_msgs type fail with "The message
# type '...' is invalid" — confirmed on real hardware — even though the
# topic itself is publishing fine.
SOURCE="source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash"
AGENT_PORT=18888

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS+1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL+1))
    fi
}

check "entrypoint.sh selects serial transport command" \
    grep -q 'MicroXRCEAgent serial --dev' "${REPO_ROOT}/docker/fcc-bridge/entrypoint.sh"
check "entrypoint.sh selects ethernet transport command" \
    grep -q 'MicroXRCEAgent udp4 --port' "${REPO_ROOT}/docker/fcc-bridge/entrypoint.sh"

command -v docker >/dev/null || { echo "==> ERROR: docker required" >&2; exit 1; }

# If drone-mc-fcc-bridge.service is already running, this smoke-test
# container's own fcc_bridge_node collides with the real one on the DDS
# domain: both use --network=host, and CycloneDDS's same-host discovery
# merges them, producing a duplicate fcc_bridge_node node name and
# cross-talk between the two (confirmed in practice — this is what broke
# this exact check on real hardware once before). Stop it for the duration
# of this check, restart on exit — same precedent as
# 10b-smoke-test-zenoh-bridge.sh. Needs root to stop/start the systemd unit.
SERVICE_WAS_ACTIVE=0
if command -v systemctl &>/dev/null \
    && systemctl is-active --quiet drone-mc-fcc-bridge.service 2>/dev/null
then
    echo "==> drone-mc-fcc-bridge.service is active — stopping it for" \
         "the duration of this check to avoid a DDS-domain collision"
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

# --network=host matches every drone-mc-*.service unit (config/systemd/) —
# without it this container gets its own netns and misses the
# net.core.rmem_max/wmem_max floor 01-install-os.sh sets on the host's
# default netns (those sysctls are per-netns), so fcc_bridge_node's
# rmw_cyclonedds_cpp domain creation aborts on startup.
cid="$(docker run -d --rm --network=host --name mc-xrce-agent-smoke \
    -e FCC_TRANSPORT=ethernet \
    -e FCC_ETH_PORT="${AGENT_PORT}" \
    -e FCC_ETH_IP=127.0.0.1 \
    -e FCC_MISSION_PORT=15558 \
    "${IMAGE}")"

check "container is running" docker inspect -f '{{.State.Running}}' "${cid}"

# Give the entrypoint a moment to spawn both processes.
sleep 2

check "MicroXRCEAgent launched with FCC_ETH_PORT (ethernet transport)" \
    docker exec "${cid}" bash -c "pgrep -fa 'MicroXRCEAgent udp4 --port ${AGENT_PORT}'"
check "fcc_bridge_node publishes /mc/fcc/vehicle_state alongside the agent" \
    docker exec "${cid}" bash -c \
        "${SOURCE} && timeout 5 ros2 topic hz /mc/fcc/vehicle_state 2>/dev/null | grep -q 'average rate'"

start_ts=$(date +%s)
docker stop --time 10 "${cid}" &>/dev/null || true
stop_elapsed=$(( $(date +%s) - start_ts ))
check "container stops cleanly (both processes) within 10s of SIGTERM" \
    bash -c "[[ ${stop_elapsed} -lt 10 ]]"
cid=""

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
