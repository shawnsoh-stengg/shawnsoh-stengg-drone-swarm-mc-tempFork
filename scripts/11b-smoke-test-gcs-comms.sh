#!/usr/bin/env bash
# 11b-smoke-test-gcs-comms.sh — Verify the gcs-comms container (GCS-005):
# gcs_mavlink_node + MAVROS launch in order, both in `ros2 node list`, and
# the container restarts cleanly after a kill. Ref: SDD §13.2 smoke test
# pattern.
#
# Static checks (always run): Dockerfile/entrypoint content, no MAVROS
# build/link dependency crept into gcs_comms's own CMakeLists.txt/package.xml
# (SDD-MC-001.md §11.1's GCS-005 update: MAVROS is a second OS process, not
# a library gcs_mavlink_node links against), params files, systemd unit
# wiring.
#
# GCS-005 AC2 (QGroundControl on a real 192.168.10.10 network showing a
# HEARTBEAT within 5 s) needs a real QGC + FCC/SITL on a real network — that
# is HW-tier verification (TP-MC-001's UT/NI/ST/HW levels), not something an
# automated smoke test can exercise, and is deliberately not attempted here.
#
# Set LIVE_CHECK=1 to additionally build/run the actual image (requires
# scripts/11-build-gcs-comms-image.sh to have succeeded on this host, and
# network access for apt to fetch ros-humble-mavros): verifies AC1 (both
# nodes registered within 30 s) and AC3 (kill -> fresh restart re-establishes
# both nodes within 15 s).
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE="${REPO_ROOT}/docker/gcs-comms/Dockerfile"
ENTRYPOINT="${REPO_ROOT}/docker/gcs-comms/entrypoint.sh"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-gcs-comms.service"
CMAKELISTS="${REPO_ROOT}/ws/src/gcs_comms/CMakeLists.txt"
PACKAGE_XML="${REPO_ROOT}/ws/src/gcs_comms/package.xml"
GCS_COMMS_YAML="${REPO_ROOT}/ws/src/gcs_comms/params/default.yaml"
MAVROS_YAML="${REPO_ROOT}/ws/src/gcs_comms/params/mavros.yaml"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/gcs-comms:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/gcs-comms:dev}"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS + 1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL + 1))
    fi
}

check "Dockerfile present"        test -f "${DOCKERFILE}"
check "entrypoint.sh present"      test -f "${ENTRYPOINT}"
check "entrypoint.sh is executable" test -x "${ENTRYPOINT}"
check "systemd unit present"       test -f "${UNIT}"

check "Dockerfile installs ros-humble-mavros" grep -q 'ros-humble-mavros' "${DOCKERFILE}"

# SDD-MC-001.md §11.1's GCS-005 update: MAVROS runs as a second OS process
# in this container, not a library dependency of gcs_mavlink_node itself —
# ws/src/gcs_comms/CMakeLists.txt's own "no MAVROS, raw mavlink.h" decision
# must stay unchanged by this ticket.
check "gcs_comms CMakeLists.txt does not link/find MAVROS" \
    bash -c "! grep -qiE 'find_package\(\s*mavros|mavros::' '${CMAKELISTS}'"
check "gcs_comms package.xml does not depend on MAVROS" \
    bash -c "! grep -qi 'mavros' '${PACKAGE_XML}'"

# AC1 ("starts gcs_mavlink_node and MAVROS in order"): verify the order is
# in the entrypoint script itself, not just assumed.
gcs_line="$(grep -n 'ros2 run gcs_comms gcs_mavlink_node' "${ENTRYPOINT}" | head -1 | cut -d: -f1)"
mavros_line="$(grep -n 'ros2 run mavros mavros_node' "${ENTRYPOINT}" | head -1 | cut -d: -f1)"
check "entrypoint launches gcs_mavlink_node before mavros_node" \
    bash -c "[[ -n '${gcs_line}' && -n '${mavros_line}' && ${gcs_line:-0} -lt ${mavros_line:-0} ]]"
check "entrypoint waits for gcs_mavlink_node registration before starting mavros" \
    grep -q "grep -q '/gcs_mavlink_node'" "${ENTRYPOINT}"
check "entrypoint traps SIGTERM/SIGINT for both processes" \
    grep -qE "trap terminate SIGTERM SIGINT" "${ENTRYPOINT}"
check "entrypoint references gcs_comms params file" \
    grep -q '/etc/drone-mc/params/gcs_comms.yaml' "${ENTRYPOINT}"
check "entrypoint references mavros params file" \
    grep -q '/etc/drone-mc/params/mavros.yaml' "${ENTRYPOINT}"

check "mavros.yaml fcu_url matches Channel A's MAVROS endpoint (14553)" \
    grep -qE 'fcu_url: "udp://:14553@"' "${MAVROS_YAML}"
check "gcs_comms params file has gcs_mavlink_node section" \
    grep -q 'gcs_mavlink_node:' "${GCS_COMMS_YAML}"

check "systemd unit runs ghcr.io/drone-mc/gcs-comms image" \
    grep -q 'ghcr.io/drone-mc/gcs-comms:' "${UNIT}"
check "systemd unit bind-mounts the shared colcon install" \
    grep -qE -- '--mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install' "${UNIT}"
check "systemd unit bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
check "systemd unit restarts on failure" grep -q 'Restart=on-failure' "${UNIT}"
check "systemd unit orders after fcc-bridge and mavlink-router" \
    grep -qE 'After=.*drone-mc-fcc-bridge\.service.*drone-mc-mavlink-router\.service' "${UNIT}"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: exercising the real container"

    if ! docker image inspect "${IMAGE}" &>/dev/null; then
        echo "==> ERROR: ${IMAGE} not found — run scripts/11-build-gcs-comms-image.sh first" >&2
        exit 1
    fi

    WORKDIR="$(mktemp -d)"
    trap 'rm -rf "${WORKDIR}"; [[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true' EXIT

    # --network=host, no /etc/drone-mc bind-mount — falls back to the
    # image's own baked-in params/config copies, same rationale
    # 08b-smoke-test-fcc-bridge.sh's RUN array comment gives for why
    # --network=host matters (CycloneDDS sysctl floor is per-netns).
    start_container() {
        cid="$(docker run -d --rm --network=host --env DRONE_ID=1 "${IMAGE}")"
    }
    nodes_ready() {
        docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && ros2 node list" \
            2>/dev/null | grep -q '/gcs_mavlink_node' \
        && docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && ros2 node list" \
            2>/dev/null | grep -qi 'mavros'
        # Tolerant match on "mavros" rather than an exact node-name anchor —
        # MAVROS's default ROS 2 node name is upstream's own convention
        # (this package neither sets nor vendors it), so this check
        # confirms *a* mavros node is present without over-committing to an
        # exact string this repo doesn't control.
    }

    start_container
    echo "==> AC1: waiting up to 30s for both nodes to register"
    ac1_ok=0
    for _ in $(seq 1 30); do
        if nodes_ready; then ac1_ok=1; break; fi
        sleep 1
    done
    check "AC1: gcs_mavlink_node and mavros both in ros2 node list within 30s" \
        bash -c "[[ ${ac1_ok} -eq 1 ]]"

    echo "==> AC3: killing container, timing a fresh restart's re-registration"
    docker kill "${cid}" &>/dev/null || true
    docker wait "${cid}" &>/dev/null || true
    start_container
    ac3_ok=0
    for _ in $(seq 1 15); do
        if nodes_ready; then ac3_ok=1; break; fi
        sleep 1
    done
    check "AC3: both nodes re-registered within 15s of a fresh restart after kill" \
        bash -c "[[ ${ac3_ok} -eq 1 ]]"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
