#!/usr/bin/env bash
# 04c-smoke-test-dds.sh — Verify the CycloneDDS unicast-discovery config
# (INFRA-004). Named 04c, not 04b, since 04b-smoke-test-ros2.sh already
# covers the base ros2-core image build.
#
# Static checks (always run): cyclonedds.xml pins eth0+lo, disables
# multicast, and declares a unicast Peers list · every drone-mc-*.service
# bind-mounts /etc/drone-mc read-only · CYCLONEDDS_URI reaches every unit
# via /etc/drone-mc/environment.
#
# Set LIVE_CHECK=1 to additionally start two ros2-core containers on the
# host network and confirm cross-container topic discovery completes
# within 5 s (requires the ros2-core image to already exist on this host).
set -euo pipefail

PASS=0; FAIL=0
IMAGE="${MC_VERSION:+ros2-core:${MC_VERSION}}"
IMAGE="${IMAGE:-ros2-core:latest}"
CDDS_XML="$(dirname "$0")/../docker/ros2-core/cyclonedds.xml"
UNIT_DIR="$(dirname "$0")/../config/systemd"

SERVICES=(
    drone-mc-health-monitor.service
    drone-mc-mavlink-router.service
    drone-mc-fcc-bridge.service
    drone-mc-gcs-comms.service
    drone-mc-swarm-coord.service
    drone-mc-mission-exec.service
    drone-mc-camera.service
    drone-mc-vision.service
)

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS + 1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL + 1))
    fi
}

check "cyclonedds.xml present"            test -f "${CDDS_XML}"
check "eth0 interface pinned"             grep -qE '<NetworkInterface name="eth0"' "${CDDS_XML}"
check "lo interface pinned"               grep -qE '<NetworkInterface name="lo"' "${CDDS_XML}"
check "eth0 multicast disabled"           grep -qE '<NetworkInterface name="eth0"[^>]*multicast="false"' "${CDDS_XML}"
check "lo multicast disabled"             grep -qE '<NetworkInterface name="lo"[^>]*multicast="false"' "${CDDS_XML}"
check "AllowMulticast false"              grep -qE '<AllowMulticast>false</AllowMulticast>' "${CDDS_XML}"
check "unicast Peers list present"        grep -qE '<Peers>' "${CDDS_XML}"
check "at least one swarm peer address"   grep -qE '<Peer address="192\.168\.10\.[0-9]+"/>' "${CDDS_XML}"

for unit in "${SERVICES[@]}"; do
    check "${unit} bind-mounts /etc/drone-mc read-only" \
        grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT_DIR}/${unit}"
    check "${unit} sources CYCLONEDDS_URI via /etc/drone-mc/environment" \
        grep -qE '^EnvironmentFile=/etc/drone-mc/environment$' "${UNIT_DIR}/${unit}"
done

check "CYCLONEDDS_URI declared in config/environment" \
    grep -qE '^CYCLONEDDS_URI=/etc/drone-mc/cyclonedds.xml$' "$(dirname "$0")/../config/environment"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: verifying cross-container discovery within 5s"
    docker run -d --rm --name mc-dds-smoke-talker --network=host \
        --mount type=bind,src="$(cd "$(dirname "${CDDS_XML}")" && pwd)",dst=/etc/drone-mc,readonly \
        -e CYCLONEDDS_URI=/etc/drone-mc/cyclonedds.xml -e ROS_DOMAIN_ID=0 \
        "${IMAGE}" bash -c \
        "source /opt/ros/humble/setup.bash && ros2 topic pub -r 10 /mc/smoke_test std_msgs/msg/String 'data: smoke'" \
        >/dev/null

    deadline=$((SECONDS + 5))
    found=0
    while [[ ${SECONDS} -lt ${deadline} ]]; do
        if docker run --rm --network=host \
            --mount type=bind,src="$(cd "$(dirname "${CDDS_XML}")" && pwd)",dst=/etc/drone-mc,readonly \
            -e CYCLONEDDS_URI=/etc/drone-mc/cyclonedds.xml -e ROS_DOMAIN_ID=0 \
            "${IMAGE}" bash -c \
            "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null | grep -q '/mc/smoke_test'; then
            found=1
            break
        fi
        sleep 1
    done
    check "/mc/smoke_test discovered within 5s" bash -c "[[ ${found} -eq 1 ]]"

    docker stop mc-dds-smoke-talker >/dev/null 2>&1 || true
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
