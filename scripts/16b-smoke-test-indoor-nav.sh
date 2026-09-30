#!/usr/bin/env bash
# 16b-smoke-test-indoor-nav.sh — Verify the indoor-nav container
# (gnss_sim_node): image/entrypoint/params wiring, and that the systemd unit
# is start-on-demand only (a fake-GPS source must never join the boot set).
#
# Static checks (always run). Set LIVE_CHECK=1 to additionally run the real
# image (requires scripts/16-build-indoor-nav-image.sh on this host) and
# confirm gnss_sim_node registers and stays up. The live run points
# FCC_ETH_IP at 127.0.0.1 and never anchors, so nothing is sent anywhere.
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE="${REPO_ROOT}/docker/indoor-nav/Dockerfile"
ENTRYPOINT="${REPO_ROOT}/docker/indoor-nav/entrypoint.sh"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-indoor-nav.service"
PARAMS_YAML="${REPO_ROOT}/ws/src/indoor_nav/params/default.yaml"
INSTALL_SCRIPT="${REPO_ROOT}/scripts/02-install-systemd-units.sh"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/indoor-nav:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/indoor-nav:dev}"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS + 1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL + 1))
    fi
}

check "Dockerfile present"          test -f "${DOCKERFILE}"
check "entrypoint.sh present"       test -f "${ENTRYPOINT}"
check "entrypoint.sh is executable" test -x "${ENTRYPOINT}"
check "systemd unit present"        test -f "${UNIT}"
check "indoor_nav params file present" test -f "${PARAMS_YAML}"

check "entrypoint launches gnss_sim_node" \
    grep -q 'ros2 run indoor_nav gnss_sim_node' "${ENTRYPOINT}"
check "entrypoint references indoor_nav params file" \
    grep -q '/etc/drone-mc/params/indoor_nav.yaml' "${ENTRYPOINT}"
check "entrypoint relaxes nounset around setup.bash (COLCON_TRACE)" \
    grep -q 'set +u' "${ENTRYPOINT}"
check "entrypoint traps SIGTERM/SIGINT" \
    grep -qE 'trap terminate SIGTERM SIGINT' "${ENTRYPOINT}"

check "systemd unit runs ghcr.io/drone-mc/indoor-nav image" \
    grep -q 'ghcr.io/drone-mc/indoor-nav:' "${UNIT}"
check "systemd unit bind-mounts the shared colcon install" \
    grep -qE -- '--mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install' "${UNIT}"
check "systemd unit bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
check "systemd unit passes the per-drone environment (DRONE_ID, FCC_ETH_IP)" \
    grep -q -- '--env-file /etc/drone-mc/environment' "${UNIT}"
check "systemd unit requires fcc-bridge (all inputs come from it)" \
    grep -qE '^Requires=.*drone-mc-fcc-bridge\.service' "${UNIT}"
check "systemd unit restarts on failure" grep -q '^Restart=on-failure' "${UNIT}"
check "systemd unit is PartOf=drone-mc.target" grep -q '^PartOf=drone-mc.target' "${UNIT}"
check "systemd unit has no [Install]/WantedBy= (start on demand only)" \
    bash -c "! grep -qE '^(\[Install\]|WantedBy=)' '${UNIT}'"
check "02-install lists indoor-nav as on-demand (never enabled)" \
    grep -qE 'ON_DEMAND_UNITS=\(.*drone-mc-indoor-nav\.service' "${INSTALL_SCRIPT}"
check "02-install provisions /etc/drone-mc/params/indoor_nav.yaml" \
    grep -q 'indoor_nav/params/default.yaml:indoor_nav.yaml' "${INSTALL_SCRIPT}"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: exercising the real container"

    if ! docker image inspect "${IMAGE}" &>/dev/null; then
        echo "==> ERROR: ${IMAGE} not found — run scripts/16-build-indoor-nav-image.sh first" >&2
        exit 1
    fi

    trap '[[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true' EXIT

    # No /etc/drone-mc bind-mount: the image's baked-in params copy is used.
    # FCC_ETH_IP=127.0.0.1 so even a stray send can't reach a real FCC.
    cid="$(docker run -d --rm --network=host -e FCC_ETH_IP=127.0.0.1 -e DRONE_ID=1 "${IMAGE}")"

    node_registered() {
        docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && ros2 node list 2>/dev/null" \
            | grep -q '/gnss_sim_node'
    }

    ready=0
    for _ in $(seq 1 30); do
        if node_registered; then ready=1; break; fi
        sleep 1
    done
    check "gnss_sim_node registered in ros2 node list within 30s" \
        bash -c "[[ ${ready} -eq 1 ]]"
    check "container still running (node didn't crash on startup)" \
        bash -c "docker inspect -f '{{.State.Running}}' '${cid}' | grep -q true"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
