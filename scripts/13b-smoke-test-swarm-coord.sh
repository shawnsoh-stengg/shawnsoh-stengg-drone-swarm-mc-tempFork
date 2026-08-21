#!/usr/bin/env bash
# 13b-smoke-test-swarm-coord.sh — Verify the swarm-coord container (DEP-001):
# swarm_coordinator_node + formation_controller_node both launched, params/
# systemd unit wiring correct.
#
# Static checks (always run): Dockerfile/entrypoint content, both nodes
# launched, params/systemd unit wiring.
#
# Unlike 12b-smoke-test-mission-exec.sh, there's no readiness topic to wait
# on here — both nodes are still §2.4/§5/§6 scaffolds (SwarmCoordinatorNode/
# FormationControllerNode just log a startup line, no topics/timers yet), so
# LIVE_CHECK=1 below only verifies both nodes actually register in
# `ros2 node list`, not any application-level readiness signal.
#
# Set LIVE_CHECK=1 to additionally build/run the actual image (requires
# scripts/13-build-swarm-coord-image.sh to have succeeded on this host).
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE="${REPO_ROOT}/docker/swarm-coord/Dockerfile"
ENTRYPOINT="${REPO_ROOT}/docker/swarm-coord/entrypoint.sh"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-swarm-coord.service"
SWARM_COORD_YAML="${REPO_ROOT}/ws/src/swarm_coord/params/default.yaml"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/swarm-coord:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/swarm-coord:dev}"

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
check "swarm_coord params file present" test -f "${SWARM_COORD_YAML}"

check "entrypoint launches swarm_coordinator_node" \
    grep -q 'ros2 run swarm_coord swarm_coordinator_node' "${ENTRYPOINT}"
check "entrypoint launches formation_controller_node" \
    grep -q 'ros2 run swarm_coord formation_controller_node' "${ENTRYPOINT}"
check "entrypoint references swarm_coord params file" \
    grep -q '/etc/drone-mc/params/swarm_coord.yaml' "${ENTRYPOINT}"
check "entrypoint relaxes nounset around setup.bash (COLCON_TRACE)" \
    grep -q 'set +u' "${ENTRYPOINT}"
check "entrypoint traps SIGTERM/SIGINT" \
    grep -qE 'trap terminate SIGTERM SIGINT' "${ENTRYPOINT}"

check "systemd unit runs ghcr.io/drone-mc/swarm-coord image" \
    grep -q 'ghcr.io/drone-mc/swarm-coord:' "${UNIT}"
check "systemd unit bind-mounts the shared colcon install" \
    grep -qE -- '--mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install' "${UNIT}"
check "systemd unit bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
check "systemd unit restarts on failure" grep -q 'Restart=on-failure' "${UNIT}"
check "systemd unit orders after gcs-comms" \
    grep -qE 'After=.*drone-mc-gcs-comms\.service' "${UNIT}"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: exercising the real container"

    if ! docker image inspect "${IMAGE}" &>/dev/null; then
        echo "==> ERROR: ${IMAGE} not found — run scripts/13-build-swarm-coord-image.sh first" >&2
        exit 1
    fi

    trap '[[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true' EXIT

    # --network=host, no /etc/drone-mc bind-mount — falls back to the
    # image's own baked-in params copy, same rationale
    # 11b-smoke-test-gcs-comms.sh's LIVE_CHECK section gives.
    cid="$(docker run -d --rm --network=host "${IMAGE}")"

    nodes_registered() {
        docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && ros2 node list 2>/dev/null" \
            | grep -q '/swarm_coordinator_node' \
        && docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && ros2 node list 2>/dev/null" \
            | grep -q '/formation_controller_node'
    }

    ready=0
    for _ in $(seq 1 30); do
        if nodes_registered; then ready=1; break; fi
        sleep 1
    done
    check "both nodes registered in ros2 node list within 30s" \
        bash -c "[[ ${ready} -eq 1 ]]"
    check "container still running (neither node crashed on startup)" \
        bash -c "docker inspect -f '{{.State.Running}}' '${cid}' | grep -q true"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
