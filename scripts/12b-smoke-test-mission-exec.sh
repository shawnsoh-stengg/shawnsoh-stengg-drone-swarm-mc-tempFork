#!/usr/bin/env bash
# 12b-smoke-test-mission-exec.sh — Verify the mission-exec container
# (EXEC-005): mission_manager_node + search_planner_node + the in-container
# watchdog supervisor. Ref: SDD §13.2 smoke test pattern.
#
# Static checks (always run): Dockerfile/entrypoint content, both nodes
# launched, watchdog supervisor present (process liveness +
# /mc/manager/heartbeat staleness check), params/systemd unit wiring.
#
# EXEC-005 AC2's full "systemctl start drone-mc.target to /mc/mission_phase
# publishing <=120s" criterion spans all 9 services' containers, most of
# which don't have images yet — that end-to-end measurement is a hardware
# test (TP-HW-005 style, same disclaimer as 02b-smoke-test-units.sh's own
# LIVE_CHECK), not something this single-service smoke test can exercise.
# What LIVE_CHECK=1 below verifies instead is the mission-exec container's
# own contribution to that budget in isolation: AC1 (both nodes in
# `ros2 node list` and /mc/mission_phase actually publishing, timed and
# logged to stdout — captured by journald when run for real under
# drone-mc-mission-exec.service, since its ExecStart runs `docker run` in
# the foreground) and AC3 (kill -> fresh restart republishes
# /mc/mission_phase within 30s).
#
# Set LIVE_CHECK=1 to additionally build/run the actual image (requires
# scripts/12-build-mission-exec-image.sh to have succeeded on this host).
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE="${REPO_ROOT}/docker/mission-exec/Dockerfile"
ENTRYPOINT="${REPO_ROOT}/docker/mission-exec/entrypoint.sh"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-mission-exec.service"
MISSION_EXEC_YAML="${REPO_ROOT}/ws/src/mission_exec/params/default.yaml"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/mission-exec:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/mission-exec:dev}"

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
check "mission_exec params file present" test -f "${MISSION_EXEC_YAML}"

# AC1 ("Container includes mission_exec, search_planner_node, and watchdog
# supervisor"): both node executables are actually launched by the
# entrypoint, not just built into the image.
check "entrypoint launches mission_manager_node" \
    grep -q 'ros2 run mission_exec mission_manager_node' "${ENTRYPOINT}"
check "entrypoint launches search_planner_node" \
    grep -q 'ros2 run mission_exec search_planner_node' "${ENTRYPOINT}"
check "entrypoint references mission_exec params file" \
    grep -q '/etc/drone-mc/params/mission_exec.yaml' "${ENTRYPOINT}"

# Watchdog supervisor: process liveness + heartbeat staleness, distinct
# from the two application nodes above.
check "entrypoint runs a watchdog supervisor loop" \
    grep -q 'watchdog supervisor active' "${ENTRYPOINT}"
check "watchdog checks mission_manager_node process liveness" \
    grep -q 'kill -0 "\${manager_pid}"' "${ENTRYPOINT}"
check "watchdog checks search_planner_node process liveness" \
    grep -q 'kill -0 "\${planner_pid}"' "${ENTRYPOINT}"
check "watchdog checks /mc/manager/heartbeat staleness" \
    grep -q '/mc/manager/heartbeat' "${ENTRYPOINT}"
check "watchdog staleness budget is configurable via EXECUTOR_WD_TIMEOUT_S" \
    grep -q 'EXECUTOR_WD_TIMEOUT_S' "${ENTRYPOINT}"
check "entrypoint traps SIGTERM/SIGINT without misreporting a clean stop as a failure" \
    grep -qE 'trap on_signal SIGTERM SIGINT' "${ENTRYPOINT}"

# AC2 ("Time from systemctl start ... to /mc/mission_phase publishing
# <=120s; logged to journald"): the entrypoint itself must measure and log
# this, since a smoke test run standalone (not through the full target)
# can't observe true target-activation time.
check "entrypoint waits for /mc/mission_phase to publish" \
    grep -q 'mission_phase_seen' "${ENTRYPOINT}"
check "entrypoint enforces the 120s PR-009 budget" \
    grep -qE 'seq 1 120' "${ENTRYPOINT}"
check "entrypoint logs boot timing to stdout (captured by journald under systemd)" \
    grep -q 'boot_elapsed_s=' "${ENTRYPOINT}"

check "systemd unit runs ghcr.io/drone-mc/mission-exec image" \
    grep -q 'ghcr.io/drone-mc/mission-exec:' "${UNIT}"
check "systemd unit bind-mounts the shared colcon install" \
    grep -qE -- '--mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install' "${UNIT}"
check "systemd unit bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
check "systemd unit restarts on failure" grep -q 'Restart=on-failure' "${UNIT}"
check "systemd unit in be.slice" grep -qE '^Slice=be.slice$' "${UNIT}"
check "systemd unit orders after swarm-coord/gcs-comms/fcc-bridge" \
    grep -qE 'After=.*drone-mc-fcc-bridge\.service.*drone-mc-gcs-comms\.service.*drone-mc-swarm-coord\.service' "${UNIT}"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: exercising the real container"

    if ! docker image inspect "${IMAGE}" &>/dev/null; then
        echo "==> ERROR: ${IMAGE} not found — run scripts/12-build-mission-exec-image.sh first" >&2
        exit 1
    fi

    trap '[[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true' EXIT

    # --network=host, no /etc/drone-mc bind-mount — falls back to the
    # image's own baked-in params copy, same rationale
    # 11b-smoke-test-gcs-comms.sh's LIVE_CHECK section gives.
    start_container() {
        cid="$(docker run -d --rm --network=host "${IMAGE}")"
    }
    mission_phase_publishing() {
        docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && timeout 2 ros2 topic echo --once /mc/mission_phase" \
            &>/dev/null
    }

    start_container
    echo "==> AC1/AC2: waiting up to 120s for /mc/mission_phase to publish"
    boot_start_s=$(date +%s)
    ac12_ok=0
    for _ in $(seq 1 120); do
        if mission_phase_publishing; then ac12_ok=1; break; fi
        sleep 1
    done
    boot_elapsed_s=$(( $(date +%s) - boot_start_s ))
    check "AC1/AC2: /mc/mission_phase publishing within 120s (elapsed ${boot_elapsed_s}s)" \
        bash -c "[[ ${ac12_ok} -eq 1 ]]"
    check "AC2: container logged its own boot timing (journald-visible via docker logs)" \
        bash -c "docker logs '${cid}' 2>&1 | grep -q 'boot_elapsed_s='"

    echo "==> AC3: killing container, timing a fresh restart's /mc/mission_phase republish"
    docker kill "${cid}" &>/dev/null || true
    docker wait "${cid}" &>/dev/null || true
    start_container
    ac3_ok=0
    for _ in $(seq 1 30); do
        if mission_phase_publishing; then ac3_ok=1; break; fi
        sleep 1
    done
    check "AC3: /mc/mission_phase republishes within 30s of a fresh restart after kill" \
        bash -c "[[ ${ac3_ok} -eq 1 ]]"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
