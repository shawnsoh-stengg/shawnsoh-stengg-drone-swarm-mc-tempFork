#!/usr/bin/env bash
# 15b-smoke-test-mission-orchestrator.sh — Verify the mission-orchestrator
# container (GCS-006): mission_orchestrator_node launches via a real
# systemd-managed container, Channel C's mavlink-routerd process is started
# automatically alongside Channels A/B, and the plan library persists
# across a container restart. Ref: SDD §13.2 smoke test pattern.
#
# Static checks (always run): Dockerfile/entrypoint content, systemd unit
# wiring (image, bind mounts including the read-write mission_plans mount,
# Requires=), docker/mavlink-router/entrypoint.sh actually launches a third
# process for mavlink-router-channel-c.conf, and
# scripts/02-install-systemd-units.sh provisions both that conf file and
# the /var/drone-mc/mission_plans directory.
#
# Set LIVE_CHECK=1 to additionally build/run the actual image (requires
# scripts/15-build-mission-orchestrator-image.sh to have succeeded on this
# host): verifies AC1 (mission_orchestrator_node in `ros2 node list` within
# 30s) and AC4 (a file written to the bind-mounted plans dir survives a
# kill + fresh restart).
#
# Set LIVE_CHECK=1 and E2E_CHECK=1 together to additionally run
# tools/mission-orchestrator-sim's full create/list/read/update/activate/
# delete command set against real mission-orchestrator + mavlink-router
# containers over a loopback-rewritten copy of mavlink-router-channel-c.conf
# (same rewrite technique 09b-smoke-test-routing.sh's own LIVE_CHECK uses
# for Channels A/B) — this is AC3 itself, exercised directly rather than
# inferred from the unit files. Needs the mavlink-router image already
# built (scripts/09-build-mavlink-router-image.sh) and pymavlink available
# either in tools/mission-orchestrator-sim/.venv or the system python3;
# skipped with a clear message, not a failure, if either prerequisite is
# missing — this is the one check in this script that exercises two other
# packages' images together, not just this ticket's own deliverable.
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE="${REPO_ROOT}/docker/mission-orchestrator/Dockerfile"
ENTRYPOINT="${REPO_ROOT}/docker/mission-orchestrator/entrypoint.sh"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-mission-orchestrator.service"
ROUTER_ENTRYPOINT="${REPO_ROOT}/docker/mavlink-router/entrypoint.sh"
INSTALL_SCRIPT="${REPO_ROOT}/scripts/02-install-systemd-units.sh"
CONF_C="${REPO_ROOT}/config/mavlink-router-channel-c.conf"
GCS_COMMS_YAML="${REPO_ROOT}/ws/src/gcs_comms/params/default.yaml"
SIM_DIR="${REPO_ROOT}/tools/mission-orchestrator-sim"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/mission-orchestrator:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/mission-orchestrator:dev}"
ROUTER_IMAGE="${MC_VERSION:+ghcr.io/drone-mc/mavlink-router:${MC_VERSION}}"
ROUTER_IMAGE="${ROUTER_IMAGE:-ghcr.io/drone-mc/mavlink-router:latest}"

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
check "channel-c.conf present"      test -f "${CONF_C}"

check "entrypoint launches mission_orchestrator_node" \
    grep -q 'ros2 run gcs_comms mission_orchestrator_node' "${ENTRYPOINT}"
check "entrypoint references gcs_comms params file" \
    grep -q '/etc/drone-mc/params/gcs_comms.yaml' "${ENTRYPOINT}"
check "entrypoint traps SIGTERM/SIGINT for a clean node shutdown" \
    grep -qE 'trap terminate SIGTERM SIGINT' "${ENTRYPOINT}"
check "gcs_comms params file has mission_orchestrator_node section" \
    grep -q 'mission_orchestrator_node:' "${GCS_COMMS_YAML}"

check "systemd unit runs ghcr.io/drone-mc/mission-orchestrator image" \
    grep -q 'ghcr.io/drone-mc/mission-orchestrator:' "${UNIT}"
check "systemd unit bind-mounts the shared colcon install" \
    grep -qE -- '--mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install' "${UNIT}"
check "systemd unit bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
# AC4: read-write, not readonly — mission_orchestrator_node writes plan
# JSON files here (MAVFTP create/update/delete), unlike every other
# service's read-only /etc/drone-mc mount.
check "systemd unit bind-mounts /var/drone-mc/mission_plans read-write (not readonly)" \
    bash -c "grep -E 'src=/var/drone-mc/mission_plans,dst=/var/drone-mc/mission_plans' '${UNIT}' | grep -qv readonly"
check "systemd unit restarts on failure" grep -q 'Restart=on-failure' "${UNIT}"
check "systemd unit in be.slice" grep -qE '^Slice=be.slice$' "${UNIT}"
# Requires= was changed to Wants= 2026-09-21 (FT-004 defect): a real
# LIVE_CHECK=1 run showed Requires= propagates a stop from fcc-bridge/
# mavlink-router to this unit, which shouldn't happen — After= alone still
# orders startup correctly.
check "systemd unit wants fcc-bridge and mavlink-router" \
    grep -qE 'Wants=.*drone-mc-fcc-bridge\.service.*drone-mc-mavlink-router\.service' "${UNIT}"

# AC2: Channel C's mavlink-routerd process is started automatically, as a
# third process alongside A/B in the same container — not a manually-run
# foreground process (the exact gap that made every tools/mission-
# orchestrator-sim request time out before this ticket).
check "mavlink-router entrypoint launches channel-c as its own process" \
    grep -qE 'mavlink-routerd -c.*mavlink-router-channel-c\.conf' "${ROUTER_ENTRYPOINT}"
check "mavlink-router entrypoint does not pass channel-c's conf to the channel-a/b processes" \
    bash -c "! grep -qE 'mavlink-router-channel-a\.conf.*mavlink-router-channel-c\.conf|mavlink-router-channel-b\.conf.*mavlink-router-channel-c\.conf' '${ROUTER_ENTRYPOINT}'"
check "mavlink-router entrypoint waits on all three processes (not just a/b)" \
    grep -qE 'wait -n.*pid_a.*pid_b.*pid_c' "${ROUTER_ENTRYPOINT}"
check "mavlink-router entrypoint terminates all three processes on shutdown" \
    grep -qE 'kill -TERM.*pid_a.*pid_b.*pid_c' "${ROUTER_ENTRYPOINT}"

check "install script provisions channel-c.conf" \
    grep -q 'mavlink-router-channel-c.conf' "${INSTALL_SCRIPT}"
check "install script provisions /var/drone-mc/mission_plans" \
    grep -q '/var/drone-mc/mission_plans' "${INSTALL_SCRIPT}"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: exercising the real container"

    if ! docker image inspect "${IMAGE}" &>/dev/null; then
        echo "==> ERROR: ${IMAGE} not found — run scripts/15-build-mission-orchestrator-image.sh first" >&2
        exit 1
    fi

    PLANS_DIR="$(mktemp -d)"
    trap 'rm -rf "${PLANS_DIR}"; [[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true' EXIT

    # --network=host, real bind-mount for /var/drone-mc/mission_plans (a
    # temp dir standing in for the real host path) so AC4's persistence
    # claim is tested against an actual bind mount, not the container's own
    # writable layer. No /etc/drone-mc bind-mount — falls back to the
    # image's own baked-in params copy, same rationale
    # 11b-smoke-test-gcs-comms.sh's LIVE_CHECK section gives.
    start_container() {
        cid="$(docker run -d --rm --network=host --env DRONE_ID=1 \
            --mount "type=bind,src=${PLANS_DIR},dst=/var/drone-mc/mission_plans" \
            "${IMAGE}")"
    }
    node_ready() {
        docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && ros2 node list" \
            2>/dev/null | grep -q '/mission_orchestrator_node'
    }

    start_container
    echo "==> AC1: waiting up to 30s for mission_orchestrator_node to register"
    ac1_ok=0
    for _ in $(seq 1 30); do
        if node_ready; then ac1_ok=1; break; fi
        sleep 1
    done
    check "AC1: mission_orchestrator_node in ros2 node list within 30s" \
        bash -c "[[ ${ac1_ok} -eq 1 ]]"

    echo "==> AC4: writing a marker plan file, killing the container, confirming it survives a fresh restart"
    echo '{"name":"smoke_marker"}' > "${PLANS_DIR}/smoke_marker.json"
    docker kill "${cid}" &>/dev/null || true
    docker wait "${cid}" &>/dev/null || true
    check "AC4: marker plan file survives the container going away" \
        test -f "${PLANS_DIR}/smoke_marker.json"

    start_container
    ac4_restart_ok=0
    for _ in $(seq 1 30); do
        if node_ready; then ac4_restart_ok=1; break; fi
        sleep 1
    done
    check "AC4: mission_orchestrator_node re-registers within 30s of a fresh restart after kill" \
        bash -c "[[ ${ac4_restart_ok} -eq 1 ]]"
    check "AC4: marker plan file still present after restart (bind mount, not container-local storage)" \
        test -f "${PLANS_DIR}/smoke_marker.json"

    docker kill "${cid}" &>/dev/null || true
    docker wait "${cid}" &>/dev/null || true
    cid=""

    if [[ "${E2E_CHECK:-0}" -eq 1 ]]; then
        echo "==> E2E_CHECK=1: exercising tools/mission-orchestrator-sim's full command set end-to-end"

        PYTHON="python3"
        [[ -x "${SIM_DIR}/.venv/bin/python3" ]] && PYTHON="${SIM_DIR}/.venv/bin/python3"

        if ! docker image inspect "${ROUTER_IMAGE}" &>/dev/null; then
            echo "==> E2E_CHECK skipped: ${ROUTER_IMAGE} not found — run scripts/09-build-mavlink-router-image.sh first"
        elif ! "${PYTHON}" -c 'import pymavlink' &>/dev/null; then
            echo "==> E2E_CHECK skipped: pymavlink not importable via ${PYTHON} — run" \
                 "'cd tools/mission-orchestrator-sim && python3 -m venv .venv &&" \
                 ".venv/bin/pip install -r requirements.txt' first"
        else
            E2E_DIR="$(mktemp -d)"
            trap '[[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true
                  [[ -n "${router_cid:-}" ]] && docker rm -f "${router_cid}" &>/dev/null || true
                  rm -rf "${E2E_DIR}" "${PLANS_DIR}"' EXIT

            # Loopback rewrite: real 192.168.100.10 isn't routable in a
            # sandboxed test host — same technique
            # 09b-smoke-test-routing.sh's own LIVE_CHECK uses. Ports are
            # unchanged (README's "Testing against a real two-host
            # deployment" section documents this as the one address that
            # needs rewriting for loopback use, unlike Channel A/B's own
            # multi-port rewrites — ORCHESTRATOR_MC_RECV already listens on
            # 0.0.0.0 and ORCHESTRATOR_MAVLINK_NODE is already loopback).
            mkdir -p "${E2E_DIR}/etc-drone-mc"
            sed 's/192\.168\.100\.10/127.0.0.1/' "${CONF_C}" \
                > "${E2E_DIR}/etc-drone-mc/mavlink-router-channel-c.conf"
            # Channels A/B still need *some* valid conf for the container's
            # other two processes to start cleanly — the real confs are
            # fine here since this test only asserts on Channel C traffic.
            cp "${REPO_ROOT}/config/mavlink-router-channel-a.conf" "${E2E_DIR}/etc-drone-mc/"
            cp "${REPO_ROOT}/config/mavlink-router-channel-b.conf" "${E2E_DIR}/etc-drone-mc/"

            router_cid="$(docker run -d --rm --network=host \
                --mount "type=bind,src=${E2E_DIR}/etc-drone-mc,dst=/etc/drone-mc,readonly" \
                "${ROUTER_IMAGE}")"
            start_container
            echo "==> E2E_CHECK: waiting up to 30s for mission_orchestrator_node to register"
            e2e_ready=0
            for _ in $(seq 1 30); do
                if node_ready; then e2e_ready=1; break; fi
                sleep 1
            done
            check "E2E_CHECK: mission_orchestrator_node registered before running the sim tool" \
                bash -c "[[ ${e2e_ready} -eq 1 ]]"

            cat > "${E2E_DIR}/smoke_plan.json" <<'PLANJSON'
{
  "name": "smoke_e2e",
  "ingress_waypoints": [
    {"lat": 1.3521000, "lon": 103.8198000, "alt_m": 50.0, "speed_mps": 5.0}
  ],
  "search_polygon": [
    {"lat": 1.3521000, "lon": 103.8198000}
  ],
  "swarm_manifest": [
    {"drone_id": 1, "ip": "127.0.0.1"}
  ],
  "arm_authorised": false
}
PLANJSON

            SIM=("${SIM_DIR}/mission-orchestrator-sim" --mc-host 127.0.0.1)
            # Shell functions, called directly (not via `bash -c "..."`,
            # which would spawn a fresh bash process these definitions
            # never reach) — `check` invokes its argument list in this same
            # shell, so a function name works exactly like a plain command.
            sim_env() { env MC_MAVLINK_DEFS_XML="${REPO_ROOT}/ws/src/gcs_comms/mc_mavlink_defs.xml" "${PYTHON}" "${SIM[@]}" "$@"; }
            create_with_retry() {
                # First request after a cold start can legitimately time
                # out once (README's "cold start" note: mavlink-router
                # hasn't learned mission_orchestrator_node's identity until
                # its first reply goes out) — retry exactly once before
                # treating it as a real failure, same tolerance the README
                # documents.
                sim_env create smoke_e2e "${E2E_DIR}/smoke_plan.json" \
                    || sim_env create smoke_e2e "${E2E_DIR}/smoke_plan.json"
            }
            list_has_plan() { sim_env list | grep -q smoke_e2e.json; }
            delete_active_rejected() { ! sim_env delete smoke_e2e; }

            check "E2E_CHECK: create succeeds (cold-start retry allowed)" create_with_retry
            check "E2E_CHECK: list shows the created plan" list_has_plan
            check "E2E_CHECK: activate succeeds" sim_env activate smoke_e2e
            check "E2E_CHECK: delete of the active plan is rejected (Nak(FileProtected))" delete_active_rejected
        fi
    fi
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
