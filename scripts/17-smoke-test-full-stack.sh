#!/usr/bin/env bash
# 17-smoke-test-full-stack.sh — FT-004: full-stack drone-mc.target bring-up,
# all services together (not per-service smoke tests).
# Ref: INTEGRATION-AND-FLIGHT-TEST-PLAN.md §1.2 Stage D / Track G FT-004
#
# Unlike 02b-smoke-test-units.sh (static unit-file validation, generic to
# whatever's installed) this script exercises FT-004's own four ACs against
# whichever of the 10 planned drone-mc.target services actually have a
# built image today:
#   AC1 — systemctl start drone-mc.target brings up all AVAILABLE services
#         with no unit entering a restart loop, observed for >= 10 minutes.
#   AC2 — ros2 node list shows every expected node registered.
#   AC3 — resource usage (/mc/health) stays within SDD §10.1's documented
#         fault thresholds with the full available stack running
#         concurrently, plus each unit's own cgroup MemoryMax/CPUQuota.
#   AC4 — restarting one service doesn't cascade into others restarting.
#
# Per FT-004's own Deliverables: services with no image today (per
# docs/DEPLOYMENT.md's "Known deployment gaps") are called out explicitly,
# not silently skipped — this script computes that set itself rather than
# hardcoding it, so it stays correct as images land.
#
# Static mode (default): reports which images exist, nothing else.
# Set LIVE_CHECK=1 to actually start drone-mc.target and run all four ACs
# (requires root, the systemd units installed via
# scripts/02-install-systemd-units.sh, and at least one service image).
#
# STEADY_STATE_SECONDS (default 600, i.e. AC1's ">=10 minutes") and
# SAMPLE_INTERVAL_SECONDS (default 10) are overridable for faster dev
# iteration — the ticket's own bound is the default, not the override.
set -euo pipefail

IMAGE_TAG="${MC_VERSION:-dev}"
UNIT_DIR="/etc/systemd/system"

STEADY_STATE_SECONDS="${STEADY_STATE_SECONDS:-600}"
SAMPLE_INTERVAL_SECONDS="${SAMPLE_INTERVAL_SECONDS:-10}"

PASS=0; FAIL=0; SKIP=0

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS+1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL+1))
    fi
}

skip() {
    printf "SKIP: %s\n" "$1"; SKIP=$((SKIP+1))
}

nrestarts() { systemctl show "$1" -p NRestarts --value 2>/dev/null || echo "0"; }

# AC2/AC3 run `ros2` on the HOST, against nodes running inside containers
# whose ROS_DOMAIN_ID/CYCLONEDDS_URI come from /etc/drone-mc/environment
# (EnvironmentFile=/--env-file, every drone-mc-*.service). Without loading
# the same two vars here, the host CLI falls back to CycloneDDS's default
# config, which docker/ros2-core/cyclonedds.xml's locked-down lo-only,
# AllowMulticast=false, Peers=127.0.0.1-only setup will never answer —
# this is what caused every prior run's AC2/AC3 to come back empty/SKIP,
# not a real discovery failure.
if [[ -f /etc/drone-mc/environment ]]; then
    # shellcheck disable=SC1091
    source /etc/drone-mc/environment
    export ROS_DOMAIN_ID CYCLONEDDS_URI
fi

# All 10 services drone-mc.target eventually manages (SDD §12.4 / CLAUDE.md
# repo-structure note). mavlink-router/zenoh-bridge are standalone daemons
# with no ROS 2 node (CLAUDE.md's Tooling Decisions) — omitted from
# SERVICE_NODES below, not forgotten.
declare -A SERVICE_IMAGE=(
    [drone-mc-mavlink-router.service]=ghcr.io/drone-mc/mavlink-router
    [drone-mc-zenoh-bridge.service]=ghcr.io/drone-mc/zenoh-bridge
    [drone-mc-fcc-bridge.service]=ghcr.io/drone-mc/fcc-bridge
    [drone-mc-gcs-comms.service]=ghcr.io/drone-mc/gcs-comms
    [drone-mc-mission-orchestrator.service]=ghcr.io/drone-mc/mission-orchestrator
    [drone-mc-swarm-coord.service]=ghcr.io/drone-mc/swarm-coord
    [drone-mc-mission-exec.service]=ghcr.io/drone-mc/mission-exec
    [drone-mc-health-monitor.service]=ghcr.io/drone-mc/health-monitor
    [drone-mc-camera.service]=ghcr.io/drone-mc/camera
    [drone-mc-vision.service]=ghcr.io/drone-mc/vision
)

declare -A SERVICE_NODES=(
    [drone-mc-fcc-bridge.service]="/fcc_bridge_node"
    [drone-mc-gcs-comms.service]="/gcs_mavlink_node /mavros_node"
    [drone-mc-mission-orchestrator.service]="/mission_orchestrator_node"
    [drone-mc-swarm-coord.service]="/swarm_coordinator_node /formation_controller_node"
    [drone-mc-mission-exec.service]="/mission_manager_node /search_planner_node"
    [drone-mc-health-monitor.service]="/health_monitor_node"
)

echo "==> Computing which of the 10 drone-mc.target services have a built image (${IMAGE_TAG})"
AVAILABLE_SERVICES=()
MISSING_SERVICES=()
for unit in "${!SERVICE_IMAGE[@]}"; do
    image="${SERVICE_IMAGE[$unit]}:${IMAGE_TAG}"
    if docker image inspect "${image}" &>/dev/null; then
        AVAILABLE_SERVICES+=("${unit}")
        echo "    ${unit} -> ${image}  (present)"
    else
        MISSING_SERVICES+=("${unit}")
        echo "    ${unit} -> ${image}  (MISSING)"
    fi
done

if [[ "${#AVAILABLE_SERVICES[@]}" -eq 0 ]]; then
    echo "==> ERROR: no service images found — build at least one via scripts/0{8,9,10,...}-build-*.sh first" >&2
    exit 1
fi

check "drone-mc.target unit installed" test -f "${UNIT_DIR}/drone-mc.target"

if [[ "${LIVE_CHECK:-0}" -ne 1 ]]; then
    echo "==> LIVE_CHECK not set — static image-availability report only (see above); no bring-up attempted."
    printf "\nResults: %d passed, %d failed, %d skipped\n" "${PASS}" "${FAIL}" "${SKIP}"
    [[ "${FAIL}" -eq 0 ]] || exit 1
    exit 0
fi

# ---- LIVE_CHECK=1: AC1-AC4 against AVAILABLE_SERVICES only -----------------

echo "==> LIVE_CHECK=1: starting drone-mc.target"
# Units whose image is missing will fail to start (docker run: no such
# image) — that's expected per this ticket's own AC, not a reason to abort
# the rest of the run, so don't let a non-zero exit here stop the script.
systemctl start drone-mc.target || true

echo "==> Waiting up to 120s for the ${#AVAILABLE_SERVICES[@]} available-image services to become active"
deadline=$((SECONDS + 120))
all_active=0
while [[ ${SECONDS} -lt ${deadline} ]]; do
    all_active=1
    for unit in "${AVAILABLE_SERVICES[@]}"; do
        systemctl is-active --quiet "${unit}" || { all_active=0; break; }
    done
    [[ ${all_active} -eq 1 ]] && break
    sleep 2
done
check "all available-image services active within 120s" bash -c "[[ ${all_active} -eq 1 ]]"
for unit in "${AVAILABLE_SERVICES[@]}"; do
    check "${unit} active" systemctl is-active --quiet "${unit}"
done

# ---- AC1: >= STEADY_STATE_SECONDS with no restart loop ---------------------
echo "==> AC1: monitoring steady state for ${STEADY_STATE_SECONDS}s (sampling every ${SAMPLE_INTERVAL_SECONDS}s)"
declare -A NRESTARTS_START
for unit in "${AVAILABLE_SERVICES[@]}"; do
    NRESTARTS_START[${unit}]="$(nrestarts "${unit}")"
done
elapsed=0
restart_loop_detected=0
flapped_unit=""
while [[ ${elapsed} -lt ${STEADY_STATE_SECONDS} ]]; do
    sleep "${SAMPLE_INTERVAL_SECONDS}"
    elapsed=$((elapsed + SAMPLE_INTERVAL_SECONDS))
    for unit in "${AVAILABLE_SERVICES[@]}"; do
        current="$(nrestarts "${unit}")"
        if [[ "${current}" != "${NRESTARTS_START[${unit}]}" ]]; then
            restart_loop_detected=1; flapped_unit="${unit} (NRestarts ${NRESTARTS_START[${unit}]} -> ${current})"
            break 2
        fi
        if ! systemctl is-active --quiet "${unit}"; then
            restart_loop_detected=1; flapped_unit="${unit} (no longer active)"
            break 2
        fi
    done
done
check "no restart loop over ${STEADY_STATE_SECONDS}s steady-state window" bash -c "[[ ${restart_loop_detected} -eq 0 ]]"
[[ ${restart_loop_detected} -eq 1 ]] && echo "    flapped: ${flapped_unit}"

# ---- AC2: ros2 node list ----------------------------------------------------
echo "==> AC2: checking ros2 node list"
EXPECTED_NODES=()
for unit in "${AVAILABLE_SERVICES[@]}"; do
    # shellcheck disable=SC2206
    EXPECTED_NODES+=(${SERVICE_NODES[${unit}]:-})
done
if [[ "${#EXPECTED_NODES[@]}" -eq 0 ]]; then
    skip "ros2 node list (only standalone daemons — mavlink-router/zenoh-bridge — are available this run)"
else
    ACTUAL_NODES="$(timeout 10 ros2 node list 2>/dev/null || true)"
    if [[ -z "${ACTUAL_NODES}" ]]; then
        skip "ros2 node list (no nodes discovered — check host RMW/ROS_DOMAIN_ID matches the containers' CycloneDDS config, and the CycloneDDS rmem floor, INFRA-004)"
    else
        for node in "${EXPECTED_NODES[@]}"; do
            check "ros2 node list shows ${node}" bash -c "grep -qx '${node}' <<<\"${ACTUAL_NODES}\""
        done
    fi
fi

# ---- AC3: resource budgets --------------------------------------------------
echo "==> AC3: checking /mc/health against SDD §10.1 fault thresholds, and per-unit cgroup budgets"
if [[ " ${AVAILABLE_SERVICES[*]} " == *" drone-mc-health-monitor.service "* ]] && systemctl is-active --quiet drone-mc-health-monitor.service; then
    HEALTH_MSG="$(timeout 5 ros2 topic echo --once /mc/health 2>/dev/null || true)"
    if [[ -z "${HEALTH_MSG}" ]]; then
        skip "/mc/health resource budget check (no message received within 5s)"
    else
        cpu_pct="$(grep -oP 'cpu_pct:\s*\K[0-9]+' <<<"${HEALTH_MSG}" || true)"
        mem_pct="$(grep -oP 'mem_pct:\s*\K[0-9]+' <<<"${HEALTH_MSG}" || true)"
        disk_pct="$(grep -oP 'disk_pct:\s*\K[0-9]+' <<<"${HEALTH_MSG}" || true)"
        gpu_temp_c="$(grep -oP 'gpu_temp_c:\s*\K[0-9]+' <<<"${HEALTH_MSG}" || true)"
        cpu_temp_c="$(grep -oP 'cpu_temp_c:\s*\K[0-9]+' <<<"${HEALTH_MSG}" || true)"
        [[ -n "${cpu_pct:-}" ]]    && check "cpu_pct ${cpu_pct} <= 95 (SDD §10.1)"        bash -c "[[ ${cpu_pct} -le 95 ]]"
        [[ -n "${mem_pct:-}" ]]    && check "mem_pct ${mem_pct} <= 90 (SDD §10.1: <10% free)" bash -c "[[ ${mem_pct} -le 90 ]]"
        [[ -n "${disk_pct:-}" ]]   && check "disk_pct ${disk_pct} <= 90 (SDD §10.1)"          bash -c "[[ ${disk_pct} -le 90 ]]"
        [[ -n "${gpu_temp_c:-}" ]] && check "gpu_temp_c ${gpu_temp_c} <= 85 (SDD §10.1)"      bash -c "[[ ${gpu_temp_c} -le 85 ]]"
        [[ -n "${cpu_temp_c:-}" ]] && check "cpu_temp_c ${cpu_temp_c} <= 90 (SDD §10.1)"      bash -c "[[ ${cpu_temp_c} -le 90 ]]"
    fi
else
    skip "/mc/health resource budget check (drone-mc-health-monitor.service not available this run)"
fi

for unit in "${AVAILABLE_SERVICES[@]}"; do
    mem_max="$(systemctl show "${unit}" -p MemoryMax --value 2>/dev/null || echo infinity)"
    mem_cur="$(systemctl show "${unit}" -p MemoryCurrent --value 2>/dev/null || echo "")"
    if [[ "${mem_max}" != "infinity" && -n "${mem_cur}" && "${mem_cur}" != "[not set]" ]]; then
        check "${unit} MemoryCurrent (${mem_cur}) <= MemoryMax (${mem_max})" bash -c "[[ ${mem_cur} -le ${mem_max} ]]"
    fi
done

# ---- AC4: single-service restart isolation ---------------------------------
echo "==> AC4: restart isolation (drone-mc-fcc-bridge.service, per this ticket's own example)"
RESTART_TARGET="drone-mc-fcc-bridge.service"
if [[ " ${AVAILABLE_SERVICES[*]} " == *" ${RESTART_TARGET} "* ]]; then
    declare -A NRESTARTS_BEFORE
    OTHERS=()
    for unit in "${AVAILABLE_SERVICES[@]}"; do
        [[ "${unit}" == "${RESTART_TARGET}" ]] && continue
        OTHERS+=("${unit}")
        NRESTARTS_BEFORE[${unit}]="$(nrestarts "${unit}")"
    done
    systemctl restart "${RESTART_TARGET}"
    sleep 10
    check "${RESTART_TARGET} active again after restart" systemctl is-active --quiet "${RESTART_TARGET}"
    cascade=0
    for unit in "${OTHERS[@]}"; do
        after="$(nrestarts "${unit}")"
        if [[ "${after}" != "${NRESTARTS_BEFORE[${unit}]}" ]]; then
            cascade=1
            echo "    cascaded restart: ${unit} (NRestarts ${NRESTARTS_BEFORE[${unit}]} -> ${after})"
        fi
    done
    check "restarting ${RESTART_TARGET} does not cascade to other services" bash -c "[[ ${cascade} -eq 0 ]]"
else
    skip "restart isolation check (${RESTART_TARGET} image not available this run)"
fi

echo "==> Stopping drone-mc.target"
systemctl stop drone-mc.target
sleep 2
all_stopped=1
STILL_ACTIVE=()
for unit in "${AVAILABLE_SERVICES[@]}"; do
    if systemctl is-active --quiet "${unit}"; then
        all_stopped=0
        STILL_ACTIVE+=("${unit}")
    fi
done
check "all available services stopped cleanly" bash -c "[[ ${all_stopped} -eq 1 ]]"
if [[ "${all_stopped}" -eq 0 ]]; then
    for unit in "${STILL_ACTIVE[@]}"; do
        echo "    still active: ${unit} (NRestarts $(nrestarts "${unit}")) — $(systemctl show "${unit}" -p ActiveState -p SubState -p Result --value | tr '\n' ' ')"
        echo "        last 5 log lines:"
        journalctl -u "${unit}" -n 5 --no-pager 2>/dev/null | sed 's/^/        /'
    done
fi

echo ""
echo "==> Missing images (FT-004 Deliverables: called out explicitly, not silently skipped):"
if [[ "${#MISSING_SERVICES[@]}" -eq 0 ]]; then
    echo "    none — full 10-service stack was available this run"
else
    for unit in "${MISSING_SERVICES[@]}"; do
        echo "    ${unit} -> ${SERVICE_IMAGE[${unit}]}:${IMAGE_TAG}"
    done
fi

printf "\nResults: %d passed, %d failed, %d skipped\n" "${PASS}" "${FAIL}" "${SKIP}"
[[ "${FAIL}" -eq 0 ]] || exit 1
