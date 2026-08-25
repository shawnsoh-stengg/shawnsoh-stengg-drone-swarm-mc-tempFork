#!/usr/bin/env bash
# entrypoint.sh — EXEC-005: launches mission_manager_node and
# search_planner_node together, waits for both to reach operational state
# (PR-009: <=120 s from container start), then runs a lightweight
# in-container watchdog supervisor over the pair for the rest of the
# container's life.
#
# The watchdog supervisor checks two things every second:
#   1. both node processes are still alive (kill -0)
#   2. mission_manager_node's own /mc/manager/heartbeat (std_msgs/Header,
#      SDD §4.4) is no older than EXECUTOR_WD_TIMEOUT_S (config/environment,
#      default 5 s — the same >5 s silence threshold FR-074/SFR-003 define)
#
# This is a container-local safety net only: on either check failing, it
# logs why and exits non-zero so drone-mc-mission-exec.service's existing
# Restart=on-failure (RestartSec=5s) brings the container back — well
# inside EXEC-005 AC3's 30 s recovery budget. It deliberately does *not*
# notify the GCS or command FCC HOLD itself — that cross-service
# escalation (health_monitor_node watching every critical node, plus a
# privileged systemd-restart interface) is HM-002/HM-003's own scope
# (SDD §10.2/§10.4), gated behind HM-001, which doesn't exist yet (see
# EXEC-004's scope note in PROJECT-PLAN-5-MONTH.md) — not duplicated here.
set -euo pipefail

# colcon-generated setup.bash scripts read $COLCON_TRACE without a default
# (`_colcon_prefix_chain_bash_source_script()`'s `[ -n "$COLCON_TRACE" ]`),
# so sourcing them under `set -u` aborts immediately with "COLCON_TRACE:
# unbound variable" — nounset is relaxed for just these two lines, verified
# against a real container run (docker/ros2-core's dustynv Humble base),
# not assumed.
set +u
source /opt/ros/humble/install/setup.bash
source /opt/drone-mc/install/setup.bash
set -u

EXECUTOR_WD_TIMEOUT_S="${EXECUTOR_WD_TIMEOUT_S:-5}"
PARAMS_FILE=/etc/drone-mc/params/mission_exec.yaml

echo "==> starting mission_manager_node"
ros2 run mission_exec mission_manager_node --ros-args --params-file "${PARAMS_FILE}" &
manager_pid=$!

echo "==> starting search_planner_node"
ros2 run mission_exec search_planner_node --ros-args --params-file "${PARAMS_FILE}" &
planner_pid=$!

shutdown_requested=0
terminate() {
    kill -TERM "${manager_pid}" "${planner_pid}" 2>/dev/null || true
    wait "${manager_pid}" 2>/dev/null || true
    wait "${planner_pid}" 2>/dev/null || true
}
on_signal() {
    shutdown_requested=1
    terminate
}
trap on_signal SIGTERM SIGINT

nodes_registered() {
    local nodes
    nodes="$(ros2 node list 2>/dev/null)" || return 1
    grep -q '/mission_manager_node' <<<"${nodes}" && grep -q '/search_planner_node' <<<"${nodes}"
}
mission_phase_seen() {
    timeout 2 ros2 topic echo --once /mc/mission_phase &>/dev/null
}

# Bounded wait, not a fixed sleep — PR-009's 120 s budget covers both nodes
# registering *and* /mc/mission_phase actually publishing (FR-005), so a
# fast boot logs a fast number to journald instead of always claiming the
# ceiling. Logged via plain stdout: drone-mc-mission-exec.service's
# ExecStart runs `docker run` in the foreground (no `-d`), so this
# container's stdout is the unit's own stdout, captured by journald
# automatically (no separate `logger`/sd_notify call needed).
boot_start_s=$(date +%s)
ready=0
for _ in $(seq 1 120); do
    if nodes_registered && mission_phase_seen; then
        ready=1
        break
    fi
    sleep 1
done
boot_elapsed_s=$(( $(date +%s) - boot_start_s ))

if [[ "${ready}" -eq 1 ]]; then
    echo "==> mission-exec ready: mission_manager_node + search_planner_node registered, /mc/mission_phase publishing (boot_elapsed_s=${boot_elapsed_s}, budget=120s, PR-009)"
else
    echo "==> mission-exec FAILED to reach ready state within 120s (boot_elapsed_s=${boot_elapsed_s}, PR-009 violated)" >&2
fi

last_heartbeat_epoch_s() {
    # `--field stamp.sec` reads the heartbeat message's *own* timestamp
    # (std_msgs/Header stamped at publish time each control-loop tick, SDD
    # §4.4), not wall-clock-of-receipt — so a publisher that's alive but
    # stuck (not re-stamping) is still caught even if the DDS transport
    # itself looks healthy.
    # `--field` still doesn't suppress the trailing "---" YAML document
    # separator `ros2 topic echo` emits after every message, so without
    # `head -n1` this returns two lines ("1787641779\n---") and the
    # arithmetic below chokes on the second — verified against a real
    # container run: entrypoint.sh crashed with "syntax error: operand
    # expected (error token is \"-\")" ~1-2s into every watchdog loop.
    timeout 2 ros2 topic echo --once --field stamp.sec /mc/manager/heartbeat 2>/dev/null | head -n1
}

echo "==> watchdog supervisor active (heartbeat staleness budget: ${EXECUTOR_WD_TIMEOUT_S}s)"
while true; do
    # A SIGTERM/SIGINT (docker stop / ExecStop) already ran on_signal via
    # the trap above — that's an intentional stop, not a fault, so exit
    # clean (0) rather than falling into the failure branches below and
    # spuriously triggering Restart=on-failure.
    [[ "${shutdown_requested}" -eq 1 ]] && break

    if ! kill -0 "${manager_pid}" 2>/dev/null; then
        echo "==> watchdog: mission_manager_node (pid ${manager_pid}) is no longer running" >&2
        terminate
        exit 1
    fi
    if ! kill -0 "${planner_pid}" 2>/dev/null; then
        echo "==> watchdog: search_planner_node (pid ${planner_pid}) is no longer running" >&2
        terminate
        exit 1
    fi

    hb_epoch_s="$(last_heartbeat_epoch_s || true)"
    if [[ -n "${hb_epoch_s}" ]]; then
        now_s=$(date +%s)
        age_s=$(( now_s - hb_epoch_s ))
        if (( age_s > EXECUTOR_WD_TIMEOUT_S )); then
            echo "==> watchdog: /mc/manager/heartbeat stale (${age_s}s > ${EXECUTOR_WD_TIMEOUT_S}s budget)" >&2
            terminate
            exit 1
        fi
    fi

    sleep 1
done

exit 0
