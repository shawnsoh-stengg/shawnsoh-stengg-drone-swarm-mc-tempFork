#!/usr/bin/env bash
# entrypoint.sh — launches swarm_coordinator_node and formation_controller_node
# together (DEP-001). Both are currently §2.4/§5/§6 scaffolds (no topics, no
# timers, just a startup log line) — this entrypoint deliberately doesn't
# invent a heartbeat/watchdog check the way docker/mission-exec/entrypoint.sh
# does for /mc/manager/heartbeat, since there's no equivalent topic to watch
# yet. Add one when a real liveness contract exists for these nodes.
#
# The two processes share this container's lifecycle: either dying takes the
# whole container down, matching docker/mavlink-router/entrypoint.sh's
# two-process pattern — drone-mc-swarm-coord.service's Restart=on-failure
# then brings both back together.
set -euo pipefail

# colcon-generated setup.bash scripts read $COLCON_TRACE without a default
# (`_colcon_prefix_chain_bash_source_script()`'s `[ -n "$COLCON_TRACE" ]`),
# so sourcing them under `set -u` aborts immediately with "COLCON_TRACE:
# unbound variable" — nounset is relaxed for just these two lines, same as
# docker/mission-exec/entrypoint.sh and docker/health-monitor/entrypoint.sh.
set +u
source /opt/ros/humble/install/setup.bash
source /opt/drone-mc/install/setup.bash
set -u

PARAMS_FILE=/etc/drone-mc/params/swarm_coord.yaml

echo "==> starting swarm_coordinator_node"
ros2 run swarm_coord swarm_coordinator_node --ros-args --params-file "${PARAMS_FILE}" &
coordinator_pid=$!

echo "==> starting formation_controller_node"
ros2 run swarm_coord formation_controller_node --ros-args --params-file "${PARAMS_FILE}" &
formation_pid=$!

terminate() {
    kill -TERM "${coordinator_pid}" "${formation_pid}" 2>/dev/null || true
    wait "${coordinator_pid}" 2>/dev/null || true
    wait "${formation_pid}" 2>/dev/null || true
}
trap terminate SIGTERM SIGINT

wait -n "${coordinator_pid}" "${formation_pid}"
exit_code=$?
terminate
exit "${exit_code}"
