#!/usr/bin/env bash
# entrypoint.sh — HEALTH-004: launches health_monitor_node as a background
# job, trapped so a `docker stop`/systemd SIGTERM reaches the actual node
# process cleanly (ros2 run execs the node as a *child* of a Python
# wrapper, not via exec — signalling that wrapper alone does not reach the
# child, per 08b-smoke-test-fcc-bridge.sh's own comment on this same
# gotcha), rather than a bare `exec ros2 run ...`.
set -euo pipefail

# See docker/mission-exec/entrypoint.sh's own comment: colcon-generated
# setup.bash scripts read $COLCON_TRACE without a default, so sourcing
# them under `set -u` aborts with "COLCON_TRACE: unbound variable".
set +u
source /opt/ros/humble/install/setup.bash
source /opt/drone-mc/install/setup.bash
set -u

PARAMS_FILE=/etc/drone-mc/params/health_monitor.yaml

echo "==> starting health_monitor_node"
ros2 run health_monitor health_monitor_node --ros-args --params-file "${PARAMS_FILE}" &
node_pid=$!

terminate() {
    kill -TERM "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
}
trap terminate SIGTERM SIGINT

wait "${node_pid}"
exit_code=$?
terminate
exit "${exit_code}"
