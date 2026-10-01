#!/usr/bin/env bash
# entrypoint.sh — launches gnss_sim_node (indoor_nav). Nothing reaches the
# FCC until an operator anchors it (MC_CMD_INDOOR_GNSS_SIM_ANCHOR, 50005),
# and IGS-005 refuses the anchor while a real GPS fix is present.
set -euo pipefail

# colcon's setup.bash reads $COLCON_TRACE unset-unsafe — see
# docker/mission-exec/entrypoint.sh.
set +u
source /opt/ros/humble/install/setup.bash
source /opt/drone-mc/install/setup.bash
set -u

PARAMS_FILE=/etc/drone-mc/params/indoor_nav.yaml

echo "==> starting gnss_sim_node (HIL_GPS -> ${FCC_ETH_IP:-<unset>}:${FCC_GPS_INPUT_PORT:-14557}, sysid ${DRONE_ID:-<unset>})"
ros2 run indoor_nav gnss_sim_node --ros-args --params-file "${PARAMS_FILE}" &
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
