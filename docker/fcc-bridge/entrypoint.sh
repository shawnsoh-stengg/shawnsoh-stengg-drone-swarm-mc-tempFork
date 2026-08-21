#!/usr/bin/env bash
# entrypoint.sh — launches the Micro XRCE-DDS Agent (FCC-008) ahead of
# fcc_bridge_node, transport selected by FCC_TRANSPORT exactly as
# HID-MC-001 §4.3 documents. XrceDdsBackend's own env var contract is
# unchanged by this — FCC_SERIAL_DEVICE/FCC_SERIAL_BAUD/FCC_ETH_PORT are
# the Agent's process args here, not read by the ROS node.
#
# The two processes share this container's lifecycle: fcc_bridge_node is
# useless without a live DDS bridge to the FCC and vice versa, so unlike
# mavlink-router's two independent GCS channels, either process dying here
# takes the whole container down — drone-mc-fcc-bridge.service's
# Restart=on-failure then brings both back together, matching the "agent
# lifecycle is supervised by systemd, not the application" requirement.
set -euo pipefail

FCC_TRANSPORT="${FCC_TRANSPORT:-serial}"

case "${FCC_TRANSPORT}" in
    serial)
        MicroXRCEAgent serial --dev "${FCC_SERIAL_DEVICE}" --baudrate "${FCC_SERIAL_BAUD}" --rts-cts &
        ;;
    ethernet)
        MicroXRCEAgent udp4 --port "${FCC_ETH_PORT}" &
        ;;
    *)
        echo "entrypoint.sh: unknown FCC_TRANSPORT '${FCC_TRANSPORT}' (expected serial|ethernet)" >&2
        exit 1
        ;;
esac
agent_pid=$!

# colcon-generated setup.bash scripts read $COLCON_TRACE without a default
# (`_colcon_prefix_chain_bash_source_script()`'s `[ -n "$COLCON_TRACE" ]`),
# so sourcing them under `set -u` aborts immediately with "COLCON_TRACE:
# unbound variable" — nounset is relaxed for just these two lines, same as
# docker/mission-exec/entrypoint.sh and docker/health-monitor/entrypoint.sh.
set +u
source /opt/ros/humble/install/setup.bash
source /opt/drone-mc/install/setup.bash
set -u
ros2 run fcc_bridge fcc_bridge_node --ros-args --params-file /etc/drone-mc/params/fcc_bridge.yaml &
node_pid=$!

terminate() {
    kill -TERM "${agent_pid}" "${node_pid}" 2>/dev/null || true
    wait "${agent_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
}
trap terminate SIGTERM SIGINT

wait -n "${agent_pid}" "${node_pid}"
exit_code=$?
terminate
exit "${exit_code}"
