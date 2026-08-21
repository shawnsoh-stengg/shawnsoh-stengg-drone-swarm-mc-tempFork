#!/usr/bin/env bash
# entrypoint.sh — launches gcs_mavlink_node ahead of MAVROS (GCS-005 AC1:
# "starts gcs_mavlink_node and MAVROS in order"). The two don't actually
# depend on each other at the transport level — gcs_mavlink_node owns
# Channel B (config/mavlink-router-channel-b.conf), MAVROS mirrors Channel A
# (mavlink-router-channel-a.conf's MAVROS_CHANNEL_A endpoint, ICD §4.2) — but
# ordering is still enforced explicitly here rather than left to a bare
# race, so container startup is deterministic and observable.
#
# The two processes share this container's lifecycle: either dying takes
# the whole container down, matching docker/fcc-bridge/entrypoint.sh's
# Agent+node pattern — drone-mc-gcs-comms.service's Restart=on-failure then
# brings both back together (GCS-005 AC3).
set -euo pipefail

# colcon-generated setup.bash scripts read $COLCON_TRACE without a default
# (`_colcon_prefix_chain_bash_source_script()`'s `[ -n "$COLCON_TRACE" ]`),
# so sourcing them under `set -u` aborts immediately with "COLCON_TRACE:
# unbound variable" — nounset is relaxed for just these lines, same as
# docker/mission-exec/entrypoint.sh and docker/health-monitor/entrypoint.sh.
set +u
source /opt/ros/humble/install/setup.bash
# ros-humble-mavros (apt package) lands under the dpkg prefix
# /opt/ros/humble, a different subtree than this base image's own
# source-built /opt/ros/humble/install — ros2-core's ENV AMENT_PREFIX_PATH/
# CMAKE_PREFIX_PATH already covers both for package *lookup*, but running
# an apt-installed node's actual binary also needs whatever
# LD_LIBRARY_PATH/PYTHONPATH the dpkg-provided /opt/ros/humble/setup.bash
# sets up — source it too, defensively, same optional-source guard
# ros2-core/Dockerfile's own .bashrc lines use.
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
source /opt/drone-mc/install/setup.bash
set -u

DRONE_ID="${DRONE_ID:-1}"

echo "==> starting gcs_mavlink_node"
ros2 run gcs_comms gcs_mavlink_node --ros-args --params-file /etc/drone-mc/params/gcs_comms.yaml &
gcs_mavlink_pid=$!

# Bounded wait, not a fixed sleep — AC1's 30 s budget covers both nodes
# appearing in `ros2 node list`, so don't spend an arbitrary slice of it
# guessing at a sleep duration.
for _ in $(seq 1 25); do
    ros2 node list 2>/dev/null | grep -q '/gcs_mavlink_node' && break
    sleep 1
done
echo "==> gcs_mavlink_node registered, starting mavros"

# mavros_node is MAVROS's own hardcoded default ROS node name — not
# something this project chose, and not renamed here.
ros2 run mavros mavros_node --ros-args \
    --params-file /etc/drone-mc/params/mavros.yaml \
    -p target_system_id:="${DRONE_ID}" &
mavros_pid=$!

terminate() {
    kill -TERM "${gcs_mavlink_pid}" "${mavros_pid}" 2>/dev/null || true
    wait "${gcs_mavlink_pid}" 2>/dev/null || true
    wait "${mavros_pid}" 2>/dev/null || true
}
trap terminate SIGTERM SIGINT

wait -n "${gcs_mavlink_pid}" "${mavros_pid}"
exit_code=$?
terminate
exit "${exit_code}"
