#!/usr/bin/env bash
# entrypoint.sh — launches three independent mavlink-routerd processes, one
# per GCS channel (GCS-001/MAVR-001 channel isolation for A/B; Channel C
# added by GCS-006 to close the gap SDD-MC-001.md §12.4 documented — this
# process previously didn't exist anywhere, so mission_orchestrator_node
# had no MAVLink path to a real GCS-side Orchestrator). They do not share a
# process or a routing table, so there is no code path by which one
# channel's traffic can reach another — see the channel .conf files'
# headers for why a single shared process can't provide this guarantee.
#
# Any process dying takes the container down (Restart=on-failure in
# drone-mc-mavlink-router.service brings all three back together), matching
# this repo's one-container-one-health-status deployment model. Channel C
# stays a third process in this same container rather than a dedicated
# unit/container of its own — same isolation guarantee (independent
# process, independent routing table) without a fourth container to
# schedule, restart-policy, and resource-limit separately for what is, like
# A and B, just one more mavlink-routerd invocation against its own .conf.
set -euo pipefail

CONF_DIR="${MAVLINK_ROUTER_CONF_DIR:-/etc/drone-mc}"

mavlink-routerd -c "${CONF_DIR}/mavlink-router-channel-a.conf" &
pid_a=$!
mavlink-routerd -c "${CONF_DIR}/mavlink-router-channel-b.conf" &
pid_b=$!
mavlink-routerd -c "${CONF_DIR}/mavlink-router-channel-c.conf" &
pid_c=$!

terminate() {
    kill -TERM "${pid_a}" "${pid_b}" "${pid_c}" 2>/dev/null || true
    wait "${pid_a}" 2>/dev/null || true
    wait "${pid_b}" 2>/dev/null || true
    wait "${pid_c}" 2>/dev/null || true
}
trap terminate SIGTERM SIGINT

wait -n "${pid_a}" "${pid_b}" "${pid_c}"
exit_code=$?
terminate
exit "${exit_code}"
