#!/usr/bin/env bash
# entrypoint.sh — launches two independent mavlink-routerd processes, one
# per GCS channel (GCS-001/MAVR-001 channel isolation). They do not share a
# process or a routing table, so there is no code path by which Channel B
# (GCS<->MC) traffic can reach Channel A (GCS<->FCC) or vice versa — see
# the channel .conf files' headers for why a single shared process can't
# provide this guarantee.
#
# Either process dying takes the container down (Restart=on-failure in
# drone-mc-mavlink-router.service brings both back together), matching this
# repo's one-container-one-health-status deployment model.
set -euo pipefail

CONF_DIR="${MAVLINK_ROUTER_CONF_DIR:-/etc/drone-mc}"

mavlink-routerd -c "${CONF_DIR}/mavlink-router-channel-a.conf" &
pid_a=$!
mavlink-routerd -c "${CONF_DIR}/mavlink-router-channel-b.conf" &
pid_b=$!

terminate() {
    kill -TERM "${pid_a}" "${pid_b}" 2>/dev/null || true
    wait "${pid_a}" 2>/dev/null || true
    wait "${pid_b}" 2>/dev/null || true
}
trap terminate SIGTERM SIGINT

wait -n "${pid_a}" "${pid_b}"
exit_code=$?
terminate
exit "${exit_code}"
