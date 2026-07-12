#!/usr/bin/env bash
# 09b-smoke-test-routing.sh — Verify mavlink-router channel isolation
# (GCS-001/MAVR-001): GCS<->FCC (Channel A) and GCS<->MC (Channel B) never
# cross-forward. Ref: SDD §13.2 smoke test pattern.
#
# Static checks (always run): each channel's .conf declares only its own
# endpoints/ports, with no address or port belonging to the other channel ·
# entrypoint.sh runs the two channels as independent mavlink-routerd
# processes, not one shared config · the systemd unit bind-mounts
# /etc/drone-mc read-only · scripts/02-install-systemd-units.sh provisions
# both conf files onto the host.
#
# Set LIVE_CHECK=1 to additionally start the built image against
# loopback-rewritten copies of both conf files and use tcpdump to confirm a
# distinctive payload sent into one channel never appears on the other
# (requires the mavlink-router image to already exist on this host, and
# `tcpdump`/`socat` available).
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONF_A="${REPO_ROOT}/config/mavlink-router-channel-a.conf"
CONF_B="${REPO_ROOT}/config/mavlink-router-channel-b.conf"
ENTRYPOINT="${REPO_ROOT}/docker/mavlink-router/entrypoint.sh"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-mavlink-router.service"
INSTALL_SCRIPT="${REPO_ROOT}/scripts/02-install-systemd-units.sh"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/mavlink-router:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/mavlink-router:latest}"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS + 1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL + 1))
    fi
}

check "channel-a.conf present"  test -f "${CONF_A}"
check "channel-b.conf present"  test -f "${CONF_B}"

# Channel A: GCS<->FCC only, port 14550.
check "channel-a: FCC endpoint at 192.168.10.31:14550" \
    grep -qE 'Address = 192\.168\.10\.31' "${CONF_A}"
check "channel-a: port 14550 declared" grep -qE 'Port = 14550' "${CONF_A}"
check "channel-a: no Channel B port 14551" bash -c "! grep -qE 'Port = 14551' '${CONF_A}'"
check "channel-a: no Channel B port 14552" bash -c "! grep -qE 'Port = 14552' '${CONF_A}'"
check "channel-a: no Channel B port 14560" bash -c "! grep -qE 'Port = 14560' '${CONF_A}'"
check "channel-a: no GCS_MAVLINK_NODE endpoint" bash -c "! grep -q 'GCS_MAVLINK_NODE' '${CONF_A}'"

# Channel B: GCS<->MC only, ports 14551/14552/14560.
check "channel-b: GCS endpoint at 192.168.10.10:14551" \
    grep -qE 'Address = 192\.168\.10\.10' "${CONF_B}"
check "channel-b: gcs_mavlink_node loopback at 127.0.0.1:14560" \
    grep -qE 'Address = 127\.0\.0\.1' "${CONF_B}"
check "channel-b: port 14551 declared" grep -qE 'Port = 14551' "${CONF_B}"
check "channel-b: port 14552 declared" grep -qE 'Port = 14552' "${CONF_B}"
check "channel-b: port 14560 declared" grep -qE 'Port = 14560' "${CONF_B}"
check "channel-b: no Channel A port 14550" bash -c "! grep -qE 'Port = 14550' '${CONF_B}'"
check "channel-b: no FCC address 192.168.10.31" bash -c "! grep -qE '192\.168\.10\.31' '${CONF_B}'"

# Isolation is by construction (two processes, no shared config) — verify
# the entrypoint actually launches two independent mavlink-routerd
# processes rather than one process reading both conf files.
check "entrypoint launches channel-a as its own process" \
    grep -qE 'mavlink-routerd -c.*mavlink-router-channel-a\.conf' "${ENTRYPOINT}"
check "entrypoint launches channel-b as its own process" \
    grep -qE 'mavlink-routerd -c.*mavlink-router-channel-b\.conf' "${ENTRYPOINT}"
check "entrypoint does not pass both conf files to one process" \
    bash -c "! grep -qE 'mavlink-router-channel-a\.conf.*mavlink-router-channel-b\.conf' '${ENTRYPOINT}'"

check "drone-mc-mavlink-router.service bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
check "install script provisions channel-a.conf" \
    grep -q 'mavlink-router-channel-a.conf' "${INSTALL_SCRIPT}"
check "install script provisions channel-b.conf" \
    grep -q 'mavlink-router-channel-b.conf' "${INSTALL_SCRIPT}"

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: verifying zero cross-channel leakage via tcpdump"

    for bin in tcpdump socat; do
        command -v "${bin}" >/dev/null || { echo "==> ERROR: ${bin} required for LIVE_CHECK" >&2; exit 1; }
    done

    WORKDIR="$(mktemp -d)"
    trap 'rm -rf "${WORKDIR}"; [[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true; [[ -n "${tcpdump_pid:-}" ]] && kill "${tcpdump_pid}" &>/dev/null || true' EXIT

    # Loopback-rewritten copies: real 192.168.10.x addresses aren't routable
    # in a sandboxed test host, so both channels run entirely on 127.0.0.1,
    # each endpoint given its own distinct port so they can coexist in one
    # test run without colliding or self-looping (channel-a.conf's two
    # endpoints share port 14550 in production, since GCS and FCC are
    # different physical addresses there — on loopback they need to differ).
    awk '/\[UdpEndpoint GCS_CHANNEL_A\]/{sec=1} /\[UdpEndpoint FCC_CHANNEL_A\]/{sec=2}
         sec==2 && /^Address/{print "Address = 127.0.0.1"; next}
         sec==2 && /^Port/{print "Port = 25553"; next}
         /^Port = 14550/{print "Port = 25550"; next}
         {print}' "${CONF_A}" > "${WORKDIR}/mavlink-router-channel-a.conf"
    sed -e 's/192\.168\.10\.10/127.0.0.1/' -e 's/14551/25551/g' -e 's/14552/25552/g' -e 's/14560/25560/g' "${CONF_B}" > "${WORKDIR}/mavlink-router-channel-b.conf"

    cid="$(docker run -d --rm --network=host \
        --mount "type=bind,src=${WORKDIR},dst=/etc/drone-mc,readonly" \
        "${IMAGE}")"
    sleep 1
    check "mavlink-router container is running" docker inspect -f '{{.State.Running}}' "${cid}"

    tcpdump -i lo -n -w "${WORKDIR}/capture.pcap" \
        'udp port 25550 or udp port 25553 or udp port 25560' &
    tcpdump_pid=$!
    sleep 1

    # Distinctive payload into Channel B's GCS-facing inbound port (25552,
    # the GCS_MC_RECV Server endpoint) — must reach gcs_mavlink_node's
    # loopback (25560) but never either Channel A port (25550 GCS-facing,
    # 25553 FCC-facing).
    printf 'CHANNEL_B_MARKER' | socat -t1 - UDP-DATAGRAM:127.0.0.1:25552,bind=127.0.0.1:0 || true
    sleep 1
    kill "${tcpdump_pid}" 2>/dev/null || true
    wait "${tcpdump_pid}" 2>/dev/null || true

    check "Channel B payload did not leak onto Channel A GCS-facing port (25550)" \
        bash -c "! tcpdump -r '${WORKDIR}/capture.pcap' -n udp port 25550 -A 2>/dev/null | grep -q CHANNEL_B_MARKER"
    check "Channel B payload did not leak onto Channel A FCC-facing port (25553)" \
        bash -c "! tcpdump -r '${WORKDIR}/capture.pcap' -n udp port 25553 -A 2>/dev/null | grep -q CHANNEL_B_MARKER"
    check "Channel B payload reached gcs_mavlink_node loopback (25560)" \
        bash -c "tcpdump -r '${WORKDIR}/capture.pcap' -n udp port 25560 -A 2>/dev/null | grep -q CHANNEL_B_MARKER"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
