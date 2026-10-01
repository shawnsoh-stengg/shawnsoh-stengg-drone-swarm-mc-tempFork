#!/usr/bin/env bash
# 09b-smoke-test-routing.sh — Verify mavlink-router channel isolation
# (GCS-001/MAVR-001): GCS<->FCC (Channel A), GCS<->MC (Channel B), and
# Orchestrator<->MC (Channel C, GCS-006) never cross-forward. Ref: SDD
# §13.2 smoke test pattern. Channel C's own deployment (its container/unit
# wiring, plan-persistence, end-to-end MAVFTP check) is
# 15b-smoke-test-mission-orchestrator.sh's job — this file only asserts
# the isolation property, matching its existing scope for A/B.
#
# Static checks (always run): each channel's .conf declares only its own
# endpoints/ports, with no address or port belonging to another channel ·
# entrypoint.sh runs each channel as its own independent mavlink-routerd
# process, not one shared config · the systemd unit bind-mounts
# /etc/drone-mc read-only · scripts/02-install-systemd-units.sh provisions
# all three conf files onto the host. Also verifies Channel A's one
# deliberate exception (GCS-005's MAVROS_CHANNEL_A endpoint, channel-a.conf's
# own header comment) is exactly what's documented — present, loopback-only,
# and not colliding with any Channel B/C port — rather than an accidental
# widening of what was previously a strict two-party channel. Also checks
# (always, when this host has been provisioned) that the *deployed*
# /etc/drone-mc/mavlink-router-channel-*.conf files were actually rendered
# from this drone's current /etc/drone-mc/environment — the template checks
# above render an in-memory copy against a representative default and can't
# catch a stale on-disk file left over from before FCC_ETH_IP/GCS_IP were
# last edited.
#
# Set LIVE_CHECK=1 to additionally start the built image against
# loopback-rewritten copies of both conf files and use tcpdump to confirm a
# distinctive payload sent into one channel never appears on the other
# (requires the mavlink-router image to already exist on this host, and
# `tcpdump`/`socat` available).
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMPL_A="${REPO_ROOT}/config/mavlink-router-channel-a.conf"
TMPL_B="${REPO_ROOT}/config/mavlink-router-channel-b.conf"
TMPL_C="${REPO_ROOT}/config/mavlink-router-channel-c.conf"
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

check "channel-a.conf present"  test -f "${TMPL_A}"
check "channel-b.conf present"  test -f "${TMPL_B}"

# INFRA-008: these three .conf files are envsubst templates now — render
# them against representative (production-default) values before asserting
# content, same as scripts/02-install-systemd-units.sh does against a real
# drone's /etc/drone-mc/environment. Every check below keeps asserting the
# exact same literal values as before this change.
RENDER_DIR="$(mktemp -d)"
trap 'rm -rf "${RENDER_DIR}"' EXIT
: "${FCC_ETH_IP:=192.168.10.31}"
: "${GCS_IP:=192.168.10.10}"
export FCC_ETH_IP GCS_IP
CONF_A="${RENDER_DIR}/mavlink-router-channel-a.conf"
CONF_B="${RENDER_DIR}/mavlink-router-channel-b.conf"
CONF_C="${RENDER_DIR}/mavlink-router-channel-c.conf"
envsubst '$FCC_ETH_IP' < "${TMPL_A}" > "${CONF_A}"
envsubst '$GCS_IP' < "${TMPL_B}" > "${CONF_B}"
envsubst '$GCS_IP' < "${TMPL_C}" > "${CONF_C}"

# Channel A: GCS<->FCC only, port 14550.
check "channel-a: FCC endpoint at 192.168.10.31:14550" \
    grep -qE 'Address = 192\.168\.10\.31' "${CONF_A}"
check "channel-a: port 14550 declared" grep -qE 'Port = 14550' "${CONF_A}"
check "channel-a: no Channel B port 14551" bash -c "! grep -qE 'Port = 14551' '${CONF_A}'"
check "channel-a: no Channel B port 14552" bash -c "! grep -qE 'Port = 14552' '${CONF_A}'"
check "channel-a: no Channel B port 14560" bash -c "! grep -qE 'Port = 14560' '${CONF_A}'"
check "channel-a: no GCS_MAVLINK_NODE endpoint" bash -c "! grep -q 'GCS_MAVLINK_NODE' '${CONF_A}'"

# MAVROS_CHANNEL_A (GCS-005) is a deliberate exception to the "no MC-facing
# endpoint" property Channel A otherwise still holds — verify it's the
# narrow, reviewed addition documented in channel-a.conf's own header, not
# an accidental extra endpoint or a cross-channel port collision.
check "channel-a: MAVROS endpoint present" grep -q 'MAVROS_CHANNEL_A' "${CONF_A}"
check "channel-a: MAVROS endpoint on loopback" \
    bash -c "awk '/\[UdpEndpoint MAVROS_CHANNEL_A\]/{f=1} f && /^Address/{print; exit}' '${CONF_A}' | grep -qE '127\.0\.0\.1'"
check "channel-a: MAVROS port 14553 declared" grep -qE 'Port = 14553' "${CONF_A}"
check "channel-a: MAVROS port does not collide with Channel B (14551/14552/14560)" \
    bash -c "! grep -qE 'Port = 1455[12]$|Port = 14560' '${CONF_A}'"
check "channel-a: MAVROS port does not collide with Channel C (14571/14572/14580)" \
    bash -c "! grep -qE 'Port = 1457[12]$|Port = 14580' '${CONF_A}'"
check "channel-a: exactly one MAVROS endpoint (no accidental duplicate)" \
    bash -c "[[ \$(grep -c '\[UdpEndpoint MAVROS_CHANNEL_A\]' '${CONF_A}') -eq 1 ]]"

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
check "channel-b: no Channel C ports (14571/14572/14580)" \
    bash -c "! grep -qE 'Port = 1457[12]$|Port = 14580' '${CONF_B}'"

# Channel C (GCS-006): Orchestrator<->MC only, ports 14571/14572/14580.
# mission_orchestrator_node's own deployment (container/unit/persistence/
# end-to-end MAVFTP) is verified by
# 15b-smoke-test-mission-orchestrator.sh — this is only the isolation
# property, matching this file's existing scope for A/B.
check "channel-c.conf present" test -f "${TMPL_C}"
check "channel-c: Orchestrator endpoint at 192.168.10.10:14571" \
    grep -qE 'Address = 192\.168\.10\.10' "${CONF_C}"
check "channel-c: mission_orchestrator_node loopback at 127.0.0.1:14580" \
    grep -qE 'Address = 127\.0\.0\.1' "${CONF_C}"
check "channel-c: port 14571 declared" grep -qE 'Port = 14571' "${CONF_C}"
check "channel-c: port 14572 declared" grep -qE 'Port = 14572' "${CONF_C}"
check "channel-c: port 14580 declared" grep -qE 'Port = 14580' "${CONF_C}"
check "channel-c: no Channel A port 14550" bash -c "! grep -qE 'Port = 14550' '${CONF_C}'"
check "channel-c: no Channel B ports (14551/14552/14560)" \
    bash -c "! grep -qE 'Port = 1455[12]$|Port = 14560' '${CONF_C}'"
check "channel-c: no FCC address 192.168.10.31" bash -c "! grep -qE '192\.168\.10\.31' '${CONF_C}'"

# Isolation is by construction (independent processes, no shared config) —
# verify the entrypoint actually launches three independent mavlink-routerd
# processes rather than one process reading multiple conf files.
check "entrypoint launches channel-a as its own process" \
    grep -qE 'mavlink-routerd -c.*mavlink-router-channel-a\.conf' "${ENTRYPOINT}"
check "entrypoint launches channel-b as its own process" \
    grep -qE 'mavlink-routerd -c.*mavlink-router-channel-b\.conf' "${ENTRYPOINT}"
check "entrypoint launches channel-c as its own process" \
    grep -qE 'mavlink-routerd -c.*mavlink-router-channel-c\.conf' "${ENTRYPOINT}"
check "entrypoint does not pass multiple conf files to one process" \
    bash -c "! grep -qE 'mavlink-router-channel-a\.conf.*mavlink-router-channel-b\.conf|mavlink-router-channel-a\.conf.*mavlink-router-channel-c\.conf|mavlink-router-channel-b\.conf.*mavlink-router-channel-c\.conf' '${ENTRYPOINT}'"

check "drone-mc-mavlink-router.service bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"
check "install script provisions channel-a.conf" \
    grep -q 'mavlink-router-channel-a.conf' "${INSTALL_SCRIPT}"
check "install script provisions channel-b.conf" \
    grep -q 'mavlink-router-channel-b.conf' "${INSTALL_SCRIPT}"
check "install script provisions channel-c.conf" \
    grep -q 'mavlink-router-channel-c.conf' "${INSTALL_SCRIPT}"

# Deployed-config drift check (always runs, no LIVE_CHECK needed — just a
# grep, not a container/tcpdump run): everything above only proves the
# *template* has the right shape when rendered against a representative
# default. It never looks at what's actually on this host, so it can't
# catch e.g. FCC_ETH_IP/GCS_IP having been edited in
# /etc/drone-mc/environment without a follow-up
# scripts/02-install-systemd-units.sh run to re-render the deployed
# .conf files. Skipped on a plain dev checkout that was never provisioned.
LIVE_ENV="/etc/drone-mc/environment"
LIVE_CONF_A="/etc/drone-mc/mavlink-router-channel-a.conf"
LIVE_CONF_B="/etc/drone-mc/mavlink-router-channel-b.conf"
LIVE_CONF_C="/etc/drone-mc/mavlink-router-channel-c.conf"
if [[ -f "${LIVE_ENV}" ]]; then
    echo "==> Checking deployed configs on this host against /etc/drone-mc/environment"
    LIVE_FCC_ETH_IP="$(grep -E '^FCC_ETH_IP=' "${LIVE_ENV}" | tail -n1 | cut -d= -f2- || true)"
    LIVE_GCS_IP="$(grep -E '^GCS_IP=' "${LIVE_ENV}" | tail -n1 | cut -d= -f2- || true)"
    if [[ -n "${LIVE_FCC_ETH_IP}" && -f "${LIVE_CONF_A}" ]]; then
        check "deployed channel-a.conf matches this drone's FCC_ETH_IP (${LIVE_FCC_ETH_IP})" \
            grep -qE "Address = ${LIVE_FCC_ETH_IP//./\\.}\$" "${LIVE_CONF_A}"
    fi
    if [[ -n "${LIVE_GCS_IP}" && -f "${LIVE_CONF_B}" ]]; then
        check "deployed channel-b.conf matches this drone's GCS_IP (${LIVE_GCS_IP})" \
            grep -qE "Address = ${LIVE_GCS_IP//./\\.}\$" "${LIVE_CONF_B}"
    fi
    if [[ -n "${LIVE_GCS_IP}" && -f "${LIVE_CONF_C}" ]]; then
        check "deployed channel-c.conf matches this drone's GCS_IP (${LIVE_GCS_IP})" \
            grep -qE "Address = ${LIVE_GCS_IP//./\\.}\$" "${LIVE_CONF_C}"
    fi
else
    echo "==> No /etc/drone-mc/environment on this host — skipping deployed-config drift check" \
         "(run scripts/02-install-systemd-units.sh first to provision it)."
fi

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: verifying zero cross-channel leakage via tcpdump"

    for bin in tcpdump socat; do
        command -v "${bin}" >/dev/null || { echo "==> ERROR: ${bin} required for LIVE_CHECK" >&2; exit 1; }
    done

    WORKDIR="$(mktemp -d)"
    trap 'rm -rf "${WORKDIR}" "${RENDER_DIR}"; [[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true; [[ -n "${tcpdump_pid:-}" ]] && kill "${tcpdump_pid}" &>/dev/null || true' EXIT

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
