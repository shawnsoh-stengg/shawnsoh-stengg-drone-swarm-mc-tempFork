#!/usr/bin/env bash
# 02b-smoke-test-units.sh — Verify the drone-mc.target systemd orchestration.
# Checks: all drone-mc-*.service units loaded · drone-mc.target enabled ·
# rt.slice/be.slice valid · every unit declares After=/Requires=,
# Restart=on-failure, RestartSec=5s · dependency order matches SDD §12.4.
# Ref: SDD §13.2 smoke test pattern
#
# This verifies the installation statically — it does not require a full
# system boot or the service container images to exist. The end-to-end
# "topics live within 120 s of systemctl start drone-mc.target" criterion is
# a hardware test (TP-HW-005 style), run separately on the Jetson with the
# images built.
#
# Set LIVE_CHECK=1 to additionally start drone-mc.target and confirm every
# unit reaches the active state before stopping it again (requires the
# service images to already exist on this host).
set -euo pipefail

PASS=0; FAIL=0
UNIT_DIR="/etc/systemd/system"

# 10 MC services per SDD §12.4 (health-monitor, mavlink-router, zenoh-bridge,
# fcc-bridge, gcs-comms, mission-orchestrator (GCS-006), swarm-coord,
# mission-exec, vision) plus camera (split out of vision — see
# drone-mc-camera.service). mission-orchestrator was missing from this list
# until FT-004 caught it — it never affected `enabled`/unit-file checks
# (those loop over what's actually installed elsewhere), but this SERVICES
# array itself silently skipped it.
SERVICES=(
    drone-mc-health-monitor.service
    drone-mc-mavlink-router.service
    drone-mc-zenoh-bridge.service
    drone-mc-fcc-bridge.service
    drone-mc-gcs-comms.service
    drone-mc-mission-orchestrator.service
    drone-mc-swarm-coord.service
    drone-mc-mission-exec.service
    drone-mc-camera.service
    drone-mc-vision.service
)

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS+1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL+1))
    fi
}

unit_declares() {
    local unit="$1" key="$2"
    grep -qE "^${key}=" "${UNIT_DIR}/${unit}"
}

unit_restart_policy() {
    local unit="$1"
    grep -qE "^Restart=on-failure$" "${UNIT_DIR}/${unit}" &&
    grep -qE "^RestartSec=5s?$" "${UNIT_DIR}/${unit}"
}

check "drone-mc.target installed"      test -f "${UNIT_DIR}/drone-mc.target"
check "rt.slice installed"             test -f "${UNIT_DIR}/rt.slice"
check "be.slice installed"             test -f "${UNIT_DIR}/be.slice"
check "drone-mc.target loaded"         systemctl list-unit-files drone-mc.target
check "drone-mc.target enabled"        systemctl is-enabled --quiet drone-mc.target
# systemd-analyze verify walks the FULL transitive dependency closure of
# drone-mc.target (After=network-online.target etc.), which on a real
# Jetson pulls in unrelated pre-existing OS units (snapd, nvs-service,
# nvargus-daemon) that emit their own harmless upstream warnings
# ("Unknown key name ... ignoring", "syslog is obsolete"). Filtering by
# known-noise substring is a losing game against those, so instead assert
# nothing verify says is actually ABOUT one of our own files — that's the
# only thing this smoke test can meaningfully claim responsibility for.
check "systemd-analyze verify: no warnings/errors about our own units" bash -c \
    "! systemd-analyze verify ${UNIT_DIR}/drone-mc.target ${UNIT_DIR}/rt.slice ${UNIT_DIR}/be.slice ${UNIT_DIR}/drone-mc-*.service 2>&1 | grep -qE '${UNIT_DIR}/(drone-mc|rt\.slice|be\.slice)'"

for unit in "${SERVICES[@]}"; do
    check "${unit} installed"          test -f "${UNIT_DIR}/${unit}"
    check "${unit} loaded"             systemctl list-unit-files "${unit}"
    check "${unit} enabled"            systemctl is-enabled --quiet "${unit}"
    check "${unit} declares After="    unit_declares "${unit}" After
    check "${unit} declares Requires=" unit_declares "${unit}" Requires
    check "${unit} PartOf=drone-mc.target" unit_declares "${unit}" PartOf
    check "${unit} Restart=on-failure/RestartSec=5s" unit_restart_policy "${unit}"
done

# SDD §12.3: fcc-bridge and swarm-coord are the only rt.slice members.
check "fcc-bridge in rt.slice"         grep -qE "^Slice=rt.slice$" "${UNIT_DIR}/drone-mc-fcc-bridge.service"
check "swarm-coord in rt.slice"        grep -qE "^Slice=rt.slice$" "${UNIT_DIR}/drone-mc-swarm-coord.service"
check "mavlink-router in be.slice"     grep -qE "^Slice=be.slice$" "${UNIT_DIR}/drone-mc-mavlink-router.service"
check "mission-exec in be.slice"       grep -qE "^Slice=be.slice$" "${UNIT_DIR}/drone-mc-mission-exec.service"

# SDD §12.4 startup ordering: verify the dependency graph via each unit's
# declared After=, rather than the live job queue (no images -> can't start).
check "fcc-bridge after mavlink-router"   grep -E "^After=" "${UNIT_DIR}/drone-mc-fcc-bridge.service" | grep -q drone-mc-mavlink-router.service
check "gcs-comms after fcc-bridge"        grep -E "^After=" "${UNIT_DIR}/drone-mc-gcs-comms.service" | grep -q drone-mc-fcc-bridge.service
check "mission-orchestrator after fcc-bridge" grep -E "^After=" "${UNIT_DIR}/drone-mc-mission-orchestrator.service" | grep -q drone-mc-fcc-bridge.service
check "swarm-coord after gcs-comms"       grep -E "^After=" "${UNIT_DIR}/drone-mc-swarm-coord.service" | grep -q drone-mc-gcs-comms.service
check "mission-exec after swarm-coord"    grep -E "^After=" "${UNIT_DIR}/drone-mc-mission-exec.service" | grep -q drone-mc-swarm-coord.service
check "vision after fcc-bridge"           grep -E "^After=" "${UNIT_DIR}/drone-mc-vision.service" | grep -q drone-mc-fcc-bridge.service

check "/etc/drone-mc/environment present" test -f /etc/drone-mc/environment

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: starting drone-mc.target"
    systemctl start drone-mc.target
    deadline=$((SECONDS + 120))
    all_active=0
    while [[ ${SECONDS} -lt ${deadline} ]]; do
        all_active=1
        for unit in "${SERVICES[@]}"; do
            systemctl is-active --quiet "${unit}" || { all_active=0; break; }
        done
        [[ ${all_active} -eq 1 ]] && break
        sleep 2
    done
    check "all ${#SERVICES[@]} services active within 120s" bash -c "[[ ${all_active} -eq 1 ]]"

    systemctl stop drone-mc.target
    sleep 2
    all_stopped=1
    for unit in "${SERVICES[@]}"; do
        systemctl is-active --quiet "${unit}" && all_stopped=0
    done
    check "all ${#SERVICES[@]} services stopped cleanly" bash -c "[[ ${all_stopped} -eq 1 ]]"
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
