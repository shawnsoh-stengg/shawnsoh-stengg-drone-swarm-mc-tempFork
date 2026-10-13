#!/usr/bin/env bash
# 01b-smoke-test-os.sh — Verify base OS configuration on the Jetson.
# TP-SM-001: Ubuntu 22.04 · aarch64 · kernel >= 5.10 · /dev/ttyTHS1 exists ·
# CycloneDDS socket buffer floor (INFRA-004)
# Ref: SDD §13.2 smoke test pattern, HID-MC-001 §3.3 (HIF-001), §4.2 (HIF-002/004)
set -euo pipefail

PASS=0; FAIL=0

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS+1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL+1))
    fi
}

kernel_at_least_5_10() {
    local ver major minor
    ver="$(uname -r | grep -oE '^[0-9]+\.[0-9]+')"
    major="${ver%%.*}"; minor="${ver##*.}"
    [[ "${major}" -gt 5 ]] || { [[ "${major}" -eq 5 && "${minor}" -ge 10 ]]; }
}

netplan_has_mc_ip() {
    # INFRA-008: MC_ETH_IP is a per-drone value rendered
    # from /etc/drone-mc/environment, not the repo's config/environment
    # defaults — checking the hardcoded defaults here would fail on every
    # drone provisioned with its own IPs (see MC_ETH_MAC/IP override note
    # in config/netplan/01-drone-mc.yaml).
    # eth0 has a single address (the FCC shares its 192.168.100.x subnet —
    # config/netplan/01-drone-mc.yaml); a leftover 192.168.12.x address means
    # this host still runs the pre-2026-09-25 dual-IP netplan and
    # 01-install-os.sh needs re-running.
    local mc_eth_ip
    mc_eth_ip="$(grep -E '^MC_ETH_IP=' /etc/drone-mc/environment | cut -d= -f2-)"
    [[ -n "${mc_eth_ip}" ]] || return 1
    ip -4 addr show eth0 | grep -q "${mc_eth_ip}/24" &&
    ! ip -4 addr show eth0 | grep -q "inet 192\.168\.12\."
}

# cyclonedds.xml requires SocketReceiveBufferSize/SocketSendBufferSize
# min="10MB"/"1MB" (docker/ros2-core/cyclonedds.xml) — below this,
# rmw_cyclonedds_cpp fails domain creation and every DDS-using node aborts.
sysctl_at_least() {
    local key="$1" min_bytes="$2"
    [[ "$(sysctl -n "${key}" 2>/dev/null)" -ge "${min_bytes}" ]]
}

check "Ubuntu 22.04"                  grep -q 'VERSION_ID="22.04"' /etc/os-release
check "aarch64"                       bash -c '[[ "$(uname -m)" == "aarch64" ]]'
check "kernel >= 5.10"                kernel_at_least_5_10
check "/dev/ttyTHS1 exists"           test -e /dev/ttyTHS1
check "/etc/netplan/01-drone-mc.yaml" test -f /etc/netplan/01-drone-mc.yaml
check "/etc/udev/rules.d/99-drone-mc.rules" test -f /etc/udev/rules.d/99-drone-mc.rules
check "eth0 has MC_ETH_IP (single address)" netplan_has_mc_ip
check "ttyTHS1 group is dialout"      bash -c '[[ "$(stat -c %G /dev/ttyTHS1)" == "dialout" ]]'
check "/etc/sysctl.d/99-drone-mc.conf" test -f /etc/sysctl.d/99-drone-mc.conf
check "net.core.rmem_max >= 10MB (CycloneDDS floor)" sysctl_at_least net.core.rmem_max 10485760
check "net.core.wmem_max >= 1MB (CycloneDDS floor)"  sysctl_at_least net.core.wmem_max 1048576

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
