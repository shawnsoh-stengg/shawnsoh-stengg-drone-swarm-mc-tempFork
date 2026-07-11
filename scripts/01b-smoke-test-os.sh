#!/usr/bin/env bash
# 01b-smoke-test-os.sh — Verify base OS configuration on the Jetson.
# TP-SM-001: Ubuntu 22.04 · aarch64 · kernel >= 5.10 · /dev/ttyTHS1 exists
# Ref: SDD §13.2 smoke test pattern, HID-MC-001 §3.3 (HIF-001), §4.2 (HIF-002/004)
set -euo pipefail

PASS=0; FAIL=0

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; ((PASS++))
    else
        printf "FAIL: %s\n" "$desc"; ((FAIL++))
    fi
}

kernel_at_least_5_10() {
    local ver major minor
    ver="$(uname -r | grep -oE '^[0-9]+\.[0-9]+')"
    major="${ver%%.*}"; minor="${ver##*.}"
    [[ "${major}" -gt 5 ]] || { [[ "${major}" -eq 5 && "${minor}" -ge 10 ]]; }
}

netplan_has_dual_ip() {
    ip -4 addr show eth0 | grep -q '192.168.10.11/24' &&
    ip -4 addr show eth0 | grep -q '192.168.12.1/24'
}

check "Ubuntu 22.04"                  grep -q 'VERSION_ID="22.04"' /etc/os-release
check "aarch64"                       bash -c '[[ "$(uname -m)" == "aarch64" ]]'
check "kernel >= 5.10"                kernel_at_least_5_10
check "/dev/ttyTHS1 exists"           test -e /dev/ttyTHS1
check "/etc/netplan/01-drone-mc.yaml" test -f /etc/netplan/01-drone-mc.yaml
check "/etc/udev/rules.d/99-drone-mc.rules" test -f /etc/udev/rules.d/99-drone-mc.rules
check "eth0 has MANET + FCC-internal IPs" netplan_has_dual_ip
check "ttyTHS1 group is dialout"      bash -c '[[ "$(stat -c %G /dev/ttyTHS1)" == "dialout" ]]'

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
