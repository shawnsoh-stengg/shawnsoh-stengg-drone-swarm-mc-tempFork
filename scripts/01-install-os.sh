#!/usr/bin/env bash
# 01-install-os.sh — Base OS configuration on the Jetson: dual-IP Netplan
# (HIF-002/HIF-004), the ttyTHS1 udev rule (HIF-001), and the CycloneDDS
# socket buffer sysctl floor (INFRA-004).
# Companion smoke test: 01b-smoke-test-os.sh
#
# Prerequisite: the Jetson module has already been flashed with JetPack 6.2.2
# via NVIDIA SDK Manager (HID-MC-001 §8.1) and boots to a login prompt.
# This script runs directly on the Jetson host (not inside Docker) — it
# configures files under /etc that must exist before the drone-mc containers
# ever start.
#
# Usage:
#   sudo ./01-install-os.sh
#
# Set ALLOW_NON_JETSON=1 to skip the aarch64/Ubuntu 22.04 precondition check
# (useful when dry-running the netplan/udev install on a dev workstation).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"

if [[ "${EUID}" -ne 0 ]]; then
    echo "ERROR: must run as root (sudo ./01-install-os.sh)" >&2
    exit 1
fi

if [[ "${ALLOW_NON_JETSON:-0}" -ne 1 ]]; then
    if [[ "$(uname -m)" != "aarch64" ]]; then
        echo "ERROR: expected aarch64 (Jetson), got $(uname -m). Set ALLOW_NON_JETSON=1 to override." >&2
        exit 1
    fi
    if ! grep -q 'VERSION_ID="22.04"' /etc/os-release 2>/dev/null; then
        echo "ERROR: expected Ubuntu 22.04. Set ALLOW_NON_JETSON=1 to override." >&2
        exit 1
    fi
fi

if ! command -v netplan &>/dev/null; then
    echo "==> netplan not found, installing netplan.io"
    if ! timeout 10 getent hosts archive.ubuntu.com &>/dev/null; then
        echo "ERROR: cannot resolve archive.ubuntu.com — check network connectivity" \
             "(eth0/wifi must be online before this script can fetch netplan.io) and retry." >&2
        exit 1
    fi
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -o Acquire::Retries=2 -o Acquire::http::Timeout=10 -o Acquire::https::Timeout=10
    apt-get install -y --no-install-recommends netplan.io
fi

echo "==> Installing /etc/netplan/01-drone-mc.yaml (HIF-002/HIF-004 dual-IP eth0)"
install -m 600 "${REPO_ROOT}/config/netplan/01-drone-mc.yaml" /etc/netplan/01-drone-mc.yaml
netplan apply

echo "==> Installing /etc/udev/rules.d/99-drone-mc.rules (HIF-001 ttyTHS1 permissions)"
install -m 644 "${REPO_ROOT}/config/udev/99-drone-mc.rules" /etc/udev/rules.d/99-drone-mc.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

echo "==> Installing /etc/sysctl.d/99-drone-mc.conf (CycloneDDS socket buffer floor)"
install -m 644 "${REPO_ROOT}/config/sysctl/99-drone-mc.conf" /etc/sysctl.d/99-drone-mc.conf
sysctl --system &>/dev/null

TARGET_USER="${SUDO_USER:-${USER}}"
if ! id -nG "${TARGET_USER}" | grep -qw dialout; then
    echo "==> Adding ${TARGET_USER} to the dialout group"
    usermod -aG dialout "${TARGET_USER}"
    echo "    NOTE: ${TARGET_USER} must log out and back in for the group change to take effect."
fi

echo "==> OS configuration complete"
if ip -4 addr show eth0 &>/dev/null; then
    ip -4 addr show eth0 | grep inet || true
else
    echo "    NOTE: eth0 not present yet — the netplan match/set-name rename of the" \
         "physical NIC (e.g. enP8p1s0) to eth0 can require a reboot to take effect" \
         "if the interface was already up. Reboot and re-run 01b-smoke-test-os.sh to confirm."
fi
ls -l /dev/ttyTHS1 2>/dev/null || echo "    NOTE: /dev/ttyTHS1 not present yet — expected before the ttyTHS1<->Pixhawk link is wired up."
