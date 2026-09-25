#!/usr/bin/env bash
# 01-install-os.sh — Base OS configuration on the Jetson: eth0 Netplan
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

if ! command -v envsubst &>/dev/null; then
    echo "==> envsubst not found, installing gettext-base"
    if ! timeout 10 getent hosts archive.ubuntu.com &>/dev/null; then
        echo "ERROR: cannot resolve archive.ubuntu.com — check network connectivity" \
             "and retry." >&2
        exit 1
    fi
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -o Acquire::Retries=2 -o Acquire::http::Timeout=10 -o Acquire::https::Timeout=10
    apt-get install -y --no-install-recommends gettext-base
fi

# INFRA-008: config/netplan/01-drone-mc.yaml is an envsubst template
# (MC_ETH_MAC/MC_ETH_IP) — /etc/drone-mc/environment is
# this drone's source of truth for those values, so it must exist before
# rendering. This mirrors 02-install-systemd-units.sh's own idempotent
# provisioning of the same file: install the repo default only if this
# drone has no per-drone copy yet, never overwrite an existing one.
echo "==> Provisioning /etc/drone-mc/environment (for netplan template rendering)"
mkdir -p /etc/drone-mc
if [[ -f /etc/drone-mc/environment ]]; then
    echo "    /etc/drone-mc/environment already exists — leaving per-drone values untouched."
else
    install -m 644 "${REPO_ROOT}/config/environment" /etc/drone-mc/environment
    echo "    Installed default environment. Edit DRONE_ID/MC_ETH_MAC/MC_ETH_IP/" \
         "FCC_ETH_IP/FCC_TRANSPORT for this vehicle, then re-run this" \
         "script to render netplan with the real values."
fi

# Single-IP FCC migration (2026-09-25): the Pixhawk's NuttX stack has
# exactly one IPv4 address, so the old dual-homed layout (FCC_ETH_IP on a
# 192.168.12.x internal subnet + FCC_GCS_IP on 192.168.10.x, and the MC's
# own second MC_ETH_INTERNAL_IP) was never real. FCC_ETH_IP is now the one
# FCC address. Fold an on-device FCC_GCS_IP into a still-192.168.12.x
# FCC_ETH_IP and drop both retired variables — before the backfill below,
# so it can't re-add them. Refuses to guess if the two disagree otherwise.
ENV_FILE=/etc/drone-mc/environment
old_fcc_eth="$(grep -E '^FCC_ETH_IP=' "${ENV_FILE}" | tail -n1 | cut -d= -f2-)"
old_fcc_gcs="$(grep -E '^FCC_GCS_IP=' "${ENV_FILE}" | tail -n1 | cut -d= -f2-)"
if [[ -n "${old_fcc_gcs}" ]]; then
    if [[ -z "${old_fcc_eth}" || "${old_fcc_eth}" == 192.168.12.* ]]; then
        sed -i '/^FCC_ETH_IP=/d' "${ENV_FILE}"
        echo "FCC_ETH_IP=${old_fcc_gcs}" >> "${ENV_FILE}"
        echo "    Migrated FCC_ETH_IP ${old_fcc_eth:-<unset>} -> ${old_fcc_gcs} (was FCC_GCS_IP; single-IP FCC)"
    elif [[ "${old_fcc_eth}" != "${old_fcc_gcs}" ]]; then
        echo "ERROR: ${ENV_FILE} has FCC_ETH_IP=${old_fcc_eth} and FCC_GCS_IP=${old_fcc_gcs}" \
             "— the FCC has one address now (FCC_ETH_IP). Set FCC_ETH_IP to the Pixhawk's" \
             "real address (\`netman show\`), delete FCC_GCS_IP, and re-run." >&2
        exit 1
    fi
    sed -i '/^FCC_GCS_IP=/d' "${ENV_FILE}"
    echo "    Retired FCC_GCS_IP from ${ENV_FILE}"
fi
if grep -qE '^FCC_ETH_IP=192\.168\.12\.' "${ENV_FILE}"; then
    echo "ERROR: ${ENV_FILE}'s FCC_ETH_IP is still on the retired 192.168.12.x subnet and" \
         "there's no FCC_GCS_IP to migrate from. Set it to the Pixhawk's real address" \
         "(\`netman show\` in the MAVLink console) and re-run." >&2
    exit 1
fi
if grep -q '^MC_ETH_INTERNAL_IP=' "${ENV_FILE}"; then
    sed -i '/^MC_ETH_INTERNAL_IP=/d' "${ENV_FILE}"
    echo "    Retired MC_ETH_INTERNAL_IP from ${ENV_FILE} (eth0 has a single address now)"
fi

# Per-variable migration: the file-level check above only seeds
# /etc/drone-mc/environment once, so a repo change to config/environment
# (e.g. INFRA-008 adding MC_ETH_MAC/MC_ETH_IP)
# never reaches an already-provisioned Jetson on its own. Backfill any
# variable present in the repo default but missing on-device, without
# touching existing per-drone values.
while IFS='=' read -r key value; do
    if ! grep -q "^${key}=" /etc/drone-mc/environment; then
        echo "${key}=${value}" >> /etc/drone-mc/environment
        echo "    Migrated new variable ${key}=${value} into /etc/drone-mc/environment" \
             "(review/edit this per-drone value if it needs to differ from the repo default)."
    fi
done < <(grep -E '^[A-Za-z_][A-Za-z0-9_]*=' "${REPO_ROOT}/config/environment")

echo "==> Rendering /etc/netplan/01-drone-mc.yaml (HIF-002/HIF-004 eth0)"
set -a
# shellcheck disable=SC1091
source /etc/drone-mc/environment
set +a
for var in MC_ETH_MAC MC_ETH_IP; do
    if [[ -z "${!var:-}" ]]; then
        echo "ERROR: ${var} is unset/empty in /etc/drone-mc/environment — envsubst would" \
             "silently render a blank value into netplan. Set it and re-run." >&2
        exit 1
    fi
done

# INFRA-008: MC_ETH_MAC must match this drone's own NIC, not a stale/copied
# value from another unit's provisioning (config/environment's repo default
# is jetson-nx-001's MAC). A mismatch means netplan's `match: macaddress:`
# matches zero interfaces — set-name never fires, eth0 never gets its
# addresses, and netplan's own duplicate-match resolution can hit an
# internal GLib assertion (g_ascii_strcasecmp on a NULL MAC) rather than
# failing cleanly. Catch it here instead, with the local MACs to compare
# against. Skippable with ALLOW_NON_JETSON=1 for a dev-workstation dry run,
# same escape hatch as the aarch64/Ubuntu precondition check above.
if [[ "${ALLOW_NON_JETSON:-0}" -ne 1 ]]; then
    mac_found=0
    for addr_file in /sys/class/net/*/address; do
        iface="$(basename "$(dirname "${addr_file}")")"
        [[ "${iface}" == "lo" ]] && continue
        if [[ "$(cat "${addr_file}")" == "${MC_ETH_MAC,,}" ]]; then
            mac_found=1
            break
        fi
    done
    if [[ "${mac_found}" -ne 1 ]]; then
        echo "ERROR: MC_ETH_MAC=${MC_ETH_MAC} in /etc/drone-mc/environment does not match" \
             "any local interface's MAC — netplan's macaddress match would select" \
             "nothing, eth0 would never get its addresses, and netplan apply may fail" \
             "with an opaque internal error instead of a clean one. Local interfaces:" >&2
        for addr_file in /sys/class/net/*/address; do
            iface="$(basename "$(dirname "${addr_file}")")"
            [[ "${iface}" == "lo" ]] && continue
            echo "    ${iface}: $(cat "${addr_file}")" >&2
        done
        echo "Set MC_ETH_MAC to this drone's real NIC MAC (usually the wired interface" \
             "above) and re-run. Set ALLOW_NON_JETSON=1 to skip this check for a" \
             "dev-workstation dry run." >&2
        exit 1
    fi
fi

RENDERED_NETPLAN="$(mktemp)"
trap 'rm -f "${RENDERED_NETPLAN}"' EXIT
envsubst '$MC_ETH_MAC $MC_ETH_IP' \
    < "${REPO_ROOT}/config/netplan/01-drone-mc.yaml" > "${RENDERED_NETPLAN}"
install -m 600 "${RENDERED_NETPLAN}" /etc/netplan/01-drone-mc.yaml
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
