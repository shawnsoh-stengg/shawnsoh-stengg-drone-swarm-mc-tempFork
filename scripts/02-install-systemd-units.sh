#!/usr/bin/env bash
# 02-install-systemd-units.sh — Install the drone-mc.target orchestration:
# the target, all drone-mc-*.service units, and the rt.slice/be.slice cgroup
# configs (SDD-MC-001 §12.2-§12.4).
# Companion smoke test: 02b-smoke-test-units.sh
#
# This script runs directly on the Jetson host (not inside Docker) — it
# installs unit files under /etc/systemd/system and enables them so that
# `systemctl start drone-mc.target` brings up every MC service container in
# the dependency order declared by each unit's After=/Requires=.
#
# Usage:
#   sudo ./02-install-systemd-units.sh
#
# Set START_NOW=1 to start drone-mc.target immediately after install
# (requires the service container images to already exist on this host —
# see scripts/04-install-ros2-core.sh and the per-package Dockerfiles).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
UNIT_SRC_DIR="${REPO_ROOT}/config/systemd"
UNIT_DST_DIR="/etc/systemd/system"

if [[ "${EUID}" -ne 0 ]]; then
    echo "ERROR: must run as root (sudo ./02-install-systemd-units.sh)" >&2
    exit 1
fi

if ! command -v systemctl &>/dev/null; then
    echo "ERROR: systemctl not found — this script must run on the Jetson host." >&2
    exit 1
fi

echo "==> Installing cgroup slices"
install -m 644 "${UNIT_SRC_DIR}/rt.slice" "${UNIT_DST_DIR}/rt.slice"
install -m 644 "${UNIT_SRC_DIR}/be.slice" "${UNIT_DST_DIR}/be.slice"

echo "==> Installing drone-mc.target"
install -m 644 "${UNIT_SRC_DIR}/drone-mc.target" "${UNIT_DST_DIR}/drone-mc.target"

echo "==> Installing service units"
SERVICE_UNITS=()
for unit_path in "${UNIT_SRC_DIR}"/drone-mc-*.service; do
    unit_name="$(basename "${unit_path}")"
    install -m 644 "${unit_path}" "${UNIT_DST_DIR}/${unit_name}"
    SERVICE_UNITS+=("${unit_name}")
    echo "    ${unit_name}"
done

echo "==> Provisioning /etc/drone-mc/environment"
mkdir -p /etc/drone-mc
if [[ -f /etc/drone-mc/environment ]]; then
    echo "    /etc/drone-mc/environment already exists — leaving per-drone values untouched."
else
    install -m 644 "${REPO_ROOT}/config/environment" /etc/drone-mc/environment
    echo "    Installed default environment. Edit DRONE_ID/MC_VERSION/FCC_TRANSPORT for this vehicle before starting drone-mc.target."
fi

# Every service bind-mounts the host's /etc/drone-mc over the image's own
# copy (INFRA-004), so config files baked into an image (cyclonedds.xml) or
# only ever shipped in the repo (the mavlink-router channel confs) must
# also land here — otherwise the bind mount just hides them with an empty
# directory.
echo "==> Provisioning /etc/drone-mc/cyclonedds.xml"
if [[ -f /etc/drone-mc/cyclonedds.xml ]]; then
    echo "    /etc/drone-mc/cyclonedds.xml already exists — leaving it untouched."
else
    install -m 644 "${REPO_ROOT}/docker/ros2-core/cyclonedds.xml" /etc/drone-mc/cyclonedds.xml
    echo "    Installed default config (same-host discovery only, lo)."
fi

echo "==> Provisioning /etc/drone-mc/zenoh-bridge-dds.json5"
if [[ -f /etc/drone-mc/zenoh-bridge-dds.json5 ]]; then
    echo "    /etc/drone-mc/zenoh-bridge-dds.json5 already exists — leaving per-drone peer list untouched."
else
    install -m 644 "${REPO_ROOT}/config/zenoh-bridge-dds.json5" /etc/drone-mc/zenoh-bridge-dds.json5
    echo "    Installed default config. Edit the connect endpoints for this vehicle's swarm before starting drone-mc.target."
fi

echo "==> Provisioning mavlink-router channel configs (GCS-001/MAVR-001)"
install -m 644 "${REPO_ROOT}/config/mavlink-router-channel-a.conf" /etc/drone-mc/mavlink-router-channel-a.conf
install -m 644 "${REPO_ROOT}/config/mavlink-router-channel-b.conf" /etc/drone-mc/mavlink-router-channel-b.conf

echo "==> Reloading systemd and enabling units"
systemctl daemon-reload
systemctl enable drone-mc.target
systemctl enable "${SERVICE_UNITS[@]}"

echo "==> Install complete"
systemctl list-dependencies drone-mc.target --no-pager || true

if [[ "${START_NOW:-0}" -eq 1 ]]; then
    echo "==> Starting drone-mc.target (START_NOW=1)"
    systemctl start drone-mc.target
    systemctl status drone-mc.target --no-pager || true
else
    echo "    Run 'sudo systemctl start drone-mc.target' to bring up all MC services."
fi
