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

if ! command -v envsubst &>/dev/null; then
    echo "==> envsubst not found, installing gettext-base"
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -o Acquire::Retries=2 -o Acquire::http::Timeout=10 -o Acquire::https::Timeout=10
    apt-get install -y --no-install-recommends gettext-base
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

# Per-variable migration: the file-level check above only seeds
# /etc/drone-mc/environment once, so a repo change to config/environment
# (e.g. INFRA-008 adding MC_ETH_MAC/MC_ETH_IP/MC_ETH_INTERNAL_IP/FCC_GCS_IP)
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

# INFRA-008: mavlink-router-channel-{a,b,c}.conf and zenoh-bridge-dds.json5
# below are envsubst templates rendered against this drone's own
# /etc/drone-mc/environment (just provisioned/confirmed above) — load it
# into the shell environment so envsubst can see it.
set -a
# shellcheck disable=SC1091
source /etc/drone-mc/environment
set +a
for var in FCC_GCS_IP GCS_IP; do
    if [[ -z "${!var:-}" ]]; then
        echo "ERROR: ${var} is unset/empty in /etc/drone-mc/environment — envsubst would" \
             "silently render a blank value into the mavlink-router configs. Set it and" \
             "re-run." >&2
        exit 1
    fi
done

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

# INFRA-008: zenoh-bridge-dds.json5's connect-endpoint peer list is now
# generated from config/fleet.yaml (drone_id -> IP) minus this drone's own
# DRONE_ID, rather than hand-edited per drone — always regenerated (like
# the mavlink-router confs below), not left-alone-if-present like
# environment above, since there is no longer a manual edit to preserve.
echo "==> Rendering /etc/drone-mc/zenoh-bridge-dds.json5 (peer list from config/fleet.yaml)"
FLEET_FILE="${REPO_ROOT}/config/fleet.yaml"
CONNECT_ENDPOINTS=""
if [[ -f "${FLEET_FILE}" ]]; then
    endpoints=()
    while read -r peer_id peer_ip; do
        [[ "${peer_id}" == "${DRONE_ID:-1}" ]] && continue
        endpoints+=("\"tcp/${peer_ip}:7447\"")
    done < <(grep -E '^[[:space:]]+[0-9]+:[[:space:]]*[0-9]+(\.[0-9]+){3}' "${FLEET_FILE}" \
             | sed -E 's/^[[:space:]]*([0-9]+):[[:space:]]*([0-9.]+).*/\1 \2/')
    IFS=,
    CONNECT_ENDPOINTS="${endpoints[*]}"
    unset IFS
    echo "    ${#endpoints[@]} peer(s) from config/fleet.yaml (excluding DRONE_ID=${DRONE_ID:-1})"
else
    echo "    ${FLEET_FILE} not found — rendering with an empty peer list (SITL/single-drone default)."
fi
export CONNECT_ENDPOINTS
envsubst '$CONNECT_ENDPOINTS' \
    < "${REPO_ROOT}/config/zenoh-bridge-dds.json5" > /etc/drone-mc/zenoh-bridge-dds.json5

echo "==> Rendering mavlink-router channel configs (GCS-001/MAVR-001/GCS-006)"
envsubst '$FCC_GCS_IP' \
    < "${REPO_ROOT}/config/mavlink-router-channel-a.conf" > /etc/drone-mc/mavlink-router-channel-a.conf
envsubst '$GCS_IP' \
    < "${REPO_ROOT}/config/mavlink-router-channel-b.conf" > /etc/drone-mc/mavlink-router-channel-b.conf
envsubst '$GCS_IP' \
    < "${REPO_ROOT}/config/mavlink-router-channel-c.conf" > /etc/drone-mc/mavlink-router-channel-c.conf

# drone-mc-mission-orchestrator.service bind-mounts this read-write for
# plan-library persistence across container restarts (GCS-006 AC4) —
# mission_orchestrator_node itself only mkdir()s it from inside the
# container, which does nothing for the host-side bind-mount source Docker
# expects to already exist.
echo "==> Provisioning /var/drone-mc/mission_plans (GCS-006)"
mkdir -p /var/drone-mc/mission_plans

# Every entrypoint.sh passes --params-file /etc/drone-mc/params/<name>.yaml,
# but each package only ships its own params/default.yaml under ws/src/ —
# nothing ever copied those into /etc/drone-mc/params under the filename
# each entrypoint actually looks for, so `--params-file` pointed at a file
# that never existed on the host (rcl aborts the whole node on a missing
# params file, it doesn't fall back to declare_parameter()'s defaults).
echo "==> Provisioning /etc/drone-mc/params/*.yaml"
mkdir -p /etc/drone-mc/params
PARAMS_SOURCES=(
    "fcc_bridge/params/default.yaml:fcc_bridge.yaml"
    "gcs_comms/params/default.yaml:gcs_comms.yaml"
    "gcs_comms/params/mavros.yaml:mavros.yaml"
    "mission_exec/params/default.yaml:mission_exec.yaml"
    "health_monitor/params/default.yaml:health_monitor.yaml"
    "swarm_coord/params/default.yaml:swarm_coord.yaml"
)
for entry in "${PARAMS_SOURCES[@]}"; do
    src="${entry%%:*}"
    dst_name="${entry##*:}"
    dst="/etc/drone-mc/params/${dst_name}"
    if [[ -f "${dst}" ]]; then
        echo "    ${dst} already exists — leaving per-drone tuning untouched."
    else
        install -m 644 "${REPO_ROOT}/ws/src/${src}" "${dst}"
        echo "    Installed ${dst} (from ws/src/${src})."
    fi
done

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
