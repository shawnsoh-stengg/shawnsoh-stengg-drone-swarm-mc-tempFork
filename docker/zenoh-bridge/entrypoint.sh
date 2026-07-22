#!/usr/bin/env bash
# entrypoint.sh — launches zenoh-bridge-dds as the container's sole process
# (exec replaces the shell so SIGTERM from `docker stop` / systemd reaches
# the bridge directly — no separate process to supervise, matching
# drone-mc-zenoh-bridge.service's single-process Restart=on-failure model).
set -euo pipefail

CONF_DIR="${ZENOH_BRIDGE_CONF_DIR:-/etc/drone-mc}"

exec zenoh-bridge-dds -c "${CONF_DIR}/zenoh-bridge-dds.json5"
