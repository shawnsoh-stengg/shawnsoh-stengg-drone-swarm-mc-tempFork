#!/usr/bin/env bash
# sync-px4-msgs.sh — Put ws/src/px4_msgs on the branch ws/px4_msgs.repos pins.
# Called by 05-build-workspace.sh and 08-build-fcc-bridge-image.sh; also safe
# to run by hand (sudo ./scripts/sync-px4-msgs.sh).
#
# px4_msgs is not vendored (ws/.gitignore). A fresh host gets a clone of the
# pinned branch; an existing checkout on any other branch (or a detached
# HEAD) is switched over, then fast-forwarded to origin. The old
# `vcs import --skip-existing` never moved an existing checkout, so DEP-009's
# release/1.14 -> release/1.17 pin bump never reached the orin-nx bench
# Jetson, whose fcc_bridge_node then decoded PX4 v1.17 traffic with v1.14
# message layouts (DDS matches on type name only, so nothing errors -- fields
# just misdecode). Plain git instead of vcs: under sudo, root's PATH doesn't
# see a `pip3 install --user` vcs, which made the build scripts bail out
# before ever reaching the pin check.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"
REPOS_FILE="${REPO_ROOT}/ws/px4_msgs.repos"
PX4_MSGS_DIR="${REPO_ROOT}/ws/src/px4_msgs"
PX4_MSGS_URL="$(awk '$1 == "url:" {print $2}' "${REPOS_FILE}")"
PX4_MSGS_PIN="$(awk '$1 == "version:" {print $2}' "${REPOS_FILE}")"

if [[ -z "${PX4_MSGS_URL}" || -z "${PX4_MSGS_PIN}" ]]; then
    echo "==> ERROR: could not read url/version from ${REPOS_FILE}" >&2
    exit 1
fi

# The checkout is root-owned when a build script ran under sudo, hence
# safe.directory (plain `git -C ws/src/px4_msgs ...` as the login user
# fails with "dubious ownership").
px4_git() { git -c safe.directory="${PX4_MSGS_DIR}" -C "${PX4_MSGS_DIR}" "$@"; }

if [[ ! -e "${PX4_MSGS_DIR}" ]]; then
    echo "==> Cloning px4_msgs (${PX4_MSGS_PIN}) into ${PX4_MSGS_DIR}"
    git clone --branch "${PX4_MSGS_PIN}" "${PX4_MSGS_URL}" "${PX4_MSGS_DIR}"
elif ! px4_git rev-parse --git-dir >/dev/null 2>&1; then
    echo "==> ERROR: ${PX4_MSGS_DIR} exists but is not a git checkout — remove it and re-run" >&2
    exit 1
else
    if [[ -n "$(px4_git status --porcelain)" ]]; then
        if [[ "$(px4_git rev-parse --abbrev-ref HEAD)" != "${PX4_MSGS_PIN}" ]]; then
            echo "==> ERROR: ${PX4_MSGS_DIR} is not on '${PX4_MSGS_PIN}' (ws/px4_msgs.repos) and has local changes — resolve by hand" >&2
            exit 1
        fi
        echo "==> WARNING: ${PX4_MSGS_DIR} has local changes — building it as-is, not updating" >&2
    else
        echo "==> Updating px4_msgs to origin/${PX4_MSGS_PIN}"
        px4_git fetch origin "+refs/heads/${PX4_MSGS_PIN}:refs/remotes/origin/${PX4_MSGS_PIN}"
        px4_git checkout -B "${PX4_MSGS_PIN}" --track "origin/${PX4_MSGS_PIN}"
    fi
fi

PX4_MSGS_HEAD="$(px4_git rev-parse --abbrev-ref HEAD)"
if [[ "${PX4_MSGS_HEAD}" != "${PX4_MSGS_PIN}" ]]; then
    echo "==> ERROR: px4_msgs is on '${PX4_MSGS_HEAD}' after sync, expected '${PX4_MSGS_PIN}'" >&2
    exit 1
fi
echo "==> px4_msgs: ${PX4_MSGS_HEAD} @ $(px4_git log -1 --format='%h %s')"

# Schema floor, checked against the msg files themselves rather than the pin
# above: the org-repo port of DEP-009 (NEAR-684) carried the v1.17 fcc_bridge
# code but dropped the ws/px4_msgs.repos line, so this script faithfully
# synced every rebuild back to release/1.14 and nothing failed. fcc_bridge's
# XrceDdsBackend and mission_exec's pre-arm checks assume PX4 v1.17's
# VehicleStatus (2-value ARMING_STATE_DISARMED/ARMED enum, v1.17 nav_state
# table). NAVIGATION_STATE_ALTITUDE_CRUISE first appears in release/1.17 --
# absent from 1.14, 1.15 and 1.16 -- so it marks the minimum schema.
VEHICLE_STATUS_MSG="${PX4_MSGS_DIR}/msg/VehicleStatus.msg"
for marker in ARMING_STATE_DISARMED NAVIGATION_STATE_ALTITUDE_CRUISE; do
    if ! grep -qw "${marker}" "${VEHICLE_STATUS_MSG}"; then
        echo "==> ERROR: ${VEHICLE_STATUS_MSG} has no ${marker} — px4_msgs is older than" >&2
        echo "    release/1.17, which fcc_bridge requires (DEP-009). Check the 'version:' line" >&2
        echo "    in ${REPOS_FILE} (currently '${PX4_MSGS_PIN}')." >&2
        exit 1
    fi
done
echo "==> px4_msgs schema check: VehicleStatus has the v1.17 markers"
