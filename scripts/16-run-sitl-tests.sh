#!/usr/bin/env bash
# 16-run-sitl-tests.sh — TEST-002: run the SITL system-test suite
# (ws/src/mission_exec/test/test_sitl_mission_scenarios.py) against a real
# PX4 SITL instance. TP-MC-001 §6 (TP-ST), single-drone scope — see that
# test file's own docstring for exactly which scenarios run vs. are
# skipped and why.
#
# Requires PX4_DIR to point at a PX4-Autopilot checkout already built with
# `make px4_sitl_default` (TP-MC-001 §3.2) — this script does not fetch or
# build PX4 itself. No paired "16b" smoke test: this script both drives
# the real run and reports colcon's own pass/fail summary, there's nothing
# separate to verify.
#
# Usage:
#   PX4_DIR=/path/to/PX4-Autopilot ./scripts/16-run-sitl-tests.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"

if [[ -z "${PX4_DIR:-}" ]]; then
    echo "ERROR: PX4_DIR is not set. Point it at a PX4-Autopilot checkout" >&2
    echo "       already built with 'make px4_sitl_default' (TP-MC-001 §3.2)." >&2
    exit 1
fi
if [[ ! -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
    echo "ERROR: no built SITL binary at ${PX4_DIR}/build/px4_sitl_default/bin/px4" >&2
    echo "       — run 'make px4_sitl_default' in that checkout first." >&2
    exit 1
fi
export PX4_DIR

command -v colcon >/dev/null || { echo "ERROR: colcon required" >&2; exit 1; }

echo "==> Running TEST-002 SITL scenarios against PX4_DIR=${PX4_DIR}"
cd "${REPO_ROOT}/ws"
colcon test --merge-install --packages-select mission_exec \
    --event-handlers console_direct+ \
    --ctest-args --tests-regex test_sitl_mission_scenarios

echo
echo "==> Result summary"
colcon test-result --verbose --test-result-base build/mission_exec
