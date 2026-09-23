#!/usr/bin/env bash
# 05b-smoke-test-colcon.sh — Verify colcon workspace at /opt/drone-mc/install.
# TP-SM-005: colcon in PATH · workspace at /opt/drone-mc/install · ros2 pkg list
# Ref: SDD §13.2 smoke test pattern
set -euo pipefail

PASS=0; FAIL=0
INSTALL_DIR="${DRONE_MC_INSTALL_DIR:-/opt/drone-mc/install}"
IMAGE="${MC_VERSION:+ros2-core:${MC_VERSION}}"
IMAGE="${IMAGE:-ros2-core:latest}"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS+1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL+1))
    fi
}

# --network none: these checks never need connectivity, only the bridge
# driver's veth setup, which some hosts' kernels lack (e.g. a Jetson
# whose /lib/modules/<kernel>/ has no veth.ko) — real drone-mc containers
# never hit this since they all run with --network=host.
RUN=(docker run --rm --network none --volume "${INSTALL_DIR}:/opt/drone-mc/install" "${IMAGE}")

check "install dir exists on host"    test -d "${INSTALL_DIR}"

# setup.bash is absent when the workspace is empty (no packages built yet).
# Once packages exist this check becomes load-bearing.
if [[ -f "${INSTALL_DIR}/setup.bash" ]]; then
    check "workspace sources cleanly" "${RUN[@]}" bash -c \
        "source /opt/ros/humble/setup.bash && source /opt/drone-mc/install/setup.bash"
    # Uncomment and extend as packages are added:
    # check "fcc_bridge registered"     "${RUN[@]}" bash -c \
    #     "source /opt/ros/humble/setup.bash && source /opt/drone-mc/install/setup.bash \
    #      && ros2 pkg list | grep -q fcc_bridge"
else
    printf "INFO: /opt/drone-mc/install/setup.bash not present — workspace is empty (no packages yet)\n"
fi

check "colcon exits 0 on empty src"   "${RUN[@]}" bash -c \
    "source /opt/ros/humble/setup.bash && \
     colcon build --merge-install --install-base /opt/drone-mc/install \
         --base-paths /tmp/empty_ws 2>&1 | grep -qE 'Summary: 0 packages finished'"

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
