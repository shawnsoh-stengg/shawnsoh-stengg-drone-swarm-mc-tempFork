#!/usr/bin/env bash
# 08b-smoke-test-fcc-bridge.sh — Verify fcc_bridge_node in the built colcon
# workspace: package/interface/executable registration, and a live
# publish + clean-shutdown check against a loopback FCC target (FCC-004).
# Ref: SDD §13.2 smoke test pattern
set -euo pipefail

PASS=0; FAIL=0
INSTALL_DIR="${DRONE_MC_INSTALL_DIR:-/opt/drone-mc/install}"
IMAGE="${MC_VERSION:+ros2-core:${MC_VERSION}}"
IMAGE="${IMAGE:-ros2-core:latest}"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; ((PASS++))
    else
        printf "FAIL: %s\n" "$desc"; ((FAIL++))
    fi
}

RUN=(docker run --rm --volume "${INSTALL_DIR}:/opt/drone-mc/install:ro" "${IMAGE}")
SOURCE="source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash"

check "fcc_bridge package registered" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 pkg list | grep -qx fcc_bridge"
check "mc_msgs package registered" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 pkg list | grep -qx mc_msgs"
check "fcc_bridge_node executable present" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 pkg executables fcc_bridge | grep -q fcc_bridge_node"
check "mc_msgs/VehicleState interface readable" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 interface show mc_msgs/msg/VehicleState"
check "mc_msgs/FaultEvent interface readable" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 interface show mc_msgs/msg/FaultEvent"

# No live Pixhawk needed: XrceDdsBackend only binds a local UDP socket for
# the mission-protocol channel (ICD §9.4). Verifies /mc/fcc/vehicle_state
# publishes at ~10 Hz (FCC-004 AC1) and the node exits 0 on SIGINT — the
# same rclcpp::shutdown() path the systemd unit's `docker stop` triggers
# (FCC-004 AC3, drone-mc-fcc-bridge.service Restart=on-failure).
#
# `ros2 run` execs the node as a *child* of a Python wrapper (subprocess.
# Popen, not exec) — signalling the wrapper's own PID does not reach the
# child outside an interactive TTY's process-group SIGINT, so the node is
# signalled by name via pkill instead; the wrapper then exits with the
# child's propagated status once it's gone. The rate threshold (9.5, not
# 10.0) leaves headroom for the same discrete-tick-vs-sampling-window
# jitter TP-NI-001's gtest works around exactly — this check just needs to
# catch gross breakage (node not running, wrong topic), not enforce the
# precise SLA.
check "fcc_bridge_node publishes vehicle_state ~10Hz and exits cleanly" \
    "${RUN[@]}" bash -c "
        ${SOURCE}
        export FCC_ETH_IP=127.0.0.1 FCC_MISSION_PORT=15557
        ros2 run fcc_bridge fcc_bridge_node &
        node_pid=\$!
        sleep 1
        rate=\$(timeout 5 ros2 topic hz /mc/fcc/vehicle_state 2>/dev/null \
            | grep -oP 'average rate: \K[0-9.]+' | tail -1)
        pkill -INT -f 'fcc_bridge_node\$'
        wait \"\${node_pid}\"
        exit_code=\$?
        [[ -n \"\${rate}\" ]] || exit 1
        awk -v r=\"\${rate}\" 'BEGIN{exit !(r >= 9.5)}' || exit 1
        [[ \${exit_code} -eq 0 ]]
    "

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
