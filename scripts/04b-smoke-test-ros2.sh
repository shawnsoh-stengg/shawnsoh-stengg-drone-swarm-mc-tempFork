#!/usr/bin/env bash
# 04b-smoke-test-ros2.sh — Verify ros2-core image: ROS 2 Humble + CycloneDDS.
# TP-SM-004: ROS 2 Humble sourced · ROS_DISTRO=humble · CycloneDDS shared library present
# Ref: SDD §13.2 smoke test pattern
set -euo pipefail

PASS=0; FAIL=0
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
RUN=(docker run --rm --network none "${IMAGE}")

check "ros2-core image exists"        docker image inspect "${IMAGE}"
check "ROS 2 Humble sourced"          "${RUN[@]}" bash -c \
    "source /opt/ros/humble/setup.bash && test \"\$ROS_DISTRO\" = humble && ros2 pkg list >/dev/null"
check "CycloneDDS shared library"     "${RUN[@]}" test -f /opt/ros/humble/install/lib/librmw_cyclonedds_cpp.so
check "rmw_cyclonedds_cpp installed"  "${RUN[@]}" bash -c \
    "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -q rmw_cyclonedds_cpp"
check "colcon in PATH"                "${RUN[@]}" which colcon
check "rosdep in PATH"                "${RUN[@]}" which rosdep
check "rosdep db initialised"         "${RUN[@]}" test -d /etc/ros/rosdep
check "/opt/drone-mc/install exists"  "${RUN[@]}" test -d /opt/drone-mc/install
check "/etc/drone-mc/ exists"         "${RUN[@]}" test -d /etc/drone-mc
check "/var/drone-mc/ exists"         "${RUN[@]}" test -d /var/drone-mc
check "cyclonedds.xml in image"       "${RUN[@]}" test -f /etc/drone-mc/cyclonedds.xml
check "RMW_IMPLEMENTATION env set"    "${RUN[@]}" bash -c 'test "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp"'
check "CYCLONEDDS_URI env set"        "${RUN[@]}" bash -c 'test -n "$CYCLONEDDS_URI"'
check "ROS_DOMAIN_ID env set"         "${RUN[@]}" bash -c 'test "$ROS_DOMAIN_ID" = "0"'

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
