# docker/fcc-bridge/Dockerfile — fcc_bridge service image (SDD-MC-001 §12.1).
#
# Stage 1 colcon-builds fcc_bridge and its workspace-local dependencies
# (mc_msgs, px4_msgs) against the ros2-core base image. Stage 2 copies the
# merged install tree into a minimal ros2-core runtime with no build
# tooling or test dependencies (BUILD_TESTING=OFF — ament_lint_auto /
# ament_cmake_gtest are test_depends only, not present in ros2-core).
#
# In production, drone-mc-fcc-bridge.service bind-mounts the host-built
# /opt/drone-mc/install over this image's own copy (SDD §12.1: "the shared
# colcon workspace is built once and bind-mounted into all containers") —
# so this image's baked-in install is only what actually runs when the
# container is launched without that mount (local dev/CI smoke tests).
#
# Build context must be the repo root (needs ws/src/*). Use
# scripts/08-build-fcc-bridge-image.sh rather than invoking docker build
# directly — it fetches px4_msgs into ws/src first.

ARG ROS2_CORE_IMAGE=ros2-core:latest

FROM ${ROS2_CORE_IMAGE} AS builder
WORKDIR /ws
COPY ws/src/fcc_bridge src/fcc_bridge
COPY ws/src/mc_msgs src/mc_msgs
COPY ws/src/px4_msgs src/px4_msgs
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    colcon build \
      --merge-install \
      --install-base /opt/drone-mc/install \
      --packages-up-to fcc_bridge \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
COPY ws/src/fcc_bridge/params/default.yaml /etc/drone-mc/params/fcc_bridge.yaml

CMD ["bash", "-c", "\
    source /opt/ros/humble/install/setup.bash && \
    source /opt/drone-mc/install/setup.bash && \
    exec ros2 run fcc_bridge fcc_bridge_node --ros-args --params-file /etc/drone-mc/params/fcc_bridge.yaml"]
