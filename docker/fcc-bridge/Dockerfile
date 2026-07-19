# docker/fcc-bridge/Dockerfile — fcc_bridge service image (SDD-MC-001 §12.1).
#
# Stage 1 colcon-builds fcc_bridge and its workspace-local dependencies
# (mc_msgs, px4_msgs) against the ros2-core base image. Stage 2 builds the
# Micro XRCE-DDS Agent (FCC-008) — standalone C++ daemon with no ROS 2
# dependency, so it's built FROM plain Ubuntu 22.04 rather than dragging in
# the ROS 2 toolchain it never uses (same reasoning as
# docker/mavlink-router/Dockerfile). Stage 3 copies both build outputs into
# a minimal ros2-core runtime with no build tooling or test dependencies
# (BUILD_TESTING=OFF — ament_lint_auto / ament_cmake_gtest are test_depends
# only, not present in ros2-core).
#
# In production, drone-mc-fcc-bridge.service bind-mounts the host-built
# /opt/drone-mc/install over this image's own copy (SDD §12.1: "the shared
# colcon workspace is built once and bind-mounted into all containers") —
# so this image's baked-in install is only what actually runs when the
# container is launched without that mount (local dev/CI smoke tests). The
# Agent binary is not part of that bind-mount — it's baked into this image
# only.
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

# Micro XRCE-DDS Agent (FCC-008) — bridges XrceDdsBackend's /fmu/in,/fmu/out
# DDS topics to the FCC's embedded uXRCE-DDS client over serial or Ethernet
# (HID-MC-001 §4.2/§4.3). No arm64 apt package exists, so it's built from
# source and pinned to the release validated against PX4 v1.14 — bump
# alongside PX4 firmware upgrades.
# https://github.com/eProsima/Micro-XRCE-DDS-Agent/releases
FROM ubuntu:22.04 AS xrce_agent_builder
ARG XRCE_AGENT_VERSION=v2.4.3
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Note: this project builds via a CMake "superbuild" (UAGENT_SUPERBUILD=ON
# default — fetches/builds Fast-DDS, Fast-CDR, spdlog as ExternalProjects),
# so there is no single combined `--target install` step; build (default
# target) and install must run as two separate `cmake --build` invocations,
# same as upstream's documented `make && make install` recipe.
RUN git clone --depth 1 --branch "${XRCE_AGENT_VERSION}" \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /build \
    && cmake -S /build -B /build/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /build/build -- -j"$(nproc)" \
    && cmake --build /build/build --target install \
    && ldconfig

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
# Exact shared-lib manifest varies by Agent version/middleware config
# (libmicroxrcedds_*, libfastcdr, ...) — copy the whole install prefix
# rather than hand-picking names that can silently drift out of date.
COPY --from=xrce_agent_builder /usr/local /usr/local
RUN ldconfig
COPY ws/src/fcc_bridge/params/default.yaml /etc/drone-mc/params/fcc_bridge.yaml
COPY docker/fcc-bridge/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
