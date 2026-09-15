# docker/gcs-comms/Dockerfile — gcs-comms service image (GCS-005,
# SDD-MC-001 §12.1). Runs gcs_mavlink_node and MAVROS together — two
# independent OS processes, not one linked binary. gcs_mavlink_node itself
# gains no MAVROS dependency from this (ws/src/gcs_comms/CMakeLists.txt is
# unchanged); MAVROS runs as a second process for a different purpose
# entirely — a live ROS 2 mirror of Channel A's GCS<->FCC vehicle
# telemetry for the still-unbuilt Mission Orchestrator App integration
# (SDD-MC-001.md §11.1's GCS-005 update).
#
# DEP-009 follow-up (2026-09-15): ros-humble-mavros (the plain apt package
# this Dockerfile used to install) no longer has a prebuilt binary on
# packages.ros.org for Humble, on any architecture — confirmed absent from
# the real upstream Packages index (amd64 and arm64 both), not an
# arm64-only or transient-network gap. Only ros-humble-mavros-msgs (pure
# .msg/.srv, no C++ node) survived. So, like fcc-bridge's Micro XRCE-DDS
# Agent, mavros/libmavconn now get their own from-source build stage —
# pinned to the real upstream mavlink/mavros.git tag 2.15.1
# (docker/gcs-comms/mavros.repos). mavros_extras/test_mavros/
# mavros_examples are never selected (only `mavros` is passed to
# --packages-up-to below) — this project only runs plain `mavros_node`
# (docker/gcs-comms/entrypoint.sh), no mavros_extras plugins.
#
# Built into /opt/ros/humble/install (ros2-core's own source-built prefix,
# baked into every service image — NOT /opt/drone-mc/install, which
# production bind-mounts over with a host-built tree that only ever
# contains this repo's own ws/src/ packages and would otherwise shadow a
# baked-in mavros/libmavconn entirely).
#
# In production, drone-mc-gcs-comms.service bind-mounts the host-built
# /opt/drone-mc/install over this image's own copy (same "shared workspace
# bind-mounted into all containers" pattern as every other service image)
# — this image's baked-in install only runs when launched without that
# mount (local dev/CI smoke tests).
#
# Build context must be the repo root (needs ws/src/gcs_comms, ws/src/mc_msgs).
# Use scripts/11-build-gcs-comms-image.sh rather than invoking docker build
# directly.

ARG ROS2_CORE_IMAGE=ros2-core:latest

FROM ${ROS2_CORE_IMAGE} AS builder
WORKDIR /ws
COPY ws/src/gcs_comms src/gcs_comms
COPY ws/src/mc_msgs src/mc_msgs
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    colcon build \
      --merge-install \
      --install-base /opt/drone-mc/install \
      --packages-up-to gcs_comms \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

# mavros/libmavconn from source (see the file-level comment above for why).
# mavros_msgs is deliberately NOT checked out — mavros's own <depend>
# mavros_msgs</depend> is satisfied by the still-real ros-humble-mavros-msgs
# apt package instead, resolved by rosdep below like every other system/ROS
# build dependency mavros's package.xml declares (eigen, geographiclib,
# libconsole-bridge-dev, diagnostic_updater, angles, eigen_stl_containers,
# mavlink, ...) — hand-enumerating those here would silently drift from
# upstream's own package.xml; rosdep is the same mechanism this repo
# already trusts colcon/ament with everywhere else.
COPY docker/gcs-comms/mavros.repos /tmp/mavros.repos
RUN mkdir -p src/mavros_ros2 \
    && vcs import src/mavros_ros2 < /tmp/mavros.repos \
    && rm -rf src/mavros_ros2/mavros_msgs
RUN apt-get update
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    rosdep update --rosdistro humble && \
    rosdep install --from-paths src/mavros_ros2 --ignore-src -y --rosdistro humble" \
    && rm -rf /var/lib/apt/lists/*
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    colcon build \
      --merge-install \
      --install-base /opt/ros/humble/install \
      --packages-up-to mavros \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
COPY --from=builder /opt/ros/humble/install /opt/ros/humble/install

# install_geographiclib_datasets.sh is NOT optional, despite this file's
# former comment claiming it only matters for plugins nothing here uses
# yet — MAVROS's UAS node calls into GeographicLib unconditionally during
# its own init and throws an uncaught std::invalid_argument
# ("File not readable .../egm96-5.pgm") if the geoid dataset is missing,
# aborting mavros_node before it ever reaches a ready state. Confirmed on
# a real ros-humble-mavros install, not assumed. Needs real internet
# access for its download, same INFRA-006 reasoning as the builder stage's
# vcs import/rosdep above — scripts/11-build-gcs-comms-image.sh's
# `docker build --network=host` already covers this RUN step too. Script
# path unchanged by the from-source build — mavros/libmavconn's own CMake
# installs it to the same relative `lib/mavros/` location regardless of
# which prefix they're installed under.
RUN /opt/ros/humble/install/lib/mavros/install_geographiclib_datasets.sh

COPY ws/src/gcs_comms/params/default.yaml /etc/drone-mc/params/gcs_comms.yaml
COPY ws/src/gcs_comms/params/mavros.yaml /etc/drone-mc/params/mavros.yaml
COPY docker/gcs-comms/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
