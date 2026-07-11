# ros2-core — shared base image for all drone-mc service containers.
#
# Built from dustynv ROS 2 Humble on L4T R36.3.0 (JetPack 6.2.2, Ubuntu 22.04 aarch64).
# Before tagging a release, pin the digest:
#   docker pull dustynv/ros:humble-ros-base-l4t-r36.3.0
#   docker inspect --format '{{index .RepoDigests 0}}' dustynv/ros:humble-ros-base-l4t-r36.3.0
# Then replace the FROM line with:
#   FROM dustynv/ros:humble-ros-base-l4t-r36.3.0@sha256:<digest>
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

# The base image's packages.ros.org apt source signs with key F42ED6FBAB17C654,
# which expired 2025-06 — apt-get update fails for that one repo, silently
# starving every ros-humble-* install below. Replace it with the current
# keyring/source via the official ros2-apt-source package (pinned; matches
# the FROM line's pin-by-digest policy above) before touching that repo.
# https://github.com/ros-infrastructure/ros-apt-source
RUN rm -f /etc/apt/sources.list.d/ros2.list \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.jammy_all.deb" \
    && apt-get install -y --no-install-recommends /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Build tooling on top of the base ROS 2 Humble install.
# ros-humble-rmw-cyclonedds-cpp: CycloneDDS RMW used on all nodes.
# ros-humble-mavlink: generated MAVLink C headers — used by fcc_bridge's
#   mission-protocol client (ICD §3.3, IR-007) and gcs_mavlink_node.
# python3-colcon-*: workspace build tooling for service container build stages (SDD §12.1).
#
# `rosdep init` is scoped in its own `|| true` subshell — `&&`/`||` share
# precedence, so without the parens a failure anywhere upstream (e.g. the
# apt repo issue above) would be swallowed by the same `|| true` and this
# whole RUN would report success with nothing installed (see git history).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-cyclonedds \
        ros-humble-mavlink \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-rosdep \
    && (rosdep init || true) \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Shared paths used by every service container (SDD §12.1, §12.5).
RUN mkdir -p /opt/drone-mc/install /etc/drone-mc /var/drone-mc

# Default CycloneDDS config. Overridden at runtime by bind-mounting /etc/drone-mc
# from the host (SDD §12.5, ICD §9.4).
COPY cyclonedds.xml /etc/drone-mc/cyclonedds.xml

# /opt/ros/humble/install is this base image's own source-built layout;
# apt-installed ros-humble-* debs (e.g. ros-humble-mavlink above) land in
# the standard dpkg prefix /opt/ros/humble instead, which nothing sources
# by default — find_package() reads CMAKE_PREFIX_PATH directly, so it must
# be listed here too or packages installed via apt are invisible to CMake.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=/etc/drone-mc/cyclonedds.xml \
    ROS_DOMAIN_ID=0 \
    AMENT_PREFIX_PATH=/opt/drone-mc/install:/opt/ros/humble/install:/opt/ros/humble \
    CMAKE_PREFIX_PATH=/opt/drone-mc/install:/opt/ros/humble/install:/opt/ros/humble \
    COLCON_HOME=/root/.colcon

# Source ROS 2 and the shared workspace on every interactive shell session.
RUN echo "source /opt/ros/humble/install/setup.bash" >> /etc/bash.bashrc \
 && echo "[[ -f /opt/drone-mc/install/setup.bash ]] && source /opt/drone-mc/install/setup.bash" >> /etc/bash.bashrc
