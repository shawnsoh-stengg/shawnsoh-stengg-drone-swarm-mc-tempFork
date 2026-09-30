# docker/indoor-nav/Dockerfile — indoor-nav service image (IGS-*,
# INDOOR-GNSS-SIMULATION-BACKLOG.md). Runs gnss_sim_node: the synthetic
# indoor WGS-84 fix streamed to the FCC as HIL_GPS on :14557 (ICD §3.3b).
#
# Same two-stage colcon pattern as docker/mission-exec/Dockerfile: builder
# compiles indoor_nav (+ mc_msgs) FROM ros2-core with BUILD_TESTING=OFF;
# runtime copies the merged install into a fresh ros2-core. The MAVLink C
# headers indoor_nav needs come from ros2-core's own ros-humble-mavlink, so
# no apt/network step here beyond the base image.
#
# In production, drone-mc-indoor-nav.service bind-mounts the host-built
# /opt/drone-mc/install over this image's copy (shared-workspace pattern) —
# the baked-in install only runs without that mount (smoke tests).
#
# Build context must be the repo root. Use
# scripts/16-build-indoor-nav-image.sh rather than docker build directly.

ARG ROS2_CORE_IMAGE=ros2-core:latest

FROM ${ROS2_CORE_IMAGE} AS builder
WORKDIR /ws
COPY ws/src/indoor_nav src/indoor_nav
COPY ws/src/mc_msgs src/mc_msgs
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    colcon build \
      --merge-install \
      --install-base /opt/drone-mc/install \
      --packages-up-to indoor_nav \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
COPY ws/src/indoor_nav/params/default.yaml /etc/drone-mc/params/indoor_nav.yaml
COPY docker/indoor-nav/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
