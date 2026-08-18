# docker/health-monitor/Dockerfile — health-monitor service image (HEALTH-004,
# DEP-001, SDD-MC-001 §12.1/§10). Runs health_monitor_node alone — one node,
# one package, same two-stage pattern every other service image follows
# (docker/fcc-bridge/Dockerfile, docker/mission-exec/Dockerfile): a builder
# stage compiles health_monitor (+ its workspace-local mc_msgs dependency)
# FROM the ros2-core base with BUILD_TESTING=OFF, and a runtime stage
# copies the merged install tree into a fresh ros2-core with no build
# tooling. No extra native-daemon stage is needed here (unlike fcc-bridge's
# Micro XRCE-DDS Agent) — health_monitor_node talks to nothing but ROS 2
# topics and the SDD §10.4 systemd-restart Unix socket (already on the host,
# bind-mounted at runtime, not baked into this image).
#
# In production, drone-mc-health-monitor.service bind-mounts the host-built
# /opt/drone-mc/install over this image's own copy (same "shared workspace
# bind-mounted into all containers" pattern every other service image
# follows) — this image's baked-in install only runs when launched without
# that mount (local dev/CI smoke tests). That same unit also bind-mounts
# the host's real /proc and /sys read-only (HEALTH-004 AC1) so
# health_monitor_node's resource-sampling (SDD §10.1) reflects genuine
# host state rather than this container's own restricted view — see that
# unit file's own comment for why.
#
# Build context must be the repo root (needs ws/src/health_monitor,
# ws/src/mc_msgs). Use scripts/14-build-health-monitor-image.sh rather than
# invoking docker build directly.

ARG ROS2_CORE_IMAGE=ros2-core:latest

FROM ${ROS2_CORE_IMAGE} AS builder
WORKDIR /ws
COPY ws/src/health_monitor src/health_monitor
COPY ws/src/mc_msgs src/mc_msgs
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    colcon build \
      --merge-install \
      --install-base /opt/drone-mc/install \
      --packages-up-to health_monitor \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
COPY ws/src/health_monitor/params/default.yaml /etc/drone-mc/params/health_monitor.yaml
COPY docker/health-monitor/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
