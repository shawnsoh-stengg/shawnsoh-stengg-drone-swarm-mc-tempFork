# docker/swarm-coord/Dockerfile — swarm-coord service image (DEP-001,
# SDD-MC-001 §12.1). Runs swarm_coordinator_node and formation_controller_node
# together — two nodes from one package, same "one container per package"
# precedent as docker/mission-exec/Dockerfile's mission_manager_node +
# search_planner_node pairing.
#
# Two-stage colcon pattern, no new pattern invented (matches
# docker/mission-exec/Dockerfile / docker/health-monitor/Dockerfile): builder
# stage compiles swarm_coord FROM the ros2-core base with BUILD_TESTING=OFF
# (ament_lint_auto / ament_cmake_gtest are test_depends only, not present in
# the ros2-core runtime); runtime stage copies the merged install tree into a
# fresh ros2-core with no build tooling. Unlike mission-exec/health-monitor,
# swarm_coord's package.xml depends on nothing but rclcpp — no mc_msgs COPY
# needed here (both nodes are still §2.4/§5/§6 scaffolds, not yet wired to
# the swarm message types).
#
# In production, drone-mc-swarm-coord.service bind-mounts the host-built
# /opt/drone-mc/install over this image's own copy (same "shared workspace
# bind-mounted into all containers" pattern every other service image
# follows) — this image's baked-in install only runs when launched without
# that mount (local dev/CI smoke tests).
#
# Build context must be the repo root (needs ws/src/swarm_coord). Use
# scripts/13-build-swarm-coord-image.sh rather than invoking docker build
# directly.

ARG ROS2_CORE_IMAGE=ros2-core:latest

FROM ${ROS2_CORE_IMAGE} AS builder
WORKDIR /ws
COPY ws/src/swarm_coord src/swarm_coord
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    colcon build \
      --merge-install \
      --install-base /opt/drone-mc/install \
      --packages-up-to swarm_coord \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
COPY ws/src/swarm_coord/params/default.yaml /etc/drone-mc/params/swarm_coord.yaml
COPY docker/swarm-coord/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
