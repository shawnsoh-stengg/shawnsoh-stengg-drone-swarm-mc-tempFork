# docker/mission-exec/Dockerfile — mission-exec service image (EXEC-005,
# DEP-001, SDD-MC-001 §12.1). Runs mission_manager_node and
# search_planner_node together — two nodes from one package, same "one
# container per package" precedent as docker/gcs-comms/Dockerfile's
# gcs_mavlink_node + MAVROS pairing (there it's two OS processes from two
# packages; here it's two OS processes from the same package).
#
# Two-stage colcon pattern, no new pattern invented (matches
# docker/fcc-bridge/Dockerfile / docker/gcs-comms/Dockerfile): builder
# stage compiles mission_exec (+ its workspace-local mc_msgs dependency)
# FROM the ros2-core base with BUILD_TESTING=OFF (ament_lint_auto /
# ament_cmake_gtest are test_depends only, not present in the ros2-core
# runtime); runtime stage copies the merged install tree into a fresh
# ros2-core with no build tooling.
#
# In production, drone-mc-mission-exec.service bind-mounts the host-built
# /opt/drone-mc/install over this image's own copy (same "shared workspace
# bind-mounted into all containers" pattern every other service image
# follows) — this image's baked-in install only runs when launched without
# that mount (local dev/CI smoke tests).
#
# Build context must be the repo root (needs ws/src/mission_exec,
# ws/src/mc_msgs). Use scripts/12-build-mission-exec-image.sh rather than
# invoking docker build directly.

ARG ROS2_CORE_IMAGE=ros2-core:latest

FROM ${ROS2_CORE_IMAGE} AS builder
WORKDIR /ws
COPY ws/src/mission_exec src/mission_exec
COPY ws/src/mc_msgs src/mc_msgs
RUN bash -c "\
    source /opt/ros/humble/install/setup.bash && \
    colcon build \
      --merge-install \
      --install-base /opt/drone-mc/install \
      --packages-up-to mission_exec \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
COPY ws/src/mission_exec/params/default.yaml /etc/drone-mc/params/mission_exec.yaml
COPY docker/mission-exec/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
