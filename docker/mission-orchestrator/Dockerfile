# docker/mission-orchestrator/Dockerfile — mission-orchestrator service
# image (GCS-006, closing the SDD-MC-001.md §12.4 deployment gap GCS-005
# left open). Runs mission_orchestrator_node alone — same package
# (gcs_comms) as docker/gcs-comms/Dockerfile, but a different executable
# and its own container, matching this repo's DEP-005/DEP-007 precedent
# (perception_front_node/video_mux_node: two nodes, one shared source
# package, two containers so one node's restart can't take the other
# down). mission_orchestrator_node was deliberately kept out of
# drone-mc-gcs-comms.service (GCS-005, that unit's own header comment) —
# this image is the container that decision implied but never built.
#
# No MAVROS, no extra apt-get install stage: mission_orchestrator_node
# hand-decodes raw MAVLink v2 frames (mission_orchestrator_node.hpp's own
# header comment) and links nlohmann_json for plan JSON (ws/src/gcs_comms/
# CMakeLists.txt) — both already covered by docker/ros2-core/Dockerfile's
# base install (nlohmann-json3-dev). Two-stage colcon pattern, matching
# docker/health-monitor/Dockerfile (the closest existing analog: one node,
# one package, no native-daemon build stage).
#
# In production, drone-mc-mission-orchestrator.service bind-mounts the
# host-built /opt/drone-mc/install over this image's own copy (same
# "shared workspace bind-mounted into all containers" pattern every other
# service image follows) — this image's baked-in install only runs when
# launched without that mount (local dev/CI smoke tests). That same unit
# also bind-mounts /var/drone-mc/mission_plans read-write so the plan
# library persists across container restarts (GCS-006 AC4).
#
# Build context must be the repo root (needs ws/src/gcs_comms, ws/src/mc_msgs).
# Use scripts/15-build-mission-orchestrator-image.sh rather than invoking
# docker build directly.

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

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install
COPY ws/src/gcs_comms/params/default.yaml /etc/drone-mc/params/gcs_comms.yaml
COPY docker/mission-orchestrator/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
