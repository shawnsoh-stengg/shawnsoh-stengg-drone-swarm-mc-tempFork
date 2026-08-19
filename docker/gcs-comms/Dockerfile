# docker/gcs-comms/Dockerfile — gcs-comms service image (GCS-005,
# SDD-MC-001 §12.1). Runs gcs_mavlink_node and MAVROS together — two
# independent OS processes, not one linked binary. gcs_mavlink_node itself
# gains no MAVROS dependency from this (ws/src/gcs_comms/CMakeLists.txt is
# unchanged); MAVROS runs as a second process for a different purpose
# entirely — a live ROS 2 mirror of Channel A's GCS<->FCC vehicle
# telemetry for the still-unbuilt Mission Orchestrator App integration
# (SDD-MC-001.md §11.1's GCS-005 update).
#
# Unlike fcc-bridge's Micro XRCE-DDS Agent (no arm64 apt package, built
# from source in its own stage), ros-humble-mavros ships a prebuilt arm64
# deb — no extra build stage needed, just an apt-get install in the
# runtime stage alongside the colcon-built install tree.
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

FROM ${ROS2_CORE_IMAGE} AS runtime
COPY --from=builder /opt/drone-mc/install /opt/drone-mc/install

# ros-humble-mavros: mavros_node executable (GCS-005). Not installed in
# ros2-core (the shared base for every service container) — no other
# service needs it, and keeping it out of the base image matches this
# project's existing minimal-toolchain-per-service pattern (mavlink-router
# and zenoh-bridge are similarly standalone, not built FROM ros2-core at
# all).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-mavros \
    && rm -rf /var/lib/apt/lists/*

# install_geographiclib_datasets.sh is NOT optional, despite this file's
# former comment claiming it only matters for plugins nothing here uses
# yet — MAVROS's UAS node calls into GeographicLib unconditionally during
# its own init and throws an uncaught std::invalid_argument
# ("File not readable .../egm96-5.pgm") if the geoid dataset is missing,
# aborting mavros_node before it ever reaches a ready state. Confirmed on
# a real ros-humble-mavros install, not assumed. Needs real internet
# access for its download, same INFRA-006 reasoning as the apt-get above
# — scripts/11-build-gcs-comms-image.sh's `docker build --network=host`
# already covers this RUN step too.
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

COPY ws/src/gcs_comms/params/default.yaml /etc/drone-mc/params/gcs_comms.yaml
COPY ws/src/gcs_comms/params/mavros.yaml /etc/drone-mc/params/mavros.yaml
COPY docker/gcs-comms/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
