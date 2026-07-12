# mavlink-router service image (GCS-001/MAVR-001) — builds mavlink-router
# via Meson/Ninja per CLAUDE.md's documented build step, pinned to a tagged
# release.
#
# Standalone C++ daemon with no ROS 2 dependency — built FROM plain Ubuntu
# 22.04 (matches the Jetson's JetPack 6.2.2 userland) rather than FROM
# ros2-core, so this image doesn't drag in a ROS 2 toolchain it never uses.
#
# Before tagging a release, pin the exact upstream commit this was built
# from (same policy as docker/ros2-core/Dockerfile's FROM-digest pin):
#   git ls-remote --tags https://github.com/mavlink-router/mavlink-router.git
FROM ubuntu:22.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates \
        build-essential meson ninja-build pkg-config \
        python3 python3-pip \
        libsystemd-dev \
    && rm -rf /var/lib/apt/lists/*

ARG MAVLINK_ROUTER_VERSION=v4
WORKDIR /build
RUN git clone --recurse-submodules --depth 1 --branch "${MAVLINK_ROUTER_VERSION}" \
        https://github.com/mavlink-router/mavlink-router.git .
RUN meson setup build . --buildtype=release \
    && ninja -C build \
    && ninja -C build install

FROM ubuntu:22.04
RUN apt-get update && apt-get install -y --no-install-recommends \
        libsystemd0 \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /etc/drone-mc

COPY --from=builder /usr/local/bin/mavlink-routerd /usr/local/bin/mavlink-routerd
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
