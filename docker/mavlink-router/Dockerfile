# mavlink-router service image (GCS-001/MAVR-001) — builds mavlink-router
# via Meson/Ninja per CLAUDE.md's documented build step, pinned to a tagged
# release.
#
# Standalone C++ daemon with no ROS 2 dependency — built FROM plain Ubuntu
# 22.04 (matches the Jetson's JetPack 6.2.2 userland) rather than FROM
# ros2-core, so this image doesn't drag in a ROS 2 toolchain it never uses.
#
# Pinned to an exact upstream commit (the tip of tag v4 as of 2026-07-23),
# not the floating tag, so a re-build can't silently pick up a moved tag
# (same policy as docker/ros2-core/Dockerfile's FROM-digest pin). To bump:
#   git ls-remote --tags https://github.com/mavlink-router/mavlink-router.git
FROM ubuntu:22.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates \
        build-essential meson ninja-build pkg-config \
        python3 python3-pip \
        libsystemd-dev \
    && rm -rf /var/lib/apt/lists/*

ARG MAVLINK_ROUTER_COMMIT=42529d55b665e0a9a29e424e186f514c56c2e5b5
WORKDIR /build
RUN git init -q . \
    && git remote add origin https://github.com/mavlink-router/mavlink-router.git \
    && git fetch --depth 1 origin "${MAVLINK_ROUTER_COMMIT}" \
    && git checkout -q FETCH_HEAD \
    && git submodule update --init --recursive --depth 1
# -Dsystemdsystemunitdir bypasses meson.build's `dependency('systemd')`
# lookup (only reachable when this option is left at its default 'auto') —
# that pkg-config module is `systemd.pc`, which is a *different* package
# than the `libsystemd.pc` libsystemd-dev provides above and isn't shipped
# by any package installed here. Harmless to skip: this project runs
# mavlink-routerd from its own entrypoint.sh/container (SDD §12.2-§12.4),
# never mavlink-router's own upstream-installed systemd unit.
RUN meson setup build . --buildtype=release -Dsystemdsystemunitdir=/usr/lib/systemd/system \
    && ninja -C build \
    && ninja -C build install

FROM ubuntu:22.04
RUN apt-get update && apt-get install -y --no-install-recommends \
        libsystemd0 \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /etc/drone-mc

COPY --from=builder /usr/bin/mavlink-routerd /usr/bin/mavlink-routerd
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
