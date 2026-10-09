# zenoh-bridge-dds service image — bridges selected DDS topics across the
# MANET mesh (192.168.100.x) via a static-peer Zenoh mesh, replacing
# CycloneDDS's own cross-host unicast discovery (docker/ros2-core/cyclonedds.xml
# is now lo-only; see config/zenoh-bridge-dds.json5 for where the per-drone
# peer list moved to).
#
# Standalone Rust daemon with no ROS 2 dependency — built FROM plain Ubuntu
# 22.04 (matches the Jetson's JetPack 6.2.2 userland), same as
# docker/mavlink-router/Dockerfile. Unlike mavlink-router, this is a
# single-stage image: Eclipse Zenoh publishes a prebuilt aarch64-gnu .deb
# per tagged release, so there's no Rust/CMake/clang builder stage to carry
# — building zenoh-bridge-dds from source needs a pinned Rust toolchain
# plus llvm-dev/libclang-dev/CMake to build CycloneDDS as a native
# dependency, meaningfully heavier than downloading the vendor's own build.
#
# Before tagging a release, confirm the pinned version below still ships an
# aarch64-unknown-linux-gnu-debian.zip asset:
#   https://github.com/eclipse-zenoh/zenoh-plugin-dds/releases
#
# NOTE: the vendor's -gnu build targets glibc; it has not yet been
# smoke-tested against JetPack 6.2.2's glibc 2.35 on real Jetson hardware —
# scripts/10b-smoke-test-zenoh-bridge.sh's LIVE_CHECK=1 path exists
# specifically to catch a compatibility mismatch before this ships.
FROM ubuntu:22.04

ARG ZENOH_BRIDGE_DDS_VERSION=1.9.0

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl unzip \
    && rm -rf /var/lib/apt/lists/*

# zenoh-plugin-dds's .deb depends on zenohd (=version) — the base Zenoh
# router daemon — but that binary ships from the separate eclipse-zenoh/zenoh
# repo's own release, not from zenoh-plugin-dds's. Both pinned to the same
# ${ZENOH_BRIDGE_DDS_VERSION} (zenoh-plugin-dds's releases track zenoh's
# versioning exactly, hence the hard `=` dependency); both .deb sets are
# installed together so apt can resolve that dependency from the local
# files instead of needing it in an apt repo.
RUN curl -fsSL -o /tmp/zenoh.zip \
        "https://github.com/eclipse-zenoh/zenoh/releases/download/${ZENOH_BRIDGE_DDS_VERSION}/zenoh-${ZENOH_BRIDGE_DDS_VERSION}-aarch64-unknown-linux-gnu-debian.zip" \
    && curl -fsSL -o /tmp/zenoh-bridge-dds.zip \
        "https://github.com/eclipse-zenoh/zenoh-plugin-dds/releases/download/${ZENOH_BRIDGE_DDS_VERSION}/zenoh-plugin-dds-${ZENOH_BRIDGE_DDS_VERSION}-aarch64-unknown-linux-gnu-debian.zip" \
    && mkdir -p /tmp/zenoh-bridge-dds-deb \
    && unzip -j /tmp/zenoh.zip 'zenohd_*.deb' -d /tmp/zenoh-bridge-dds-deb \
    && unzip -j /tmp/zenoh-bridge-dds.zip '*.deb' -d /tmp/zenoh-bridge-dds-deb \
    && apt-get update && apt-get install -y --no-install-recommends /tmp/zenoh-bridge-dds-deb/*.deb \
    && rm -rf /tmp/zenoh.zip /tmp/zenoh-bridge-dds.zip /tmp/zenoh-bridge-dds-deb /var/lib/apt/lists/*

RUN mkdir -p /etc/drone-mc

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
