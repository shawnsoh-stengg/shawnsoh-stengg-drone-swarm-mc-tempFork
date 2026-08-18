#!/usr/bin/env bash
# 14b-smoke-test-health.sh — Verify the health-monitor container (HEALTH-004):
# read-only host /proc,/sys access, systemd hardening, and health_monitor_node
# itself. Ref: SDD §13.2 smoke test pattern.
#
# Static checks (always run): Dockerfile/entrypoint content, package/
# interface/executable registration, and the systemd unit's read-only
# mounts + hardening directives (AC1 — these are unit-file facts, checkable
# without a live systemd/docker environment).
#
# A live publish check (AC2's "/mc/health published within 10s") runs
# against ros2-core:latest + the bind-mounted install, same pattern
# 08b-smoke-test-fcc-bridge.sh uses — it doesn't need the actual
# health-monitor image built first, since the entrypoint is just
# `ros2 run health_monitor health_monitor_node`.
#
# Set LIVE_CHECK=1 to additionally build/run the actual image (requires
# scripts/14-build-health-monitor-image.sh to have succeeded on this host)
# and confirm: the real container reaches the same 10s publish budget,
# `systemd-analyze security` scores the hardening (needs a real systemd
# install — most dev/CI hosts don't have one usable this way), and AC3
# (this container is unaffected by another MC service's container
# restarting).
#
# NOT smoke-testable in any environment: AC2's "GCS receives health
# telemetry in QGC" needs a real QGroundControl instance on the far end of
# a real MAVLink link — same class of gap PROJECT-PLAN-5-MONTH.md's own
# GCS-001 smoke-test note already documents for HEARTBEAT-in-QGC.
set -euo pipefail

PASS=0; FAIL=0
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKERFILE="${REPO_ROOT}/docker/health-monitor/Dockerfile"
ENTRYPOINT="${REPO_ROOT}/docker/health-monitor/entrypoint.sh"
UNIT="${REPO_ROOT}/config/systemd/drone-mc-health-monitor.service"
HEALTH_MONITOR_YAML="${REPO_ROOT}/ws/src/health_monitor/params/default.yaml"
INSTALL_DIR="${DRONE_MC_INSTALL_DIR:-/opt/drone-mc/install}"
CORE_IMAGE="${MC_VERSION:+ros2-core:${MC_VERSION}}"
CORE_IMAGE="${CORE_IMAGE:-ros2-core:latest}"
IMAGE="${MC_VERSION:+ghcr.io/drone-mc/health-monitor:${MC_VERSION}}"
IMAGE="${IMAGE:-ghcr.io/drone-mc/health-monitor:dev}"

check() {
    local desc="$1"; shift
    if "$@" &>/dev/null; then
        printf "PASS: %s\n" "$desc"; PASS=$((PASS + 1))
    else
        printf "FAIL: %s\n" "$desc"; FAIL=$((FAIL + 1))
    fi
}

check "Dockerfile present"          test -f "${DOCKERFILE}"
check "entrypoint.sh present"       test -f "${ENTRYPOINT}"
check "entrypoint.sh is executable" test -x "${ENTRYPOINT}"
check "systemd unit present"        test -f "${UNIT}"
check "health_monitor params file present" test -f "${HEALTH_MONITOR_YAML}"

check "entrypoint launches health_monitor_node" \
    grep -q 'ros2 run health_monitor health_monitor_node' "${ENTRYPOINT}"
check "entrypoint references health_monitor params file" \
    grep -q '/etc/drone-mc/params/health_monitor.yaml' "${ENTRYPOINT}"
check "entrypoint traps SIGTERM/SIGINT for a clean node shutdown" \
    grep -qE 'trap terminate SIGTERM SIGINT' "${ENTRYPOINT}"

check "systemd unit runs ghcr.io/drone-mc/health-monitor image" \
    grep -q 'ghcr.io/drone-mc/health-monitor:' "${UNIT}"
check "systemd unit bind-mounts the shared colcon install" \
    grep -qE -- '--mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install' "${UNIT}"
check "systemd unit bind-mounts /etc/drone-mc read-only" \
    grep -qE -- '--mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly' "${UNIT}"

# AC1: "Read-only access to /sys, /proc" — the host's real procfs/sysfs,
# bind-mounted read-only, so resource_sampler.cpp's /proc/stat,
# /proc/meminfo, /sys/class/thermal/* reads (SDD §10.1) reflect genuine
# host state rather than this container's own restricted view.
check "systemd unit bind-mounts host /proc read-only" \
    grep -qE -- '--mount type=bind,src=/proc,dst=/proc,readonly' "${UNIT}"
check "systemd unit bind-mounts host /sys read-only" \
    grep -qE -- '--mount type=bind,src=/sys,dst=/sys,readonly' "${UNIT}"

# AC1: "ProtectSystem=strict, PrivateTmp=yes" plus the additional
# systemd-analyze security hardening this unit adds beyond every other
# service's own baseline (see the unit file's own comment for why only
# this one gets the fuller treatment).
check "systemd unit sets ProtectSystem=strict" grep -qE '^ProtectSystem=strict$' "${UNIT}"
check "systemd unit sets PrivateTmp=yes" grep -qE '^PrivateTmp=yes$' "${UNIT}"
check "systemd unit sets NoNewPrivileges=yes" grep -qE '^NoNewPrivileges=yes$' "${UNIT}"
check "systemd unit sets ProtectKernelTunables=yes" grep -qE '^ProtectKernelTunables=yes$' "${UNIT}"
check "systemd unit sets ProtectKernelModules=yes" grep -qE '^ProtectKernelModules=yes$' "${UNIT}"
check "systemd unit sets ProtectControlGroups=yes" grep -qE '^ProtectControlGroups=yes$' "${UNIT}"
check "systemd unit sets RestrictNamespaces=yes" grep -qE '^RestrictNamespaces=yes$' "${UNIT}"
check "systemd unit sets LockPersonality=yes" grep -qE '^LockPersonality=yes$' "${UNIT}"
check "systemd unit sets RestrictAddressFamilies" grep -qE '^RestrictAddressFamilies=' "${UNIT}"

check "systemd unit restarts on failure" grep -q 'Restart=on-failure' "${UNIT}"
check "systemd unit in be.slice" grep -qE '^Slice=be.slice$' "${UNIT}"

# AC3: "Container survives restart of any other MC service" is structural,
# not behavioral — this unit must not Require= (or otherwise depend on)
# any other drone-mc-*.service, so systemd never stops/restarts it as a
# side effect of another service's own restart. (Its own header comment
# already states it has "no peer dependencies" — this checks that claim
# against the actual unit file.)
check "systemd unit requires only docker.service, no peer MC service" \
    bash -c "grep -E '^Requires=' '${UNIT}' | grep -qvE 'drone-mc-'"

# --network=host matches every drone-mc-*.service unit — see
# 08b-smoke-test-fcc-bridge.sh's own comment on why (CycloneDDS's
# net.core.rmem_max/wmem_max sysctl floor is per-netns, not global).
RUN=(docker run --rm --network=host --volume "${INSTALL_DIR}:/opt/drone-mc/install:ro" "${CORE_IMAGE}")
SOURCE="source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash"

check "health_monitor package registered" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 pkg list | grep -qx health_monitor"
check "mc_msgs package registered" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 pkg list | grep -qx mc_msgs"
check "health_monitor_node executable present" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 pkg executables health_monitor | grep -q health_monitor_node"
check "mc_msgs/SystemHealth interface readable" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 interface show mc_msgs/msg/SystemHealth"
check "mc_msgs/FaultEvent interface readable" "${RUN[@]}" bash -c \
    "${SOURCE} && ros2 interface show mc_msgs/msg/FaultEvent"

# AC2 ("/mc/health published within 10s of container start"): same node,
# same entrypoint command, run against ros2-core:latest rather than the
# built health-monitor image — the entrypoint does nothing but this one
# `ros2 run`, so this is a faithful stand-in that doesn't require the
# image to be built first. `ros2 run` execs the node as a child of a
# Python wrapper (not via exec), so it's signalled by name via pkill —
# same reasoning as 08b-smoke-test-fcc-bridge.sh's own equivalent check.
check "health_monitor_node publishes /mc/health within 10s and exits cleanly" \
    "${RUN[@]}" bash -c "
        ${SOURCE}
        ros2 run health_monitor health_monitor_node &
        node_pid=\$!
        published=0
        for _ in \$(seq 1 10); do
            if timeout 2 ros2 topic echo --once /mc/health &>/dev/null; then
                published=1
                break
            fi
            sleep 1
        done
        pkill -INT -f 'health_monitor_node\$'
        wait \"\${node_pid}\"
        exit_code=\$?
        [[ \${published} -eq 1 ]] || exit 1
        [[ \${exit_code} -eq 0 ]]
    "

if [[ "${LIVE_CHECK:-0}" -eq 1 ]]; then
    echo "==> LIVE_CHECK=1: exercising the real container"

    if ! docker image inspect "${IMAGE}" &>/dev/null; then
        echo "==> ERROR: ${IMAGE} not found — run scripts/14-build-health-monitor-image.sh first" >&2
        exit 1
    fi

    trap '[[ -n "${cid:-}" ]] && docker rm -f "${cid}" &>/dev/null || true
          [[ -n "${other_cid:-}" ]] && docker rm -f "${other_cid}" &>/dev/null || true' EXIT

    # --network=host, no /etc/drone-mc or host /proc,/sys bind-mounts here
    # (this is a standalone container smoke test, not the real systemd
    # unit) — falls back to the image's own baked-in params copy, same
    # rationale 11b-smoke-test-gcs-comms.sh's LIVE_CHECK section gives.
    cid="$(docker run -d --rm --network=host "${IMAGE}")"

    echo "==> AC2: waiting up to 10s for /mc/health to publish"
    health_published() {
        docker exec "${cid}" bash -c \
            "source /opt/ros/humble/install/setup.bash && source /opt/drone-mc/install/setup.bash && timeout 2 ros2 topic echo --once /mc/health" \
            &>/dev/null
    }
    ac2_ok=0
    for _ in $(seq 1 10); do
        if health_published; then ac2_ok=1; break; fi
        sleep 1
    done
    check "AC2: /mc/health publishing within 10s of real container start" \
        bash -c "[[ ${ac2_ok} -eq 1 ]]"

    # AC1: systemd-analyze security scores the *installed unit*, not this
    # ad-hoc container — needs the real unit installed via
    # scripts/02-install-systemd-units.sh on a host that actually has
    # systemd-analyze (most dev/CI containers don't).
    if command -v systemd-analyze >/dev/null 2>&1 && [[ -f /etc/systemd/system/drone-mc-health-monitor.service ]]; then
        echo "==> AC1: systemd-analyze security drone-mc-health-monitor.service"
        systemd-analyze security drone-mc-health-monitor.service || true
    else
        echo "==> AC1: systemd-analyze unavailable or unit not installed here — skipping score check" \
             "(install via scripts/02-install-systemd-units.sh on a real systemd host first)"
    fi

    # AC3: start an unrelated container under the same "mc-*" naming
    # convention and kill/restart it, then confirm health-monitor's own
    # container never stopped and is still publishing. This stands in for
    # "restart of any other MC service" without needing all 9 services
    # actually deployed.
    echo "==> AC3: restarting an unrelated container, confirming health-monitor is unaffected"
    other_cid="$(docker run -d --rm --name mc-smoke-other --network=host "${CORE_IMAGE}" sleep 300)"
    docker kill "${other_cid}" &>/dev/null || true
    docker wait "${other_cid}" &>/dev/null || true
    other_cid=""
    check "AC3: health-monitor container still running after an unrelated container restarted" \
        bash -c "docker inspect -f '{{.State.Running}}' '${cid}' | grep -qx true"
    check "AC3: health-monitor still publishing /mc/health after an unrelated container restarted" \
        health_published
fi

printf "\nResults: %d passed, %d failed\n" "${PASS}" "${FAIL}"
[[ "${FAIL}" -eq 0 ]] || exit 1
