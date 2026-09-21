# Deployment (systemd + Docker)

How the MC software actually gets onto a Jetson and stays running, grounded
in `config/systemd/`, `docker/*/Dockerfile`, and `scripts/02-install-systemd-units.sh`
as they exist today. For the design rationale behind these choices see
`SDD-MC-001.md` §12; for the build chain that produces the images this
document deploys, see `docs/GETTING-STARTED.md`.

**Not Docker Compose.** Every service is one Docker container, but
lifecycle, restart policy, and resource limits are all owned by `systemd`
— each container is `docker run --rm` inside a `.service` unit, not
`docker-compose up`. See `CLAUDE.md`'s Key Design Patterns for why.

## Implementation status

Three different states exist among the 9 service units in
`config/systemd/`, and it matters which one you're looking at before you
try to run something:

- ✅ **buildable and deployable** — has a `docker/<name>/Dockerfile`, a
  `scripts/NN-build-*-image.sh`, and its systemd unit's `ExecStart` image
  reference resolves to something you can actually build.
- 🧭 **unit installed, image not buildable yet** — the `.service` file
  exists under `config/systemd/` and `scripts/02-install-systemd-units.sh`
  will happily install and enable it, but there is no corresponding
  `docker/<name>/` directory or build script. Starting `drone-mc.target`
  today will fail at `docker run: image not found` for these until their
  images exist.
- ⛔ **no unit at all** — the package's nodes have no `drone-mc-*.service`
  in `config/systemd/` yet; deploying them isn't wired up even at the unit
  level.

| Service | Package(s) | Status |
|---|---|---|
| `drone-mc-fcc-bridge` | `fcc_bridge` | ✅ |
| `drone-mc-mavlink-router` | (external daemon) | ✅ |
| `drone-mc-zenoh-bridge` | (external daemon) | ✅ |
| `drone-mc-gcs-comms` | `gcs_comms` (`gcs_mavlink_node` + MAVROS) | ✅ |
| `drone-mc-mission-orchestrator` | `gcs_comms` (`mission_orchestrator_node`, Channel C) | ✅ (GCS-006) |
| `drone-mc-mission-exec` | `mission_exec` (`mission_manager_node` + `search_planner_node`) | ✅ |
| `drone-mc-health-monitor` | `health_monitor` | ✅ (HEALTH-004) |
| `drone-mc-swarm-coord` | `swarm_coord` | ✅ (`DEP-001`, 2026-08-19 — image builds; `swarm_coordinator_node`/`formation_controller_node` are still §2.4/§5/§6 scaffolds, tracked separately under Track A) |
| `drone-mc-vision` | `vision` | 🧭 (`ghcr.io/drone-mc/vision`, no `docker/vision/` yet) |
| `drone-mc-camera` | (gscam2 capture, no ROS 2 package of its own yet) | 🧭 (`ghcr.io/drone-mc/camera`, no `docker/camera/` yet) |
| — | `video_downlink` (`perception_front_node`, `video_mux_node`) | ⛔ — no `drone-mc-video-ingest`/`drone-mc-video-downlink` unit exists; `CLAUDE.md`'s repo-layout note flags this as SDD §12.4 design only |

Also note: `mission_orchestrator_node` (Channel C, package `gcs_comms`) is
deliberately **not** part of `drone-mc-gcs-comms.service` — it's out of
scope for that container per its own header comment. As of `GCS-006`
(2026-08-26) it has its own unit/container instead, `drone-mc-mission-
orchestrator`, listed in the table above.

## Topology and startup order

`drone-mc.target` is what you actually start; every service unit declares
`PartOf=drone-mc.target` (so stopping the target stops everything) and
`WantedBy=drone-mc.target` (so enabling the target pulls all of them in).
Startup order is derived entirely from each unit's own `After=`/`Requires=`:

```
drone-mc-health-monitor          (no peer deps — up before everything it watches)
        |
        +-- drone-mc-mavlink-router        (no peer deps beyond health-monitor)
        +-- drone-mc-zenoh-bridge          (no peer deps beyond health-monitor)
        +-- drone-mc-camera                (no peer deps beyond health-monitor)
        |
        +-- drone-mc-fcc-bridge  <-- requires mavlink-router
                |
                +-- drone-mc-gcs-comms  <-- requires fcc-bridge + mavlink-router
                |       |
                |       +-- drone-mc-swarm-coord  <-- requires gcs-comms
                |       |       |
                |       |       +-- drone-mc-mission-exec  <-- requires fcc-bridge + gcs-comms + swarm-coord
                |
                +-- drone-mc-mission-orchestrator  <-- requires fcc-bridge + mavlink-router (GCS-006)
                |
                +-- drone-mc-vision  <-- requires camera + fcc-bridge
```

`rt.slice` (cgroup v2, `CPUWeight=800`, `MemoryHigh=4G`) holds the
latency-sensitive services: `fcc-bridge`, `swarm-coord`. Everything else —
including `mavlink-router` and `zenoh-bridge` — runs in `be.slice`
(`CPUWeight=200`, `MemoryHigh=8G`). See `SDD-MC-001.md` §12.3 for why
`swarm-coord` gets `rt.slice` despite being "best-effort" by the
Layer-4-vs-Layer-2 split elsewhere in the docs — it's a latency
classification here, not a layer classification.

`vision`'s `MemoryMax=6G` (be.slice) is watched from the outside, not just
capped: `health_monitor_node` reads that cgroup's `memory.current`/
`memory.max` directly (`INFRA-007`) and publishes `VISION_MEM_HIGH`
(`/mc/fault`) once usage has stayed at/above 80% of the cap continuously
for 30 s — long enough that a TensorRT engine reload's transient spike
doesn't false-alarm, short enough to give real headroom before an
OOM-kill. See `ICD-MC-001.md` §10.8 and `ws/src/health_monitor/params/
default.yaml`'s `vision_mem_*` params to retune either number.

## Per-service unit anatomy

Every `drone-mc-*.service` follows the same shape (`drone-mc-fcc-bridge.service` shown):

```ini
[Unit]
PartOf=drone-mc.target
After=docker.service drone-mc-health-monitor.service drone-mc-mavlink-router.service
Requires=docker.service drone-mc-mavlink-router.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Slice=rt.slice
EnvironmentFile=/etc/drone-mc/environment
ExecStart=/usr/bin/docker run --rm --name mc-fcc-bridge \
  --device /dev/ttyTHS1 \
  --network=host \
  --mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install \
  --mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly \
  --env-file /etc/drone-mc/environment \
  ghcr.io/drone-mc/fcc-bridge:${MC_VERSION}
ExecStop=/usr/bin/docker stop mc-fcc-bridge
Restart=on-failure
RestartSec=5s
MemoryMax=512M
CPUQuota=50%
DeviceAllow=/dev/ttyTHS1 rw
PrivateTmp=yes
ProtectSystem=strict

[Install]
WantedBy=drone-mc.target
```

Points worth knowing when you add a new unit:

- **`--network=host`** everywhere — DDS and MAVLink UDP traffic needs the
  host's real interfaces, not Docker's bridge network.
- **Two bind mounts, not one image copy** — `/opt/drone-mc/install` (the
  shared colcon workspace, built once by `scripts/05-build-workspace.sh`)
  and `/etc/drone-mc` (runtime config, read-only) are mounted from the
  host into every ROS 2 service container. The image's own baked-in
  install (from its Dockerfile's builder stage) only matters when the
  container runs *without* that mount — i.e. local dev/CI smoke tests, not
  production. `mavlink-router` and `zenoh-bridge` skip the workspace mount
  entirely — they're standalone daemons with no colcon/ROS 2 dependency.
- **`Restart=on-failure` + `StartLimitIntervalSec=60`/`StartLimitBurst=3`**
  on every unit — systemd, not the application, supervises the container's
  lifecycle. A service that crash-loops more than 3 times in 60 s is left
  down rather than infinitely retried.
- **Device passthrough** is per-service and minimal: `fcc-bridge` gets
  `/dev/ttyTHS1`, `camera` gets `/dev/video0`, `vision` gets `--runtime
  nvidia` plus `/dev/nvhost-gpu`/`/dev/nvmap` — nothing gets more device
  access than its one job requires.
- **`ProtectSystem=strict` + `PrivateTmp=yes`** on every unit as a systemd
  hardening baseline, independent of whatever hardening the container
  image itself does. `health-monitor` goes further (HEALTH-004):
  `NoNewPrivileges`, `Protect{Home,Hostname,Clock,KernelTunables,
  KernelModules,KernelLogs,ControlGroups}=yes`, `RestrictNamespaces`,
  `RestrictSUIDSGID`, `RestrictRealtime`, `LockPersonality`, `RemoveIPC`,
  `MemoryDenyWriteExecute`, `PrivateNetwork=yes` (safe here specifically
  because the `docker` CLI only ever talks to `dockerd` over its `AF_UNIX`
  socket, empirically confirmed not to affect a `--network=host`
  container's own networking), `RestrictAddressFamilies`,
  `SystemCallFilter=@system-service`, `KeyringMode=private`, `UMask=0077`
  — measured via `systemd-analyze security` (a real, installed systemd
  user-unit test in this repo's own dev sandbox) at exposure 9.2 `UNSAFE`
  before these additions, 4.1 `OK` after. `User=`/`DynamicUser=` and a
  restricted `CapabilityBoundingSet=` were deliberately left alone —
  dropping capabilities from an already-root process needs privilege an
  unprivileged test session can't grant, so a wrong guess there couldn't
  be verified without risking a hard failure (`status=218/CAPABILITIES`)
  on real hardware; that's real follow-up hardening, not done here. It
  also bind-mounts the host's real `/proc` and `/sys` read-only (AC1) —
  directly, not shadowed at `/host/proc`/`/host/sys` — since
  `health_monitor_node`'s own resource-sampling (SDD §10.1) only ever
  reads `/proc/stat`, `/proc/meminfo`, and `/sys/class/thermal/*`, none of
  which are per-PID paths.
- **`MemoryMax`/`CPUQuota`** are per-service resource caps (systemd-level,
  layered on top of the slice's `MemoryHigh`), sized to what that one
  service actually needs — e.g. `vision` gets `MemoryMax=6G` for the
  TensorRT pipeline, `mavlink-router`/`zenoh-bridge` get `MemoryMax=256M`.

## Docker image structure

### `ros2-core` — the shared base image

`docker/ros2-core/Dockerfile` is `FROM dustynv/ros:humble-ros-base-l4t-r36.3.0`
(JetPack 6.2.2 / Ubuntu 22.04 aarch64) and every ROS 2 service image builds
`FROM` it. It installs the CycloneDDS RMW, generated MAVLink C headers,
`geographic_msgs`, `nlohmann-json3-dev`, and the colcon build toolchain,
then bakes in a default `cyclonedds.xml` (same-host `lo`-only discovery,
overridden at runtime by the `/etc/drone-mc` bind mount). `mavlink-router`
and `zenoh-bridge` are **not** built from this image — they're standalone
daemons (`FROM ubuntu:22.04`) with no ROS 2 dependency of their own.

### Per-service images: the multi-stage pattern

Every ROS 2 service Dockerfile (`fcc-bridge`, `gcs-comms`, `mission-exec`)
follows a builder → runtime split: a `builder` stage `FROM ros2-core`
colcon-builds just that package (`--packages-up-to <pkg>`,
`-DBUILD_TESTING=OFF`) into `/opt/drone-mc/install`, and a `runtime` stage
`FROM ros2-core` copies only that build output in — no build tooling or
test dependencies ship in the final image. `fcc-bridge`'s Dockerfile adds a
third stage that builds the Micro XRCE-DDS Agent (FCC-008) from source
`FROM ubuntu:22.04` (no arm64 apt package exists), whose output is then
copied into the same runtime stage — a second, independent builder rather
than dragging the ROS 2 toolchain into a build that never uses it.

Build context for every service image is the **repo root** (they need
`ws/src/*`), and each has a numbered `scripts/NN-build-*-image.sh` wrapper
you should use instead of invoking `docker build` directly — the wrapper
fetches `px4_msgs` into `ws/src/` first when needed. See
`docs/GETTING-STARTED.md` for the full ordered build chain.

### Dual-process supervision inside one container

Two of the built images run more than one process under one container
lifecycle, each with the same `trap` + `wait -n` shape in `entrypoint.sh`
— either process dying takes the whole container down, and
`Restart=on-failure` brings both back together:

- **`fcc-bridge`**: the Micro XRCE-DDS Agent (transport picked by
  `FCC_TRANSPORT=serial|ethernet`, per `HID-MC-001.md` §4.3) plus
  `fcc_bridge_node` — the node is useless without a live agent and vice
  versa.
- **`mavlink-router`**: two independent `mavlink-routerd` processes, one
  per config file (`mavlink-router-channel-a.conf`, `-channel-b.conf`) —
  run as separate processes specifically so a bug or flood on one GCS
  channel can't reach the other (GCS-001/MAVR-001 channel isolation; see
  each `.conf`'s own header comment for why one shared process can't
  provide that guarantee).

## Configuration: `/etc/drone-mc`

Every container bind-mounts host `/etc/drone-mc` read-only over its own
copy — anything baked into an image (`cyclonedds.xml`) or only ever shipped
in the repo (the mavlink-router channel confs) has to actually land in
`/etc/drone-mc` on the host, or the bind mount just hides it behind an
empty directory. `scripts/02-install-systemd-units.sh` provisions:

| File | Source | First-install behavior |
|---|---|---|
| `/etc/drone-mc/environment` | `config/environment` | Only installed if absent — existing per-drone values are never overwritten. This is the one file an operator hand-edits; everything below is now rendered *from* it (`INFRA-008`) |
| `/etc/drone-mc/cyclonedds.xml` | `docker/ros2-core/cyclonedds.xml` | Only installed if absent |
| `/etc/drone-mc/zenoh-bridge-dds.json5` | `config/zenoh-bridge-dds.json5` (envsubst template) | Always regenerated from `config/fleet.yaml` + this drone's `DRONE_ID` — no longer hand-edited per drone |
| `/etc/drone-mc/mavlink-router-channel-a.conf` | `config/mavlink-router-channel-a.conf` (envsubst template) | Always re-rendered against `FCC_GCS_IP` |
| `/etc/drone-mc/mavlink-router-channel-b.conf` | `config/mavlink-router-channel-b.conf` (envsubst template) | Always re-rendered against `GCS_IP` |
| `/etc/drone-mc/mavlink-router-channel-c.conf` | `config/mavlink-router-channel-c.conf` (envsubst template) | Always re-rendered against `GCS_IP` |
| `/etc/drone-mc/params/fcc_bridge.yaml` | `ws/src/fcc_bridge/params/default.yaml` | Only installed if absent |
| `/etc/drone-mc/params/gcs_comms.yaml` | `ws/src/gcs_comms/params/default.yaml` | Only installed if absent |
| `/etc/drone-mc/params/mavros.yaml` | `ws/src/gcs_comms/params/mavros.yaml` | Only installed if absent |
| `/etc/drone-mc/params/mission_exec.yaml` | `ws/src/mission_exec/params/default.yaml` | Only installed if absent |
| `/etc/drone-mc/params/health_monitor.yaml` | `ws/src/health_monitor/params/default.yaml` | Only installed if absent |

`scripts/01-install-os.sh` provisions one more, ahead of the netplan apply
it needs those values for:

| File | Source | First-install behavior |
|---|---|---|
| `/etc/netplan/01-drone-mc.yaml` | `config/netplan/01-drone-mc.yaml` (envsubst template) | Always re-rendered against `MC_ETH_MAC`/`MC_ETH_IP`/`MC_ETH_INTERNAL_IP`, then `netplan apply` |

Every `docker/*/entrypoint.sh` passes `--params-file /etc/drone-mc/params/<name>.yaml`
by that exact filename — each package only ever shipped its own
`params/default.yaml` under `ws/src/`, and nothing copied it into
`/etc/drone-mc/params` under the name the entrypoint looks for, until this
provisioning block was added (2026-08-19). Before that fix, every affected
node (`health_monitor_node`, `mission_manager_node`/`search_planner_node`,
`fcc_bridge_node`, `gcs_mavlink_node`, `mavros_node`) aborted at startup
with `rclcpp::exceptions::RCLInvalidROSArgsError` / "Error opening YAML
file" — `rcl` treats a missing `--params-file` target as fatal, it does
not fall back to each parameter's own `declare_parameter(name, default)`
value.

**Before first `systemctl start drone-mc.target` on a given vehicle**, edit
`/etc/drone-mc/environment` — set `DRONE_ID`, `MC_VERSION` (image tag),
`FCC_TRANSPORT` (`serial` for production ttyTHS1, `ethernet` for the
192.168.12.x fallback), and this vehicle's own network identity
(`MC_ETH_MAC`/`MC_ETH_IP`/`MC_ETH_INTERNAL_IP`, `FCC_GCS_IP`). Full
variable reference in `config/environment`'s own comments. That one file
is now the only thing to hand-edit — see "Bringing up drone N" below.

### Bringing up drone N (`INFRA-008`)

Before this ticket, four separate files needed hand-editing per physical
unit (`mavlink-router-channel-a.conf`'s FCC address, `netplan/
01-drone-mc.yaml`'s MAC/IPs, `zenoh-bridge-dds.json5`'s peer list, and
implicitly `/etc/drone-mc/environment` itself) with no error if one was
missed — a stale hardcoded value just meant that one channel silently
didn't come up. `mavlink-router-channel-{a,b,c}.conf`, `netplan/
01-drone-mc.yaml`, and `zenoh-bridge-dds.json5` are now `envsubst`
templates (`${VAR}` placeholders), rendered by `01-install-os.sh`/
`02-install-systemd-units.sh` against `/etc/drone-mc/environment` — so
bringing up drone N is:

1. Add drone N's `DRONE_ID` → `MC_ETH_IP` mapping to `config/fleet.yaml`
   (used to generate every drone's zenoh peer list, self excluded).
2. On drone N's Jetson, run `01-install-os.sh` once — this seeds
   `/etc/drone-mc/environment` from `config/environment`'s defaults.
3. Edit `/etc/drone-mc/environment` on drone N: `DRONE_ID`, `MC_ETH_MAC`
   (this unit's real NIC MAC — see the file that used to hardcode this,
   `config/netplan/01-drone-mc.yaml`, for why it must be matched by MAC
   not interface name), `MC_ETH_IP`, `MC_ETH_INTERNAL_IP`, `FCC_GCS_IP`
   (this unit's own Pixhawk address).
4. Re-run `01-install-os.sh` (renders + applies netplan with the real
   values) and `02-install-systemd-units.sh` (renders the mavlink-router
   confs and the zenoh peer list).

No `.conf`/`.yaml` file under `/etc/drone-mc` or `/etc/netplan` is ever
hand-edited directly — `GCS_IP` and the channel ports stay fleet-wide
constants (same value on every drone) and are not per-drone inputs.

## Known deployment gaps

- ~~Channel C (mission plan library) has no routed transport in
  production.~~ **Closed by `GCS-006` (2026-08-26).**
  `docker/mavlink-router/entrypoint.sh` now launches a third
  `mavlink-routerd` process against `mavlink-router-channel-c.conf`
  alongside `-a`/`-b`, `scripts/02-install-systemd-units.sh` provisions
  that conf file and `/var/drone-mc/mission_plans` onto the host, and
  `drone-mc-mission-orchestrator.service` runs `mission_orchestrator_node`
  in its own container. See `PROJECT-PLAN-5-MONTH.md`'s `GCS-006` entry.
- **Two service units reference images that don't build yet**
  (`vision`, `camera` — see the status table above; `swarm-coord` closed
  this gap in `DEP-001` (2026-08-19), `health-monitor` in `HEALTH-004` —
  this bullet previously listed `swarm-coord` here too, stale as of
  `FT-004`'s 2026-09-21 bench run, which confirmed
  `ghcr.io/drone-mc/swarm-coord` builds and starts). `02b-smoke-test-
  units.sh` verifies the *unit files* are valid and enabled; it does not
  verify the images they reference exist. `START_NOW=1 ./scripts/02-
  install-systemd-units.sh` (or a manual `systemctl start drone-mc.target`)
  will fail partway through until these are built — see
  `INTEGRATION-AND-FLIGHT-TEST-PLAN.md`'s `FT-004` for the dedicated
  full-stack bring-up test (`scripts/17-smoke-test-full-stack.sh`), which
  computes the available/missing image set itself rather than assuming it.
- **`14b-smoke-test-health.sh`'s "GCS receives health telemetry in QGC"
  criterion (HEALTH-004) is not smoke-testable in any environment** — it
  needs a real QGroundControl instance on the far end of a real MAVLink
  link, the same class of gap `PROJECT-PLAN-5-MONTH.md` already accepts
  for GCS-001's own "QGroundControl shows drone HEARTBEAT" criterion.
  `/mc/health` already reaches the GCS side today as the custom
  `MC_HEALTH` (50103) MAVLink message (`gcs_mavlink_node`'s existing
  `mc_status_timer_` mirror, GCS-003) — rendering it inside stock
  QGroundControl would need a QGC-side dialect plugin, out of scope for
  this MC-side repo (same class of "another team's app" boundary as the
  Mission Orchestrator App, GCS-004).
- **`video_downlink` has no deployment story yet at all** — no unit file,
  no Dockerfile, no build script for `perception_front_node` or
  `video_mux_node`.

## Operating a running deployment

```bash
# Bring everything up / down
sudo systemctl start drone-mc.target
sudo systemctl stop drone-mc.target

# Status of the whole target and its dependency graph
systemctl status drone-mc.target
systemctl list-dependencies drone-mc.target --no-pager

# One service
sudo systemctl restart drone-mc-fcc-bridge.service
journalctl -u drone-mc-fcc-bridge -f

# Re-tag the image set for a release (per-drone, in /etc/drone-mc/environment)
MC_VERSION=1.0.0   # then restart the target
```

`scripts/02b-smoke-test-units.sh` verifies units/target/slices are
installed, valid, and enabled without starting anything real;
`LIVE_CHECK=1 ./scripts/02b-smoke-test-units.sh` additionally does a real
boot test. See `docs/TESTING.md` for the full TP-SM smoke-test picture.

## Cutting a release (TEST-005)

**Status: manual — the automated release workflow was removed.**
`TEST-005` originally built `.github/workflows/release.yml`, which
triggered on a pushed `v*` tag (e.g. `v1.0.0`) and built every
`docker/*/Dockerfile` that existed at that point, pushing each to
`ghcr.io/drone-mc/<service>:<tag>`. That workflow (along with `ci.yml` and
`nightly-sitl.yml`) was deleted on 2026-09-05 and has not been reinstated.

Cutting a release today means doing what `release.yml` used to do by
hand: run the numbered `scripts/NN-build-*-image.sh` chain (see
`CLAUDE.md`'s Build System section) for every service, tag each image
`ghcr.io/drone-mc/<service>:<tag>` — the same
`ghcr.io/drone-mc/<service>:${MC_VERSION}` naming the systemd units above
already reference — and push the tags yourself. Set `MC_VERSION` on each
drone to the tag pushed and restart `drone-mc.target` to roll it out. See
`docs/CONTRIBUTING.md`'s [CI/CD (TEST-005)](CONTRIBUTING.md#cicd-test-005)
section for the manual equivalents of the other two removed workflows
(`ci.yml`, `nightly-sitl.yml`).

## Where to go next

- Full build chain that produces these images, in order: `docs/GETTING-STARTED.md`.
- Design rationale for the slice/container split and dependency ordering: `SDD-MC-001.md` §12.
- Which ROS 2 topics each of these services actually publishes/subscribes to once running: `docs/API-REFERENCE.md`.
- MAVLink channel wire formats (A/B/C) these containers route between: `ICD-MC-001.md`.
