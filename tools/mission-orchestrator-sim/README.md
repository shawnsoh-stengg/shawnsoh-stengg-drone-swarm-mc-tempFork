# mission-orchestrator-sim (TEST-004)

A standalone Python 3 CLI that simulates the Mission Orchestrator app's
mission-plan CRUD actions (create/list/read/update/delete/activate) against
`mission_orchestrator_node`'s Channel C interface
(`MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md` §4-§7, `ICD-MC-001.md` §12,
IFD-008) — MAVFTP for everything filesystem-shaped, plus the two custom
`MC_MISSION_PLAN_ACTIVATE`/`MC_MISSION_PLAN_ACK` messages for `activate`.

It exists so the MC-side implementation can be exercised interactively and in
CI without waiting on the real Mission Orchestrator app, which is a different
team's deliverable (GCS-004's own ticket description) built for and run at
the GCS, not on the Jetson MC. This tool is **not** that app — it is a test
double covering the same wire protocol, and it is written to double as a
worked reference for whoever *does* build that app: every subcommand maps
directly onto one row of the interface spec's §6 workflows.

Runs anywhere with Python 3 and `pymavlink` — no ROS 2, no colcon, no
`ws/src/` build step. It reads `ws/src/gcs_comms/mc_mavlink_defs.xml`
directly (see "Running outside this checkout" below if that file isn't
available at its usual path) to generate the two custom messages via
`mavgen`; MAVFTP itself is already part of `pymavlink`'s bundled `common.xml`
dialect.

## Install

```bash
cd tools/mission-orchestrator-sim
python3 -m venv .venv && source .venv/bin/activate   # optional but recommended
pip install -r requirements.txt
```

## Usage

```
./mission-orchestrator-sim --mc-host <MC_IP> [options] <command> [args]
```

| Option | Default | Meaning |
|---|---|---|
| `--mc-host` | *(required)* | The drone's Channel C address — the Jetson's LAN IP (`192.168.100.11`-class address), or `127.0.0.1` for local/loopback testing |
| `--drone-id` | `1` | Target drone's `system_id` (the MC's `DRONE_ID`) |
| `--recv-port` | `14571` | Local UDP port this tool listens on (`ORCHESTRATOR_MAV_PORT_RECV`) |
| `--send-port` | `14572` | MC's Channel C UDP port this tool sends to (`ORCHESTRATOR_MAV_PORT_SEND`) |
| `--timeout` | `5.0` | Per-request reply timeout, seconds |
| `--regen-dialect` | off | Force-regenerate the cached `mc_mavlink_defs` dialect module |

Commands:

```
create   <name> <plan.json>   # CreateFile — Nak(FileExists) if <name>.json already exists
list                          # ListDirectory — every stored plan, plus which one is active
read     <name>                # OpenFileRO + BurstReadFile — Nak(FileNotFound) if missing
update   <name> <plan.json>   # OpenFileWO — Nak(FileNotFound) if it doesn't already exist
delete   <name>                # RemoveFile — Nak(FileProtected) if <name> is the active plan
activate <name>                # MC_MISSION_PLAN_ACTIVATE -> MC_MISSION_PLAN_ACK
```

`<name>` is the plan's identity with or without the `.json` suffix (both
`recon_alpha` and `recon_alpha.json` work) — per the interface spec, the
filename *is* the plan's identity, there is no separate ID.

### Example session (against a real MC on the LAN)

```bash
$ ./mission-orchestrator-sim --mc-host 192.168.100.11 create recon_alpha plans/recon_alpha.json
OK: created recon_alpha.json (412 bytes)

$ ./mission-orchestrator-sim --mc-host 192.168.100.11 list
recon_alpha.json	412 bytes
active plan: (none)

$ ./mission-orchestrator-sim --mc-host 192.168.100.11 activate recon_alpha
MC_MISSION_PLAN_ACK: client_request_id=1 filename=recon_alpha.json result=ACCEPTED

$ ./mission-orchestrator-sim --mc-host 192.168.100.11 delete recon_alpha
ERROR: Nak(FileProtected) for opcode 8
```

That last line is the delete-protection case TEST-004's AC calls out
explicitly: MAVFTP's own `RemoveFile` opcode replies `Nak(FileProtected)`
when `<name>.json` is the currently-active plan (interface spec §6.5) —
`activate` must succeed *before* `delete` can be shown failing this way, as
in the example above.

`activate` exits `0` on `result=ACCEPTED` and `1` on any rejection
(`REJECTED_INVALID_JSON`/`REJECTED_NOT_FOUND`/`REJECTED_PHASE`), so it's
usable directly in a shell `&&` chain or a CI assertion.

### Local / loopback testing (no real drone)

Run `mission_orchestrator_node` and a `mavlink-routerd` process configured
with a loopback-rewritten copy of `config/mavlink-router-channel-c.conf`
(same technique `scripts/09b-smoke-test-routing.sh`'s `LIVE_CHECK=1` path
uses for Channels A/B — rewrite `Address = 192.168.100.10` to `127.0.0.1`,
ports unchanged), then point this tool at `--mc-host 127.0.0.1`. See that
script for the exact rewrite pattern.

### Testing against a real two-host deployment

`config/mavlink-router-channel-c.conf` is already written for the repo's
standard addresses (GCS laptop `192.168.100.10`, Jetson MC `192.168.100.11` —
see `CLAUDE.md`'s Network/IP Reference) — **no rewrite is needed** for this
case, unlike the loopback setup above.

As of `GCS-006`, both `mission_orchestrator_node` and Channel C's
`mavlink-routerd` process are started by `drone-mc.target` like every other
MC service — no manual `ros2 run`/`mavlink-routerd` step on the Jetson. On
the Jetson MC (`192.168.100.11`), once `scripts/02-install-systemd-units.sh`
has been run (it provisions `/etc/drone-mc/mavlink-router-channel-c.conf`
and `/var/drone-mc/mission_plans` for you) and `drone-mc-mission-
orchestrator`/`drone-mc-mavlink-router` images are built
(`scripts/15-build-mission-orchestrator-image.sh`,
`scripts/09-build-mavlink-router-image.sh`):

```bash
sudo systemctl start drone-mc.target
# or, to bring up just these two services if the rest of the target isn't ready yet:
sudo systemctl start drone-mc-mavlink-router.service drone-mc-mission-orchestrator.service

systemctl status drone-mc-mission-orchestrator.service   # confirm it's running
```

Confirm the node's `DRONE_ID` env var (`/etc/drone-mc/environment` on the
Jetson) matches whatever `--drone-id` you pass below (both default to `1`
if you set neither).

On the GCS laptop (`192.168.100.10`):

```bash
cd tools/mission-orchestrator-sim
pip install -r requirements.txt

./mission-orchestrator-sim --mc-host 192.168.100.11 create recon_alpha plans/recon_alpha.json
./mission-orchestrator-sim --mc-host 192.168.100.11 list
./mission-orchestrator-sim --mc-host 192.168.100.11 activate recon_alpha
./mission-orchestrator-sim --mc-host 192.168.100.11 delete recon_alpha   # expect Nak(FileProtected)
```

`--mc-host` is the only flag that needs to change from the loopback case —
`--recv-port`/`--send-port` already default to 14571/14572 to match the
conf file.

Things worth checking before the first run:

- UDP 14571/14572 must be open between the two hosts — same subnet per the
  IP reference above, so usually fine, but worth a quick `nc -u` sanity
  check if the first command times out.
- `docker/mavlink-router/entrypoint.sh` launches Channel C alongside
  Channels A/B as a third independent `mavlink-routerd` process — if
  `drone-mc-mavlink-router.service` predates `GCS-006` on this Jetson,
  rebuild the image (`scripts/09-build-mavlink-router-image.sh`) and
  restart the service to pick up the change.
- Cold start: `transport.py`'s `ChannelCClient` docstring covers why
  requests address `target_system=0` (broadcast) rather than the drone's
  real sysid — mavlink-router hasn't "learned" `mission_orchestrator_node`'s
  identity until it sends its first reply. This is handled automatically;
  if the very first `create` still times out, retry it once.

### Plan JSON format

One JSON file per plan — see the interface spec §6.1 for the full field
reference. Minimal example:

```json
{
  "name": "recon_alpha",
  "ingress_waypoints": [
    {"lat": 1.3521000, "lon": 103.8198000, "alt_m": 50.0, "speed_mps": 5.0}
  ],
  "search_polygon": [
    {"lat": 1.3521000, "lon": 103.8198000}
  ],
  "swarm_manifest": [
    {"drone_id": 1, "ip": "192.168.100.11"}
  ],
  "arm_authorised": true
}
```

## Notes for whoever builds the real Mission Orchestrator app

- This tool's `mission_orchestrator_sim/transport.py` hand-rolls a
  synchronous, one-request-one-reply MAVFTP client on top of
  `pymavlink.mavftp_op.FTP_OP` rather than using `pymavlink.mavftp.MAVFTP`
  directly — that class has no dispatch case for `OpenFileWO`, so it can't
  drive `update` at all, and its `cmd_put`/`cmd_get` are built for a
  pipelined multi-block-in-flight transfer sized for large files over lossy
  links. Every plan here is a few KB of JSON; a real app can use either
  approach, but note the `OpenFileWO` gap if reaching for the stock
  `MAVFTP` class.
- `read`/`_active` both go through `BurstReadFile` (opcode 15), matching
  `mission_orchestrator_node`'s own `handle_burst_read()` — plain
  `ReadFile` (opcode 5) is also implemented server-side, but burst is the
  documented/exercised path.
- MAVFTP paths must include the leading `/` (`/<name>.json`,
  `/_active`) — the server does not add it.
- `mission_orchestrator_sim/dialect.py` generates the custom
  `MC_MISSION_PLAN_ACTIVATE`/`ACK` messages via `mavgen` against
  `ws/src/gcs_comms/mc_mavlink_defs.xml` and installs the result as
  `pymavlink`'s active dialect, so `FILE_TRANSFER_PROTOCOL` and the two
  custom messages all decode through the same `mav.parse_char()` path — no
  hand-rolled raw-frame decoding is needed on the client side (unlike
  `mission_orchestrator_node.cpp`, which can't do this on the C++ side; see
  that file's own header comment for why).
- Wire identity: this tool's `system_id`/`component_id` are `255`/`190`
  (`MAV_COMP_ID_MISSIONPLANNER`), matching the interface spec §3.3 — a real
  app should use the same identity, since `mission_orchestrator_node`
  echoes the request's `sysid`/`compid` back as the reply's MAVLink target.

## Running outside this checkout

If `ws/src/gcs_comms/mc_mavlink_defs.xml` isn't available at its usual
monorepo-relative path (e.g. this directory was copied out on its own), set
`MC_MAVLINK_DEFS_XML=/path/to/mc_mavlink_defs.xml`. The generated dialect
module is cached under `tools/mission-orchestrator-sim/_generated/` (keyed
by a hash of the XML's contents); override the cache location with
`MISSION_ORCHESTRATOR_SIM_CACHE` if needed.
