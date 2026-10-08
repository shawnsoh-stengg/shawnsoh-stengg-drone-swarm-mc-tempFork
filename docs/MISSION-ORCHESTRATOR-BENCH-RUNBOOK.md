# Mission Orchestrator (Channel C) Bench Runbook — Step by Step

This is the operator's walkthrough for testing the mission-plan library
on a real bench: a GCS laptop running `tools/mission-orchestrator-sim`
against the real `mission_orchestrator_node` on a Jetson MC, over the
real Channel C `mavlink-routerd`. It tells you what to type, in what
order, and what you should see.

It covers `FT-001`'s Channel C acceptance criterion
(`INTEGRATION-AND-FLIGHT-TEST-PLAN.md`): the full CRUD cycle, including
the `Nak(FileProtected)` case, against the **deployed** containers. It
also follows an activated plan into `mission_manager_node`, which is
where it actually takes effect.

Background, if you need it:
`MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md` (the protocol and plan
JSON format, §6), `docs/GCS-MAVLINK-API-REFERENCE.md` (Channel C
reference), and `tools/mission-orchestrator-sim/README.md` (the tool).

---

## 0. Before you start

**What the test exercises**

```
GCS laptop (GCS_IP)                    Jetson MC (MC_ETH_IP)
mission-orchestrator-sim  ──UDP 14572──►  mavlink-routerd (Channel C)  ──► mission_orchestrator_node
                          ◄─UDP 14571──                                      │  plans in /var/drone-mc/mission_plans
                                                                             ▼  activate
                                                              /mc/gcs/mission_upload ──► mission_manager_node
```

- Create, list, read, update and delete are MAVFTP file operations on
  the plan library.
- `activate` sends the custom `MC_MISSION_PLAN_ACTIVATE` message. On
  success, `mission_orchestrator_node` publishes the plan on
  `/mc/gcs/mission_upload`, and `mission_manager_node` stores it.
- **Activating a plan does not send anything to the FCC.** Waypoints
  reach PX4 later, during the mission phases. No Pixhawk is needed for
  this test, and nothing can fly or spin.

**Hardware**

- Jetson MC and the GCS laptop on the same 192.168.100.x switch.
- The laptop's IP **must equal `GCS_IP`** in the Jetson's
  `/etc/drone-mc/environment` (default `192.168.100.10`). Channel C only
  sends replies to that address; from any other address, every command
  times out.
- A native Linux laptop is easiest. Under WSL2 the default NAT
  networking doesn't deliver inbound UDP to port 14571.

**Time:** about 45 minutes.

**Software on the laptop:** a checkout of this repo (the tool reads
`ws/src/gcs_comms/mc_mavlink_defs.xml` from it), and Python 3.

---

## 1. Jetson setup

All commands in this section run on the Jetson host.

### 1.1 Settings

```bash
set -a; source /etc/drone-mc/environment; set +a
echo "DRONE_ID=$DRONE_ID MC_ETH_IP=$MC_ETH_IP GCS_IP=$GCS_IP MC_VERSION=$MC_VERSION"
ip -4 -br addr show eth0                        # must show $MC_ETH_IP
```

Write down `DRONE_ID` and `MC_ETH_IP`; you need both on the laptop.

### 1.2 Channel C router config

```bash
grep -A3 "ORCHESTRATOR_CHANNEL_C" /etc/drone-mc/mavlink-router-channel-c.conf
```

`Address =` must be the laptop's IP (`$GCS_IP`). If it's blank or still
a `192.168.10.x` address, the installed file is stale. Re-run
`sudo ./scripts/02-install-systemd-units.sh` to re-render it, then
restart the router (§1.4).

### 1.3 Images (only if missing or the code changed)

```bash
sudo docker images | grep -E "mission-orchestrator|mavlink-router|mission-exec"
```

All three should be listed with tag `$MC_VERSION`. If one is missing or
out of date:

```bash
sudo ./scripts/05-build-workspace.sh
sudo ./scripts/09-build-mavlink-router-image.sh
sudo ./scripts/15-build-mission-orchestrator-image.sh
sudo ./scripts/12-build-mission-exec-image.sh
```

### 1.4 Start the services

```bash
sudo systemctl start drone-mc-mavlink-router drone-mc-mission-orchestrator drone-mc-mission-exec
systemctl --no-pager status drone-mc-mavlink-router drone-mc-mission-orchestrator drone-mc-mission-exec | grep -E "●|Active:"
```

All three should be `active (running)`. Starting the whole
`drone-mc.target` is also fine.

`mission-exec` matters because `mission_manager_node` publishes the
mission phase, and `activate` is only allowed in `PREFLIGHT`. On a
bench that never arms, the phase stays `PREFLIGHT`.

### 1.5 Check the ports

```bash
sudo ss -ulpn | grep -E ":14572|:14580"
```

You should see `mavlink-routerd` on `14572` (from the laptop) and
`mission_orchestrator_node` on `14580` (loopback).

### 1.6 Open a log window

Leave this running in its own terminal for the whole test:

```bash
journalctl -fu drone-mc-mission-orchestrator
```

### 1.7 A shell for ROS 2 checks

Later steps check ROS 2 topics. Open a shell inside the running
mission-orchestrator container. It already has ROS 2 and the right
network settings:

```bash
sudo docker exec -it mc-mission-orchestrator bash
set +u; source /opt/ros/humble/install/setup.bash; source /opt/drone-mc/install/setup.bash
ros2 node list          # must include mission_orchestrator_node and mission_manager_node
ros2 topic echo /mc/mission_phase --once     # phase should be PREFLIGHT
```

Keep this shell open; it's called **J-ROS** below.

---

## 2. Laptop setup

### 2.1 Network

```bash
ip -4 -br addr                  # one interface must show GCS_IP, e.g. 192.168.100.10
ping -c 3 <MC_ETH_IP>
```

If a firewall is on, allow the reply port:

```bash
sudo ufw allow 14571/udp        # only if ufw is active
```

Make sure nothing else on the laptop is using UDP 14571 (for example a
second copy of the tool):

```bash
ss -ulpn | grep 14571           # should print nothing
```

### 2.2 Install the tool

From the repo root on the laptop:

```bash
cd tools/mission-orchestrator-sim
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

### 2.3 A shortcut

Every command needs `--mc-host` and `--drone-id`. Set them once per
terminal:

```bash
export MC=<MC_ETH_IP>           # from §1.1, e.g. 192.168.100.11
export DID=<DRONE_ID>           # from §1.1
mo() { ./mission-orchestrator-sim --mc-host "$MC" --drone-id "$DID" "$@"; }
```

From here on, `mo list` means
`./mission-orchestrator-sim --mc-host $MC --drone-id $DID list`.

### 2.4 Test plan files

Create a scratch folder with three plan files:

```bash
mkdir -p ~/mo-bench && cd ~/mo-bench
```

`bench_alpha.json`, a complete plan. Every list must be non-empty,
otherwise `mission_manager_node`'s pre-arm check
`mission_upload_complete` fails later. Set `drone_id` and `ip` in
`formation_offsets`/`swarm_manifest` to this drone's `DRONE_ID` and
`MC_ETH_IP`:

```bash
cat > bench_alpha.json <<'EOF'
{
  "name": "bench_alpha",
  "ingress_waypoints": [
    {"lat": 1.3000000, "lon": 103.8000000, "alt_m": 30.0, "speed_mps": 2.0}
  ],
  "formation_offsets": [
    {"drone_id": 1, "north_m": 0.0, "east_m": 0.0, "down_m": 0.0}
  ],
  "search_polygon": [
    {"lat": 1.3000000, "lon": 103.8000000},
    {"lat": 1.3001000, "lon": 103.8000000},
    {"lat": 1.3001000, "lon": 103.8001000}
  ],
  "swarm_manifest": [
    {"drone_id": 1, "ip": "192.168.100.11"}
  ],
  "arm_authorised": false,
  "low_battery_threshold": 0.20,
  "offboard_phase_mask": 0
}
EOF
```

`arm_authorised` is `false` on purpose: the bench never needs to arm.

`bench_alpha_v2.json`, the same plan with one visible change, for the
update test:

```bash
sed 's/"speed_mps": 2.0/"speed_mps": 3.0/' bench_alpha.json > bench_alpha_v2.json
```

`bench_bad.json`, invalid JSON, for the rejection test:

```bash
echo '{"name": "bench_bad", "ingress_waypoints": [' > bench_bad.json
```

Go back to the tool folder before running commands, or use the full
paths to these files.

---

## 3. Test procedure

Run each step from the laptop. Fill in the results table in §5 as you
go, from what you actually see.

### 3.1 First contact — `list`

```bash
mo list
```

Expected: either `(no stored plans)` or a list of plans, then
`active plan: …`. Any answer at all proves the full path: laptop → router
→ node → router → laptop.

If it times out, retry once. The router learns the node's address from
its first reply (tool README, "Cold start"). Two timeouts in a row → §6.

If the library already has plans named `bench_*` from an earlier run,
delete them first (`mo delete bench_alpha`). If one is the active plan,
see §3.7 step 3.

### 3.2 Create

```bash
mo create bench_alpha ~/mo-bench/bench_alpha.json
```

Expected: `OK: created bench_alpha.json (<n> bytes)`.

Check on the Jetson host that the file landed:

```bash
sudo ls -l /var/drone-mc/mission_plans/
```

**Duplicate create** must be refused:

```bash
mo create bench_alpha ~/mo-bench/bench_alpha.json
```

Expected: `ERROR: Nak(FileExists) …`.

### 3.3 List and read

```bash
mo list
mo read bench_alpha -o ~/mo-bench/readback.json
diff ~/mo-bench/bench_alpha.json ~/mo-bench/readback.json && echo SAME
```

Expected: `bench_alpha.json` in the list, and `SAME`: the readback is
byte-identical.

Reading a plan that doesn't exist must fail:

```bash
mo read no_such_plan
```

Expected: `ERROR: Nak(FileNotFound) …`.

### 3.4 Update

```bash
mo update bench_alpha ~/mo-bench/bench_alpha_v2.json
mo read bench_alpha | grep speed_mps          # now 3.0
```

Updating a plan that doesn't exist must fail:

```bash
mo update no_such_plan ~/mo-bench/bench_alpha_v2.json
```

Expected: `ERROR: Nak(FileNotFound) …`.

### 3.5 Activate

In **J-ROS**, start watching the topic first:

```bash
ros2 topic echo /mc/gcs/mission_upload
```

On the laptop:

```bash
mo activate bench_alpha
```

Expected:

- Laptop: `MC_MISSION_PLAN_ACK: … filename=bench_alpha.json result=ACCEPTED`.
- J-ROS: one `MissionUpload` message with the plan's contents
  (`speed_mps: 3.0` from the update) and an `upload_id`. Write the
  `upload_id` down.
- Log window: `activated mission plan bench_alpha.json (upload_id=…)`.

Then:

```bash
mo list                              # active plan: bench_alpha.json
mo read _active                      # prints bench_alpha.json
```

**`mission_manager_node` stored it.** On the Jetson host:

```bash
sudo docker exec mc-mission-exec cat /var/drone-mc/mission_upload.yaml | head -20
```

Expected: the same `upload_id` and plan contents.

### 3.6 Activate rejections

Each one must print a non-`ACCEPTED` result (the tool exits with code 1),
and the active plan must stay `bench_alpha.json`:

```bash
mo activate no_such_plan             # result=REJECTED_NOT_FOUND

mo create bench_bad ~/mo-bench/bench_bad.json   # accepted: MAVFTP can't refuse content
mo activate bench_bad                # result=REJECTED_INVALID_JSON
mo list                              # active plan: still bench_alpha.json
```

The node also sends a MAVLink `STATUSTEXT` warning
(`… has invalid JSON`) back to the laptop when `bench_bad` is created.
The tool doesn't display `STATUSTEXT`, and the node doesn't log it, so
there's nothing to look for. The file stays stored; only `activate`
refuses it.

`REJECTED_PHASE` (activate outside `PREFLIGHT`) can't be reached on a
bench that never arms. Write NOT RUN for it; the node tests cover it.

### 3.7 Delete

```bash
mo delete bench_alpha                # ERROR: Nak(FileProtected) — it's the active plan
mo delete bench_bad                  # OK: deleted bench_bad.json
mo list                              # bench_bad gone, bench_alpha still active
```

To delete `bench_alpha` at the end, first make another plan active:

```bash
mo create bench_beta ~/mo-bench/bench_alpha.json
mo activate bench_beta
mo delete bench_alpha                # now OK
```

### 3.8 Restart behaviour

**Plan library survives a mission-orchestrator restart.** On the Jetson
host:

```bash
sudo systemctl restart drone-mc-mission-orchestrator
```

Wait about 10 s, then on the laptop:

```bash
mo list
```

Expected: `bench_beta.json` is still listed. The plans live on the host
in `/var/drone-mc/mission_plans`. Record what `active plan:` shows: the
node keeps the active-plan name in memory only, so expect `(none)` after
a restart. Retry once if the first `list` times out.

**`mission_manager_node` reloads its stored plan after a restart.** On
the Jetson host:

```bash
sudo systemctl restart drone-mc-mission-exec
sleep 10
journalctl -u drone-mc-mission-exec -n 50 --no-pager | grep -i "reloaded persisted mission upload"
sudo docker exec mc-mission-exec cat /var/drone-mc/mission_upload.yaml | head -5
```

The design (SDD §4.5) expects a `reloaded persisted mission upload …`
line and the same `upload_id` as §3.5. **Known risk:** the
`drone-mc-mission-exec` unit doesn't bind-mount `/var/drone-mc` from the
host, and the container is started with `--rm`. So the file may be lost
on restart, and there would be no reload line. Record exactly what you
see; if it's missing, that is a finding, not a setup error.

### 3.9 Clean up

```bash
mo list
```

Plans with a `bench_` prefix can stay or be deleted. The active one
can only be deleted after activating another (§3.7). Leave the library
the way the next user expects it.

---

## 4. Evidence to keep

- The laptop terminal output for every step (copy it, or run the
  session under `script ~/mo-bench/session.log`).
- `journalctl -u drone-mc-mission-orchestrator --since "<start time>"`
  saved to a file.
- The J-ROS `MissionUpload` echo from §3.5.
- `sudo ls -l /var/drone-mc/mission_plans/` before and after.

Save the results table below, filled in, as
`test-results/FT-001-ac3-channel-c-<date>.md`, with the evidence next
to it or linked.

---

## 5. Results table

Record the unit (hostname/login and eth0 MAC), the repo and commit on
the Jetson (`git remote -v`, `git log -1 --oneline`), `DRONE_ID`,
`MC_ETH_IP`, `GCS_IP`, and the date and operator at the top.

| Step | Check | Expected | Observed | Result |
|---|---|---|---|---|
| 3.1 | `list` answers | any reply, ≤ 1 retry | | |
| 3.2 | `create` | `OK: created` + file in `/var/drone-mc/mission_plans` | | |
| 3.2 | duplicate `create` | `Nak(FileExists)` | | |
| 3.3 | `read` round-trip | byte-identical | | |
| 3.3 | `read` missing plan | `Nak(FileNotFound)` | | |
| 3.4 | `update` | new content read back | | |
| 3.4 | `update` missing plan | `Nak(FileNotFound)` | | |
| 3.5 | `activate` | `result=ACCEPTED` | | |
| 3.5 | `/mc/gcs/mission_upload` | one message, plan contents, `upload_id` | | |
| 3.5 | `list` / `read _active` | shows `bench_alpha.json` | | |
| 3.5 | `mission_manager_node` stored it | `mission_upload.yaml` has the same `upload_id` | | |
| 3.6 | activate missing plan | `REJECTED_NOT_FOUND`, active unchanged | | |
| 3.6 | activate invalid JSON | `REJECTED_INVALID_JSON`, active unchanged | | |
| 3.6 | activate outside `PREFLIGHT` | `REJECTED_PHASE` | NOT RUN (bench never arms) | — |
| 3.7 | delete active plan | `Nak(FileProtected)` | | |
| 3.7 | delete inactive plan | `OK: deleted` | | |
| 3.8 | library after orchestrator restart | plans still listed; record `active plan:` | | |
| 3.8 | `mission_manager_node` reload after restart | reload log line + same `upload_id` (known risk, see §3.8) | | |

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Every command times out | Laptop isn't at `GCS_IP`, or the router config points elsewhere | §2.1 `ip addr`; §1.2 `Address =` must be the laptop's IP |
| Every command times out, addresses correct | Laptop firewall, or WSL2 NAT, drops inbound UDP 14571 | `sudo ufw allow 14571/udp`; use native Linux |
| First command times out, retry works | Cold start: the router hadn't learned the node's address yet | Normal, documented. Only a problem if it keeps happening |
| `OSError: Address already in use` | Something else on the laptop holds UDP 14571 | `ss -ulpn \| grep 14571`; close the other copy |
| `activate` → `REJECTED_PHASE` on the bench | `mission_manager_node` reports a phase other than `PREFLIGHT` | J-ROS: `ros2 topic echo /mc/mission_phase --once`. Restart `drone-mc-mission-exec` to return to `PREFLIGHT` |
| `ACCEPTED` but no message in J-ROS | J-ROS shell started without sourcing, or started echo after activating | Source both setup files; start the echo before `activate` |
| `ros2 node list` lacks `mission_manager_node` | mission-exec not running | `systemctl status drone-mc-mission-exec`; `journalctl -u drone-mc-mission-exec -n 50` |
| Tool error mentioning `mc_mavlink_defs.xml` | Running the tool outside the repo checkout | Run from the repo, or set `MC_MAVLINK_DEFS_XML=/path/to/mc_mavlink_defs.xml` |
| `ss` shows nothing on 14572 | Channel C's `mavlink-routerd` isn't running (old router image) | Rebuild with `09-build-mavlink-router-image.sh`, restart `drone-mc-mavlink-router` |
