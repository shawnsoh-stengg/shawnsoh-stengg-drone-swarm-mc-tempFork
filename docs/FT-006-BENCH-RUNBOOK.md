# FT-006 Real-FCC Bench Runbook — Step by Step

This is the operator's walkthrough for running FT-006, the MC+IGS bench
integration test: a real Jetson MC running `gnss_sim_node` against a real
Pixhawk 6X, props off. It tells you what to type, in what order, and what
you should see.

It does **not** replace the results log. Record every observation in
`test-results/FT-006-mc-igs-bench-integration.md`. That file holds the
acceptance criteria, the expected-value tables, and the results of earlier
runs. Section numbers like "log §3.2" below point into it.

Background, if you need it: the ticket is in
`INTEGRATION-AND-FLIGHT-TEST-PLAN.md` (FT-006), the wire interfaces are
`ICD-MC-001.md` §3.3a (`:14556` flow/baro relay) and §3.3b (`:14557`
`HIL_GPS`), and the FCC setup is `HID-MC-001.md` §10.6/§10.7.

> **Not to be confused with HITL-Sim.** `HITL-SIMULATION-TEST-PLAN.md`
> is a different rig with PX4 SITL on a laptop and no real FCC. This
> runbook is for the real-FCC bench only.

---

## 0. Before you start

**Hardware**

- Jetson MC (the DUT) and Pixhawk 6X on the same 192.168.100.x switch,
  and the ttyTHS1 serial link wired if `FCC_TRANSPORT=serial`.
- **Props removed, no airframe.** FCC on bench power.
- Holybro H-Flow on Pixhawk CAN1, pointed at a textured surface 0.3–1 m
  away. If you don't have one, the helper's `flow` command mocks it.
  FT-006 allows that substitution, but it must be recorded in log §1.
- Optional for log §3.5: a real GPS receiver on the Pixhawk's GPS port
  with sky view, or a GNSS simulator. Without one, §3.5 is NOT RUN.
- A laptop with QGroundControl, for the PX4 MAVLink Console, parameter
  edits and the `.ulg` log download.

**Time:** about 2 hours for a first run, excluding the one-time FCC setup
in §1. §3.3 alone takes over 3 minutes of continuous recording.

**Order matters.** Log §3.1 (the negative path) must run while
`MAV_USEHILGPS` is still `0`. Don't set it to `1` during setup.

**Variables used below.** Every per-drone value comes from the Jetson's
`/etc/drone-mc/environment`. Defaults are `DRONE_ID=1`,
`MC_ETH_IP=192.168.100.11`, `FCC_ETH_IP=192.168.100.31`, but bench units
override them (for example orin-nx is `DRONE_ID=2`, MC `.39`, FCC `.19`).
Always read the real file; don't assume the defaults.

---

## 1. FCC setup (one-time per FCC, then verify each session)

Run PX4 shell commands from QGC → **Analyze Tools → MAVLink Console**.

### 1.1 Ethernet address

```
netman show
```

`IPADDR` must equal the Jetson's `FCC_ETH_IP`. If it doesn't, edit
`/fs/microsd/net.cfg` (`IPADDR=…`, `NETMASK=255.255.255.0`,
`BOOTPROTO=static`) and `reboot`. See `HID-MC-001.md` §4.2.

### 1.2 uXRCE-DDS link

For `FCC_TRANSPORT=serial`:

```
param show UXRCE_DDS_CFG       # 102 (TELEM2)
param show UXRCE_DDS_BAUD      # 921600
param show UXRCE_DDS_PTCFG     # 2 — mandatory, otherwise no /fmu/* topics reach the MC
```

For `FCC_TRANSPORT=ethernet`, see `HID-MC-001.md` §4 (`UXRCE_DDS_AG_IP`
must encode the MC's **current** IP).

### 1.3 H-Flow (skip if mocking)

Follow `HID-MC-001.md` §10.6 steps **A–C**: DroneCAN on, `uavcan status`
shows the H-Flow node, and `listener sensor_optical_flow` /
`listener distance_sensor` update with sane values.

### 1.4 `extras.txt` — the two MAVLink instances FT-006 needs

```
cat /fs/microsd/etc/extras.txt
```

It must contain both blocks below (`HID-MC-001.md` §10.6 D/E). Replace
`<MC_ETH_IP>` with this drone's **current** MC address. On 2026-10-02 a
stale `-t` address meant no baro input ever reached the MC, and the
2026-10-06 subnet move changed every address, so check it.

```
mavlink start -u 14556 -o 14556 -t <MC_ETH_IP> -m custom -r 10000
mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 20
mavlink stream -u 14556 -s DISTANCE_SENSOR -r 10
mavlink stream -u 14556 -s ALTITUDE -r 10
mavlink start -u 14557 -m custom -r 1000
```

The `ALTITUDE` stream is required even with mocked flow; the helper only
mocks flow, never baro.

### 1.5 Parameters

```
param set MAV_SYS_ID <DRONE_ID>   # must equal the MC's DRONE_ID
param set SENS_BAR_AUTOCAL 0      # default 1 — gives a +40 m height jump otherwise
param show EKF2_GPS_CTRL          # must be 7 — 0 discards HIL_GPS entirely
param show MAV_USEHILGPS          # LEAVE AT 0 for now (log §3.1)
param show EKF2_REQ_EPH           # record (default 3.0)
param show EKF2_REQ_NSATS         # record (default 6)
param show SENS_GPS_MASK          # record (default 7)
param save
reboot
```

### 1.6 Verify after reboot

```
mavlink status          # an instance on UDP 14556 and one on UDP 14557
mavlink status streams  # 14556: OPTICAL_FLOW_RAD, DISTANCE_SENSOR, ALTITUDE at the set rates
ver all                 # record the PX4 version in log §1
```

On the `14556` instance, `tx` should be counting up. `tx 0` with
`txerr > 0` means the `-t` address is wrong.

---

## 2. MC setup (each session)

All commands in this section run on the Jetson host, from the repo
checkout the Jetson actually uses.

### 2.1 Identify the unit and the code

Both bench Jetsons report hostname `ubuntu`, so identify them by login
and MAC:

```bash
whoami; ip -br link show eth0          # MAC identifies the unit; record it in log §1
git remote -v; git log -1 --oneline    # record both in log §1
```

The Jetsons usually run the org repo (`AI-DA-STC/drone-swarm-mc`) with
hand-ported commits. Confirm the build has everything FT-006 depends on:

```bash
ls tools/ft006_bench/ft006_bench.py                           # the helper exists
grep -n baro_vz_filter_tau_s ws/src/indoor_nav/src/gnss_sim_node.cpp   # vd LPF (c8f61f4 / NEAR-723)
```

Also confirm the three 2026-09-26 fixes listed in log §1 are present
(baro vertical input, XRCE GNSS-field aggregation,
publish-each-flow-frame-once).

### 2.2 Environment and link

```bash
set -a; source /etc/drone-mc/environment; set +a
echo "DRONE_ID=$DRONE_ID MC_ETH_IP=$MC_ETH_IP FCC_ETH_IP=$FCC_ETH_IP FCC_TRANSPORT=$FCC_TRANSPORT MC_VERSION=$MC_VERSION"
ip -4 -br addr show eth0     # must show $MC_ETH_IP
ping -c 3 $FCC_ETH_IP
```

Check: `DRONE_ID` equals the FCC's `MAV_SYS_ID` (§1.5), and `MC_ETH_IP`
equals the `-t` address in the FCC's `extras.txt` (§1.4).

### 2.3 Build (only if the code changed since the last build)

```bash
sudo ./scripts/05-build-workspace.sh          # note the px4_msgs branch it prints: must be release/1.17
sudo ./scripts/08-build-fcc-bridge-image.sh
sudo ./scripts/16-build-indoor-nav-image.sh
```

Record the `px4_msgs` branch in log §1.

### 2.4 Start the services

Start fcc-bridge first and check it on its own, then `gnss_sim_node`:

```bash
sudo systemctl start drone-mc-fcc-bridge
journalctl -u drone-mc-fcc-bridge -n 50 --no-pager   # Agent up, no repeated errors

sudo systemctl start drone-mc-indoor-nav             # start-on-demand, never in the boot set
journalctl -fu drone-mc-indoor-nav                   # leave this open as an extra log window
```

If fcc-bridge was already running from `drone-mc.target`, that's fine.

### 2.5 Prepare a run directory

Everything the run produces goes here. The helper container mounts it
as `/out`.

```bash
RUN=$(date +%Y%m%d-%H%M)
export RUN_DIR=~/ft006/$RUN
mkdir -p "$RUN_DIR"
cat > "$RUN_DIR/ft006.rc" <<'EOF'
set +u
source /opt/ros/humble/install/setup.bash
source /opt/drone-mc/install/setup.bash
alias ft006='python3 /tools/ft006_bench/ft006_bench.py'
cd /out
EOF
echo "$RUN"
```

---

## 3. Helper container and terminals

### 3.1 Start the helper container

**Why this step exists.** `ft006_bench.py` is a Python program that talks
to the drone's ROS 2 nodes. It needs ROS 2 and the project's message
definitions, and on the Jetson those exist only inside the Docker
containers, not on the Jetson itself. So it can't run from a normal
Jetson shell. The running service containers don't have the repo's
`tools/` folder either.

The fix is one extra, temporary container that has ROS 2, the same
network and settings as the real drone services (so it can see their
topics), and the repo's `tools/` folder. You type every helper command
(T1, T2, T3, T6) inside it.

**What the command does, line by line**

| Line | Meaning |
|---|---|
| `sudo docker run --rm -it` | Start a container. `-it` gives you an interactive shell in it. `--rm` deletes it when you exit |
| `--name mc-ft006` | Name it `mc-ft006`, so you can open more terminals into it later |
| `--network=host` | Use the Jetson's network directly, the same as the drone services. Without it, it can't see their topics |
| `--mount …/opt/drone-mc/install…` | Make the built project code (messages, nodes) available inside |
| `--mount …/etc/drone-mc…` | Make the drone's config files available inside, including the DDS config |
| `--mount …"$PWD/tools",dst=/tools…` | Make the repo's `tools/` folder appear as `/tools` inside. `$PWD` means "the folder I'm in now", so you must run this from the repo folder |
| `--mount …"$RUN_DIR",dst=/out` | Make your run folder (§2.5) appear as `/out` inside, so logs survive after exit |
| `--env-file /etc/drone-mc/environment` | Load the drone's settings (`DRONE_ID`, `CYCLONEDDS_URI`, `ROS_DOMAIN_ID`, …) |
| `--entrypoint bash` | Give you a shell, instead of starting a second copy of the Agent and fcc_bridge |
| `ghcr.io/drone-mc/fcc-bridge:${MC_VERSION}` | The image to use: the fcc-bridge image, already on the Jetson |

**Step by step**

**Terminal 1** (SSH into the Jetson):

```bash
# 1. Go to the repo folder on the Jetson (use your actual path)
cd ~/drone-swarm-mc
ls tools/ft006_bench/ft006_bench.py        # must exist

# 2. Load the drone settings into this shell (provides MC_VERSION etc.)
set -a; source /etc/drone-mc/environment; set +a
echo $MC_VERSION                            # must print something, e.g. dev

# 3. Check that the image exists
sudo docker images | grep fcc-bridge        # must list a line with that tag

# 4. Make sure RUN_DIR is set (from §2.5, in this same shell)
echo $RUN_DIR                               # must print e.g. /home/<you>/ft006/20261008-1430
```

If `echo $RUN_DIR` prints nothing, run the §2.5 block again in this
terminal. Variables only exist in the shell where you set them.

```bash
# 5. Start the helper container — paste the whole block
sudo docker run --rm -it --name mc-ft006 --network=host \
  --mount type=bind,src=/opt/drone-mc/install,dst=/opt/drone-mc/install,readonly \
  --mount type=bind,src=/etc/drone-mc,dst=/etc/drone-mc,readonly \
  --mount type=bind,src="$PWD/tools",dst=/tools,readonly \
  --mount type=bind,src="$RUN_DIR",dst=/out \
  --env-file /etc/drone-mc/environment \
  --entrypoint bash \
  ghcr.io/drone-mc/fcc-bridge:${MC_VERSION}
```

The prompt changes to something like `root@ubuntu:/#`. You are now
**inside** the container. Then:

```bash
# 6. Set up ROS 2 in this shell, then check you can see the drone
source /out/ft006.rc
ros2 node list                              # must list fcc_bridge_node and gnss_sim_node
ros2 topic hz /mc/fcc/vehicle_state         # ≈ 10 Hz, Ctrl-C after a few lines
ros2 topic list | grep /fmu/out             # PX4 topics present; none → UXRCE_DDS_PTCFG ≠ 2
```

If all three checks pass, it works. This terminal becomes T2 (§3.2).

**Terminals 2, 3, … (T1, T3, T6):** open a new SSH session to the
Jetson for each one, and attach to the same container:

```bash
sudo docker exec -it mc-ft006 bash
source /out/ft006.rc
```

**When finished:** type `exit` in Terminal 1. The container stops and is
deleted, and the other terminals attached to it close too. Your logs
stay in `$RUN_DIR` on the Jetson.

**Common errors**

| Error | Meaning |
|---|---|
| `invalid reference format` | `MC_VERSION` is empty. Do step 2 in this shell |
| `bind source path does not exist` | `RUN_DIR` is empty, or you're not in the repo folder. Do step 4, or `cd` to the repo |
| `Conflict. The container name "/mc-ft006" is already in use` | A previous one is still running: `sudo docker rm -f mc-ft006`, then retry |
| `ros2 node list` shows nothing | The services aren't running: `systemctl status drone-mc-fcc-bridge drone-mc-indoor-nav` |
| `ros2: command not found` inside the container | You skipped `source /out/ft006.rc` in this terminal |

### 3.2 Open the terminals

| Term | Where | Command | Purpose |
|---|---|---|---|
| T1 | helper | `ft006 flow` | Mock flow at 20 Hz. **Skip with real H-Flow.** Must be running before every anchor |
| T2 | helper | `ft006 watch \| tee -a watch.log` | 1 Hz side-by-side: sim status, baro, PX4's GPS. **Start before anchoring**, and leave it running all session |
| T3 | helper | `ros2 topic echo /mc/status_text \| tee -a status_text.log` | Anchor accepted/rejected, lockout engaged/cleared |
| T4 | Jetson host | see below | Wire capture of `HIL_GPS` sends |
| T5 | QGC MAVLink Console | PX4 shell | `listener …`, `param …`, `commander …` |
| T6 | helper | anchor commands (`ft006 anchor $ANCHOR`) | One-shot commands |

**T4** (host shell). Use one capture file per log section so the stop
events are easy to find later:

```bash
set -a; source /etc/drone-mc/environment; set +a
sudo tcpdump -ni eth0 -w "$RUN_DIR/s31.pcap" udp dst port 14557 and dst host $FCC_ETH_IP
```

Ctrl-C and restart it with the next name (`s32.pcap`, `s33.pcap`, …) at
the start of each section. For a quick live rate check in another host
shell (expect ≈ 50 in 10 s, i.e. 5 pkt/s):

```bash
sudo timeout 10 tcpdump -ni eth0 -l udp dst port 14557 and dst host $FCC_ETH_IP 2>/dev/null | wc -l
```

If `tcpdump` is missing: `sudo apt install tcpdump`.

### 3.3 Check the inputs before any anchor

In T2, before anchoring, every line should show:

- `baro=…m age=0.0s` (or < 1 s). `baro: none` → see §6.
- `sim: active=0 …` or `sim: no status yet`.

Also check the flow input once in T6:

```bash
ros2 topic hz /mc/sensors/optical_flow     # ≈ 20 Hz
ros2 topic hz /mc/sensors/baro_altitude    # ≈ 10 Hz
```

### 3.4 Choose and set the anchor

In T6 (and in any other shell you'll anchor from):

```bash
export ANCHOR="<lat> <lon> <alt_msl>"   # degrees, degrees, meters MSL
```

Pick an obviously fake but valid position near the bench (a round number
is fine) and a realistic MSL altitude. Use the same value all session,
because re-anchoring at a different altitude trips PX4's
`GPS Vertical Pos Drift too high` check for 20–30 s. Write it in log §2.
Leave `$ANCHOR` unquoted when you use it.

---

## 4. Test procedure

Each step below maps to a section of the log. Fill in that section's
table as you go, from what you actually see.

### 4.1 Log §3.1 — Negative path (`MAV_USEHILGPS=0`)

1. T5: `param show MAV_USEHILGPS` → must be `0`. If it's `1`:
   `param set MAV_USEHILGPS 0`, `param save`, `reboot`, wait for the FCC
   to come back (T2's baro age returns to < 1 s).
2. T4: start `s31.pcap`. T1 (if mocking), T2, T3 running.
3. T6: `ft006 anchor $ANCHOR` → prints `anchor: sent 50005 …`.
   T3 shows the anchor accepted.
4. Watch for 30 s:
   - T4 rate check: ≈ 5 pkt/s. The MC **is** sending.
   - T2 PX4 column: `no vehicle_gps_position yet`, or `src=other` only.
   - T5: `listener sensor_gps` shows no instance with devtype `0xAF`.
5. Fill in the log §3.1 table.

### 4.2 Log §3.2 — Positive path (`MAV_USEHILGPS=1`)

1. T5:
   ```
   param set MAV_USEHILGPS 1
   param save
   reboot
   ```
   After it comes back: `param show MAV_USEHILGPS` → `1`.
2. The FCC reboot drops `gnss_sim_node`'s session; it doesn't resume on
   its own. T1/T2/T3 keep running. Confirm T2 shows baro age < 1 s again.
3. T4: start `s32.pcap`.
4. T6: `ft006 anchor $ANCHOR`, and write the wall-clock time in the log.
5. Within a few seconds, T2 should show
   `src=HIL_GPS(bus=6/… sysid=<DRONE_ID> devtype=0xAF)`, `fix=3`, and
   lat/lon matching the anchor to about 1e-6°.
6. T5, after ≈ 10–20 s:
   ```
   listener sensor_gps                # an instance with devtype 0xAF
   listener estimator_status_flags    # cs_gnss_pos and cs_gnss_vel = True
   ```
   QGC's GPS indicator should show a 3D fix and the satellite count.
7. Check the altitude: PX4's altitude should equal the anchor altitude.
   About 40 m too high means `SENS_BAR_AUTOCAL` was still `1`. Set it to
   `0`, save, and re-anchor (§6).
8. Fill in the log §3.2 table.

**Optional — sysid mismatch.** On the Jetson host, edit
`/etc/drone-mc/environment` so `DRONE_ID` ≠ the FCC's `MAV_SYS_ID`, then
`sudo systemctl restart drone-mc-indoor-nav` (only that unit; don't
restart fcc-bridge). Anchor again: PX4 must ignore the stream, as in
§4.1. Then restore `DRONE_ID`, restart `drone-mc-indoor-nav` again, and
re-anchor before continuing.

### 4.3 Log §3.2a — "vertical velocity unstable" re-check

Continue from §4.2 with `cs_gnss_pos`/`cs_gnss_vel` true. Don't touch
the bench.

1. Leave it stationary for **at least 120 s**. During that window, run
   in T5 at least 5 times, spread out:
   ```
   listener estimator_aid_src_gnss_vel   # record the max |test_ratio_filtered[2]| and the innovation[2] range
   listener vehicle_gps_position         # record the vel_d_m_s range
   ```
2. At the end of the window:
   ```
   listener estimator_status    # pre_flt_fail_innov_vel_vert must be False
   commander check              # must NOT say "vertical velocity unstable"
   ```
   Record any other preflight failures as separate findings.
3. Optional, props **off**: `commander arm`, then `commander disarm`.
4. Fill in the log §3.2a table.

### 4.4 Log §3.3 — eph/epv/satellites over time

§4.3 used up most of the ≈ 200 s window, so start this section with a
fresh anchor, then **don't re-anchor until step 4**.

1. T4: start `s33.pcap`.
2. T6: `ft006 anchor $ANCHOR`. T2's `t=` restarts from 0.
3. Copy T2's lines at t = 0, 30, 60, ≈ 67, 100, 150 and ≈ 200 s into the
   log §3.3 table (they're also in `watch.log`). Alongside, poll
   `listener estimator_status_flags` in T5 and note the time
   `cs_gnss_pos` goes false.
   - Expected: eph/epv on both sides grow and never decrease. PX4 keeps
     GNSS fused through the 67 s and 100 s marks; dropping there is a
     finding.
   - At ≈ 200 s (eph 5.0 m): T2 prints
     `LATCHED INDOOR_GNSS_SIM_LOST: …` with a drift reason, T4's rate
     goes to 0, and PX4 then loses GPS on its timeout.
4. T6: `ft006 anchor $ANCHOR`. The latch clears, T4 resumes, and eph/epv
   reset.
5. Tick the checkboxes under the log §3.3 table.

### 4.5 Log §3.4 — Staleness watchdog on the wire

1. T4: start `s34.pcap`. Make sure T2 shows a steady HIL fix and T4
   ≈ 5 pkt/s. Re-anchor first if the §4.4 session is past ≈ 150 s.
2. Stall the flow input, and note the exact time in a host shell
   right as you do it: `date +%T.%N`
   - Mocked flow: Ctrl-C in T1.
   - Real H-Flow: in T5,
     `mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 0`
     (cleaner than unplugging CAN1 with the FCC powered).
3. Within ≈ 1 s, expect:
   - T2: `LATCHED INDOOR_GNSS_SIM_LOST: /mc/sensors/optical_flow input stale …`
   - T4: no more packets.
   - T5: the HIL `sensor_gps` instance times out; `cs_gnss_pos` goes
     false. Record PX4's own GPS-lost indication and when it appeared.
4. Restore the flow (restart T1, or
   `mavlink stream -u 14556 -s OPTICAL_FLOW_RAD -r 20`). The latch must
   **stay** and T4 must stay silent. Then `ft006 anchor $ANCHOR`: the
   stream resumes.
5. After the section, get the last packet time from the capture and
   compare it with the stall time:
   ```bash
   sudo tcpdump -nr "$RUN_DIR/s34.pcap" -tttt | tail -3
   ```
6. Optional — baro stall: in T5,
   `mavlink stream -u 14556 -s ALTITUDE -r 0`. Expect the reason
   `baro_altitude input stale`. Restore with `-r 10` and re-anchor.
7. Fill in the log §3.4 table.

### 4.6 Log §3.5 — Real-GPS lockout

Needs a real fix on the Pixhawk's GPS port (`fix_type ≥ 3`,
`hdop < 2.0`). Without one, write **NOT RUN (no receiver)** in the log
and go to §5. Read the log's "Known limitation" note first.

In a helper terminal, watch what the lockout sees:

```bash
ros2 topic echo /mc/fcc/vehicle_state --field fix_type
ros2 topic echo /mc/fcc/vehicle_state --field hdop      # in another terminal
```

1. **A — refuse to anchor.** Start from a fresh session:
   `sudo systemctl restart drone-mc-indoor-nav` on the host. Bring up the
   real fix. T3: `real GPS fix detected -- indoor lockout engaged …`.
   Anchor: T3 shows `rejected anchor command -- a real GPS fix is currently present`,
   and T4 stays silent.
2. **B — auto-clear.** Remove the real fix (cover/disconnect the antenna,
   or stop the simulator). T3: `real GPS fix not present -- indoor lockout clear`,
   with no operator action. Anchor again: accepted, stream starts.
3. **C — stop an active stream.** With B's session streaming, bring the
   real fix back. Expect lockout engaged in T3, T4 stops, and no latched
   fault. Record the device_id/src that T2 shows PX4 selecting. Remove
   the fix again: T4 must resume **without** a re-anchor.
4. Fill in the log §3.5 table.

---

## 5. Collect evidence and wrap up

1. Stop T4 and the helper terminals (Ctrl-C), then `exit` the helper
   container. `watch.log`, `status_text.log` and the `s3*.pcap` files
   are in `$RUN_DIR`.
2. From the FCC (T5 or QGC):
   - `param show -c` → save the output as `$RUN_DIR/params.txt`, or QGC
     → Parameters → Tools → Save to file.
   - `cat /fs/microsd/etc/extras.txt` → save as `$RUN_DIR/extras.txt`.
   - QGC → Analyze Tools → Log Download → the `.ulg` for this session.
3. Paste key snapshots into log §4, fill in log §5's summary table and
   the header's **Status** line, and file any real-firmware discrepancy
   as a ticket (log §5, deliverable 3).
4. Stop `gnss_sim_node`:
   ```bash
   sudo systemctl stop drone-mc-indoor-nav
   ```
5. FCC parameters afterwards:
   - Next session is indoor flight (`TP-FT-002`): leave
     `MAV_USEHILGPS=1` and `SENS_BAR_AUTOCAL=0`, and apply
     `HID-MC-001.md` §10.7 B.
   - Next session is outdoor on a real receiver: `MAV_USEHILGPS 0`,
     `SENS_BAR_AUTOCAL 1`, `param save`, `reboot`.

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `anchor: FAIL -- no /mc/gcs/command subscriber` | `gnss_sim_node` not running, or the helper can't see it over DDS | `systemctl status drone-mc-indoor-nav`; helper started with `--network=host` and `--env-file`? `ros2 node list` in the helper |
| `ros2 node list` empty or hangs in the helper | DDS config or socket buffer floor | `--env-file` missing; host `net.core.rmem_max` below CycloneDDS's 10 MB floor → re-run `sudo ./scripts/01-install-os.sh` |
| T2 `baro: none` | FCC isn't streaming `ALTITUDE` to this MC | `mavlink status streams` on the FCC; check `extras.txt` has the `ALTITUDE` line and `-t` = current `MC_ETH_IP` (`tx 0 / txerr > 0` means wrong `-t`) |
| `LATCHED … input stale` right after anchoring | Flow or baro not arriving when the anchor landed | T1 running (or H-Flow streaming)? baro age < 1 s? Fix, then re-anchor |
| T2 `px4: no vehicle_gps_position yet` with `MAV_USEHILGPS=1` and T4 sending | sysid gate, missing `:14557` instance, or no `/fmu/*` topics | `MAV_SYS_ID` = `DRONE_ID`; `mavlink status` lists UDP 14557; `ros2 topic list \| grep fmu` (empty → `UXRCE_DDS_PTCFG` ≠ 2) |
| `watch` crashes on `latitude_deg` | Workspace built with the 1.14 `px4_msgs` layout | Rebuild with `release/1.17` (`05-build-workspace.sh`) |
| T4 silent after anchoring | Anchor rejected, or watchdog latched | Read T3 and T2's `LATCHED` line. A rejection may mean a real GPS fix is present (lockout) |
| PX4 altitude ≈ anchor + 40 m; `height estimate not stable` | `SENS_BAR_AUTOCAL` fired this boot | `param set SENS_BAR_AUTOCAL 0`, `param save`, re-anchor |
| `vertical velocity unstable` | Build without the `vd` low-pass filter | Check §2.1's `grep`; rebuild |
| `GPS Vertical Pos Drift too high` | Re-anchored at a different altitude | Wait 20–30 s; keep the same `$ANCHOR` all session |
| `First waypoint far away from home` | Stale mission on the FCC far from the fake anchor | Clear the mission in QGC's Plan view. Irrelevant to the FT-006 checks |
| Nothing works after an FCC reboot | The `gnss_sim_node` session doesn't survive an FCC reboot | Re-anchor |
