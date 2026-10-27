# gcs-test-panel (TEST-007)

A desktop GUI that stands in for the ground Mission Orchestrator app on
MAVLink **Channel B** (GCS ↔ `gcs_mavlink_node`) and **Channel C** (Mission
Orchestrator ↔ `mission_orchestrator_node`). A test engineer can run, watch
and record MC functional tests from one screen, without scripting. Channel A
(PX4 autopilot) stays with QGroundControl. The panel never binds QGC's port
14550, so both run side by side on the GCS laptop.

It is a test double, not the real Mission Orchestrator app (another team's
deliverable). Every action maps onto one row of
`docs/GCS-MAVLINK-API-REFERENCE.md`, so it also works as a reference for
that team.

- **Language:** Python 3 + PySide6 (Qt 6) + pyqtgraph. It reuses
  `tools/mission-orchestrator-sim`'s verified dialect generation and MAVFTP
  client instead of re-implementing them. See `PROJECT-PLAN-5-MONTH.md`
  `TEST-007` for the full rationale.
- **No ROS 2/colcon dependency.** It runs on any GCS-class laptop (Ubuntu
  22.04/24.04, Windows) that has Python ≥ 3.10.

## Install and run

```bash
cd tools/gcs-test-panel
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
./gcs-test-panel --mc-host 192.168.100.11 --drone-id 1      # then press Connect
```

The panel needs `tools/mission-orchestrator-sim/` next to it (it imports
that package) and `ws/src/gcs_comms/mc_mavlink_defs.xml` (for the custom
messages). Both are present in a normal checkout. If you run it outside a
checkout, set `MISSION_ORCHESTRATOR_SIM_ROOT` and `MC_MAVLINK_DEFS_XML`.

| Option | Default | Meaning |
|---|---|---|
| `--mc-host` | `192.168.100.11` | The MC's `MC_ETH_IP` (`127.0.0.1` for a local/loopback rig) |
| `--drone-id` | `1` | The MC's `DRONE_ID` (= its MAVLink `system_id`) |
| `--connect` | off | Connect on startup |
| `--test-case` | — | Initial test-case ID, e.g. `"FT-001 AC2"` |
| `--log-dir` | `~/gcs-test-panel-logs` | Session log root (env `GCS_TEST_PANEL_LOG_DIR`) |
| `--bind` | `0.0.0.0` | Local address for the listen ports |
| `--b-recv-port` / `--b-send-port` | `14551` / `14552` | Channel B: GCS listen (`GCS_CHANNEL_B`) / MC receive (`GCS_MC_RECV`) |
| `--c-recv-port` / `--c-send-port` | `14571` / `14572` | Channel C: Orchestrator listen / MC receive |

Everything except `--log-dir` can also be changed in the left-hand
**Connection & session** panel before connecting.

**Network prerequisite:** the MC's `mavlink-router` sends Channel B/C
traffic to a fixed `GCS_IP` (`192.168.100.10`). Run the panel on the
machine that owns that address. Like `tools/ft001_ac2_channel_b`, the panel
needs both sockets (send and listen) to reach it. Close the TEST-004 CLI
and any other copy of the panel first, because they bind the same listen
ports.

## The screen

| Area | What it does | Pass/fail it grades |
|---|---|---|
| **Status bar lamps** | Channel B: green while MC `HEARTBEAT`s arrive (red after 3 s without one). Channel C: `mission_orchestrator_node` never sends anything unsolicited, so this lamp shows whether the *last request* got a reply. Tick *Auto-refresh* on the plans tab to keep it current. | — |
| **Monitor** | `MC_STATUS` tiles, decoded `fault_mask` (ICD §10.8), `MC_HEALTH` bars, decoded `node_alive_mask` (§10.9), plots of altitude, battery and phase, a local east/north map (track, detections and the editor's plan; no map tiles, so it works offline in the field) and a `MC_DETECTION` table | Shows `MC_STATUS` < 2 Hz (FR-062) and `MC_HEALTH` < 1 Hz (FR-071) in red, and logs each drop/recovery |
| **Channel B — Commands** | `MC_CMD_PHASE_TRANSITION` (the hint shows the one transition the MC accepts from the current phase and what its guard needs), `ARM_AUTHORISE` (with a confirm: the MC may arm the FCC straight away; its upload ID fills in from a Channel B upload, or as a placeholder `1` after a Channel C activate, which doesn't report its ID. Upload ID 0 *cancels* authorisation, so the panel asks before sending it), `GCS_OVERRIDE` and `END_MISSION`. **Indoor GNSS sim anchor** (`MC_CMD_INDOOR_GNSS_SIM_ANCHOR` 50005: lat/lon/alt MSL, remembered between sessions, with a confirm) replaces `ft006 anchor` for FT-006 bench runs; its result (`gnss_sim_node: anchored …` or `… rejected …`) arrives as a `STATUSTEXT` in the Event log tab. Disarm, RTL and the manual-mode `DO_SET_MODE` are boxed in red and need a confirm. **GCS heartbeat** checkbox for the link-loss test. | **ACK:** `ACCEPTED` within 500 ms (FR-061/FR-003). **Effect:** whether `MC_STATUS` then shows the expected phase within 3 s (see below). With the heartbeat off, passes if `GCS_LINK_LOST` appears 10–13 s after the last heartbeat sent (FR-073/SFR-007) |
| **Mission plans** | One JSON editor (validated live against the plan format), two ways in: **Channel C** create/update/read/delete/activate on the MC's plan library, or **Channel B** `MC_MISSION_UPLOAD` (items `50200`–`50204`), with per-item progress and re-requests | Upload: `MISSION_ACK` ACCEPTED. The returned `upload_id` is copied into the ARM_AUTHORISE box |
| **Message inspector** | Every frame, both channels, both directions (TX in blue), with a text filter, and a per-type count/rate table | — |
| **Event log** | The live view of `events.jsonl` | — |

**ACK is not execution.** `gcs_mavlink_node` ACKs every `COMMAND_LONG`
`ACCEPTED` as soon as it has decoded it. Its design is zero business logic:
it just republishes the command on `/mc/gcs/command`. `mission_manager_node`
then decides. For `MC_CMD_PHASE_TRANSITION` its verdict comes back as a second
`COMMAND_ACK` (shown as `IN_PROGRESS → ACCEPTED/DENIED`, with the reason).
Everything the MC publishes on `/mc/status_text` (pre-arm failures, anchor
results) arrives as `STATUSTEXT` in the Event log tab; texts over 50
chars are reassembled from their chunks. The **Effect** column still watches
`MC_STATUS` for the phase change itself. What to expect:

- The MC moves **one phase at a time** (`PREFLIGHT → ARMING → TAKEOFF →
  INGRESS → …`). Asking for any other target is rejected (FR-002), e.g.
  TAKEOFF or INGRESS from PREFLIGHT.
- **PREFLIGHT → ARMING** needs `MC_CMD_ARM_AUTHORISE` *and* all 7 pre-arm
  checks (`mission_exec/pre_arm_checks.cpp`): FCC link alive, FCC disarmed,
  ARM authorised, battery above threshold + margin, live peers in
  PREFLIGHT/ARMING, a complete mission upload, and GNSS OK or visual
  localisation alive. "Complete" depends on the plan's `skip_phase_mask`:
  ingress waypoints always; formation offsets, swarm manifest and search
  polygon unless the plan is transit. The live-peer check is skipped for
  transit plans, and only counts peers in the plan's swarm manifest
  otherwise. Once they hold, the MC enters ARMING **by itself and
  arms the FCC**; no phase command is needed.
- Send ARM_AUTHORISE **after** the Channel B upload or Channel C activate.
  Every new mission upload resets the authorisation to the plan's own
  `arm_authorised` (false in the example plan).
- ARMING → TAKEOFF is automatic once the FCC reports armed. TAKEOFF →
  INGRESS needs the GCS command *and* the drone at takeoff altitude.
- To see *why* the MC refused, read the mission-exec log on the Jetson:
  `docker logs -f mc-mission-exec 2>&1 | grep -E "pre-arm|rejected"`.

**How verdicts work:**
- *PASS/FAIL* appears only where a requirement defines the expected result
  (ACK latency, upload accepted, telemetry rates, link-loss timing).
- A plan-library **Nak** (`FileExists`, `FileProtected`, `FileNotFound`) and
  an activate **`REJECTED_*`** are logged with no verdict. They are valid MC
  replies, and whether one was expected depends on the test case. Only
  *no reply at all* is a FAIL.
- Fault raised/cleared and node-liveness changes are also logged as
  observations, without a verdict.

## Example plans

`examples/` holds one plan per `skip_phase_mask` stage set (EXEC-008). Load
one with **Plans → Load file…**, then push it over Channel C (save to the
library, then Activate) or Channel B (upload):

| File | `skip_phase_mask` | Mission |
|---|---|---|
| `swarm_full.json` | `0` | TAKEOFF → INGRESS → MUSTER → ISR_SELECTION → VORONOI_PARTITION → SEARCH → RETURN → LAND (3 drones) |
| `solo_transit.json` | `240` | TAKEOFF → INGRESS → RETURN → LAND |

- The coordinates are placeholders near Singapore (the same ones
  `plan.example_plan()` uses). Replace them with your test site's before
  activating on a real drone.
- `arm_authorised` is `false` in every example, so send ARM_AUTHORISE
  after the upload.
- INGRESS ends at the last ingress waypoint. `solo_transit` sends RTL
  there.
- `swarm_full` still stalls at ISR_SELECTION until `swarm_coord` is built.
- Solo search (`skip_phase_mask` `112`: INGRESS → SEARCH over the whole
  polygon → RETURN) is refused at upload and pre-arm until
  `search_planner_node` publishes a coverage path — without one the drone
  would hold at the end of INGRESS forever.
- Over Channel B, `solo_transit` ends with an `MC_MISSION_CONFIG` (50205)
  item carrying the mask.

`tests/test_plan.py` checks every file here is valid and that every
accepted stage set is covered.

## Indoor GNSS sim: satellite count and fix decay

After an **Indoor GNSS sim anchor**, `gnss_sim_node` reports a falling
satellite count to PX4 in `HIL_GPS` (`satellites_visible`), from 10 at
the anchor down to 4. The panel only sends the anchor. The count and
how fast it falls are set MC-side in `ws/src/indoor_nav/params/default.yaml`
(`gnss_sim_node.ros__parameters`):

| Parameter | Default | Effect |
|---|---|---|
| `hil_gps_sats_at_anchor` | `10.0` | Satellite count just after an anchor |
| `hil_gps_min_sats` | `4.0` | Floor the count falls to |
| `eph_growth_rate_m_per_s` | `0.02` | How fast the horizontal error (`eph`) grows, so how fast the count falls. Not in the YAML yet; add it to override the code default |
| `eph_at_anchor_m` | `1.0` | `eph` at the anchor. Not in the YAML yet either |
| `max_eph_before_fault_m` | `5.0` | `eph` at which the sim latches `INDOOR_GNSS_SIM_LOST` |

- The count falls in a straight line from `hil_gps_sats_at_anchor` to
  `hil_gps_min_sats` as `eph` grows from `eph_at_anchor_m` to 3 m, PX4's
  `EKF2_REQ_EPH` default (`send_hil_gps()` in `gnss_sim_node.cpp`). So
  with the defaults it reaches 4 about 100 s after the anchor, when PX4
  also stops accepting the fix. The 5 m fault follows at about 200 s.
- To fly longer between anchors, lower `eph_growth_rate_m_per_s`. Changing
  only the two satellite counts doesn't change the timing.
- The YAML is copied into the image when it's built
  (`docker/indoor-nav/Dockerfile`), so rebuild and restart after editing:
  `sudo ./scripts/16-build-indoor-nav-image.sh && sudo systemctl restart drone-mc-indoor-nav`.
  The Jetsons build from the org repo (`AI-DA-STC/drone-swarm-mc`), so
  make the edit there too.
- Send a new anchor to reset the count to `hil_gps_sats_at_anchor`.

## Logs (test evidence)

Each launch writes a new session folder `~/gcs-test-panel-logs/<YYYYmmdd-HHMMSS>/`:

- `channel_b.tlog`, `channel_c.tlog`: every frame, both directions, in
  standard MAVLink tlog format (`mavlogdump.py`, QGC/MAVExplorer, or the
  panel's own **File → Replay .tlog…**).
- `events.jsonl`: one JSON object per action, result or alarm, each stamped
  with the **test case** set in the left panel. Notes typed into the panel
  go here too.
- `session.json`: connection settings, repo version (`git describe`), host,
  tester.

**Export session (.zip)** bundles the folder for attaching under
`test-results/`. The MC links carry no software-version message, so the
`repo_version` in `session.json` is the panel's checkout. Record the MC's
own version in a note (`git -C ~/swarm-drones describe` on the Jetson).

**Duplicate frames:** an MC running a Channel B/C router config from before
2026-10-18 delivers each MC frame twice when the GCS sends from its own
listen port. One copy comes through the fixed
`GCS_CHANNEL_B`/`ORCHESTRATOR_CHANNEL_C` endpoint, the other through the
Server endpoint that learned the GCS as a client. The current configs make
that Server endpoint ingress-only, so this no longer happens. The panel
still drops exact repeats before display, as a guard against stale
on-device configs. A non-zero "duplicate frames dropped" counter in the
status bar means the MC's router config needs re-installing
(`scripts/02-install-systemd-units.sh`). The `.tlog` keeps every copy, as
the raw record.

## A typical session

1. Set **Test case** (e.g. `FT-001 AC2`), press **Connect**, and wait for
   the Channel B lamp to turn green. Confirm the Monitor shows `MC_STATUS`
   at ≥ 2 Hz.
2. **Mission plans:** edit the plan (or *Load file…*), then **Create on MC
   (Channel C)** and **Activate**. Or use **Upload via Channel B**.
3. **Commands:** ARM_AUTHORISE (the upload ID is pre-filled after a
   Channel B upload), then phase transitions. Watch the phase on the
   Monitor.
4. **Link-loss test:** untick *Send 1 Hz GCS HEARTBEAT*, wait for
   `GCS_LINK_LOST` (the Event log grades the timing), then tick it again.
5. Add notes as you go, then **Export session** and attach the zip to the
   test record.

## Tests

```bash
env -u PYTHONPATH .venv/bin/python -m pytest -q tests     # unset a sourced ROS PYTHONPATH: its pytest plugins break collection
```

- `test_plan.py`, `test_channels.py`, `test_logging.py`, `test_ui.py`:
  these run against `gcs_test_panel/fake_mc.py`, a minimal loopback MC
  (both channels). `test_ui.py` drives a whole session through the real
  widgets headless (`QT_QPA_PLATFORM=offscreen`). Set
  `GCS_PANEL_SHOTS=<dir>` to save a PNG of every tab.
- `test_real_mc.py`: the acceptance run against a **real** MC. It is
  skipped unless `GCS_PANEL_REAL_MC=<MC IP>` is set (plus
  `GCS_PANEL_DRONE_ID`). `GCS_PANEL_REAL_MC_PHASE_GATE=1` also checks
  activate → `REJECTED_PHASE` outside PREFLIGHT, which drives a phase
  transition, so use it on a bench rig only.

`python3 -m gcs_test_panel.fake_mc` starts the fake MC on loopback for
learning the panel without hardware: `./gcs-test-panel --mc-host 127.0.0.1`.
The fake is **not** a reference. Only the real nodes are.

## Code layout

```
gcs_test_panel/
  core/            # no Qt — also importable from HSR-003's CLI and HSR-004's pytest suite
    defs.py        # ports, ids, command/enum/bit tables (cited to ICD/API reference)
    link.py        # SplitPortLink + LinkWorker: one reader thread per socket, fan-out, dedup
    channel_b.py   # heartbeat, COMMAND_LONG + timed ACK, MC_MISSION_UPLOAD sender
    channel_c.py   # TEST-004's ChannelCClient fed from the reader thread
    plan.py        # plan JSON validation + Channel B item encoding
    telemetry.py   # latest MC_STATUS/MC_HEALTH, rate meters, phase/fault/rate events
    session.py     # session folder, events.jsonl, export zip
    tlog.py, replay.py
  ui/              # PySide6 widgets; bridge.py moves results back onto the GUI thread
  fake_mc.py       # loopback test double
examples/          # example plans, one per skip_phase_mask stage set
```
