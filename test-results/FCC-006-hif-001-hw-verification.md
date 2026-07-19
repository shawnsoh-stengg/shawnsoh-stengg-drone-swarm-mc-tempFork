# FCC-006 — HIF-001 (ttyTHS1) Hardware Verification — Test Results Log

| Field | Value |
|---|---|
| Ticket | FCC-006 (external tracker — "Verify HIF-001 (ttyTHS1) on hardware"; distinct from this repo's own FCC-006 in `FCC-MISSION-CHANNEL-BACKLOG.md`) |
| Test procedure | TP-SM-006 (`scripts/06b-smoke-test-fcc-links.sh`, `LIVE_CHECK=1`), per HID-MC-001 §3.4 |
| Requirement | HIF-001 |
| Status | **NOT RUN — blocked, see below** |

## Result

Not executed. This log is a template for whoever runs the live check on
the physical DUT; it does not contain real measurements.

## Blockers encountered while preparing this ticket (2026-07-15)

1. **No network path to the DUT.** `192.168.10.11` (Jetson MC, per
   `CLAUDE.md`'s network table) is unreachable from the workstation this
   work was done on (`ssh: connect to host 192.168.10.11 port 22: No
   route to host`). The live check must be run from a machine with an
   actual route to the Jetson, or directly on the Jetson. **Still
   blocking as of 2026-07-19** — unrelated to blocker 2 below.
2. ~~`MicroXRCEAgent` is not yet installed anywhere in the image
   build.~~ **Resolved 2026-07-19 by FCC-008**:
   `docker/fcc-bridge/Dockerfile` now builds the Agent from source in a
   dedicated `xrce_agent_builder` stage, and `docker/fcc-bridge/
   entrypoint.sh` launches it (transport selected by `FCC_TRANSPORT`)
   before `fcc_bridge_node` starts. `06b-smoke-test-fcc-links.sh`'s
   "fcc-bridge image has MicroXRCEAgent binary" check should now pass
   once the image is rebuilt (`scripts/08-build-fcc-bridge-image.sh`).
   See `scripts/06c-smoke-test-xrce-agent.sh` for FCC-008's own
   transport-selection/liveness checks.

## To complete this ticket

1. ~~Land FCC-008~~ — done, see above.
2. On the Jetson DUT, with a powered Pixhawk 6X wired via HIF-001 and
   `UXRCE_DDS_CFG=102` / `UXRCE_DDS_BAUD=921600` set via QGroundControl
   (HID-MC-001 §3.3):
   ```bash
   LIVE_CHECK=1 sudo ./scripts/06b-smoke-test-fcc-links.sh
   ```
   Blocker 1 (no network route to a DUT) still applies — this step still
   needs to be run from a machine with an actual route to the Jetson, or
   directly on the Jetson.
3. Paste the script's PASS/FAIL output and the measured
   `average rate` values below, replacing this template's placeholders.

```
<paste 06b-smoke-test-fcc-links.sh output here>
```

| Check | Expected | Measured | Result |
|---|---|---|---|
| `/fmu/out/vehicle_status` rate | ≥ 5 Hz | — | — |
| `/fmu/out/vehicle_local_position` rate | ≥ 10 Hz | — | — |
