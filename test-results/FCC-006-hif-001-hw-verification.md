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
   actual route to the Jetson, or directly on the Jetson.
2. **`MicroXRCEAgent` is not yet installed anywhere in the image build.**
   Neither `docker/ros2-core/Dockerfile` nor `docker/fcc-bridge/Dockerfile`
   installs the Micro XRCE-DDS Agent binary. This matches the gap already
   tracked as FCC-008 ("MicroXRCEAgent lifecycle") in
   `FCC-MISSION-CHANNEL-BACKLOG.md`, which states FCC-008 "blocks any real
   hardware-in-the-loop use of FCC-002 today." Until FCC-008 (or an
   equivalent image change) lands, `06b-smoke-test-fcc-links.sh`'s
   `LIVE_CHECK=1` path cannot pass — the container will fail to start
   `MicroXRCEAgent` at all.

## To complete this ticket

1. Land FCC-008 (or otherwise add a `MicroXRCEAgent` install step to the
   fcc-bridge image).
2. On the Jetson DUT, with a powered Pixhawk 6X wired via HIF-001 and
   `UXRCE_DDS_CFG=102` / `UXRCE_DDS_BAUD=921600` set via QGroundControl
   (HID-MC-001 §3.3):
   ```bash
   LIVE_CHECK=1 sudo ./scripts/06b-smoke-test-fcc-links.sh
   ```
3. Paste the script's PASS/FAIL output and the measured
   `average rate` values below, replacing this template's placeholders.

```
<paste 06b-smoke-test-fcc-links.sh output here>
```

| Check | Expected | Measured | Result |
|---|---|---|---|
| `/fmu/out/vehicle_status` rate | ≥ 5 Hz | — | — |
| `/fmu/out/vehicle_local_position` rate | ≥ 10 Hz | — | — |
