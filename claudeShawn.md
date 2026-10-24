# Notes: plan-sequencer MC built on drone-swarm-mc

Paths below are relative to the repo root.

## Goal

MC software on a drone that receives a set of mission plan files from the
GCS, then uploads them to the FCC one at a time, loading the next plan each
time the current mission completes.

## Reuse as-is

- **Receiving plans from the GCS: `ws/src/gcs_comms` → `mission_orchestrator_node`**
  - MAVFTP server on MAVLink Channel C (create/update/list/read/delete) over
    `/var/drone-mc/mission_plans/`, so the GCS can push every plan up front.
  - Own container `docker/mission-orchestrator/`, router config
    `config/mavlink-router-channel-c.conf`, GCS-side simulator
    `tools/mission-orchestrator-sim/`.
  - `include/gcs_comms/upload_id.hpp`: unique, restart-safe upload IDs.
- **Uploading to the FCC: `ws/src/fcc_bridge`**
  - `FccBridgeNode` subscribes `/mc/fcc/mission_upload` (`mc_msgs/FccMission`),
    runs MISSION_COUNT → MISSION_ITEM_INT×N → MISSION_ACK, and publishes the
    result on `/mc/fcc/mission_ack` (`mc_msgs/FccMissionAck`).
  - Code: `MissionProtocolClient` behind `FccBackend::upload_mission()`.
    Works on both the real FCC (XRCE-DDS) and SITL (MAVLink UDP) backends.
- **Supporting pieces**
  - `mc_msgs`: `FccMission`, `FccMissionItem`, `FccMissionAck`, `FccCommand`,
    `MissionPhase`, `StatusText`.
  - `mission_exec/mission_persistence`: atomic save/load pattern, useful for
    saving the plan queue position across restarts.
  - `health_monitor`, `indoor_nav` (indoor GPS sim): no changes needed.

## Gaps to fill

1. **No auto-advance.** A plan reaches the FCC only on an explicit
   `MC_MISSION_PLAN_ACTIVATE` from the GCS, and only in `PREFLIGHT`
   (`load_and_activate_plan()` in `mission_orchestrator_node.cpp`). No queue.
2. **Not QGroundControl `.plan` format.** `parse_plan_json()` reads a custom
   search-and-rescue JSON (`name`, `ingress_waypoints`, `formation_offsets`,
   `search_polygon`, …) into `MissionUpload`. A QGC `.plan` has its items under
   `mission.items[]`, so a new parser is needed.
3. **Waypoint-only mission items.** `fcc_bridge::MissionItem` and
   `mc_msgs/FccMissionItem` carry only frame/command/lat/lon/alt/acceptance
   radius. `send_mission_item_int()` hardcodes param1/3/4 and `autocontinue`.
   Add `param1`–`param4` + `autocontinue` to both and pass them through, or
   loiter times, speed changes and `DO_*` commands are lost.
4. **No mission-complete signal.** `VehicleState` has no mission progress and
   fcc_bridge does not decode `MISSION_CURRENT` / `MISSION_ITEM_REACHED`.
   - Missions end on the ground: reuse `fcc_landed_and_disarmed()` from
     `mission_manager_node.cpp` (disarmed + |vz| small).
   - Plans chain in the air: add progress decoding to fcc_bridge and publish
     a new topic (e.g. `mc_msgs/FccMissionProgress`).

## Skip `mission_manager_node`

It is built around the search-and-rescue flow (Voronoi split, coverage
paths, swarm role election) and generates its own FCC missions per phase
instead of passing a plan through. Borrow pieces only: the landed check,
ack timeout/retry logic, arm/mode commands via `FccCommand`.

## Proposed layout

```
GCS ──MAVFTP (Channel C)──► mission_orchestrator_node  (reuse: stores plan files)
                                   │ plan files on disk + ordered list
                                   ▼
                          plan_sequencer_node  (NEW)
                            - QGC .plan → FccMission
                            - on mission complete → load next plan
                            - ack retry/timeout, persist queue position
                                   │ /mc/fcc/mission_upload   ▲ /mc/fcc/mission_ack
                                   ▼                          │
                             fcc_bridge_node  (reuse + params/autocontinue
                                               + mission-progress publish)
                                   ▼
                                  FCC
```

Plan order from the GCS: simplest is a manifest file (e.g. `_sequence.json`)
uploaded over the same MAVFTP channel, so no new MAVLink message is needed.
Alternative: extend `MC_MISSION_PLAN_ACTIVATE` with a "queue" flag.

## Repo quirks

- `README.md` lists `MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md` and
  `CLAUDE.md`, but neither file is in the repo. The Channel C interface is
  documented in `ICD-MC-001.md` §12.
