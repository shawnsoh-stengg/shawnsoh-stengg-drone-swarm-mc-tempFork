# MC↔FCC Flight-Mode Switching (MISSION ⇄ OFFBOARD) — Proposal

**Status:** Draft for review. Nothing in this doc has been applied to
SRS-MC-001.md / ICD-MC-001.md / SDD-MC-001.md yet — it's a proposal to
amend them, written the same way `FCC-MISSION-CHANNEL-BACKLOG.md` was
before FCC-005/006 landed.

## Why this doc exists

New requirement: `mission_manager_node` must be able to switch FCC flight
control between **MISSION mode** (waypoints, MAVLink mission protocol,
Ethernet UDP) and **OFFBOARD mode** (continuous setpoints, uXRCE-DDS,
serial) depending on what the mission plan calls for on a given leg.

This directly reverses a **deliberate, already-documented design
decision**, not a gap:

- **FR-012** (SRS-MC-001.md:199): MC may only command HOLD/RTL/MISSION/TAKEOFF.
- **FR-020/FR-023** (SRS-MC-001.md:209,212): INGRESS formation-keeping
  explicitly stays in MISSION mode, "no OFFBOARD mode," by replacing a
  `FORMATION_SLOT` mission item at ≥5 Hz instead of streaming setpoints.
- **ICD-MC-001.md:325**: *"The MC never commands `OFFBOARD` (FR-012)."*
- **SDD-MC-001.md:259,412**: contrasts the current design against "the old
  `trajectory_setpoint`/`offboard_control_mode` pair" — an earlier revision
  used OFFBOARD and it was removed specifically because OFFBOARD requires a
  ≥2 Hz setpoint floor or PX4 auto-exits the mode, whereas MISSION mode has
  no such floor (a gap doesn't cause a mode change).

Reintroducing OFFBOARD reintroduces that exact reliability tradeoff. This
proposal scopes it narrowly — **only on legs the mission plan explicitly
flags** — so phases that don't need it keep the current gap-tolerant
behavior.

The transport split the requirement describes (OFFBOARD over uXRCE-DDS/
serial, MISSION over MAVLink/Ethernet UDP) needs no channel-assignment
change: the mission-protocol channel already always rides the dedicated
`192.168.12.31:14555` UDP link regardless of `FCC_TRANSPORT` (ICD-MC-001.md
§3.1, IR-007), and `vehicle_command`/`vehicle_state` already ride
uXRCE-DDS. The new part is that uXRCE-DDS must *also* carry an OFFBOARD
setpoint stream when a leg calls for it.

`mission_manager_node` and `formation_controller_node` are both still
empty stubs (constructor-only), so no shipped MISSION-only logic needs to
be unwound — this is a good point to fold the change in before ME-001/SC-001
implementation starts.

**Design decisions already made (see conversation record):**
1. `formation_controller_node` owns OFFBOARD setpoint generation/streaming.
   `mission_manager_node` only decides *which* mode a leg needs and tells it.
2. Scope is data-driven — a per-phase flag in the GCS mission upload, not a
   hardcoded phase list.
3. Stream-staleness fallback is MC-driven (health_monitor watchdog), not
   left to PX4's `COM_OBL_ACT` parameter.

---

## Requirement deltas (SRS-MC-001.md)

- **Amend FR-012**: MC may command HOLD, RTL, MISSION, TAKEOFF, **or
  OFFBOARD** (add OFFBOARD to the PX4 v1.14 command set), still exclusively
  via uXRCE-DDS `vehicle_command`.
- **Amend FR-020**: formation-keeping during INGRESS uses MISSION-mode
  `FORMATION_SLOT` replacement *unless* the mission plan flags that leg for
  OFFBOARD, in which case `formation_controller_node` streams setpoints
  instead. Drop the blanket "no OFFBOARD mode" clause.
- **Amend FR-023**: scope the ≥5 Hz mission-item-replace requirement to
  "when in MISSION mode"; add the OFFBOARD-mode equivalent (see new FR
  below).
- **New FR (mode arbitration)**: `mission_manager_node` shall read the
  per-phase control-mode flag from the active mission plan and command the
  corresponding FCC flight mode at each phase entry.
- **New FR (OFFBOARD entry sequencing)**: before commanding `DO_SET_MODE
  OFFBOARD`, the MC shall establish an OFFBOARD setpoint stream at ≥2 Hz for
  at least 1 s (PX4 pre-arm-for-mode-entry requirement), then issue the mode
  command, then continue streaming without interruption while OFFBOARD is
  active.
- **New FR (OFFBOARD setpoint rate)**: while OFFBOARD is active,
  `formation_controller_node` shall publish `/mc/fcc/offboard_setpoint` at
  ≥5 Hz (tracking-quality target, mirrors FR-023's existing cadence),
  never below the PX4-imposed 2 Hz floor.
- **New SFR (stream-staleness fallback)**: if the OFFBOARD setpoint stream
  age exceeds 500 ms (mirrors the existing 500 ms GCS-override response
  bound, FR-003), `health_monitor_node` shall command a fallback to MISSION
  mode (if a valid mission is loaded) or HOLD, rather than relying on the
  FCC's own `COM_OBL_ACT` failsafe timing.
- **New FR (exit sequencing)**: before commanding `DO_SET_MODE MISSION` to
  leave OFFBOARD, the MC shall confirm a valid mission is already loaded in
  FCC-side dataman for the upcoming leg (no gap where the FCC has neither a
  setpoint stream nor a mission to fly).

## Interface deltas (ICD-MC-001.md)

- **§3.3 note at ICD-MC-001.md:325** — replace *"The MC never commands
  `OFFBOARD` (FR-012)"* with the conditional language above, and cross-ref
  the new mission-upload flag.
- **§3.3 PX4 custom-mode table** — add an `OFFBOARD` row. PX4 v1.14
  `px4_custom_mode.h` defines `PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6` with
  `custom_sub_mode = 0`; **verify against the actual v1.14 firmware source
  before committing this to the ICD** — flight-mode encoding is not
  something to get wrong from memory. (Worth also re-verifying the
  existing HOLD/MISSION/RTL/TAKEOFF rows at ICD-MC-001.md:316-321 while
  touching this table — TAKEOFF's `custom_main=2` looks inconsistent with
  the AUTO-submode pattern the other three rows use, but that's a
  pre-existing discrepancy, not introduced here.)
- **New message `mc_msgs/FccOffboardSetpoint`** (new file, mirrors the
  existing `FccMissionItem`/`FccCommand` pattern):
  ```
  # Continuous OFFBOARD position setpoint, published while an OFFBOARD leg
  # is active. NED frame, matches px4_msgs::msg::TrajectorySetpoint /
  # OffboardControlMode field layout on the XrceDdsBackend side.
  std_msgs/Header header
  float32 x            # N, m
  float32 y            # E, m
  float32 z            # D, m
  float32 yaw          # rad, NED
  ```
  (Velocity/acceleration fields deliberately omitted for the initial cut —
  formation-tracking only needs position+yaw, same as the existing
  `FORMATION_SLOT` mission-item-replace path it's superseding on
  OFFBOARD-flagged legs. Extend later if a use case needs velocity control.)
- **New topic** `/mc/fcc/offboard_setpoint` — `formation_controller_node` →
  `fcc_bridge_node`, ≥5 Hz while active, same reentrant-callback-group
  treatment FCC-005 already gives `/mc/fcc/mission_item_update` (must not
  starve `/mc/fcc/vehicle_state`).
- **Extend `mc_msgs/MissionUpload`** (ICD-MC-001.md:1026, still design-only
  — no `.msg` file exists yet) with:
  ```
  uint16 offboard_phase_mask   # bit N = Mission Phase ID N (§10.6) requires
                                # OFFBOARD for its FCC-facing legs; 0 = all
                                # MISSION mode (today's behavior)
  ```
- **§10.8 Fault Mask Bits** — bit 11 is next free (10 used by
  `GIMBAL_LINK_LOST`, 11–15 reserved). Add:
  | 11 | `OFFBOARD_STREAM_STALE` | OFFBOARD setpoint stream age > 500 ms while OFFBOARD active |
- **§3.4 FccBackend abstraction** — add to the interface described in ICD
  §3.4 (implemented in `ws/src/fcc_bridge/include/fcc_bridge/fcc_backend.hpp`):
  ```cpp
  // Stream a continuous OFFBOARD position setpoint. Caller (formation_
  // controller_node) is responsible for sustaining >=2 Hz while OFFBOARD
  // is the commanded mode; the backend does not buffer or extrapolate.
  virtual void stream_offboard_setpoint(const OffboardSetpoint & sp) = 0;
  ```
  `XrceDdsBackend` implements this by publishing `px4_msgs::msg::
  OffboardControlMode` (position=true, all else false) and `px4_msgs::msg::
  TrajectorySetpoint` on `/fmu/in/*` — reintroducing the exact publish path
  SDD-MC-001.md:259 describes as removed. `MavlinkUdpBackend` (SITL, still
  unimplemented — FCC-007) needs `SET_POSITION_TARGET_LOCAL_NED` streamed
  at the same rate; add this as a new FCC-007 acceptance criterion rather
  than a separate ticket, since FCC-007 hasn't started.

## SDD deltas (SDD-MC-001.md)

- **§3 channel design**: note that uXRCE-DDS now carries three things
  instead of two — `vehicle_state` (in), `vehicle_command` (out), and
  conditionally `offboard_setpoint` (out, ≥2 Hz, only while an
  OFFBOARD-flagged leg is active).
- **§4 mission_manager_node**: add mode-arbitration logic to phase entry —
  on entering each phase, read `offboard_phase_mask` for that phase ID; if
  set, sequence OFFBOARD entry (stream → confirm → `DO_SET_MODE`) before
  handing control to `formation_controller_node`; if clear, keep today's
  MISSION-mode upload/replace behavior unchanged.
- **§5 formation_controller_node**: becomes dual-mode. Keep the existing
  MISSION-mode `FORMATION_SLOT` replace path (SDD-MC-001.md:374) as the
  default; add an OFFBOARD path that publishes `FccOffboardSetpoint` at
  ≥5 Hz instead, selected by whichever mode `mission_manager_node` last
  commanded for the current phase. Self-supervision (already planned per
  SC-001/FR-024 for the MISSION-mode path) extends symmetrically: if the
  node's own `WallTimer` stalls while it's supposed to be streaming
  OFFBOARD setpoints, it self-commands HOLD exactly as it would for a
  stalled mission-item-replace, per the existing pattern.
- **§10.3 failsafe decision tree** (health_monitor): add
  `OFFBOARD_STREAM_STALE` → command MISSION-mode fallback if a mission is
  loaded, else HOLD — same "restart-then-fallback" shape as the existing
  `MANAGER_DEAD` branch, but the fallback here is a mode command, not a
  process restart.

---

## New/amended backlog tickets

Dependency order:

```
FCC-009 (FccBackend OFFBOARD interface + XrceDdsBackend impl)
  └──> SC-005 (formation_controller_node dual-mode: OFFBOARD stream / MISSION replace)
  └──> ME-003 (mission_manager_node mode arbitration)
              └──> HM-004 (OFFBOARD stream-staleness fault branch)

FCC-007 amended (MavlinkUdpBackend gets a SET_POSITION_TARGET_LOCAL_NED
  equivalent) — independent, no dependency on the above, gates SITL-side
  OFFBOARD testing only.
```

### FCC-009: FccBackend — OFFBOARD setpoint interface + XrceDdsBackend implementation

**Points:** 8

**As a** developer, **I want** `FccBackend` to expose an OFFBOARD setpoint
stream method backed by real `px4_msgs::msg::TrajectorySetpoint`/
`OffboardControlMode` publishing in `XrceDdsBackend`, **so that**
`formation_controller_node` can drive OFFBOARD-flagged legs without
touching transport-specific code, matching the existing abstraction
boundary FCC-001 established.

**Acceptance Criteria:**
- `FccBackend::stream_offboard_setpoint(const OffboardSetpoint&)` added to
  `fcc_backend.hpp`; existing `upload_mission()`/`update_mission_item()`
  doc comments' "MISSION mode only — no OFFBOARD setpoint path" claim is
  removed/corrected.
- `XrceDdsBackend` publishes `OffboardControlMode` (position=true) +
  `TrajectorySetpoint` to `/fmu/in/*` on each call, at whatever rate the
  caller invokes it — no internal buffering/extrapolation between calls
  (matches the existing `update_mission_item()` contract).
- Does **not** itself send `DO_SET_MODE OFFBOARD` — mode commands stay in
  `send_command()`, called by `mission_manager_node` (ME-003), so
  `FccBackend` doesn't need OFFBOARD-specific sequencing logic.
- `mc_msgs/FccOffboardSetpoint.msg` added; ICD-MC-001.md §3.3/§3.4/§7.2
  updated with the new message, method, and (pending firmware
  verification) the OFFBOARD custom-mode table row.

**Deliverables:**
- `fcc_backend.hpp` interface addition; `xrce_dds_backend.{hpp,cpp}`
  implementation.
- `mc_msgs/msg/FccOffboardSetpoint.msg`.
- Node integration test extending `test_fcc_bridge_node.cpp`: a published
  `FccOffboardSetpoint` reaches the backend and results in the expected
  `TrajectorySetpoint`/`OffboardControlMode` publish (mirrors FCC-005's
  existing mission-upload test pattern).

**Relationships:** No blockers (independent of FCC-005/006/mission
channel work). Blocks SC-005.

---

### SC-005: formation_controller_node — dual-mode control (MISSION replace / OFFBOARD stream)

**Points:** 8

**As a** developer, **I want** `formation_controller_node` to switch
between MISSION-mode `FORMATION_SLOT` replacement and OFFBOARD setpoint
streaming based on which mode `mission_manager_node` has commanded for the
current phase, **so that** OFFBOARD-flagged legs get tighter tracking
without changing behavior on legs that don't need it.

**Acceptance Criteria:**
- Subscribes to the active flight-mode state (from ME-003, e.g.
  `/mc/mission_phase` plus the commanded mode, or a dedicated
  `/mc/fcc/active_control_mode` — exact wiring TBD with ME-003).
- In MISSION mode: unchanged from SC-001 — replaces `FORMATION_SLOT` at
  ≥5 Hz via `update_mission_item()`.
- In OFFBOARD mode: publishes `mc_msgs/FccOffboardSetpoint` at ≥5 Hz via
  `stream_offboard_setpoint()`, same PD math (SDD §5.2) feeding a different
  output path — no duplication of the leader-tracking/NED-frame logic
  between the two modes.
- Self-supervision extends symmetrically: a stalled `WallTimer` in OFFBOARD
  mode triggers the same self-HOLD behavior SC-001 already defines for a
  stalled MISSION-mode replace (FR-024).

**Deliverables:**
- `formation_controller_node.{hpp,cpp}` extended with the OFFBOARD path and
  mode-select logic.
- Node integration test: forcing OFFBOARD mode → `FccOffboardSetpoint`
  published at expected rate with correct PD output; forcing MISSION mode
  → existing SC-001 behavior unchanged; self-HOLD fires in both modes on
  stall.

**Relationships:** Depends on FCC-009, SC-001 (base PD controller), SC-002
(peer/leader state). Depends on ME-003 for the mode signal.

---

### ME-003: mission_manager_node — flight-mode arbitration

**Points:** 5

**As a** developer, **I want** `mission_manager_node` to read the
`offboard_phase_mask` from the active mission plan and sequence FCC
flight-mode transitions accordingly, **so that** OFFBOARD is only ever
active where the mission plan explicitly calls for it, entered/exited
safely.

**Acceptance Criteria:**
- On each phase entry, checks `offboard_phase_mask` (from the
  `MissionUpload` that produced the active plan) for the entering phase's
  bit.
- If set: commands `formation_controller_node` into OFFBOARD-stream mode,
  waits for ≥1 s of confirmed `FccOffboardSetpoint` publication, *then*
  calls `send_command(MAV_CMD_DO_SET_MODE, OFFBOARD)`.
- If clear (today's behavior): unchanged MISSION-mode upload/replace flow
  from ME-001.
- On phase exit from an OFFBOARD-flagged phase: confirms a valid mission is
  loaded for the next leg *before* commanding `DO_SET_MODE MISSION` and
  signaling `formation_controller_node` to stop streaming — no gap with
  neither a stream nor a mission active.

**Deliverables:**
- `mission_manager_node.{hpp,cpp}` extended with mode-arbitration/sequencing
  logic, layered on top of ME-001's phase state machine.
- Node integration test: a plan with `offboard_phase_mask` set for INGRESS
  → correct entry sequencing (stream-then-switch) and correct exit
  sequencing (mission-loaded-then-switch); a plan with the mask clear →
  ME-001's existing MISSION-only behavior, byte-for-byte unchanged.

**Relationships:** Depends on ME-001 (base state machine), FCC-009. Blocks
HM-004 (needs the mode-arbitration signal to know when
`OFFBOARD_STREAM_STALE` is even meaningful to check).

---

### HM-004: health_monitor_node — OFFBOARD stream-staleness fault branch

**Points:** 3

**As a** developer, **I want** `health_monitor_node` to detect when the
OFFBOARD setpoint stream goes stale while OFFBOARD is the FCC's commanded
mode, **so that** the MC proactively falls back to MISSION/HOLD instead of
relying on the FCC's own `COM_OBL_ACT` failsafe.

**Acceptance Criteria:**
- Tracks `/mc/fcc/offboard_setpoint` liveness only while ME-003 reports
  OFFBOARD as the active commanded mode (no false trigger while a phase
  legitimately stays in MISSION mode and the topic is idle).
- Stream age > 500 ms while active → publish `/mc/fault` with
  `OFFBOARD_STREAM_STALE` and `failsafe_action` = fallback-to-MISSION (if a
  mission is loaded) else HOLD — extends the HM-003 decision tree, same
  edge-triggered (no re-fire while persisting) behavior as its other
  branches.

**Deliverables:**
- `health_monitor_node.{hpp,cpp}` extended with the new liveness check and
  decision-tree branch.
- Node integration test: OFFBOARD active + stream stalls >500 ms →
  `OFFBOARD_STREAM_STALE` fault + correct fallback action; same stall while
  OFFBOARD is *not* the active mode → no fault.

**Relationships:** Depends on HM-002/HM-003 (existing decision tree), ME-003
(mode-arbitration signal).

---

## Explicitly out of scope for this proposal

- Any change to TAKEOFF, ARMING, PREFLIGHT, VORONOI_PARTITION, SEARCH,
  RETURN, LAND, FAULT — these stay MISSION-mode/command-only unless a
  future mission plan flags them too; the mechanism is general, this
  proposal doesn't itself extend OFFBOARD to those phases.
- Implementing `MavlinkUdpBackend` (FCC-007) — only amending its future
  acceptance criteria to include OFFBOARD parity; the backend itself is
  still unstarted.
- Exact PX4 v1.14 custom-mode enum values — flagged above for verification
  against firmware source before the ICD table is actually edited.
- Renumbering `ME-*`/`SC-*`/`HM-*`/`FCC-*` to match whatever the real
  ticket tracker uses (same caveat `FCC-MISSION-CHANNEL-BACKLOG.md` already
  carries).
