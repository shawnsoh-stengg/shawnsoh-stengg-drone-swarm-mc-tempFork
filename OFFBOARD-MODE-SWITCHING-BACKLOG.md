# MC↔FCC Flight-Mode Switching (MISSION ⇄ OFFBOARD) — Proposal

**Status:** Draft for review. Nothing in this doc has been applied to
SRS-MC-001.md / ICD-MC-001.md / SDD-MC-001.md yet — it's a proposal to
amend them, written the same way `FCC-MISSION-CHANNEL-BACKLOG.md` was
before FCC-005/006 landed.

**Revision note (2026-07-20):** revised after design review to close two
soundness gaps in the original draft: (1) nothing confirmed the FCC actually
entered/exited OFFBOARD before downstream logic assumed it had, and (2)
`health_monitor_node` and `mission_manager_node` both had independent
authority to command a flight-mode change, with no arbitration rule and no
shared source for the "is a mission loaded" check. See design decisions 4-5
below and the inline changes throughout.

**Revision note (2026-07-20, applied):** the requirement/interface/SDD
deltas above have now been applied to `SRS-MC-001.md` (FR-012/020 amended,
FR-026–029 and SFR-009 added, §4.1.3a), `ICD-MC-001.md` (OFFBOARD custom-mode
row, `mc_msgs/FccOffboardSetpoint`, `/mc/fcc/offboard_setpoint`,
`offboard_phase_mask`, fault bit 11), and `SDD-MC-001.md` (§4.1a, §5.3a,
§10.3 branch) — this doc's "Requirement/Interface/Parameter/SDD deltas"
sections are now historical record of *why*, not a pending to-do. FCC-009
below was corrected accordingly (its ICD section references were wrong —
§9.1/§9.2, not §7.2, which is the gimbal-camera topic section) and to reflect
that `MavlinkUdpBackend` (FCC-007) is no longer "still unimplemented" as
originally assumed — it shipped, which changes FCC-009's scope: adding a new
pure-virtual method to `FccBackend` now requires implementing it in both
concrete backends or the workspace stops compiling.

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
3. Stream-staleness *detection* is MC-driven (health_monitor watchdog), not
   left to PX4's `COM_OBL_ACT` parameter as the primary fallback trigger.
4. `mission_manager_node` is the **sole** flight-mode command authority.
   `health_monitor_node` never calls `send_command()` for a flight-mode
   change itself — on stream staleness it only publishes `/mc/fault`
   (`OFFBOARD_STREAM_STALE`); `mission_manager_node` executes the resulting
   fallback through the same exit-sequencing path it already uses for a
   normal OFFBOARD→MISSION phase exit. This avoids two nodes racing to
   command the FCC's mode at the same time, and avoids duplicating the
   "is a mission loaded" check in `health_monitor_node` when
   `mission_manager_node` already owns it.
5. PX4's own `COM_OBL_ACT`/`COM_OBL_RT_ACT` OFFBOARD-loss failsafe stays
   **enabled**, but configured with a timeout longer than the MC's 500 ms
   watchdog (see Parameter deltas below) — a silent backstop for an
   unresponsive MC software stack, not a competing primary path.

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
  active. The MC shall confirm entry by observing
  `/mc/fcc/vehicle_state.nav_state == OFFBOARD` (§10.1) within 1 s of
  issuing the command; if `nav_state` has not transitioned within that
  window, the MC shall retry the mode command once, and if still
  unconfirmed, abort the OFFBOARD leg, remain in the vehicle's current
  mode, and raise fault `OFFBOARD_ENTRY_FAILED` rather than proceeding as if
  OFFBOARD were active. `formation_controller_node` shall not be signaled
  to treat OFFBOARD as the active mode until this confirmation lands —
  closes the gap where PX4 silently rejects the mode change (e.g. a failed
  pre-arm/EKF check) and the MC keeps streaming setpoints PX4 never acts on.
- **New FR (OFFBOARD setpoint rate)**: while OFFBOARD is active,
  `formation_controller_node` shall publish `/mc/fcc/offboard_setpoint` at
  ≥5 Hz (tracking-quality target, mirrors FR-023's existing cadence),
  never below the PX4-imposed 2 Hz floor.
- **New SFR (stream-staleness fallback)**: if the OFFBOARD setpoint stream
  age exceeds 500 ms while OFFBOARD is the FCC's *confirmed* active mode
  (mirrors the existing 500 ms GCS-override response bound, FR-003),
  `health_monitor_node` shall publish `/mc/fault` with
  `OFFBOARD_STREAM_STALE` and a recommended `failsafe_action` of
  fallback-to-MISSION-or-HOLD. `health_monitor_node` shall **not** call
  `send_command()` itself. `mission_manager_node` shall treat this fault as
  a phase-exit trigger and execute the exit-sequencing FR above
  (mission-loaded check → `DO_SET_MODE MISSION`, or HOLD if no valid
  mission is loaded → confirm via `nav_state`) — the same code path used for
  a normal phase exit, not a separate implementation. This keeps
  `mission_manager_node` as the sole flight-mode command authority (design
  decision 4) and avoids duplicating the "is a mission loaded" check in
  `health_monitor_node`. PX4's own `COM_OBL_ACT` failsafe remains enabled as
  a slower backstop — see Parameter deltas.
- **New FR (exit sequencing)**: before commanding `DO_SET_MODE MISSION` to
  leave OFFBOARD, the MC shall confirm a valid mission is already loaded in
  FCC-side dataman for the upcoming leg (no gap where the FCC has neither a
  setpoint stream nor a mission to fly). After issuing the command, the MC
  shall confirm exit the same way entry is confirmed — observing
  `/mc/fcc/vehicle_state.nav_state == MISSION` within 1 s, with the same
  retry-once-then-fault behavior as entry — before `formation_controller_node`
  is told to stop streaming; if unconfirmed, keep streaming rather than
  stopping into a mode that never actually changed. This exit path is the
  **single** code path used both for a normal ME-003-driven phase exit and
  for the `health_monitor_node`-triggered stream-staleness fallback below —
  there is no second, independent fallback-execution path.

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

## Parameter deltas (PX4 side)

- **`COM_OBL_ACT`** (OFFBOARD-loss failsafe action) and the associated
  timeout param (`COM_OBL_RT_ACT` or equivalent — verify exact v1.14 param
  name against firmware source alongside the custom-mode verification
  above) stay **enabled**, not disabled. Set the timeout clearly longer
  than the MC's 500 ms staleness threshold (proposed: 1.5-2 s) so the
  MC-driven fallback (SFR above) always fires first under normal
  conditions; PX4's own failsafe becomes a backstop only for the case
  where `health_monitor_node` or the whole MC software stack is
  unresponsive, not a second primary path racing the MC's watchdog.
  Disabling it outright would remove the FCC's last line of defense if the
  companion computer itself dies mid-OFFBOARD leg — exactly the scenario
  this parameter exists to cover.
- Record the chosen value alongside the other PX4 param overrides for this
  airframe (wherever those already live — e.g. `HID-MC-001.md` — this is a
  Pixhawk param, not MC-side YAML config) and in the ICD's PX4 custom-mode
  table alongside the new `OFFBOARD` row.

## SDD deltas (SDD-MC-001.md)

- **§3 channel design**: note that uXRCE-DDS now carries three things
  instead of two — `vehicle_state` (in), `vehicle_command` (out), and
  conditionally `offboard_setpoint` (out, ≥2 Hz, only while an
  OFFBOARD-flagged leg is active).
- **§4 mission_manager_node**: add mode-arbitration logic to phase entry —
  on entering each phase, read `offboard_phase_mask` for that phase ID; if
  set, sequence OFFBOARD entry (stream ≥2 Hz for 1 s → `DO_SET_MODE
  OFFBOARD` → confirm `/mc/fcc/vehicle_state.nav_state == OFFBOARD` within
  1 s, one retry of the command on timeout, else abort the leg and raise
  `OFFBOARD_ENTRY_FAILED`) before handing control to
  `formation_controller_node`; if clear, keep today's MISSION-mode
  upload/replace behavior unchanged. `mission_manager_node` is the sole
  node that calls `send_command()` for a flight-mode change (design
  decision 4) — its exit-sequencing logic (mission-loaded check →
  `DO_SET_MODE MISSION` → confirm `nav_state == MISSION`) is a single
  shared code path invoked both on a normal phase exit and on an
  `OFFBOARD_STREAM_STALE` fault from `health_monitor_node`, not two
  independent implementations.
- **§5 formation_controller_node**: becomes dual-mode. Keep the existing
  MISSION-mode `FORMATION_SLOT` replace path (SDD-MC-001.md:374) as the
  default; add an OFFBOARD path that publishes `FccOffboardSetpoint` at
  ≥5 Hz instead, selected by whichever mode `mission_manager_node` last
  *confirmed* (not merely commanded) for the current phase — the node only
  begins streaming/self-supervising once `mission_manager_node` signals
  confirmed OFFBOARD entry. Self-supervision (already planned per
  SC-001/FR-024 for the MISSION-mode path) extends symmetrically: if the
  node's own `WallTimer` stalls while it's supposed to be streaming
  OFFBOARD setpoints, it self-commands HOLD exactly as it would for a
  stalled mission-item-replace, per the existing pattern.
- **§10.3 failsafe decision tree** (health_monitor): add
  `OFFBOARD_STREAM_STALE` → **publish** `/mc/fault` with a recommended
  fallback-to-MISSION-if-loaded-else-HOLD action; `health_monitor_node`
  does not itself command a mode change. `mission_manager_node` consumes
  this fault and executes the fallback through the shared exit-sequencing
  path described in §4 above — same "restart-then-fallback" shape as the
  existing `MANAGER_DEAD` branch, but execution stays with
  `mission_manager_node` rather than `health_monitor_node` reaching into
  the FCC directly, so there is exactly one node with mode-command
  authority at all times.
- **New §(PX4 params)**: `COM_OBL_ACT`/`COM_OBL_RT_ACT` (verify exact param
  name against v1.14 firmware source) stay enabled with a timeout longer
  than the MC's 500 ms watchdog (proposed 1.5-2 s) — a backstop for an
  unresponsive MC, not a competing primary path. See Parameter deltas
  below.

---

## New/amended backlog tickets

Dependency order:

```
FCC-009 (FccBackend OFFBOARD interface + XrceDdsBackend AND
         MavlinkUdpBackend impls — both required, FccBackend is abstract)
  └──> SC-005 (formation_controller_node dual-mode: OFFBOARD stream / MISSION replace)
  └──> ME-003 (mission_manager_node mode arbitration)
              └──> HM-004 (OFFBOARD stream-staleness fault branch)
```

(Previously listed a separate "FCC-007 amended" entry for
`MavlinkUdpBackend`'s `SET_POSITION_TARGET_LOCAL_NED` equivalent, described
as independent and SITL-testing-only. That assumed FCC-007 hadn't started.
It has — `MavlinkUdpBackend` is a shipped concrete subclass of `FccBackend`
today, so adding `stream_offboard_setpoint()` as a new pure-virtual method
is not optional/independent: every concrete backend must implement it for
the workspace to compile. Folded into FCC-009 below.)

### FCC-009: FccBackend — OFFBOARD setpoint interface + XrceDdsBackend/MavlinkUdpBackend implementations

**Points:** 10 (was 8 — raised to cover the `MavlinkUdpBackend` implementation
folded in below, which the original estimate assumed was someone else's
problem via a separate "FCC-007 amended" ticket)

**As a** developer, **I want** `FccBackend` to expose an OFFBOARD setpoint
stream method backed by real publishing in **both** concrete backends,
**so that** `formation_controller_node` can drive OFFBOARD-flagged legs
without touching transport-specific code, matching the existing abstraction
boundary FCC-001 established — and so the workspace keeps compiling, since
`FccBackend` is an abstract class and both `XrceDdsBackend` and
`MavlinkUdpBackend` are shipped concrete subclasses today
(`ws/src/fcc_bridge/src/xrce_dds_backend.cpp`,
`ws/src/fcc_bridge/src/mavlink_udp_backend.cpp`).

**Acceptance Criteria:**
- `FccBackend::stream_offboard_setpoint(const OffboardSetpoint&)` added as a
  pure-virtual method to `fcc_backend.hpp`; the `upload_mission()` doc
  comment's "there is no OFFBOARD setpoint path" claim (currently
  `fcc_backend.hpp:28`) is removed/corrected to point at the conditional
  design (SDD §4.1a/§5.3a).
- `XrceDdsBackend` publishes `px4_msgs::msg::OffboardControlMode`
  (position=true, all else false) + `px4_msgs::msg::TrajectorySetpoint` to
  `/fmu/in/*` on each call, at whatever rate the caller invokes it — no
  internal buffering/extrapolation between calls (matches the existing
  `update_mission_item()` contract in the same file).
- `MavlinkUdpBackend` publishes `SET_POSITION_TARGET_LOCAL_NED` to the known
  SITL peer on each call, mirroring `send_command()`'s existing
  peer-address/`ensure`-pattern in the same file: if no peer has been seen
  yet (`peer_known_` false), log a warning and drop the call rather than
  throwing — same convention `send_command()` already uses for "no SITL peer
  yet."
- Neither backend itself sends `DO_SET_MODE OFFBOARD` — mode commands stay
  in `send_command()`, called by `mission_manager_node` (ME-003), so
  `FccBackend` doesn't need OFFBOARD-specific sequencing logic.
- `FccBackendContract`'s `FakeBackend`
  (`ws/src/fcc_bridge/test/test_fcc_backend_contract.cpp`) gets a
  `stream_offboard_setpoint()` override and a new
  `StreamOffboardSetpointReachesImplementationThroughBasePointer`-style test,
  mirroring the existing per-method coverage in that file — otherwise the
  contract test target itself fails to compile once the pure-virtual method
  is added.
- `mc_msgs/FccOffboardSetpoint.msg` added, registered in
  `ws/src/mc_msgs/CMakeLists.txt`'s `rosidl_generate_interfaces()` call.
  `ICD-MC-001.md` §3.3 (custom-mode table/note), §3.4 (`FccBackend`
  abstraction table + method description), §9.1 (topic catalogue), and
  §9.2 (message definition) are **already updated** (applied 2026-07-20,
  see revision note above) — this AC item is now "confirm the shipped
  `.msg` file matches §9.2's definition," not "update the ICD."

**Deliverables:**
- `fcc_backend.hpp` interface addition.
- `xrce_dds_backend.{hpp,cpp}` implementation.
- `mavlink_udp_backend.{hpp,cpp}` implementation.
- `mc_msgs/msg/FccOffboardSetpoint.msg` + `mc_msgs/CMakeLists.txt` entry.
- Node integration test extending `test_fcc_bridge_node.cpp`: a published
  `FccOffboardSetpoint` reaches the backend and results in the expected
  `TrajectorySetpoint`/`OffboardControlMode` publish (mirrors FCC-005's
  existing mission-upload test pattern) — plus the `MavlinkUdpBackend`-side
  equivalent in `test_mavlink_udp_backend.cpp` (mirrors that file's existing
  `send_command()`/no-peer-yet coverage).
- `test_fcc_backend_contract.cpp` update per the AC above.

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
  calls `send_command(MAV_CMD_DO_SET_MODE, OFFBOARD)`, then waits for
  `/mc/fcc/vehicle_state.nav_state == OFFBOARD` (timeout 1 s, one retry of
  the command on timeout, else abort the leg and raise
  `OFFBOARD_ENTRY_FAILED`). `formation_controller_node` is not told
  OFFBOARD is "active" for self-supervision purposes until this
  confirmation lands.
- If clear (today's behavior): unchanged MISSION-mode upload/replace flow
  from ME-001.
- On phase exit from an OFFBOARD-flagged phase — including a
  `health_monitor_node`-triggered `OFFBOARD_STREAM_STALE` fault, which
  drives this *same* code path, not a separate one — confirms a valid
  mission is loaded for the next leg *before* commanding `DO_SET_MODE
  MISSION`, confirms the switch via `nav_state == MISSION`, and only then
  signals `formation_controller_node` to stop streaming — no gap with
  neither a stream nor a mission active, and no window where the MC's
  bookkeeping says "back in MISSION" while `nav_state` says otherwise.
- Is the **only** node in the system that calls `send_command()` for a
  flight-mode change; `health_monitor_node` (HM-004) publishes a fault and
  nothing else.

**Deliverables:**
- `mission_manager_node.{hpp,cpp}` extended with mode-arbitration/sequencing
  logic, layered on top of ME-001's phase state machine.
- Node integration test: a plan with `offboard_phase_mask` set for INGRESS
  → correct entry sequencing (stream-then-switch-then-confirm) and correct
  exit sequencing (mission-loaded-then-switch-then-confirm); a plan with the
  mask clear → ME-001's existing MISSION-only behavior, byte-for-byte
  unchanged; a simulated `nav_state` that never transitions to `OFFBOARD`
  after the mode command → one retry, then abort-and-fault, never silently
  treated as success; a simulated `OFFBOARD_STREAM_STALE` fault from
  `health_monitor_node` → exit-sequencing path runs exactly as a normal
  phase exit would.

**Relationships:** Depends on ME-001 (base state machine), FCC-009. Blocks
HM-004 (needs the mode-arbitration signal to know when
`OFFBOARD_STREAM_STALE` is even meaningful to check).

---

### HM-004: health_monitor_node — OFFBOARD stream-staleness fault branch

**Points:** 3

**As a** developer, **I want** `health_monitor_node` to detect when the
OFFBOARD setpoint stream goes stale while OFFBOARD is the FCC's *confirmed*
active mode, **so that** the MC proactively triggers a fallback to
MISSION/HOLD instead of relying on the FCC's own `COM_OBL_ACT` failsafe as
the primary path.

**Acceptance Criteria:**
- Tracks `/mc/fcc/offboard_setpoint` liveness only while ME-003 reports
  OFFBOARD as the **confirmed** active mode (i.e. after ME-003's own
  `nav_state == OFFBOARD` confirmation, not merely "commanded") — no false
  trigger while a phase legitimately stays in MISSION mode, the topic is
  idle, or an OFFBOARD entry is still being confirmed/retried.
- Stream age > 500 ms while active → publish `/mc/fault` with
  `OFFBOARD_STREAM_STALE` and a recommended `failsafe_action` of
  fallback-to-MISSION-or-HOLD — extends the HM-003 decision tree, same
  edge-triggered (no re-fire while persisting) behavior as its other
  branches.
- **Does not** call `send_command()` or otherwise command the FCC directly.
  `mission_manager_node` (ME-003) is the sole flight-mode command authority
  (design decision 4); `health_monitor_node`'s job ends at publishing the
  fault. This avoids `health_monitor_node` needing its own copy of the
  "is a mission loaded" check that ME-003 already owns, and avoids two
  nodes racing to change the FCC's mode.

**Deliverables:**
- `health_monitor_node.{hpp,cpp}` extended with the new liveness check and
  decision-tree branch (fault-publish only).
- Node integration test: OFFBOARD confirmed-active + stream stalls >500 ms
  → `OFFBOARD_STREAM_STALE` fault published (and, in a combined test with
  ME-003, that fault correctly drives ME-003's exit-sequencing path); same
  stall while OFFBOARD is commanded but not yet confirmed → no fault; same
  stall while OFFBOARD is not the active mode → no fault.

**Relationships:** Depends on HM-002/HM-003 (existing decision tree), ME-003
(mode-confirmation signal and exit-sequencing execution).

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
