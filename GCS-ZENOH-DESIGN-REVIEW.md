# GCS ↔ MC Communication: MAVLink vs. Zenoh Design Review

Design review evaluating whether `zenoh-bridge-dds` (added 2026-07-22 for
cross-host MC↔MC DDS transport, see `ZBD-001`/`DEP-006` in
`PROJECT-PLAN-5-MONTH.md`) could also replace MAVLink for GCS↔MC
communication — data exchange, mission/system status, and mission plan
upload/update.

**Revision note (2026-07-22, corrected):** the initial pass of this review
assumed "GCS" meant a single actor (stock QGroundControl) and flagged an
undocumented gap around who renders the MC-specific custom MAVLink traffic.
Clarified same day: **GCS is actually two separate actors** — QGroundControl
(vehicle telemetry/control, tied to the FCC) and a **Mission Orchestrator**
app (mission plan creation/upload/update, built by another team, out of
scope for this project to implement). This resolves the mission-upload part
of the original flagged risk — see "Two GCS-side actors" below — and
reopens the Zenoh question specifically for that interface, since the
Orchestrator isn't QGC-constrained. The system/status-telemetry half of the
original flagged risk (`MC_STATUS`/`MC_DETECTION`/`MC_HEALTH` — not
mentioned as part of the Orchestrator's scope) remains open and unresolved;
see Flagged Risk below.

## Why this doc exists

Requested 2026-07-22 as a system design review. Not prompted by an incident
or a specific requirement gap — a natural follow-on question once
`zenoh-bridge-dds` existed for the MC↔MC leg: does the same rationale
(static-peer mesh, no reliance on DDS's own discovery, better fit for a
lossy/bandwidth-constrained MANET link) extend to the GCS↔MC leg too?

## Two GCS-side actors

"GCS" is not one application:

- **QGroundControl** — stock, unmodified, tied to vehicle telemetry/control
  to/from the FCC. GCS software is explicitly out of scope (SRS §1.2), and
  QGC is the actor that exclusion is actually about — everything in the
  original review's "Channel B traffic breakdown" involving standard
  MAVLink (arm/disarm, mode, `HEARTBEAT`, `STATUSTEXT`, `PARAM_*`) is QGC's
  domain and is unaffected by anything below.
- **Mission Orchestrator** — a separate application responsible for mission
  plan creation, upload, and update across a *library* of named plans (not
  today's single-active-mission model). Built by another team; **out of
  scope for this project to implement.** This project's deliverable is the
  **interface specification** that team builds against — see
  `MISSION-ORCHESTRATOR-INTERFACE-SPEC.md`, which defines that interface as
  Zenoh-native (reusing `mc_msgs` schemas over the existing
  `zenoh-bridge-dds` mesh), decided 2026-07-22 precisely because the
  Orchestrator, unlike QGC, has no MAVLink/QGC-compatibility constraint.

## Question evaluated

Can MAVLink (ICD-MC-001.md §4, IFD-002) be replaced by Zenoh for:
1. General data exchange between GCS and MC.
2. Mission/system status information (telemetry, health, detections).
3. Uploading/updating mission plans.

## Current architecture

### What the core docs say about GCS today (QGC-only — predates the Orchestrator split)

SRS-MC-001.md §1.2 states explicitly: **"Out of scope: GCS software, FCC
firmware (PX4), MANET radio firmware, airframe design, battery management
electronics."** §1.3 defines GCS as "Ground Control Station —
QGroundControl laptop at 192.168.10.10" — i.e. stock QGC, not a custom
application. This is reinforced consistently across diagrams 01, 03, 08,
and 21 (`mock_gcs_node`'s test strategy models the real GCS as
"QGroundControl on GCS laptop (real GCS software)"). None of these docs
mention a Mission Orchestrator yet — that's new context from this review's
2026-07-22 correction, not yet folded into SRS/ICD/SDD themselves.

GCS never joins the ROS 2/DDS domain today: `gcs_mavlink_node` binds a
loopback-only UDP socket (`ws/src/gcs_comms/src/gcs_mavlink_node.cpp`,
`open_loopback_socket()`), fed by `mavlink-router-channel-b.conf`'s
`GCS_MAVLINK_NODE` endpoint at `127.0.0.1:14560` (ICD §4.2). All GCS↔MC
traffic is MAVLink 2/UDP end to end.

### Channel B traffic breakdown

| Traffic | Message type | QGC-native? |
|---|---|---|
| Arm/disarm, mode, `HEARTBEAT`, `STATUSTEXT`, `PARAM_*` (ICD §4.3/4.4) | Standard MAVLink `common.xml` | **Yes** — QGC's stock UI renders these with zero extra code |
| `MC_STATUS` (50100), `MC_DETECTION` (50101), `MC_HEALTH` (50103) (ICD §4.6) | Custom dialect, user-defined ID range 50100–50199, defined in `mc_mavlink_defs.xml` | **No** |
| `MC_MISSION_UPLOAD` (50102) mission items, codes 50200–50204, `MAV_MISSION_TYPE=3` (ICD §4.6) | Custom mission type riding the standard mission *protocol* | **No** |

SDD-MC-001.md §11.1 confirms the MC side needs a custom MAVROS plugin to
handle `MC_STATUS`/`MC_DETECTION`/`MC_HEALTH`: *"gcs_mavlink_node uses
MAVROS as a library ... Custom messages ... are implemented as a MAVROS
plugin."* **No equivalent QGC-side plugin is documented anywhere** in
ICD/SDD/PROJECT-PLAN-5-MONTH.md — see Flagged Risk below.

### Mission plan upload — two legs, two different answers

- **GCS→MC (superseded by the Mission Orchestrator interface):** the
  original `MC_MISSION_UPLOAD`-over-MAVLink design (`MISSION_COUNT`→
  `MISSION_REQUEST_INT`→`MISSION_ITEM_INT`×N→`MISSION_ACK`, `MAV_MISSION_TYPE=3`,
  item codes 50200–50204) is **not** the path forward for this leg — it was
  never QGC-native anyway (same custom-dialect situation as the
  status/detection messages), and the actual owner of mission plan
  creation/upload/update is the Mission Orchestrator, not QGC. See
  `MISSION-ORCHESTRATOR-INTERFACE-SPEC.md` for the Zenoh-native replacement.
- **MC→FCC:** genuine standard MAVLink mission protocol over
  `192.168.12.31:14555` (ICD §3.2/§3.3, IR-007), because **"stock PX4 has
  no DDS-writable mission topic."** This leg is locked to MAVLink by PX4
  firmware itself — also explicitly out of scope (SRS §1.2) — and is
  unaffected by anything decided about the GCS leg.

### Link characteristics

GCS, FCC Channel A, and peer-drone traffic all share the same physical
`192.168.10.0/24` MANET mesh (diagram 25), under the same assumed link
model: **"maximum link latency ≤ 500 ms and loss rate < 5%"** (SRS-MC-001.md
§7, non-functional assumptions) — the identical characterization used to
justify `zenoh-bridge-dds` over CycloneDDS's own discovery for MC↔MC
traffic (SRS SW-008: *"the bandwidth-constrained, lossy MANET link"*).
Nothing about the link itself favors MAVLink over Zenoh.

## Analysis

Splitting the traffic in two:

**Standard vehicle telemetry/command** (arm, mode, `HEARTBEAT`,
`STATUSTEXT`, params) is genuinely MAVLink-dependent — this is what gives
QGC its native map/telemetry/mission-planner UI with zero custom code on
either end. Replacing it with Zenoh would mean replacing or extending QGC,
which is GCS software and explicitly out of scope. **This can't move to
Zenoh without a scope change the project has already ruled out.**

**MC-specific status/detection/health traffic** (`MC_STATUS`/`MC_DETECTION`/
`MC_HEALTH`), however, already isn't in QGC's native surface — it rides a
custom dialect (`mc_mavlink_defs.xml`) that QGC cannot decode without an
undocumented GCS-side plugin. This traffic gets **no actual benefit today**
from being MAVLink: the project is already paying to maintain a bespoke
wire format (custom XML dialect) for data that's also defined once already
as `mc_msgs` ROS 2 types — effectively two schemas for the same
information. Unlike mission upload, though, no second GCS-side actor has
been named as owning this data yet, so it's not yet clear who, if anyone,
would run the Zenoh session on the receiving end — see Flagged Risk.

**Mission plan creation/upload/update** was in the same position as the
status/detection/health traffic in the original pass of this review — but
is now resolved differently: the Mission Orchestrator, not QGC, owns this
traffic, and has no MAVLink-compatibility constraint. `mc_msgs` reuse over
Zenoh is a clean fit here; full design in
`MISSION-ORCHESTRATOR-INTERFACE-SPEC.md`.

**Mission plan upload's MC→FCC leg** is locked to MAVLink by PX4 firmware
(no DDS-writable mission topic), independent of anything decided here.

**Link engineering is a non-issue.** Same MANET link, same latency/loss
assumptions already used to justify Zenoh for MC↔MC. There's no bandwidth-
or reliability-based argument for preferring MAVLink over Zenoh — the
constraint is purely "must interoperate with unmodified QGC."

## Verdict

**Split verdict, now that GCS is understood as two actors:**

| Traffic | Verdict |
|---|---|
| Standard vehicle telemetry/command (QGC) | Stays MAVLink — permanently, as long as QGC is the GCS. Not Zenoh-eligible; QGC has no Zenoh/DDS support and is out of scope to modify. |
| MC-specific status/detection/health (`MC_STATUS`/`MC_DETECTION`/`MC_HEALTH`) | **Still unresolved** — not confirmed as the Mission Orchestrator's responsibility, and no QGC-side plugin is documented either. See Flagged Risk below. |
| Mission plan creation/upload/update | **Resolved — moves to Zenoh.** Owned by the Mission Orchestrator (not QGC), which has no MAVLink-compatibility constraint. See `MISSION-ORCHESTRATOR-INTERFACE-SPEC.md`. |
| Mission upload, MC→FCC leg | Hard-locked to MAVLink by PX4 firmware (no DDS-writable mission topic) — unrelated to this decision, unaffected by anything above. |

For standard vehicle telemetry/command: not because Zenoh is technically
unsuited to the link (the opposite — the same MANET-link rationale that
justified `zenoh-bridge-dds` for MC↔MC applies equally well here), but
because QGC itself has no Zenoh/DDS support and is out of scope to change.

## Flagged risk — not a ticket, needs a scope decision first

**Narrowed by the 2026-07-22 correction** to just the status/telemetry
half of the original concern (the mission-upload half is now resolved via
the Mission Orchestrator interface). **No QGC-side plugin/dialect for
`MC_STATUS`/`MC_DETECTION`/`MC_HEALTH` is documented anywhere** in
ICD/SDD/PROJECT-PLAN-5-MONTH.md, even though the MC side's custom MAVROS
plugin is (SDD §11.1). As designed, it's unclear how an operator's stock,
unmodified QGC would actually render this data today — and it wasn't named
as part of the Mission Orchestrator's scope either (that app's stated scope
is mission plan creation/upload/update only). This isn't something to fix
by just adding a ticket to `PROJECT-PLAN-5-MONTH.md` — building a QGC
plugin is itself GCS software, which is the exact category SRS §1.2
excludes. Whoever owns the GCS-side deliverable needs to decide: a QGC
plugin (a scope exception), folding status/telemetry into the Mission
Orchestrator's own scope too, a third companion app, or this traffic simply
isn't visualized by anything today. Flagging for that decision, not
actioning it here.
