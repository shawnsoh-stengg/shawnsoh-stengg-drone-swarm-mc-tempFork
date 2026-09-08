# Introduction

Welcome to `swarm-drones`. This doc is the front door: what this project
is, why it exists, the jargon you'll hit in your first week, and which of
the other docs to read next and in what order. It deliberately doesn't
duplicate the technical depth of `docs/ARCHITECTURE.md` or the build steps
in `docs/GETTING-STARTED.md` — it just gets you oriented enough to use
them.

## What we're building

A **swarm of autonomous drones** that fly a coordinated search-and-rescue
(ISR) mission with minimal human input. Each drone carries an identical
onboard computer — the **mission computer (MC)** — that:

1. Talks to its own flight controller to fly the aircraft.
2. Talks to its peer drones' MCs to coordinate: agree who leads, split up
   the search area, avoid re-covering ground.
3. Talks to a human operator at a ground control station (GCS), who can
   always override or take back control.
4. Runs onboard perception (camera + AI) to detect and geolocate targets
   as it flies, and streams video/detections back to the GCS.

The GCS operator hands the swarm a search area; the drones take off,
elect a coordinator among themselves, carve up the area (Voronoi
partition), fly a lawnmower ("boustrophedon") search pattern over their
assigned cell, and report anything they find — all while the GCS retains
override authority at all times, by design and by requirement (FR-003:
GCS commands must take effect within 500 ms).

This repository is the **mission computer software** — not the flight
controller firmware (that's stock PX4) and not the GCS application (that's
QGroundControl plus a purpose-built Mission Orchestrator tool). It's the
layer in between that makes a *swarm* out of individually-flyable drones.

## Where things stand today

Design is complete (see the formal documents below); implementation of
the ROS 2 packages in `ws/src/` is underway. Check `README.md`'s Status
section and `PROJECT-PLAN-5-MONTH.md` for exactly what's built vs.
in-progress right now — that changes week to week, so don't take this
doc's word for it.

## The jargon, fast

You'll see these constantly in code, docs, and commit messages:

| Term | Meaning |
|---|---|
| **MC** | Mission computer — the Jetson-based software this repo builds |
| **FCC** | Flight controller (the Pixhawk 6X hardware + PX4 firmware) — not part of this repo, but the MC talks to it constantly |
| **GCS** | Ground control station — the human operator's laptop/software |
| **ISR** | The search-and-rescue role/mission the swarm flies (also used loosely for "the elected coordinator role") |
| **MANET** | The radio mesh network drones use to talk to each other and the GCS in the field |
| **DDS / uXRCE-DDS** | The pub-sub messaging protocol ROS 2 uses internally, and the lightweight variant the MC uses to talk to the FCC |
| **Channel A / B / C** | Three separate MAVLink conversations that share the wire: A = GCS↔FCC (standard PX4/QGC traffic), B = GCS↔MC (custom commands/telemetry), C = MC↔FCC mission-plan upload (the one place MAVLink is used *between* MC and FCC, because PX4 has no DDS-writable mission topic) |
| **SITL** | Software-in-the-loop — running against a simulated PX4 instead of real hardware, for dev/test |
| **Voronoi partition** | How the swarm divides up the search area: one cell per drone, computed independently by each drone and cross-checked against peers |
| **Boustrophedon** | The lawnmower-pattern flight path used to cover a cell |

If you're unsure what an acronym means, `grep` it across the root
`*.md` files — nearly everything is defined in `SRS-MC-001.md` or
`ICD-MC-001.md`.

## Suggested reading order

Don't try to read everything below in one sitting — the formal documents
(`SRS`/`ICD`/`SDD`/`HID`/`TP`) are reference material, not onboarding
material. A practical path:

1. **This doc** — orientation (you're here).
2. **`docs/ARCHITECTURE.md`** — the 4-layer design, packages, key patterns
   and algorithms. This is the real technical entry point.
3. **`docs/GETTING-STARTED.md`** — get something building and running,
   either the full Jetson/Docker stack or a single package natively on
   your workstation.
4. **Whichever package you're assigned** — read its code alongside
   `docs/API-REFERENCE.md` (topics/messages it publishes and subscribes
   to) and the relevant section of `SDD-MC-001.md` (its authoritative
   design).
5. **As needed, not up front**:
   - `SRS-MC-001.md` — if you need to trace a requirement (FR/IR/PR/SFR
     tags).
   - `ICD-MC-001.md` — if you're touching a wire format (topic schema,
     MAVLink message, MAVFTP layout).
   - `docs/TESTING.md` / `docs/DEPLOYMENT.md` / `docs/CONTRIBUTING.md` —
     when you're about to write tests, touch systemd/Docker deployment,
     or open your first PR.
   - `PROJECT-PLAN-5-MONTH.md` and the `*-BACKLOG.md` files — when
     picking up a ticket.

`CLAUDE.md` at the repo root has the full document map if you need
something not listed here.

## First-week checklist

- [ ] Read `docs/ARCHITECTURE.md` end to end once, even the parts that
      don't seem relevant to your ticket yet.
- [ ] Follow `docs/GETTING-STARTED.md`'s "Native package development"
      path and get one package building + its gtest suite passing
      locally — you don't need a Jetson or Pixhawk for this.
- [ ] Skim diagram `02` (`docs/diagrams/02-software-architecture.puml`,
      rendered to the matching `.png`) for the node topology, and
      diagram `04` for the mission state machine.
- [ ] Find your assigned package in the table in `docs/ARCHITECTURE.md`
      and read its node(s) source in `ws/src/<package>/`.
- [ ] Read `docs/CONTRIBUTING.md` before your first commit.

## Who to ask

For anything not answered by the docs above, ask in whatever channel
pointed you here — this document doesn't try to be a substitute for a
human answering a question about intent or history that predates the
written record.
