# Contributing

Code style and commit/PR conventions actually in use in this repo, pulled
from the existing `ws/src/` source and `git log` rather than an idealized
style guide. For test-writing conventions specifically, see
`docs/TESTING.md`; for package/file layout, `docs/API-REFERENCE.md` and
`docs/ARCHITECTURE.md`.

## Repo shape today

Development happens on short-lived feature branches merged into `main` via
GitHub PRs — 10 have landed so far (#1-#10). Branch names follow
`feature/near-<NEAR-ID>-mc-<prefix>-<NNN>-<slug>`, e.g.
`feature/near-433-mc-exec-004-watchdog-heartbeat-mission-upload-disk`: the
`near-<NEAR-ID>` segment is the ticket's ID in the team's external tracker
(Linear); `mc-<prefix>-<NNN>` is this repo's own subsystem-ticket ID (see
[Ticket IDs](#ticket-ids)) for the same piece of work. One branch/PR per
ticket. PRs are merged either as a single squash commit (subject ends
`(#N)`) or as a regular merge preserving the branch's individual commits —
both show up in `git log`; match whichever your reviewer/merge button
produces, don't force one over the other.

## C++ style

### File header

Every `.cpp`/`.hpp` starts with exactly this two-line header, second line
tailored to the file's subject:

```cpp
// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).
```

Cite whichever `docs/design/SDD-MC-001.md`/`docs/design/ICD-MC-001.md`
sections that file actually implements — not a generic boilerplate
reference.

### Headers and includes

- `#pragma once`, not include guards.
- Include order: standard library first, one blank line, then other
  packages' headers in alphabetical order (angle-bracket or quoted,
  matching what's already in the file), one blank line, then this
  package's own headers last:

  ```cpp
  #include <memory>

  #include "mc_msgs/msg/fcc_command.hpp"
  #include "mc_msgs/msg/vehicle_state.hpp"
  #include "rclcpp/rclcpp.hpp"

  #include "fcc_bridge/fcc_backend.hpp"
  ```

  `ament_cpplint` (wired into every package's `BUILD_TESTING` block) checks
  this — see [Linting](#linting-and-formatting).

### Naming

Standard ROS 2 / Google-derived C++ naming, consistent across the
workspace:

| Kind | Convention | Example |
|---|---|---|
| Class / struct | `PascalCase` | `FccBridgeNode`, `MissionItem` |
| Function / method | `snake_case` | `on_mission_upload()`, `publish_mission_ack()` |
| Private member | `snake_case_` (trailing underscore) | `backend_`, `pub_vehicle_state_` |
| Local constant | `kPascalCase` | `kDefaultVehicleStateRateHz`, `kSeverityCritical` |
| Namespace | matches the package name | `namespace fcc_bridge { ... }` |
| Node class member split | `<node>_lib` target (testable) vs `<node>_main.cpp` (just calls `rclcpp::spin`) | see `docs/TESTING.md` |

### Comment culture: cite the ticket, state the boundary, flag the gap

This codebase's comments lean heavier than a typical "no comments unless
non-obvious" rule — because they carry traceability the project doesn't
have a separate requirements-tracing tool for. Three patterns recur
throughout `ws/src/` and are worth following:

1. **Cite the ticket and spec section**, not just what the code does:

   ```cpp
   // FCC-004: fcc_bridge_node — topic adapter, liveness monitoring, fault
   // publication (SDD §3.4, ICD §7). Zero business logic: it instantiates a
   // FccBackend via the factory, republishes its aggregated VehicleState...
   ```

2. **State the boundary explicitly** — what this code deliberately does
   *not* do, and whose job that is instead:

   ```cpp
   // Still zero business logic: deciding HOLD/RTL/mission-abort from
   // repeated failures stays mission_manager's/formation_controller's job
   // (SDD §10.3 ownership split).
   ```

3. **Flag known gaps as `GAP NOTE`** when something is authored ahead of
   (or in place of) an upstream spec that doesn't fully cover it yet, so a
   future reader — or the actual spec owner — knows it needs review:

   ```cpp
   // GAP NOTE: ICD-MC-001 §9's topic table has cited mc_msgs/IsrRole since
   // it was added, but no message body existed anywhere in the ICD
   // (v0.1-DRAFT, 2026-06-20) until this file. Authored locally 2026-07-26
   // to close that hole — not sourced from an upstream ICD revision; needs
   // ICD-owner review/ratification.
   ```

Don't narrate *what* the code does when the identifier already says it —
these three patterns are about *why* and *whose responsibility*, which
nothing else in the repo captures.

## Linting and formatting

Every package's `CMakeLists.txt` wires `ament_lint_auto` into
`BUILD_TESTING` — running `colcon test` runs `cpplint`, `uncrustify`,
`lint_cmake`, and `xmllint` alongside the gtest suites, so a clean
`colcon test-result` already means the style checks passed too. Two things
every new package needs to carry over:

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright only recognizes OSI license templates (apache2, bsd*,
  # mit, gpl*, ...); this package.xml declares license="Proprietary", which
  # has no matching template, so the check would always fail regardless of
  # header content. Skipped rather than attaching a misleading OSS notice.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ...
```

If `uncrustify`/`cpplint` flag a violation, fix it manually to match the
surrounding code rather than reformatting the whole file — there's no
project-specific `.clang-format`/`.uncrustify.cfg` here, just the ROS 2
default `ament_uncrustify` style already present throughout `ws/src/`.

## `package.xml` conventions

- `<version>0.1.0</version>` for every first-party package — pre-1.0, not
  yet bumped per-change. (`px4_msgs`, vendored, keeps its own upstream
  version.)
- `<license>Proprietary</license>` — this is what makes the
  `ament_copyright` skip above necessary.
- `<maintainer email="...">` — real maintainer, not a placeholder.
- `<test_depend>ament_lint_auto</test_depend>`,
  `<test_depend>ament_lint_common</test_depend>`,
  `<test_depend>ament_cmake_gtest</test_depend>` at minimum.

## Ticket IDs

Two ID systems coexist and both matter — don't conflate them:

- **`NEAR-<N>`** — the ticket's ID in the team's external tracker (Linear).
  Shows up in branch names and, inconsistently, in commit subjects/PR
  titles. This repo carries no backlog doc anymore (the root-level
  `PROJECT-PLAN-5-MONTH.md` and subsystem backlog docs were deleted in
  `e8c60b8`) — Linear is now the only source of truth for what's planned,
  in progress, or done. Don't expect to find a ticket list in this repo.
- **`<PREFIX>-<NNN>`** (e.g. `FCC-006`, `GCS-003`, `EXEC-004`) — this
  repo's own subsystem-scoped ID, cited in code comments (see
  [Comment culture](#comment-culture-cite-the-ticket-state-the-boundary-flag-the-gap))
  and embedded in branch names alongside the `NEAR-<N>` for the same piece
  of work, e.g. `near-433-mc-exec-004-...` ties `NEAR-433` to `EXEC-004`.
  There's no numeric relationship between the two — always read the
  mapping off the branch name, never assume `NEAR-<N>` and `<PREFIX>-<NNN>`
  correlate by number.

Prefix maps roughly to subsystem:

| Prefix | Subsystem |
|---|---|
| `FCC` | `fcc_bridge` |
| `GCS` | `gcs_comms` |
| `EXEC`, `ME` | `mission_exec` |
| `SC` | `swarm_coord` |
| `VIS`, `VU` | `vision` / `video_downlink` |
| `HEALTH`, `HM` | `health_monitor` |
| `MSG` | `mc_msgs` |
| `MAVR` | `mavlink-router` |
| `ZBD` | `zenoh-bridge-dds` |
| `INFRA`, `DEP` | cross-cutting infra / deployment |
| `TEST` | test tooling |

Code comments cite the `<PREFIX>-<NNN>` ID (see
[Comment culture](#comment-culture-cite-the-ticket-state-the-boundary-flag-the-gap));
commit subjects and PR titles, when they cite a ticket at all, tend to use
the `NEAR-<N>` ID instead. Ticket status (planned/in-progress/done) lives
in Linear now, not in this repo.

## Commit messages

Two eras are visible in `git log`, and the current one is what to match
going forward:

- **Pre-PR era** (everything up to `1efbe26`, before feature
  branches/PRs started): strict and consistent —
  `<Verb> <TICKET-ID>: <short imperative summary>` subject, a
  multi-paragraph body with design rationale (spec sections, real
  identifiers, deferred scope), a
  `colcon test --packages-select <pkg>: N tests, 0 failures` evidence
  line, and a `Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>`
  trailer on every Claude-Code-authored commit.
- **Current, PR-era practice** (`NEAR-425` onward): looser. Typical
  subject is `<NEAR-ID> - <short summary>` (dash, not colon) or just a
  plain descriptive summary with no ticket ID at all (e.g. `a575bab`,
  `e8c60b8`). Most of these commits carry no body, and neither the
  test-evidence line nor the `Co-Authored-By` trailer reliably appears
  anymore (compare `3142bae`, `811d396`, `96463d1` to the pre-PR examples
  above).

For new work, match current practice, with two things worth still doing
even though recent history is inconsistent about them:

- Subject: `<NEAR-ID> - <short imperative summary>` when the commit maps
  cleanly to one ticket.
- **Test evidence is worth keeping** when the commit touches tested code
  — a bare `colcon test --packages-select <pkg>: N tests, 0 failures`
  line costs little and is proof the change is green, not just a claim.
- **Docs move with the code that motivates them.** When a change affects
  requirements/interfaces/design/test plan, the same PR (or a fast
  follow-up PR) updates the relevant `SRS-MC-001.md`/`ICD-MC-001.md`/
  `SDD-MC-001.md`/`TP-MC-001.md` section under `docs/design/` and, if the
  architecture picture changed, the corresponding `docs/diagrams/*.puml`
  (+ re-rendered `.png`). Don't let docs drift from what the code
  actually does — this repo already has real examples of design docs
  going stale (see `docs/API-REFERENCE.md`'s and `docs/DEPLOYMENT.md`'s
  "implemented vs. designed" callouts) and it costs a full audit to
  notice.
- Keep the `Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>`
  trailer if Claude Code produced the commit — it's dropped out of most
  recent commits, but it's still the right thing to include.

## Opening a PR

10 PRs have landed so far (#1-#10), one per `feature/near-<ID>-mc-<prefix>-<NNN>-<slug>`
branch (see [Repo shape today](#repo-shape-today)). In practice, PR titles
are a short descriptive summary (the ticket ID isn't consistently
included) and bodies are one or two prose paragraphs — no
"Summary"/"Test plan" headers have actually been used, so don't feel
obliged to impose that structure. Keep a PR scoped to one ticket, matching
how branches map 1:1 to tickets today — resist bundling unrelated tickets
into one PR just because they touched nearby files. If you have test
evidence worth recording, put it in the PR body — past PRs mostly haven't,
but it's cheap and useful for a reviewer.

## Where to go next

- Writing or running tests for your change: `docs/TESTING.md`.
- Which topics/messages your change needs to wire up or update: `docs/API-REFERENCE.md`.
- Design specs a change may need to update: `docs/design/SRS-MC-001.md`,
  `docs/design/ICD-MC-001.md`, `docs/design/SDD-MC-001.md`.
- Ticket backlog and current delivery status: tracked in Linear
  (`NEAR-<N>` IDs) — not in this repo.
