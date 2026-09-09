# Contributing

Code style and commit/PR conventions actually in use in this repo, pulled
from the existing `ws/src/` source and `git log` rather than an idealized
style guide. For test-writing conventions specifically, see
`docs/TESTING.md`; for package/file layout, `docs/API-REFERENCE.md` and
`docs/ARCHITECTURE.md`.

## Repo shape today

There is one branch, `main` — no long-lived feature branches, no merged PR
history yet. Development so far is a sequence of self-contained commits
directly to `main`, each implementing one backlog ticket (see
`PROJECT-PLAN-5-MONTH.md`) end to end: code + tests + doc updates in the
same commit. If you open a PR against this repo, match that same
granularity and the commit-message conventions below in the PR title/body
— see [Commit messages](#commit-messages) and [Opening a PR](#opening-a-pr).

## C++ style

### File header

Every `.cpp`/`.hpp` starts with exactly this two-line header, second line
tailored to the file's subject:

```cpp
// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).
```

Cite whichever `SDD-MC-001.md`/`ICD-MC-001.md` sections that file actually
implements — not a generic boilerplate reference.

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

## Python style

Two different flavors of Python live in this repo, and they follow
different conventions because only one of them goes through `ament_lint_auto`:

| | ROS 2 packages (`ws/src/*/`, e.g. `mc_test_utils`) | Standalone tools (`tools/`, e.g. `mission-orchestrator-sim`) |
|---|---|---|
| Build | `ament_cmake_python` + `ament_python_install_package()` in `CMakeLists.txt` | Plain `pip install -r requirements.txt` into a venv — no ROS 2, no colcon |
| Lint | `ament_lint_auto` (flake8 + pep257) rides along in `colcon test`, same as C++ | None wired up — style is by convention/review only |
| Quotes | `'single'` (flake8's default; observed consistently in `mc_test_utils/*.py`) | `"double"` (observed consistently in `mission_orchestrator_sim/*.py`) |
| Type hints | Present but not exhaustive (mock nodes, callbacks) | Used consistently on every function signature (`cli.py`, `dialect.py`, `transport.py`) |

Both flavors agree on:

- `from __future__ import annotations` as the first import, in every file
  that uses type hints.
- Import order: standard library, blank line, then everything else
  (third-party and local) alphabetized by module name in one block —
  `from X import Y` and `import Z` are sorted together, not grouped
  separately by style. Example (`ws/src/mc_test_utils/mc_test_utils/bag_playback.py`):

  ```python
  from __future__ import annotations

  from rclpy.serialization import deserialize_message
  import rosbag2_py
  from rosidl_runtime_py.utilities import get_message
  ```

- A module-level docstring citing the ticket/spec section, in the same
  voice as the C++ [comment culture](#comment-culture-cite-the-ticket-state-the-boundary-flag-the-gap)
  above — what the module does, which ticket/ICD section it implements, and
  any gap notes (see `mc_test_utils/swarm_protocol.py`'s `PEER_STATE_REQUEST`
  and byte-count gap notes for the pattern). No copyright header block like
  the C++ two-liner — none of the existing `.py` files carry one.
- Executables get a `#!/usr/bin/env python3` shebang line before the
  docstring (see `mc_test_utils/mock_fcc_node.py`); importable modules with
  no standalone entry point don't need one.
- `snake_case` functions/methods/variables, `PascalCase` classes — standard
  PEP 8, no project-specific deviation.

New Python code should match whichever column above it belongs to: a new
`ws/src/` package's `.py` files follow the ROS/flake8 conventions (and must
add the same `ament_lint_auto`/`set(ament_cmake_copyright_FOUND TRUE)`
`CMakeLists.txt` wiring as `mc_test_utils`, per
[Linting and formatting](#linting-and-formatting) below); a new standalone
`tools/` script follows the double-quote/type-hint style of
`mission-orchestrator-sim` instead of introducing a third convention.

## Linting and formatting

Every package's `CMakeLists.txt` wires `ament_lint_auto` into
`BUILD_TESTING` — running `colcon test` runs `cpplint`, `uncrustify`,
`lint_cmake`, and `xmllint` alongside the gtest suites, so a clean
`colcon test-result` already means the style checks passed too. A Python
package (`ament_cmake_python` build type, e.g. `mc_test_utils`) picks up
`flake8` and `pep257` from the same `ament_lint_auto` call instead — no
separate wiring needed. Two things every new package needs to carry over:

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

Work is tracked as tickets in `PROJECT-PLAN-5-MONTH.md` (and the follow-up
backlog docs — `FCC-MISSION-CHANNEL-BACKLOG.md`,
`OFFBOARD-MODE-SWITCHING-BACKLOG.md`). Prefix maps roughly to subsystem:

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

Every commit implementing a ticket cites its ID in the subject line (see
below); every ticket landed gets marked done in `PROJECT-PLAN-5-MONTH.md`
in the same or an immediately following commit. Note: an *external* tracker
(if your organization has one) may number `FCC-NNN` etc. differently from
this repo's own backlog doc — don't assume the numbers line up across
systems.

## Commit messages

Observed shape, consistently, across `git log`:

```
<Verb> <TICKET-ID>[/<TICKET-ID-2>]: <short imperative summary>

<Multi-paragraph prose: what changed, in which files/functions, and why —
design rationale, not a diff restatement. Cite SDD/ICD/SRS section numbers
and specific identifiers (class/function/topic names) where relevant. Call
out what's deliberately deferred/out of scope, same as the code comments
do.>

<Test evidence, if applicable — e.g.:>
colcon test --packages-select mission_exec: 128 tests, 0 failures.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
```

- **Subject verb** is one of: `Implement`, `Add`, `Fix`, `Update`,
  `Enhance`, `Document`, `Flag`, `Rename`, `Mark ... done in
  PROJECT-PLAN-5-MONTH.md`, `Revert`. Pick the one that actually describes
  the change — don't default to `Update` for everything.
- **Body explains why**, referencing spec sections and real identifiers,
  the same way the in-code comments do — a reviewer (or future you) should
  be able to read the commit message alone and understand the design
  decision, not just that "code changed."
- **Test evidence line** when the commit touches tested code — a bare
  `colcon test --packages-select <pkg>: N tests, 0 failures` line is
  enough; it's there so the message itself is proof of green tests, not
  just a claim.
- **Docs move with the code that motivates them.** When a change affects
  requirements/interfaces/design/test plan, the same commit (or the very
  next one) updates the relevant `SRS-MC-001.md`/`ICD-MC-001.md`/
  `SDD-MC-001.md`/`TP-MC-001.md` section and, if the architecture picture
  changed, the corresponding `docs/diagrams/*.puml` (+ re-rendered `.png`).
  Don't let root-level docs drift from what the code actually does — this
  repo already has real examples of design docs going stale (see
  `docs/API-REFERENCE.md`'s and `docs/DEPLOYMENT.md`'s "implemented vs.
  designed" callouts) and it costs a full audit to notice.
- Keep the `Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>`
  trailer if Claude Code produced the commit, matching every prior commit
  in this repo's history that it authored.

## CI/CD (TEST-005)

**Status: removed, not currently automated.** `TEST-005` built three
GitHub Actions workflows under `.github/workflows/`
(`ci.yml`, `nightly-sitl.yml`, `release.yml`) — done 2026-08-28. **All
three were deleted** ("Remove GitHub Actions workflows", 2026-09-05) and
have not been reinstated as of this writing. Nothing in this section runs
automatically today; every gate below is a manual step a developer runs
by hand before opening or merging a PR:

| Stage | What `ci.yml` used to do | Manual equivalent today |
|---|---|---|
| Build + test | `colcon build` + `colcon test` across all of `ws/src/*` | Run `colcon build`/`colcon test` yourself per `docs/TESTING.md` before pushing |
| Lint | `ament_lint_auto` (cpplint/cppcheck/uncrustify/xmllint), already wired into every package's `BUILD_TESTING` block | Run the same lint targets locally — see [Linting and formatting](#linting-and-formatting) above |
| Nightly SITL | Placeholder no-op (`TEST-002`'s SITL harness didn't exist yet) | `TEST-002`'s SITL harness now exists (partial — see `docs/TESTING.md`'s TP-ST row); run it manually via `scripts/16-run-sitl-tests.sh` |
| Release image build | `release.yml` built every `docker/*/Dockerfile` on a `v*` tag push | Run the numbered `scripts/NN-build-*-image.sh` chain by hand and push tags yourself — see `docs/DEPLOYMENT.md`'s "Cutting a release" section |

Branch protection on `main` was never enabled either way — GitHub gates
both classic branch protection and repository rulesets behind a paid plan
for private repos, and this repo is intentionally private. So even while
`ci.yml` existed, a failing run never actually blocked a merge; it was
advisory only. That constraint is unrelated to the workflows' removal and
still applies if they're ever restored.

See `INTEGRATION-AND-FLIGHT-TEST-PLAN.md` §2 for the original design
rationale (runner architecture, caching strategy) behind `ci.yml`, kept as
the target to restore rather than a description of current automation.

## Opening a PR

No PR has landed here yet, but if you open one: give it the same
subject/body shape as a commit message above (ticket ID, what changed and
why, test evidence), and use a "Summary" + "Test plan" body structure if
your PR spans multiple commits. Keep a PR scoped to one ticket, matching
how commits are scoped today — resist bundling unrelated tickets into one
PR just because they touched nearby files.

## Where to go next

- Writing or running tests for your change: `docs/TESTING.md`.
- Which topics/messages your change needs to wire up or update: `docs/API-REFERENCE.md`.
- Ticket backlog and current delivery status: `PROJECT-PLAN-5-MONTH.md`.
- Cutting a release / the manual build-test-lint process that replaces the removed CI/CD pipeline: [CI/CD (TEST-005)](#cicd-test-005) above and `docs/DEPLOYMENT.md`.
