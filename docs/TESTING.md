# Testing

How to run the existing test suite and how to write new tests that fit the
conventions already established in `ws/src/`. For the full test-case
catalog (every `TP-UT-*`/`TP-NI-*`/`TP-ST-*`/`TP-HW-*` ID, inputs, and pass
criteria) see `TP-MC-001.md` — this document is about mechanics and
conventions, not a restatement of that catalog.

## The 5-level strategy, and what actually exists today

`TP-MC-001.md` §2.1 defines five test levels:

```
TP-UT  Unit Tests          gtest                      Algorithms, guards, state logic in isolation
TP-NI  Node Integration    launch_testing + rclpy     ROS 2 node pub/sub/service behaviour
TP-ST  SITL System Tests   pytest + rclpy + PX4 SITL  Full mission phases, failsafe scenarios
TP-HW  Hardware-in-the-Loop                            Jetson + Pixhawk    Performance, latency, resource bounds
TP-SM  Smoke Tests         Bash scripts               Deployment validation after installation
```

**What's implemented right now:**

- ✅ **TP-UT** — real, via `gtest` + `colcon test`, one `test_*.cpp` per
  package (see the table below).
- ✅ **TP-SM** — real, via the `scripts/<n>b-smoke-test-*.sh` /
  `<n>c-smoke-test-*.sh` companions described in `CLAUDE.md`'s Build System
  section and `docs/GETTING-STARTED.md`.
- ✅ **TP-NI** — real, via `launch_testing`/`pytest` (TEST-001), in the new
  `ws/src/mc_test_utils` package: the 4 mock nodes `TP-MC-001.md` §2.4
  specifies (`mock_fcc_node`, `mock_camera_node`, `mock_peer_node`,
  `mock_gcs_node`) plus one example `launch_testing` test per mock — see
  [Node-integration tests](#node-integration-tests-tp-ni) below. The
  gtest-based approximation described there previously (node-level gtest
  suites with a harness `rclcpp::Node`) still exists for the packages that
  already had it and is not being migrated retroactively — new TP-NI
  coverage for a node under test should use `mc_test_utils`'s mocks and the
  `launch_testing` pattern instead, per the example tests.
- 🧭 **TP-ST** and **TP-HW** — designed only. No PX4 SITL harness and no CI
  pipeline wiring the `TP-SM → TP-UT → TP-NI → TP-ST` sequence (§2.1) exist
  in this repo yet. Two of `TP-MC-001.md` §3.4's seven named fixtures now
  exist (`mc_test_utils/test_data/bags/fcc_nominal`, `camera_outdoor` —
  synthetic, built for TP-NI, see below); the other five
  (`gnss_lost.bag`, `search_polygon_500m.json`, `mission_upload_3drone.yaml`,
  `detection_ground_truth.json`, `gimbal_outdoor.mp4`) are TP-ST/TP-HW-only
  and still don't exist. `ros2 launch mission_exec
  sitl_mission_test.launch.py` in `CLAUDE.md` is the intended entry point
  once TP-ST lands.

## Running tests

### Unit tests (what exists today)

```bash
source /opt/ros/humble/install/setup.bash

# One package
colcon build --merge-install --packages-select fcc_bridge
colcon test --packages-select fcc_bridge
colcon test-result --verbose

# Whole workspace
colcon build --merge-install
colcon test
colcon test-result --all --verbose
```

`colcon test-result --verbose` is what actually shows failing assertions —
`colcon test`'s own output just reports pass/fail per package, so always
follow it with `test-result` when a test fails.

To run a single gtest binary directly (faster iteration than a full
`colcon test` pass) or filter to one test case:

```bash
./build/mission_exec/test_mission_manager_node
./build/mission_exec/test_mission_manager_node --gtest_filter='MissionManagerNodeTest.PreArmFailsOnLowBattery'
```

`ROS_LOCALHOST_ONLY=1` is worth setting if DDS discovery stalls or picks up
unrelated nodes on a shared network while running node-level tests locally.

### Smoke tests (TP-SM)

Each numbered build/install script (`scripts/01-*.sh`, `04-*.sh`, ...) has a
`<n>b-` or `<n>c-` smoke-test companion that verifies it — see
`docs/GETTING-STARTED.md`'s build walkthrough for the full ordered list.
Some accept `LIVE_CHECK=1` to additionally exercise a real running
container/service rather than just checking config and image state; that
mode needs the CycloneDDS sysctl floor from `scripts/01-install-os.sh`
(INFRA-004) already applied on the host, since a too-low `net.core.rmem_max`
will make DDS discovery fail under a live check.

One documented exception to the "every numbered pair has a plain `NN-*.sh`"
rule: `06b-smoke-test-fcc-links.sh`/`06c-smoke-test-xrce-agent.sh` have no
`06-*.sh` counterpart. Both verify config/behavior that other scripts
already installed or built — HIF-001 (ttyTHS1) wiring from
`01-install-os.sh`, and the MicroXRCEAgent-bearing image from
`08-build-fcc-bridge-image.sh` — rather than installing or building
anything of their own, so there was never a `06` step for them to pair
with.

### Node-integration tests (TP-NI)

```bash
source /opt/ros/humble/install/setup.bash
colcon build --merge-install --packages-select mc_msgs mc_test_utils
source install/setup.bash   # or the merge-install's own setup.bash
colcon test --packages-select mc_test_utils
colcon test-result --verbose
```

`mc_test_utils` (`ws/src/mc_test_utils`) is an `ament_cmake` + Python
package (`ament_cmake_python`'s `ament_python_install_package()`, plus
`launch_testing_ament_cmake`'s `add_launch_test()` in its
`CMakeLists.txt`) — this is what actually wires a `launch_testing`/`pytest`
file into `colcon test`; a plain `ament_python` package's `test/` directory
is *not* auto-discovered by `colcon test` the way an `ament_cmake` package's
`ament_add_gtest`/`add_launch_test` targets are, which is easy to get wrong
starting from scratch (see git history on this package for the false start).

The 4 mocks, one per `TP-MC-001.md` §2.4 row:

| Mock | Stands in for | Interface |
|---|---|---|
| `mock_fcc_node` | `fcc_bridge_node` itself (not just the PX4 hardware beneath it — TP-MC-001 §2.4 replaces the *node*) | `/mc/fcc/vehicle_state`, `/mc/fcc/odometry`, `/mc/fcc/imu` (published from a looped bag); subscribes `/mc/fcc/offboard_setpoint`, echoes to `/mock/fcc/setpoint_echo` |
| `mock_camera_node` | the not-yet-implemented `camera_node` | `/camera/image_raw`, `/camera/camera_info` (published from a looped bag, >= 15 FPS) |
| `mock_peer_node` | a swarm UDP peer (ICD-MC-001 §5, IFD-003) | raw UDP on 14900 (heartbeat)/14901 (coordination) — not a ROS topic |
| `mock_gcs_node` | the GCS side of Channel B (ICD-MC-001 §4, IFD-002) | raw MAVLink 2/UDP loopback — not a ROS topic; `/mock/gcs/last_rx_msgid` is a test-only introspection extra |

`mock_fcc_node`/`mock_camera_node` play back a rosbag2 bag via
`mc_test_utils/bag_playback.py`'s `BagPlayback` (loads the whole bag into
memory, replays it looped, preserving original inter-message timing — not
`ros2 bag play`, which is a one-shot CLI process, not an in-process,
loopable source a mock node's own timer can drive). The bags themselves
(`test_data/bags/fcc_nominal`, `test_data/bags/camera_outdoor`) are
synthetic — no hardware capture exists yet — generated by
`test_data/generate_bags.py`; re-run that script manually (inside a sourced
workspace with `mc_msgs` built) if the fixtures need to change, the
committed bags don't regenerate themselves.

`mock_peer_node`/`mock_gcs_node` speak raw UDP wire protocols instead of
ROS topics (`mc_test_utils/swarm_protocol.py`, `mc_test_utils/mavlink_lite.py`
respectively) — hand-packed, not via a generator dependency, following the
same toolchain decision `gcs_mavlink_node`/`mc_mavlink_defs.xml` already
made for MAVLink (see `mavlink_lite.py`'s module docstring for how its
`CRC_EXTRA`/wire-order constants were derived and verified). A TP-NI test
against either one talks to the mock with a plain `socket`, not `rclpy`.

Each mock has one example `test/test_mock_<name>.py` demonstrating the
pattern to follow for a real node under test: for a ROS-topic interface
(`mock_fcc_node`/`mock_camera_node`), launch the mock via
`generate_test_description()` and drive/observe it with a plain `rclpy`
client node in the test process; for a raw-UDP interface
(`mock_peer_node`/`mock_gcs_node`), do the same but talk to the mock with a
plain `socket` instead. Retry rather than send-once when a test's first
action is sending something to the mock — right after launch there's no
guarantee the mock's socket is bound yet (see
`test_mock_peer_node.py::test_answers_peer_state_request`'s retry loop and
comment for a case that actually hit this).

## Where tests live and how they're wired into the build

Every package follows the same `CMakeLists.txt` shape:

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)   # see note below
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_<thing> test/test_<thing>.cpp)
  target_link_libraries(test_<thing> <node>_lib)   # link the lib target, not the executable
endif()
```

- `set(ament_cmake_copyright_FOUND TRUE)` skips `ament_copyright`: it only
  recognizes OSI license templates (Apache-2, BSD, MIT, GPL, ...), and every
  `package.xml` here declares `license="Proprietary"`, which has no matching
  template — the check would fail regardless of header content, so it's
  disabled rather than attaching a misleading OSS notice. Carry this line
  into any new package's `CMakeLists.txt`.
- Node logic is built as a `<node>_lib` target separate from the
  `_node_main.cpp` executable specifically so tests can link the lib
  without pulling in `main()`. When adding a new node, split it the same
  way (see any existing package for the pattern) rather than testing
  through the executable.
- Test file naming is `test_<thing>.cpp`, one gtest binary per
  logical unit (a class, a node, or a focused behavior like
  `test_mission_persistence.cpp`) — not one binary per package.

### Current test inventory

| Package | Test binaries |
|---|---|
| `fcc_bridge` | `test_fcc_backend_contract`, `test_factory`, `test_mission_protocol_client`, `test_xrce_dds_backend`, `test_mavlink_udp_backend`, `test_fcc_bridge_node` |
| `gcs_comms` | `test_gcs_mavlink_node`, `test_mission_orchestrator_node` |
| `mission_exec` | `test_mission_manager_node`, `test_pre_arm_checks`, `test_command_arbiter`, `test_mission_persistence`, `test_mission_manager_node_persistence`, `test_search_planner_node` |
| `health_monitor` | `test_resource_sampler`, `test_health_monitor_node` (HEALTH-001: real resource-sampling tests; liveness/failsafe still land in HEALTH-002/003) |
| `swarm_coord` | `test_swarm_coordinator_node`, `test_formation_controller_node` (placeholders — stubs) |
| `vision` | `test_visual_localisation_node`, `test_perception_down_node` (placeholders — stubs) |
| `video_downlink` | `test_perception_front_node`, `test_video_mux_node` (placeholders — stubs) |

This table is TP-UT (`gtest`) only. `mc_test_utils`'s TP-NI (`launch_testing`)
tests are a different level and live in `ws/src/mc_test_utils/test/` — see
[Node-integration tests](#node-integration-tests-tp-ni) above.

## Writing a unit test

Every test file opens with a comment citing what it verifies and why,
tying back to a ticket ID and/or `SRS`/`SDD`/`TP` section — follow this,
it's how traceability is kept without a separate requirements-tracing tool:

```cpp
// FCC-001: FccBackend is a pure abstract interface — this test exercises it
// through a minimal fake implementation to confirm the contract (every
// method reachable via a base-class pointer, defaults are sane) without
// depending on any concrete transport.
```

### Testing an interface in isolation: the fake pattern

For pure-interface classes like `FccBackend`, write a minimal fake
implementation local to the test file (`test_fcc_backend_contract.cpp`)
rather than a mocking framework — the fake records what it was called with
in plain member fields, and the test asserts against those fields:

```cpp
class FakeBackend : public fcc_bridge::FccBackend
{
public:
  bool upload_mission(const std::vector<fcc_bridge::MissionItem> & items) override
  {
    last_uploaded_mission_ = items;
    return upload_result_;
  }
  // ... one field per method's "what was I called with" / "what should I return"
};
```

This keeps the test dependency-free (no PX4, no real transport) while still
exercising the real interface contract.

### Node-level tests

This is the pattern used for `fcc_bridge_node`, `gcs_mavlink_node`,
`mission_orchestrator_node`, `mission_manager_node`, and `health_monitor_node`
today — it plays the role TP-NI's `launch_testing` is meant to play, using
plain `gtest` and a harness node instead:

1. **One global `rclcpp::init`/`shutdown`** per test binary via a
   `::testing::Environment`, not per-test — registered once at file scope:

   ```cpp
   class RclcppEnvironment : public ::testing::Environment
   {
   public:
     void SetUp() override {rclcpp::init(0, nullptr);}
     void TearDown() override {rclcpp::shutdown();}
   };
   ::testing::Environment * const kRclcppEnv =
     ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);
   ```

2. **A `::testing::Test` fixture** whose `SetUp()` constructs the real node
   under test plus a second plain `rclcpp::Node` (conventionally named
   `harness_`) that publishes on the node's documented input topics and
   subscribes to its documented output topics — standing in for whatever
   peer nodes would normally be there, per `TP-MC-001.md` §2.4's mock
   strategy. Every publish from the node under test that a test cares about
   gets collected into a `std::vector` via the subscription callback, so
   assertions can inspect message history after the fact.

3. **`rclcpp::NodeOptions::parameter_overrides()`** to shrink timeouts,
   periods, and quorum counts for fast, deterministic tests — e.g.
   `control_loop_period_s: 0.02`, `mission_ack_timeout_s: 1.0`. Don't rely
   on production defaults in a test; override anything time-based
   explicitly.

4. **A `spin_until` helper** instead of a fixed `sleep`, to avoid both
   flakiness and slow tests:

   ```cpp
   template<typename Predicate>
   bool spin_until(
     std::initializer_list<rclcpp::Node::SharedPtr> nodes, Predicate predicate,
     std::chrono::milliseconds timeout)
   {
     const auto deadline = std::chrono::steady_clock::now() + timeout;
     while (std::chrono::steady_clock::now() < deadline) {
       for (const auto & n : nodes) {rclcpp::spin_some(n);}
       if (predicate()) {return true;}
       std::this_thread::sleep_for(std::chrono::milliseconds(5));
     }
     return false;
   }
   ```

   Use it both to wait out discovery before a test body starts asserting
   (`SetUp()` typically waits for `get_publisher_count() > 0` on a key
   subscription) and to wait for an expected message/state change inside a
   test case.

5. **`TearDown()`** resets both node pointers explicitly rather than
   relying on scope exit, to guarantee clean shutdown ordering between
   tests in the same binary.

### Placeholder tests for stub nodes

`swarm_coordinator_node`, `formation_controller_node`,
`perception_down_node`, `visual_localisation_node`, `perception_front_node`,
`video_mux_node`, and `search_planner_node` are currently empty constructor
stubs (see `docs/API-REFERENCE.md`'s implementation-status note). Their test
files are placeholders that just confirm the node constructs under its
expected name:

```cpp
TEST(VideoMuxNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<video_downlink::VideoMuxNode>();
  EXPECT_STREQ(node->get_name(), "video_mux_node");
  rclcpp::shutdown();
}
```

When you implement one of these nodes for real, replace its placeholder
test with a proper node-level test suite following the pattern above —
the topics to wire up are already documented (as 🧭 designed) in
`docs/API-REFERENCE.md`, and the corresponding `TP-UT-*`/`TP-NI-*` cases to
satisfy are in `TP-MC-001.md`.

## Adding a new test to an existing package

1. Add `test/test_<thing>.cpp` following the fake-backend or node-harness
   pattern above, whichever fits what you're testing.
2. Add the `ament_add_gtest(...)` + `target_link_libraries(...)` pair to
   the package's `CMakeLists.txt`, inside the existing `if(BUILD_TESTING)`
   block.
3. Open the file with a comment citing the ticket/requirement it verifies.
4. `colcon test --packages-select <pkg> && colcon test-result --verbose`.

## Where to go next

- Full test-case catalog, pass/fail criteria, mock strategy table, and the
  SITL/HIL environment specs: `TP-MC-001.md`.
- Which topics a not-yet-implemented node needs to wire up when you give it
  a real node-level test: `docs/API-REFERENCE.md`.
- Build/install chain the smoke tests verify: `docs/GETTING-STARTED.md`.
