// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/fcc_command.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/link_status.hpp>
#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/status_text.hpp>
#include <mc_msgs/msg/system_health.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "health_monitor/health_monitor_node.hpp"

namespace
{
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

::testing::Environment * const kRclcppEnv =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

template<typename Predicate>
bool spin_until(
  std::initializer_list<rclcpp::Node::SharedPtr> nodes, Predicate predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    for (const auto & n : nodes) {
      rclcpp::spin_some(n);
    }
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// HEALTH-003: shared harness for the failsafe-decision-tree tests --
// bundles every failsafe-tree input publisher and output subscriber a
// branch test might need. Heap-allocated (rather than returned by value)
// so its own subscription callbacks can safely capture `this`.
class FailsafeHarness
{
public:
  explicit FailsafeHarness(const std::string & name)
  : node(std::make_shared<rclcpp::Node>(name))
  {
    vehicle_state_pub = node->create_publisher<mc_msgs::msg::VehicleState>(
      "/mc/fcc/vehicle_state", rclcpp::QoS(10));
    link_status_pub = node->create_publisher<mc_msgs::msg::LinkStatus>(
      "/mc/gcs/link_status", rclcpp::QoS(10));
    mission_phase_pub = node->create_publisher<mc_msgs::msg::MissionPhase>(
      "/mc/mission_phase", rclcpp::QoS(10));
    gcs_command_pub = node->create_publisher<mc_msgs::msg::GcsCommand>(
      "/mc/gcs/command", rclcpp::QoS(10));

    fault_sub = node->create_subscription<mc_msgs::msg::FaultEvent>(
      "/mc/fault", rclcpp::QoS(10),
      [this](mc_msgs::msg::FaultEvent::SharedPtr msg) {faults.push_back(msg);});
    command_sub = node->create_subscription<mc_msgs::msg::FccCommand>(
      "/mc/fcc/send_command", rclcpp::QoS(10),
      [this](mc_msgs::msg::FccCommand::SharedPtr msg) {commands.push_back(msg);});
    status_text_sub = node->create_subscription<mc_msgs::msg::StatusText>(
      "/mc/status_text", rclcpp::QoS(10),
      [this](mc_msgs::msg::StatusText::SharedPtr msg) {status_texts.push_back(msg);});
  }

  // True once this harness's /mc/fault subscription has discovered
  // health_monitor_node's publisher -- the common "safe to publish an
  // input and expect the resulting fault to actually arrive" gate.
  bool ready() const {return fault_sub->get_publisher_count() > 0;}

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<mc_msgs::msg::VehicleState>::SharedPtr vehicle_state_pub;
  rclcpp::Publisher<mc_msgs::msg::LinkStatus>::SharedPtr link_status_pub;
  rclcpp::Publisher<mc_msgs::msg::MissionPhase>::SharedPtr mission_phase_pub;
  rclcpp::Publisher<mc_msgs::msg::GcsCommand>::SharedPtr gcs_command_pub;
  std::vector<mc_msgs::msg::FaultEvent::SharedPtr> faults;
  std::vector<mc_msgs::msg::FccCommand::SharedPtr> commands;
  std::vector<mc_msgs::msg::StatusText::SharedPtr> status_texts;

private:
  rclcpp::Subscription<mc_msgs::msg::FaultEvent>::SharedPtr fault_sub;
  rclcpp::Subscription<mc_msgs::msg::FccCommand>::SharedPtr command_sub;
  rclcpp::Subscription<mc_msgs::msg::StatusText>::SharedPtr status_text_sub;
};

constexpr uint32_t kMcCmdGcsOverride = 50004;
constexpr uint32_t kMavCmdNavReturnToLaunch = 20;
}  // namespace

TEST(HealthMonitorNode, ConstructsWithExpectedName)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  EXPECT_STREQ(node->get_name(), "health_monitor_node");
}

// HEALTH-001: node-integration test with mocked /proc/nvidia-smi-equivalent
// readings -> correct published values on /mc/health. Uses sample_and_
// publish() directly (see its own doc comment) rather than waiting out a
// real 1 Hz WallTimer tick.
TEST(HealthMonitorNode, PublishesSystemHealthFromMockedSources)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("proc_stat_path", std::string("/no/such/proc/stat")),
    rclcpp::Parameter("meminfo_path", std::string("/no/such/meminfo")),
    rclcpp::Parameter(
      "cpu_thermal_zone_paths", std::vector<std::string>{"/no/such/thermal/zone"}),
  });
  auto node = std::make_shared<health_monitor::HealthMonitorNode>(opts);
  node->resource_sampler().set_disk_usage_provider(
    [](const std::string &) {return std::optional<uint8_t>(42);});
  node->resource_sampler().set_gpu_temp_line_provider(
    [] {return std::optional<std::string>("RAM ... GPU@51C ...");});

  auto harness = std::make_shared<rclcpp::Node>("health_monitor_node_test_harness");
  std::vector<mc_msgs::msg::SystemHealth::SharedPtr> received;
  auto sub = harness->create_subscription<mc_msgs::msg::SystemHealth>(
    "/mc/health", rclcpp::QoS(10),
    [&received](mc_msgs::msg::SystemHealth::SharedPtr msg) {received.push_back(msg);});

  ASSERT_TRUE(
    spin_until(
      {node, harness}, [&] {return sub->get_publisher_count() > 0;}, std::chrono::seconds(2)));

  node->sample_and_publish();

  ASSERT_TRUE(
    spin_until({node, harness}, [&] {return !received.empty();}, std::chrono::seconds(2)));

  const auto & msg = *received.front();
  EXPECT_EQ(msg.cpu_pct, 0);   // no /proc/stat source -> left at 0
  EXPECT_EQ(msg.mem_pct, 0);   // no /proc/meminfo source -> left at 0
  EXPECT_EQ(msg.disk_pct, 42);
  EXPECT_EQ(msg.gpu_temp_c, 51);
  EXPECT_EQ(msg.cpu_temp_c, 0);  // no thermal zone source -> left at 0
}

// HEALTH-002: a tracked topic going silent past its configured timeout
// flips the corresponding node_alive_mask bit back off (SDD §10.2). Uses
// gcs_mavlink_node/`/mc/gcs/link_status` as the exemplar tracked topic,
// with its timeout shrunk via param override so the test doesn't have to
// wait out the real 2 s SDD default.
TEST(HealthMonitorNode, NodeAliveMaskFlipsOnTopicSilence)
{
  constexpr double kTimeoutS = 0.3;

  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("gcs_mavlink_node_timeout_s", kTimeoutS),
  });
  auto node = std::make_shared<health_monitor::HealthMonitorNode>(opts);

  auto harness = std::make_shared<rclcpp::Node>("health_monitor_node_test_harness_liveness");
  auto link_pub = harness->create_publisher<mc_msgs::msg::LinkStatus>(
    "/mc/gcs/link_status", rclcpp::QoS(10));

  mc_msgs::msg::SystemHealth::SharedPtr latest_health;
  auto health_sub = harness->create_subscription<mc_msgs::msg::SystemHealth>(
    "/mc/health", rclcpp::QoS(10),
    [&latest_health](mc_msgs::msg::SystemHealth::SharedPtr msg) {latest_health = msg;});

  ASSERT_TRUE(
    spin_until(
      {node, harness}, [&] {return link_pub->get_subscription_count() > 0;},
      std::chrono::seconds(2)));

  link_pub->publish(mc_msgs::msg::LinkStatus());

  // Poll sample_and_publish() (rather than a fixed sleep) so the elapsed
  // time between the message landing and observing the alive bit stays
  // well under kTimeoutS regardless of scheduling jitter.
  bool became_alive = false;
  const auto alive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < alive_deadline) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(harness);
    node->sample_and_publish();
    if (latest_health &&
      (latest_health->node_alive_mask & mc_msgs::msg::SystemHealth::GCS_MAVLINK_ALIVE))
    {
      became_alive = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(became_alive);

  // Let kTimeoutS lapse with no further publish, then re-sample.
  std::this_thread::sleep_for(std::chrono::duration<double>(kTimeoutS * 2));
  node->sample_and_publish();
  rclcpp::spin_some(harness);

  ASSERT_TRUE(
    spin_until(
      {node, harness},
      [&] {
        return latest_health &&
        !(latest_health->node_alive_mask & mc_msgs::msg::SystemHealth::GCS_MAVLINK_ALIVE);
      },
      std::chrono::seconds(2)));
}

// HEALTH-002/FR-074: mission_manager_node heartbeat silence past
// `manager_heartbeat_timeout_s` triggers the restart-path seam (the real
// systemd-restart transport is HEALTH-003) and publishes a MANAGER_DEAD
// fault event for the GCS.
TEST(HealthMonitorNode, ManagerHeartbeatWatchdogTriggersRestartAndFault)
{
  constexpr double kTimeoutS = 0.3;

  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("manager_heartbeat_timeout_s", kTimeoutS),
  });
  auto node = std::make_shared<health_monitor::HealthMonitorNode>(opts);

  bool restart_triggered = false;
  std::string restart_service;
  node->set_restart_trigger(
    [&](const std::string & service) {
      restart_triggered = true;
      restart_service = service;
    });

  auto harness = std::make_shared<rclcpp::Node>("health_monitor_node_test_harness_manager");
  auto heartbeat_pub = harness->create_publisher<std_msgs::msg::Header>(
    "/mc/manager/heartbeat", rclcpp::QoS(10));

  mc_msgs::msg::FaultEvent::SharedPtr latest_fault;
  auto fault_sub = harness->create_subscription<mc_msgs::msg::FaultEvent>(
    "/mc/fault", rclcpp::QoS(10),
    [&latest_fault](mc_msgs::msg::FaultEvent::SharedPtr msg) {latest_fault = msg;});

  ASSERT_TRUE(
    spin_until(
      {node, harness}, [&] {return heartbeat_pub->get_subscription_count() > 0;},
      std::chrono::seconds(2)));

  std_msgs::msg::Header hb;
  hb.stamp = harness->now();
  heartbeat_pub->publish(hb);

  // Settle well under kTimeoutS so the "still fresh" sample below can't
  // itself race past the shrunk timeout.
  spin_until({node, harness}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();
  EXPECT_FALSE(restart_triggered);  // heartbeat still fresh -> no watchdog fire yet

  // Let kTimeoutS lapse with no further heartbeat, then re-sample.
  std::this_thread::sleep_for(std::chrono::duration<double>(kTimeoutS * 2));
  node->sample_and_publish();

  EXPECT_TRUE(restart_triggered);
  EXPECT_EQ(restart_service, "mission-exec.service");

  ASSERT_TRUE(
    spin_until({node, harness}, [&] {return latest_fault != nullptr;}, std::chrono::seconds(2)));
  EXPECT_EQ(latest_fault->fault_id, mc_msgs::msg::FaultEvent::MANAGER_DEAD);
}

// HEALTH-003/FR-026: FCC_LINK_LOST publishes an annotated /mc/fault
// (ACTION_RTL, documenting what mission_manager_node's own pre-existing
// fault-handling will do) but never calls send_command() itself -- FR-026
// makes mission_manager_node the sole node authorised to command a
// flight-mode change. Edge-triggered: does not re-fire on persistence.
TEST(HealthMonitorNode, FccLinkLostPublishesFaultWithoutCommandingFcc)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  auto harness = std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_fcc_link");

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {
        return harness->vehicle_state_pub->get_subscription_count() > 0 && harness->ready();
      }, std::chrono::seconds(2)));

  mc_msgs::msg::VehicleState ok_state;
  ok_state.fcc_link_ok = true;
  ok_state.battery_pct = 1.0F;  // keep LOW_BATTERY out of this test's way
  harness->vehicle_state_pub->publish(ok_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  mc_msgs::msg::VehicleState lost_state;
  lost_state.fcc_link_ok = false;
  lost_state.battery_pct = 1.0F;
  harness->vehicle_state_pub->publish(lost_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::FCC_LINK_LOST);
  EXPECT_EQ(harness->faults[0]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_RTL);

  // Persistence -- no new message, just re-sample -- must not re-fire, and
  // FR-026 means no FccCommand is ever published for this branch.
  node->sample_and_publish();
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(100));
  EXPECT_EQ(harness->faults.size(), 1u);
  EXPECT_TRUE(harness->commands.empty());
}

// HEALTH-003/FR-074/FR-075: MANAGER_DEAD restart-then-RTL escalation across
// ONE continuous outage that never recovers on its own -- the scenario
// FR-075 actually targets. Retries happen on a manager_restart_retry_s_
// cooldown (not a fresh alive->dead edge each time, since a truly hung
// node only ever produces one such edge): attempts 1-3 request a systemd
// restart (ACTION_NONE), attempt 4 exhausts the budget and commands RTL
// directly (ACTION_RTL, FR-075's sole exception to FR-026) exactly once,
// then further persistence must not re-fire either action.
TEST(HealthMonitorNode, ManagerDeadRetriesThreeTimesThenCommandsRtlOnce)
{
  constexpr double kTimeoutS = 0.1;
  constexpr double kRetryS = 0.2;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("manager_heartbeat_timeout_s", kTimeoutS),
    rclcpp::Parameter("manager_restart_retry_s", kRetryS),
  });
  auto node = std::make_shared<health_monitor::HealthMonitorNode>(opts);

  std::vector<std::string> restart_calls;
  node->set_restart_trigger(
    [&](const std::string & service) {restart_calls.push_back(service);});

  auto harness = std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_manager");
  auto heartbeat_pub = harness->node->create_publisher<std_msgs::msg::Header>(
    "/mc/manager/heartbeat", rclcpp::QoS(10));

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return heartbeat_pub->get_subscription_count() > 0;},
      std::chrono::seconds(2)));

  // One heartbeat, then silence forever -- a genuinely hung node, never
  // republished, unlike HEALTH-002's own single-shot watchdog test.
  std_msgs::msg::Header hb;
  hb.stamp = harness->node->now();
  heartbeat_pub->publish(hb);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(30));
  node->sample_and_publish();  // registers alive

  // Attempt 1: the initial alive -> dead edge.
  std::this_thread::sleep_for(std::chrono::duration<double>(kTimeoutS * 3));
  node->sample_and_publish();
  // Attempts 2-4: each one retry-cooldown period later, still no heartbeat.
  for (int i = 0; i < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::duration<double>(kRetryS * 1.5));
    node->sample_and_publish();
  }

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return harness->faults.size() >= 4;}, std::chrono::seconds(2)));
  ASSERT_EQ(restart_calls.size(), 3u);
  for (const auto & service : restart_calls) {
    EXPECT_EQ(service, "mission-exec.service");
  }
  ASSERT_EQ(harness->faults.size(), 4u);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(harness->faults[i]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_NONE);
  }
  EXPECT_EQ(harness->faults[3]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_RTL);

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->commands.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->commands.size(), 1u);  // only the escalation sends an FCC command
  EXPECT_EQ(harness->commands[0]->cmd, kMavCmdNavReturnToLaunch);

  // Further persistence past the escalation -- no re-fire of either action.
  std::this_thread::sleep_for(std::chrono::duration<double>(kRetryS * 1.5));
  node->sample_and_publish();
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  EXPECT_EQ(restart_calls.size(), 3u);
  EXPECT_EQ(harness->faults.size(), 4u);
  EXPECT_EQ(harness->commands.size(), 1u);
}

// HEALTH-003/FR-026/FR-072: LOW_BATTERY publishes an annotated /mc/fault
// (ACTION_RTL, CRITICAL severity -- enough for mission_manager_node's own
// pre-existing is_critical_fault()/on_fault() to auto-RTL) without this
// node ever calling send_command() itself, using the 0.20 default
// threshold since no MissionUpload has been published.
TEST(HealthMonitorNode, LowBatteryPublishesFaultWithoutCommandingFcc)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  auto harness = std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_battery");

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {
        return harness->vehicle_state_pub->get_subscription_count() > 0 && harness->ready();
      }, std::chrono::seconds(2)));

  mc_msgs::msg::VehicleState ok_state;
  ok_state.fcc_link_ok = true;
  ok_state.battery_pct = 0.5F;
  harness->vehicle_state_pub->publish(ok_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  mc_msgs::msg::VehicleState low_state;
  low_state.fcc_link_ok = true;
  low_state.battery_pct = 0.1F;  // below the 0.20 default low_battery_threshold
  harness->vehicle_state_pub->publish(low_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::LOW_BATTERY);
  EXPECT_EQ(harness->faults[0]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_RTL);

  node->sample_and_publish();
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(100));
  EXPECT_EQ(harness->faults.size(), 1u);
  EXPECT_TRUE(harness->commands.empty());
}

// TEST-002: fcc_bridge's battery_pct=-1.0 "unknown" sentinel (published
// legitimately for the first few /mc/fcc/vehicle_state messages after any
// backend (re)start, before a real BATTERY_STATUS sample arrives) must not
// read as "critically low" -- confirmed live against a real PX4 SITL
// instance to cause a spurious immediate LOW_BATTERY/auto-RTL otherwise.
TEST(HealthMonitorNode, UnknownBatterySentinelDoesNotTriggerLowBattery)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  auto harness = std::make_shared<FailsafeHarness>(
    "health_monitor_node_test_harness_battery_unknown");

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {
        return harness->vehicle_state_pub->get_subscription_count() > 0 && harness->ready();
      }, std::chrono::seconds(2)));

  mc_msgs::msg::VehicleState unknown_state;
  unknown_state.fcc_link_ok = true;
  unknown_state.battery_pct = -1.0F;  // fcc_bridge::VehicleState::battery_remaining's default
  harness->vehicle_state_pub->publish(unknown_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(200));
  EXPECT_TRUE(harness->faults.empty());

  // A subsequent real, genuinely low reading still fires normally --
  // confirms this isn't just permanently suppressing LOW_BATTERY outright.
  mc_msgs::msg::VehicleState low_state;
  low_state.fcc_link_ok = true;
  low_state.battery_pct = 0.1F;
  harness->vehicle_state_pub->publish(low_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::LOW_BATTERY);
}

// HEALTH-003: MC_CMD_GCS_OVERRIDE (ICD §4.5/§10.5, cmd=50004) with
// OVERRIDE_LOW_BATTERY_RTL active suppresses the RTL -- a FaultEvent still
// notifies the GCS (ACTION_NONE), but no FccCommand is sent.
TEST(HealthMonitorNode, LowBatterySuppressedByGcsOverride)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  auto harness =
    std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_battery_override");

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {
        return harness->vehicle_state_pub->get_subscription_count() > 0 &&
        harness->gcs_command_pub->get_subscription_count() > 0 && harness->ready();
      }, std::chrono::seconds(2)));

  mc_msgs::msg::GcsCommand override_cmd;
  override_cmd.command = kMcCmdGcsOverride;
  override_cmd.params[0] = 1.0F;  // enable
  override_cmd.params[1] = 1.0F;  // OVERRIDE_LOW_BATTERY_RTL (ICD §10.7)
  harness->gcs_command_pub->publish(override_cmd);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));

  mc_msgs::msg::VehicleState ok_state;
  ok_state.fcc_link_ok = true;
  ok_state.battery_pct = 0.5F;
  harness->vehicle_state_pub->publish(ok_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  mc_msgs::msg::VehicleState low_state;
  low_state.fcc_link_ok = true;
  low_state.battery_pct = 0.1F;
  harness->vehicle_state_pub->publish(low_state);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::LOW_BATTERY);
  EXPECT_EQ(harness->faults[0]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_NONE);

  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(100));
  EXPECT_TRUE(harness->commands.empty());
}

// HEALTH-003/FR-026: GCS_LINK_LOST during a GCS-supervised mission_phase
// publishes a /mc/fault annotated ACTION_HOLD (the SDD §10.3-intended
// action) at non-critical severity -- deliberately kept below
// mission_manager_node's critical-fault->RTL threshold, since HOLD has no
// existing fault-consumption path to actually enact it (a documented,
// out-of-scope gap, not silently reinterpreted as RTL) -- and never calls
// send_command() itself.
TEST(HealthMonitorNode, GcsLinkLostPublishesHoldFaultWithoutCommandingFcc)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  auto harness =
    std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_gcslink_hold");

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {
        return harness->link_status_pub->get_subscription_count() > 0 &&
        harness->mission_phase_pub->get_subscription_count() > 0 && harness->ready();
      }, std::chrono::seconds(2)));

  mc_msgs::msg::MissionPhase phase;
  phase.phase = mc_msgs::msg::MissionPhase::SEARCH;
  harness->mission_phase_pub->publish(phase);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));

  mc_msgs::msg::LinkStatus ok_status;
  ok_status.gcs_link_ok = true;
  harness->link_status_pub->publish(ok_status);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  mc_msgs::msg::LinkStatus lost_status;
  lost_status.gcs_link_ok = false;
  harness->link_status_pub->publish(lost_status);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::GCS_LINK_LOST);
  EXPECT_EQ(harness->faults[0]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_HOLD);
  EXPECT_EQ(harness->faults[0]->severity, 4);  // MAV_SEVERITY WARNING -- below mission_manager_
                                               // node's severity<=CRITICAL(2) auto-RTL threshold

  node->sample_and_publish();
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(100));
  EXPECT_EQ(harness->faults.size(), 1u);
  EXPECT_TRUE(harness->commands.empty());
}

// HEALTH-003: GCS_LINK_LOST outside the GCS-supervised phase set publishes
// a fault notification only -- no HOLD is commanded (SDD §10.3).
TEST(HealthMonitorNode, GcsLinkLostNoActionOutsideSupervisedPhase)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  auto harness =
    std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_gcslink_noaction");

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {
        return harness->link_status_pub->get_subscription_count() > 0 && harness->ready();
      }, std::chrono::seconds(2)));
  // No mission_phase publish -- phase stays unknown, so the gate is false.

  mc_msgs::msg::LinkStatus ok_status;
  ok_status.gcs_link_ok = true;
  harness->link_status_pub->publish(ok_status);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  mc_msgs::msg::LinkStatus lost_status;
  lost_status.gcs_link_ok = false;
  harness->link_status_pub->publish(lost_status);
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  node->sample_and_publish();

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::GCS_LINK_LOST);
  EXPECT_EQ(harness->faults[0]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_NONE);

  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(100));
  EXPECT_TRUE(harness->commands.empty());
}

// HEALTH-003: GPU_OVERTEMP -> inference throttled + WARNING statustext
// (SDD §10.3), edge-triggered.
TEST(HealthMonitorNode, GpuOvertempThrottlesInferenceAndDoesNotRefire)
{
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  node->resource_sampler().set_gpu_temp_line_provider(
    [] {return std::optional<std::string>("RAM ... GPU@90C ...");});

  auto harness = std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_gpu");

  ASSERT_TRUE(
    spin_until({node, harness->node}, [&] {return harness->ready();}, std::chrono::seconds(2)));

  node->sample_and_publish();  // gpu_temp_c=90 > 85 threshold -> overtemp edge

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::GPU_OVERTEMP);
  EXPECT_EQ(harness->faults[0]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_NONE);

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->status_texts.empty();},
      std::chrono::seconds(2)));
  ASSERT_EQ(harness->status_texts.size(), 1u);
  EXPECT_EQ(harness->status_texts[0]->severity, mc_msgs::msg::StatusText::SEVERITY_WARNING);

  node->sample_and_publish();
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  EXPECT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->status_texts.size(), 1u);
}

// INFRA-007: VISION_MEM_HIGH is time-windowed, not edge-on-first-sample --
// it must not fire until usage has stayed at/above threshold continuously
// for vision_mem_sustained_s.
TEST(HealthMonitorNode, VisionMemHighDoesNotFireBeforeSustainedWindowElapses)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
    {rclcpp::Parameter("vision_mem_sustained_s", 30.0)});  // real default, not shrunk
  auto node = std::make_shared<health_monitor::HealthMonitorNode>(opts);
  node->resource_sampler().set_vision_mem_pct_provider(
    [] {return std::optional<uint8_t>(95);});  // >= 80% threshold

  auto harness = std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_vision_mem_1");
  ASSERT_TRUE(
    spin_until({node, harness->node}, [&] {return harness->ready();}, std::chrono::seconds(2)));

  node->sample_and_publish();  // window just started -- 30s hasn't elapsed
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  EXPECT_TRUE(harness->faults.empty());
}

TEST(HealthMonitorNode, VisionMemHighFiresAfterSustainedWindowAndDoesNotRefire)
{
  constexpr double kSustainedS = 0.05;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
    {rclcpp::Parameter("vision_mem_sustained_s", kSustainedS)});
  auto node = std::make_shared<health_monitor::HealthMonitorNode>(opts);
  node->resource_sampler().set_vision_mem_pct_provider(
    [] {return std::optional<uint8_t>(95);});

  auto harness = std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_vision_mem_2");
  ASSERT_TRUE(
    spin_until({node, harness->node}, [&] {return harness->ready();}, std::chrono::seconds(2)));

  node->sample_and_publish();  // window starts
  std::this_thread::sleep_for(std::chrono::milliseconds(80));  // exceed kSustainedS
  node->sample_and_publish();  // still over threshold -- window now satisfied

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);
  EXPECT_EQ(harness->faults[0]->fault_id, mc_msgs::msg::FaultEvent::VISION_MEM_HIGH);
  EXPECT_EQ(harness->faults[0]->failsafe_action, mc_msgs::msg::FaultEvent::ACTION_NONE);

  node->sample_and_publish();  // condition persists -- must not re-fire
  spin_until({node, harness->node}, [] {return false;}, std::chrono::milliseconds(50));
  EXPECT_EQ(harness->faults.size(), 1u);
}

TEST(HealthMonitorNode, VisionMemHighRearmsAfterDroppingBelowThreshold)
{
  constexpr double kSustainedS = 0.05;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
    {rclcpp::Parameter("vision_mem_sustained_s", kSustainedS)});
  auto node = std::make_shared<health_monitor::HealthMonitorNode>(opts);
  bool over_threshold = true;
  node->resource_sampler().set_vision_mem_pct_provider(
    [&over_threshold] {return std::optional<uint8_t>(over_threshold ? 95 : 10);});

  auto harness = std::make_shared<FailsafeHarness>("health_monitor_node_test_harness_vision_mem_3");
  ASSERT_TRUE(
    spin_until({node, harness->node}, [&] {return harness->ready();}, std::chrono::seconds(2)));

  node->sample_and_publish();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  node->sample_and_publish();
  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return !harness->faults.empty();}, std::chrono::seconds(2)));
  ASSERT_EQ(harness->faults.size(), 1u);

  over_threshold = false;
  node->sample_and_publish();  // drops below threshold -- clears active state

  over_threshold = true;
  node->sample_and_publish();  // re-crosses -- window restarts
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  node->sample_and_publish();  // sustained again -- fires a second, distinct fault

  ASSERT_TRUE(
    spin_until(
      {node, harness->node}, [&] {return harness->faults.size() >= 2u;}, std::chrono::seconds(2)));
  EXPECT_EQ(harness->faults[1]->fault_id, mc_msgs::msg::FaultEvent::VISION_MEM_HIGH);
}
