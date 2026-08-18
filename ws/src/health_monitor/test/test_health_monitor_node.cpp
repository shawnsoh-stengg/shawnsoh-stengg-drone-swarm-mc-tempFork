// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/link_status.hpp>
#include <mc_msgs/msg/system_health.hpp>
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
