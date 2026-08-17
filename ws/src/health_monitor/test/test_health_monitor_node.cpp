// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <mc_msgs/msg/system_health.hpp>
#include <rclcpp/rclcpp.hpp>

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
