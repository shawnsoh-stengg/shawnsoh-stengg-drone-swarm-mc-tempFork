#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "health_monitor/health_monitor_node.hpp"

TEST(HealthMonitorNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<health_monitor::HealthMonitorNode>();
  EXPECT_STREQ(node->get_name(), "health_monitor_node");
  rclcpp::shutdown();
}
