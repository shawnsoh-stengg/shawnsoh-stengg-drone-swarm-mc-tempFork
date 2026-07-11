#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "mission_exec/mission_executor_node.hpp"

TEST(MissionExecutorNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<mission_exec::MissionExecutorNode>();
  EXPECT_STREQ(node->get_name(), "mission_executor_node");
  rclcpp::shutdown();
}
