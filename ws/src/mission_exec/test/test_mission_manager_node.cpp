#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "mission_exec/mission_manager_node.hpp"

TEST(MissionManagerNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<mission_exec::MissionManagerNode>();
  EXPECT_STREQ(node->get_name(), "mission_manager_node");
  rclcpp::shutdown();
}
