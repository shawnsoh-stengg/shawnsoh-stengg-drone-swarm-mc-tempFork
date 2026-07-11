#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "mission_exec/search_planner_node.hpp"

TEST(SearchPlannerNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<mission_exec::SearchPlannerNode>();
  EXPECT_STREQ(node->get_name(), "search_planner_node");
  rclcpp::shutdown();
}
