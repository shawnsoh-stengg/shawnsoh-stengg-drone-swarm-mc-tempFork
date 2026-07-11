#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "swarm_coord/swarm_coordinator_node.hpp"

TEST(SwarmCoordinatorNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<swarm_coord::SwarmCoordinatorNode>();
  EXPECT_STREQ(node->get_name(), "swarm_coordinator_node");
  rclcpp::shutdown();
}
