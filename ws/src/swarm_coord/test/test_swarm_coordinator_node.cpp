// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §6, ICD-MC-001 §5).

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
