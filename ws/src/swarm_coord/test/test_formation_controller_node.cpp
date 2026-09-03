// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §5, ICD-MC-001 §5).

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "swarm_coord/formation_controller_node.hpp"

TEST(FormationControllerNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<swarm_coord::FormationControllerNode>();
  EXPECT_STREQ(node->get_name(), "formation_controller_node");
  rclcpp::shutdown();
}
