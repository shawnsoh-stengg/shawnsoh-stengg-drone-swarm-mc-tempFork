// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §9, ICD-MC-001 §6).
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vision/perception_down_node.hpp"

TEST(PerceptionDownNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<vision::PerceptionDownNode>();
  EXPECT_STREQ(node->get_name(), "perception_down_node");
  rclcpp::shutdown();
}
