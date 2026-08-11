// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §7, ICD-MC-001 §9).

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
