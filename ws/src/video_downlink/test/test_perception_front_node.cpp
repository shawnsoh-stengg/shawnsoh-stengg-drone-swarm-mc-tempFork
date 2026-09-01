// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §9a, ICD-MC-001 §7).
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "video_downlink/perception_front_node.hpp"

TEST(PerceptionFrontNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<video_downlink::PerceptionFrontNode>();
  EXPECT_STREQ(node->get_name(), "perception_front_node");
  rclcpp::shutdown();
}
