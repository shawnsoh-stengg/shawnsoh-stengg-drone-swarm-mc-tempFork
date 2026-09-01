// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §8, ICD-MC-001 §3).
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vision/visual_localisation_node.hpp"

TEST(VisualLocalisationNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<vision::VisualLocalisationNode>();
  EXPECT_STREQ(node->get_name(), "visual_localisation_node");
  rclcpp::shutdown();
}
