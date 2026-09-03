// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §9, ICD-MC-001 §6).

#include <rclcpp/rclcpp.hpp>

#include "vision/perception_down_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision::PerceptionDownNode>());
  rclcpp::shutdown();
  return 0;
}
