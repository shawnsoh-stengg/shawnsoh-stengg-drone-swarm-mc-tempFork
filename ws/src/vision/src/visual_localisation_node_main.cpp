// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §8, ICD-MC-001 §6).

#include <rclcpp/rclcpp.hpp>

#include "vision/visual_localisation_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision::VisualLocalisationNode>());
  rclcpp::shutdown();
  return 0;
}
