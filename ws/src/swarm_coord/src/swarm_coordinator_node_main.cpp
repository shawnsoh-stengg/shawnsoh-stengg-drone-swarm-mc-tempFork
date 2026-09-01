// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §6, ICD-MC-001 §5).
#include <rclcpp/rclcpp.hpp>

#include "swarm_coord/swarm_coordinator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_coord::SwarmCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
