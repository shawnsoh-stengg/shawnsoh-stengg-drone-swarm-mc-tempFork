// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §5, ICD-MC-001 §5).

#include <rclcpp/rclcpp.hpp>

#include "swarm_coord/formation_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_coord::FormationControllerNode>());
  rclcpp::shutdown();
  return 0;
}
