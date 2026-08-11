// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §7, ICD-MC-001 §9).

#include <rclcpp/rclcpp.hpp>

#include "mission_exec/search_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_exec::SearchPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
