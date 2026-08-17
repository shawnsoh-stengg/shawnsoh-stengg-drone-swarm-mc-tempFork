// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include <rclcpp/rclcpp.hpp>

#include "health_monitor/health_monitor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<health_monitor::HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
