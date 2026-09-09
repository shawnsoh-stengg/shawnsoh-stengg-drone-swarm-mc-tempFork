// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md, IGS-001).

#include <rclcpp/rclcpp.hpp>

#include "indoor_nav/gnss_sim_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<indoor_nav::GnssSimNode>());
  rclcpp::shutdown();
  return 0;
}
