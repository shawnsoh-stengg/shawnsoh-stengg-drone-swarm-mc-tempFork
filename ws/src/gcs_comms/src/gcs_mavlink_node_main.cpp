// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

#include <rclcpp/rclcpp.hpp>

#include "gcs_comms/gcs_mavlink_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gcs_comms::GcsMavlinkNode>();

  // Threading model (SDD §2.1): MultiThreadedExecutor per the standard
  // pattern (matches fcc_bridge_node_main.cpp) — this node currently runs
  // both its timers on one mutually-exclusive group (header comment,
  // gcs_mavlink_node.hpp), but the executor is set up per the project-wide
  // convention so future subscriptions slot in without a node-level rewrite.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
