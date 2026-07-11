// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include <rclcpp/rclcpp.hpp>

#include "fcc_bridge/fcc_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fcc_bridge::FccBridgeNode>();

  // Threading model (SDD §2.1): MultiThreadedExecutor with dedicated
  // callback groups — this node currently only owns the mutually-exclusive
  // publish timer, but the executor is set up per the standard pattern so
  // future subscriptions (mission upload/command forwarding) slot in on a
  // reentrant group without a node-level rewrite.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
