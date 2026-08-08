// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#include <rclcpp/rclcpp.hpp>

#include "mission_exec/mission_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mission_exec::MissionManagerNode>();

  // Threading model (SDD §2.1): MultiThreadedExecutor. All of this node's
  // own callbacks share one mutually-exclusive group (state_cb_group_'s doc
  // comment in the header) since they all touch the same state machine —
  // the executor is still multi-threaded so this node's callbacks never
  // block other nodes' executors if ever composed into the same process.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
