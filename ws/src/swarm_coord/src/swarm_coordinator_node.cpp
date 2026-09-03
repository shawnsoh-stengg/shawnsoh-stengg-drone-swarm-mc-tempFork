// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §6, ICD-MC-001 §5).

#include "swarm_coord/swarm_coordinator_node.hpp"

namespace swarm_coord
{

SwarmCoordinatorNode::SwarmCoordinatorNode()
: Node("swarm_coordinator_node")
{
  RCLCPP_INFO(get_logger(), "swarm_coordinator_node started");
}

}  // namespace swarm_coord
