// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §9, ICD-MC-001 §6).
#include "vision/perception_down_node.hpp"

namespace vision
{

PerceptionDownNode::PerceptionDownNode()
: Node("perception_down_node")
{
  RCLCPP_INFO(get_logger(), "perception_down_node started");
}

}  // namespace vision
