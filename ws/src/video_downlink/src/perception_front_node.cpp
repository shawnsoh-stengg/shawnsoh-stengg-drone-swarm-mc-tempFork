// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §9a, ICD-MC-001 §7).

#include "video_downlink/perception_front_node.hpp"

namespace video_downlink
{

PerceptionFrontNode::PerceptionFrontNode()
: Node("perception_front_node")
{
  RCLCPP_INFO(get_logger(), "perception_front_node started");
}

}  // namespace video_downlink
