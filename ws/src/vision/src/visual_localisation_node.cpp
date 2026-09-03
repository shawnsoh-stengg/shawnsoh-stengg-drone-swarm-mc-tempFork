// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §8, ICD-MC-001 §6).

#include "vision/visual_localisation_node.hpp"

namespace vision
{

VisualLocalisationNode::VisualLocalisationNode()
: Node("visual_localisation_node")
{
  RCLCPP_INFO(get_logger(), "visual_localisation_node started");
}

}  // namespace vision
