// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §7, ICD-MC-001 §9).

#include "mission_exec/search_planner_node.hpp"

namespace mission_exec
{

SearchPlannerNode::SearchPlannerNode()
: Node("search_planner_node")
{
  RCLCPP_INFO(get_logger(), "search_planner_node started");
}

}  // namespace mission_exec
