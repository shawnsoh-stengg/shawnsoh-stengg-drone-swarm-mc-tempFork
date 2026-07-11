#include "mission_exec/search_planner_node.hpp"

namespace mission_exec
{

SearchPlannerNode::SearchPlannerNode()
: Node("search_planner_node")
{
  RCLCPP_INFO(get_logger(), "search_planner_node started");
}

}  // namespace mission_exec
