#include "mission_exec/mission_manager_node.hpp"

namespace mission_exec
{

MissionManagerNode::MissionManagerNode()
: Node("mission_manager_node")
{
  RCLCPP_INFO(get_logger(), "mission_manager_node started");
}

}  // namespace mission_exec
