#include "mission_exec/mission_executor_node.hpp"

namespace mission_exec
{

MissionExecutorNode::MissionExecutorNode()
: Node("mission_executor_node")
{
  RCLCPP_INFO(get_logger(), "mission_executor_node started");
}

}  // namespace mission_exec
