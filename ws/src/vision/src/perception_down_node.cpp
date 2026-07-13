#include "vision/perception_down_node.hpp"

namespace vision
{

PerceptionDownNode::PerceptionDownNode()
: Node("perception_down_node")
{
  RCLCPP_INFO(get_logger(), "perception_down_node started");
}

}  // namespace vision
