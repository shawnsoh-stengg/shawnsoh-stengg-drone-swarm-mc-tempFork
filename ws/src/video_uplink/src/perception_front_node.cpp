#include "video_uplink/perception_front_node.hpp"

namespace video_uplink
{

PerceptionFrontNode::PerceptionFrontNode()
: Node("perception_front_node")
{
  RCLCPP_INFO(get_logger(), "perception_front_node started");
}

}  // namespace video_uplink
