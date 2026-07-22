#include "video_downlink/perception_front_node.hpp"

namespace video_downlink
{

PerceptionFrontNode::PerceptionFrontNode()
: Node("perception_front_node")
{
  RCLCPP_INFO(get_logger(), "perception_front_node started");
}

}  // namespace video_downlink
