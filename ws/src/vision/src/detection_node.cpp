#include "vision/detection_node.hpp"

namespace vision
{

DetectionNode::DetectionNode()
: Node("detection_node")
{
  RCLCPP_INFO(get_logger(), "detection_node started");
}

}  // namespace vision
