#include "vision/visual_localisation_node.hpp"

namespace vision
{

VisualLocalisationNode::VisualLocalisationNode()
: Node("visual_localisation_node")
{
  RCLCPP_INFO(get_logger(), "visual_localisation_node started");
}

}  // namespace vision
