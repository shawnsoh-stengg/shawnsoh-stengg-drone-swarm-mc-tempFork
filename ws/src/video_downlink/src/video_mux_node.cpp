#include "video_downlink/video_mux_node.hpp"

namespace video_downlink
{

VideoMuxNode::VideoMuxNode()
: Node("video_mux_node")
{
  RCLCPP_INFO(get_logger(), "video_mux_node started");
}

}  // namespace video_downlink
