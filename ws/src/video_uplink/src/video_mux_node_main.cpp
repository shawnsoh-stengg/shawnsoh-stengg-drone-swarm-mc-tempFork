#include <rclcpp/rclcpp.hpp>

#include "video_uplink/video_mux_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<video_uplink::VideoMuxNode>());
  rclcpp::shutdown();
  return 0;
}
