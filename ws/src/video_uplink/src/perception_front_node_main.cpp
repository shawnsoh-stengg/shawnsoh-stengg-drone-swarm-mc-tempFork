#include <rclcpp/rclcpp.hpp>

#include "video_uplink/perception_front_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<video_uplink::PerceptionFrontNode>());
  rclcpp::shutdown();
  return 0;
}
