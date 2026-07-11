#include <rclcpp/rclcpp.hpp>

#include "vision/detection_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision::DetectionNode>());
  rclcpp::shutdown();
  return 0;
}
