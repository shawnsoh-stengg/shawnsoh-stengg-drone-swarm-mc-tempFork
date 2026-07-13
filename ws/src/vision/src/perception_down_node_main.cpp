#include <rclcpp/rclcpp.hpp>

#include "vision/perception_down_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision::PerceptionDownNode>());
  rclcpp::shutdown();
  return 0;
}
