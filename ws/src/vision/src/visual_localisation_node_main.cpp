#include <rclcpp/rclcpp.hpp>

#include "vision/visual_localisation_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision::VisualLocalisationNode>());
  rclcpp::shutdown();
  return 0;
}
