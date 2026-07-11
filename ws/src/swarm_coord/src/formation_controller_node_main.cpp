#include <rclcpp/rclcpp.hpp>

#include "swarm_coord/formation_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_coord::FormationControllerNode>());
  rclcpp::shutdown();
  return 0;
}
