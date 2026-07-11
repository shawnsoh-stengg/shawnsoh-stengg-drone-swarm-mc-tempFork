#include <rclcpp/rclcpp.hpp>

#include "swarm_coord/swarm_coordinator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_coord::SwarmCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
