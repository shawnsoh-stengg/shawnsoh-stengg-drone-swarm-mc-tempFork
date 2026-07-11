#include <rclcpp/rclcpp.hpp>

#include "mission_exec/mission_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_exec::MissionExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
