#include <rclcpp/rclcpp.hpp>

#include "mission_exec/search_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_exec::SearchPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
