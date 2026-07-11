#include <rclcpp/rclcpp.hpp>

#include "health_monitor/health_monitor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<health_monitor::HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
