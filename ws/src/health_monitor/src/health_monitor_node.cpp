#include "health_monitor/health_monitor_node.hpp"

namespace health_monitor
{

HealthMonitorNode::HealthMonitorNode()
: Node("health_monitor_node")
{
  RCLCPP_INFO(get_logger(), "health_monitor_node started");
}

}  // namespace health_monitor
