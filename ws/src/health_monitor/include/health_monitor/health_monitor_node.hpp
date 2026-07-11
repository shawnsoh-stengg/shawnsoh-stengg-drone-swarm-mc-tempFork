#pragma once

#include <rclcpp/rclcpp.hpp>

namespace health_monitor
{

// System health monitoring, node liveness tracking, and failsafe decision
// tree (SDD-MC-001 §10). Scaffold only — resource monitoring, liveness
// tracking, and the failsafe decision tree land with the health_monitor
// implementation work.
class HealthMonitorNode : public rclcpp::Node
{
public:
  HealthMonitorNode();
};

}  // namespace health_monitor
