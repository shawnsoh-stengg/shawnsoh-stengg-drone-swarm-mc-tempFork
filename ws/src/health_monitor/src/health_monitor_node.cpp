// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#include "health_monitor/health_monitor_node.hpp"

#include <chrono>

namespace health_monitor
{

HealthMonitorNode::HealthMonitorNode(const rclcpp::NodeOptions & options)
: Node("health_monitor_node", options)
{
  ResourceSamplerConfig config;
  config.proc_stat_path = declare_parameter("proc_stat_path", config.proc_stat_path);
  config.meminfo_path = declare_parameter("meminfo_path", config.meminfo_path);
  config.disk_path = declare_parameter("disk_path", config.disk_path);
  config.cpu_thermal_zone_paths =
    declare_parameter("cpu_thermal_zone_paths", config.cpu_thermal_zone_paths);
  config.gpu_temp_command = declare_parameter("gpu_temp_command", config.gpu_temp_command);
  sampler_ = ResourceSampler(config);

  const double sample_period_s = declare_parameter("sample_period_s", 1.0);

  start_time_ = now();

  pub_health_ = create_publisher<mc_msgs::msg::SystemHealth>("/mc/health", rclcpp::QoS(10));

  sample_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sample_period_s)),
    [this] {sample_and_publish();});

  RCLCPP_INFO(get_logger(), "health_monitor_node started");
}

void HealthMonitorNode::sample_and_publish()
{
  mc_msgs::msg::SystemHealth msg;
  msg.header.stamp = now();
  sampler_.sample(msg);
  msg.uptime_s = static_cast<uint32_t>((now() - start_time_).seconds());
  pub_health_->publish(msg);
}

}  // namespace health_monitor
