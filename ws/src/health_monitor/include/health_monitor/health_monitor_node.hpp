// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#pragma once

#include <mc_msgs/msg/system_health.hpp>
#include <rclcpp/rclcpp.hpp>

#include "health_monitor/resource_sampler.hpp"

namespace health_monitor
{

// System health monitoring, node liveness tracking, and failsafe decision
// tree (SDD-MC-001 §10). HEALTH-001 wires up the 1 Hz resource-sampling
// timer and its /mc/health publisher; node_alive_mask liveness tracking
// (§10.2) and the failsafe decision tree (§10.3) land in HEALTH-002/003 on
// top of this same node and message.
class HealthMonitorNode : public rclcpp::Node
{
public:
  explicit HealthMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Test seam: exposes the sampler so a test can install its disk/GPU
  // provider seams (see resource_sampler.hpp) before triggering a sample,
  // per HEALTH-001's own "mocked /proc/nvidia-smi-equivalent readings"
  // acceptance criterion.
  ResourceSampler & resource_sampler() {return sampler_;}

  // Test seam: same body as the 1 Hz timer callback, callable directly so
  // a test doesn't have to wait out a real WallTimer period.
  void sample_and_publish();

private:
  ResourceSampler sampler_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::Publisher<mc_msgs::msg::SystemHealth>::SharedPtr pub_health_;
  rclcpp::Time start_time_;
};

}  // namespace health_monitor
