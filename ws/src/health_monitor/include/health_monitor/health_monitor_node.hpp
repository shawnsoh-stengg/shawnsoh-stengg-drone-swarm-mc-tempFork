// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/system_health.hpp>
#include <rclcpp/rclcpp.hpp>

#include "health_monitor/resource_sampler.hpp"

namespace health_monitor
{

// System health monitoring, node liveness tracking, and failsafe decision
// tree (SDD-MC-001 §10). HEALTH-001 wired up the 1 Hz resource-sampling
// timer and its /mc/health publisher. HEALTH-002 (this) adds node_alive_mask
// liveness tracking (§10.2) and the mission_manager_node heartbeat watchdog
// (FR-074). The full failsafe decision tree (§10.3) and the real
// systemd-restart transport (§10.4) are HEALTH-003, built on top of the
// same node/message.
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

  // Test seam: HEALTH-003 owns the real systemd-restart transport (a Unix
  // socket to the privileged drone-mc-watchdog helper, SDD §10.4). Until
  // that lands, the FR-074 watchdog calls this seam instead of a real
  // socket — defaults to a log-only stub. Argument is the systemd unit
  // name to restart (e.g. "mission-exec.service").
  void set_restart_trigger(std::function<void(const std::string &)> trigger);

private:
  // One tracked "is this critical node alive" source (SDD §10.2). Only
  // arrival timing matters, never payload content, so a single
  // rclcpp::GenericSubscription per entry covers every topic regardless of
  // its message type — the type is only needed as a runtime-resolved
  // string for rclcpp's generic-subscription typesupport lookup, so
  // tracking these 9 topics doesn't require depending on every message
  // package (sensor_msgs, geometry_msgs, ...) at build time.
  struct NodeWatch
  {
    std::string node_name;
    uint16_t mask_bit{0};
    double timeout_s{0.0};
    rclcpp::Time last_received;
    bool ever_received{false};
    rclcpp::GenericSubscription::SharedPtr sub;
  };

  void wire_node_watches();
  bool is_alive(const NodeWatch & watch, const rclcpp::Time & current) const;
  void on_manager_heartbeat_lost();

  ResourceSampler sampler_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::Publisher<mc_msgs::msg::SystemHealth>::SharedPtr pub_health_;
  rclcpp::Publisher<mc_msgs::msg::FaultEvent>::SharedPtr pub_fault_;
  rclcpp::Time start_time_;

  std::vector<NodeWatch> node_watches_;
  std::size_t manager_watch_index_{0};
  bool manager_was_alive_{false};
  double manager_heartbeat_timeout_s_{5.0};
  std::function<void(const std::string &)> restart_trigger_;
};

}  // namespace health_monitor
