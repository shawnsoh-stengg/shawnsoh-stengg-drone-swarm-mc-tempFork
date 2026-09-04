// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §10, ICD-MC-001 §9).

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/fcc_command.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/link_status.hpp>
#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <mc_msgs/msg/status_text.hpp>
#include <mc_msgs/msg/system_health.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "health_monitor/resource_sampler.hpp"

namespace health_monitor
{

// System health monitoring, node liveness tracking, and failsafe decision
// tree (SDD-MC-001 §10). HEALTH-001 wired up the 1 Hz resource-sampling
// timer and its /mc/health publisher. HEALTH-002 added node_alive_mask
// liveness tracking (§10.2) and the mission_manager_node heartbeat watchdog
// (FR-074). HEALTH-003 (this) adds the failsafe decision tree (§10.3) --
// FCC_LINK_LOST, MANAGER_DEAD restart-then-RTL, LOW_BATTERY, GCS_LINK_LOST,
// GPU_OVERTEMP -- and the real systemd-restart transport (§10.4).
//
// FR-026 vs SDD §10.3/FR-072/FR-075: the SDD's decision tree and FR-072/
// FR-075 read as if health_monitor_node commands the FCC directly for
// HOLD/RTL, but FR-026 is explicit and absolute: "mission_manager_node...
// shall be the sole node in the system that commands a flight-mode change;
// health_monitor_node shall never call send_command() for a flight-mode
// change (FR-012, SFR-009)." This implementation follows FR-026: for
// FCC_LINK_LOST and LOW_BATTERY, this node only publishes an annotated
// /mc/fault -- mission_manager_node's own pre-existing, independent
// on_fault()/guard_any_fault()/action_command_rtl() machinery (see
// mission_manager_node.cpp; this is how FCC_LINK_LOST already auto-RTLs
// today via fcc_bridge_node's own fault publish) is what actually issues
// the RTL, so FR-072's outcome is achieved with zero new send_command()
// calls from this node. The one deliberate exception is FR-075's
// literal, narrow grant: after 3 failed mission-exec.service restart
// attempts, mission_manager_node is presumed unreachable to react to its
// own death, so health_monitor_node commands RTL directly as the sole
// remaining path to bring the vehicle home.
//
// GCS_LINK_LOST->HOLD is a known, documented gap rather than silently
// reinterpreted: mission_manager_node's existing fault-consumption only
// ever RTLs (there is no HOLD-on-fault path), and adding one is a
// mission_exec change outside this ticket's stated file scope
// (health_monitor_node.{hpp,cpp} only) -- the same class of accepted,
// out-of-scope inconsistency already established for OFFBOARD_STREAM_STALE
// (SDD §10.3's own note) and GIMBAL_LINK_LOST (FR-084). This node publishes
// the GCS_LINK_LOST fault annotated ACTION_HOLD (the SDD-intended action)
// at a non-critical severity specifically so it does NOT fall into
// mission_manager_node's blanket "any critical fault -> RTL" reaction --
// an unintended RTL would be worse than the documented gap of no HOLD.
//
// `OFFBOARD_STREAM_STALE` and `GIMBAL_LINK_LOST` remain out of scope here
// for the reasons above/FR-084.
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

  // Test seam: overrides the default systemd-restart transport (a real
  // SDD §10.4 Unix-socket request, see systemd_restart_client.hpp) with
  // `trigger`. Argument is the systemd unit name to restart (e.g.
  // "mission-exec.service").
  void set_restart_trigger(std::function<void(const std::string &)> trigger);

private:
  // One tracked "is this critical node alive" source (SDD §10.2). Most
  // entries only care about arrival timing (a single
  // rclcpp::GenericSubscription per entry, message type only needed as a
  // runtime-resolved string). Two entries (fcc_bridge_node,
  // gcs_mavlink_node) are wired as typed subscriptions instead -- HEALTH-003
  // also needs their message content (fcc_link_ok/battery_pct,
  // gcs_link_ok), not just arrival timing -- but still update the same
  // bookkeeping fields here, so node_alive_mask computation in
  // sample_and_publish() stays uniform across all 9 regardless of which
  // kind of subscription backs a given entry.
  struct NodeWatch
  {
    std::string node_name;
    uint16_t mask_bit{0};
    double timeout_s{0.0};
    rclcpp::Time last_received;
    bool ever_received{false};
    rclcpp::SubscriptionBase::SharedPtr sub;
  };

  void wire_node_watches();
  bool is_alive(const NodeWatch & watch, const rclcpp::Time & current) const;

  void on_gcs_command(mc_msgs::msg::GcsCommand::SharedPtr msg);

  // Each check_*() reads whatever continuous-state input it needs (a
  // cached latest message, or a value already computed this tick) and:
  //   1. updates the corresponding fault_mask_ bit to reflect current
  //      truth (always, every tick -- mirrors node_alive_mask's own
  //      always-accurate semantics), then
  //   2. on the false->true edge only, publishes the annotated /mc/fault
  //      FaultEvent and (MANAGER_DEAD's final-resort case only) commands
  //      the FCC (HEALTH-003's own "does not repeatedly re-fire ... while
  //      the condition persists" acceptance criterion).
  // check_manager_heartbeat additionally retries on a cooldown (not a
  // single edge) so a node that never comes back on its own still gets 3
  // real restart attempts before RTL escalation -- see its own comment.
  void check_manager_heartbeat(bool alive_now, const rclcpp::Time & current);
  void check_fcc_link();
  void check_low_battery();
  void check_gcs_link();
  void check_gpu_overtemp(uint8_t gpu_temp_c);
  // INFRA-007: vision.service cgroup memory sustained above threshold
  // (ICD §10.8 VISION_MEM_HIGH). Unlike the other check_*() methods, this
  // one is time-windowed rather than edge-on-first-sample -- it must stay
  // above vision_mem_threshold_pct_ for vision_mem_sustained_s_ before
  // firing, so a transient spike (e.g. a TensorRT engine reload) doesn't
  // false-alarm.
  void check_vision_memory(const rclcpp::Time & current);

  bool mission_phase_requires_gcs_link_hold() const;
  float effective_low_battery_threshold() const;
  // The ONLY legitimate caller of this is check_manager_heartbeat()'s
  // FR-075 final-resort RTL -- see this file's own top-of-file note on why
  // every other branch deliberately does not call this.
  void send_fcc_command(uint32_t cmd);
  void publish_fault(
    uint16_t fault_id, uint8_t severity, const std::string & description, uint8_t action);
  void publish_status_text(uint8_t severity, const std::string & text);
  void throttle_inference_rate();

  ResourceSampler sampler_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::Publisher<mc_msgs::msg::SystemHealth>::SharedPtr pub_health_;
  rclcpp::Publisher<mc_msgs::msg::FaultEvent>::SharedPtr pub_fault_;
  rclcpp::Publisher<mc_msgs::msg::FccCommand>::SharedPtr pub_send_command_;
  rclcpp::Publisher<mc_msgs::msg::StatusText>::SharedPtr pub_status_text_;
  rclcpp::Time start_time_;

  std::vector<NodeWatch> node_watches_;
  std::size_t manager_watch_index_{0};
  bool manager_was_alive_{false};
  double manager_heartbeat_timeout_s_{5.0};
  std::function<void(const std::string &)> restart_trigger_;
  std::string watchdog_socket_path_;
  int watchdog_timeout_ms_{500};

  // FR-075 restart-then-RTL escalation state. manager_restart_count_ counts
  // attempts made against the CURRENT continuous outage (reset to 0 the
  // moment the heartbeat is next seen alive, so a fresh, unrelated future
  // outage gets its own full budget). manager_escalated_ latches once the
  // direct RTL has been commanded, so the decision doesn't repeatedly
  // re-fire every retry period while the manager stays dead afterward.
  int manager_restart_count_{0};
  bool manager_escalated_{false};
  rclcpp::Time last_manager_action_time_;
  double manager_restart_retry_s_{10.0};

  rclcpp::Subscription<mc_msgs::msg::MissionPhase>::SharedPtr sub_mission_phase_;
  rclcpp::Subscription<mc_msgs::msg::MissionUpload>::SharedPtr sub_mission_upload_;
  rclcpp::Subscription<mc_msgs::msg::GcsCommand>::SharedPtr sub_gcs_command_;
  mc_msgs::msg::VehicleState::SharedPtr latest_vehicle_state_;
  mc_msgs::msg::LinkStatus::SharedPtr latest_link_status_;
  mc_msgs::msg::MissionPhase::SharedPtr latest_mission_phase_;
  mc_msgs::msg::MissionUpload::SharedPtr latest_mission_upload_;

  uint16_t fault_mask_{0};
  bool fcc_link_was_ok_{false};
  bool gcs_link_was_ok_{false};
  bool low_battery_was_active_{false};
  bool gpu_overtemp_was_active_{false};
  uint8_t gpu_overtemp_threshold_c_{85};

  // INFRA-007: VISION_MEM_HIGH sustained-threshold state.
  // vision_mem_over_since_ is set the moment usage first crosses the
  // threshold and cleared the moment it drops back below -- so it always
  // reflects the start of the CURRENT excursion, not any prior one.
  bool vision_mem_high_active_{false};
  std::optional<rclcpp::Time> vision_mem_over_since_;
  uint8_t vision_mem_threshold_pct_{80};
  double vision_mem_sustained_s_{30.0};

  bool override_low_battery_rtl_{false};
  bool override_gcs_link_hold_{false};

  rclcpp::AsyncParametersClient::SharedPtr perception_down_param_client_;
};

}  // namespace health_monitor
