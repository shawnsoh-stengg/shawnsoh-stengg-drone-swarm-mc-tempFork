// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <memory>

#include "mc_msgs/msg/fcc_command.hpp"
#include "mc_msgs/msg/fcc_mission.hpp"
#include "mc_msgs/msg/fcc_mission_ack.hpp"
#include "mc_msgs/msg/fcc_mission_item.hpp"
#include "mc_msgs/msg/fault_event.hpp"
#include "mc_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "fcc_bridge/fcc_backend.hpp"

// FCC-004: fcc_bridge_node — topic adapter, liveness monitoring, fault
// publication (SDD §3.4, ICD §7). Zero business logic: it instantiates a
// FccBackend via the factory, republishes its aggregated VehicleState, and
// reports FCC_LINK_LOST when the backend's link goes stale.
// FCC-005: adds mission upload/item-update/command forwarding — still zero
// business logic, an adapter only (no mission validation, frame conversion,
// or retry policy). The /mc/fcc/imu, /mc/fcc/odometry relays (SDD §3.4 item
// 4) remain tracked separately.
// FCC-006: adds /mc/fcc/mission_ack — raw success/failure of each forwarded
// upload/item-update, reported by FccBackend. Still zero business logic:
// deciding HOLD/RTL/mission-abort from repeated failures stays
// mission_manager's/formation_controller's job (SDD §10.3 ownership split).
namespace fcc_bridge
{

class FccBridgeNode : public rclcpp::Node
{
public:
  // FCC-005 test seam: pass a non-null `backend` to bypass the
  // SIMULATION_MODE factory (make_fcc_backend) — used by node-level tests
  // that need to observe mission/command forwarding without a live FCC or
  // real MAVLink socket. Production call sites never pass this.
  explicit FccBridgeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<FccBackend> backend = nullptr);

private:
  void on_publish_timer();
  void on_mission_upload(const mc_msgs::msg::FccMission::SharedPtr msg);
  void on_mission_item_update(const mc_msgs::msg::FccMissionItem::SharedPtr msg);
  void on_send_command(const mc_msgs::msg::FccCommand::SharedPtr msg);
  mc_msgs::msg::VehicleState to_msg(const VehicleState & state, bool link_alive) const;
  static MissionItem to_mission_item(const mc_msgs::msg::FccMissionItem & msg);
  void publish_mission_ack(uint8_t request_type, uint32_t upload_id, uint16_t seq, bool success);

  std::unique_ptr<FccBackend> backend_;
  double link_timeout_s_;

  rclcpp::Publisher<mc_msgs::msg::VehicleState>::SharedPtr pub_vehicle_state_;
  rclcpp::Publisher<mc_msgs::msg::FaultEvent>::SharedPtr pub_fault_;
  rclcpp::Publisher<mc_msgs::msg::FccMissionAck>::SharedPtr pub_mission_ack_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Mission upload and item-update share one mutually-exclusive group: both
  // drive the same underlying MissionProtocolClient/socket in FccBackend,
  // which is not safe for concurrent invocation, so they must serialize
  // against each other and against themselves — and this group is distinct
  // from timer_cb_group_ so mission traffic never blocks the 10 Hz
  // vehicle_state publish (FCC-005 AC).
  rclcpp::CallbackGroup::SharedPtr mission_cb_group_;
  // send_command() only touches the DDS command publisher, independent of
  // the mission socket — a separate reentrant group so it is never queued
  // behind in-flight mission handshakes.
  rclcpp::CallbackGroup::SharedPtr command_cb_group_;

  rclcpp::Subscription<mc_msgs::msg::FccMission>::SharedPtr sub_mission_upload_;
  rclcpp::Subscription<mc_msgs::msg::FccMissionItem>::SharedPtr sub_mission_item_update_;
  rclcpp::Subscription<mc_msgs::msg::FccCommand>::SharedPtr sub_send_command_;

  // Edge-triggers FCC_LINK_LOST — publish once per alive->dead transition,
  // not on every tick the link stays down. Starts true: FccBackend also
  // treats "no messages yet" as alive within its own grace window (see
  // XrceDdsBackend ctor), so this matches the backend's own startup state.
  bool link_was_alive_{true};
};

}  // namespace fcc_bridge
