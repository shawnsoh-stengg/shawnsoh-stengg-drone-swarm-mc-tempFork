// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <memory>

#include "mc_msgs/msg/fcc_command.hpp"
#include "mc_msgs/msg/fcc_mission.hpp"
#include "mc_msgs/msg/fcc_mission_ack.hpp"
#include "mc_msgs/msg/fcc_mission_item.hpp"
#include "mc_msgs/msg/fcc_offboard_setpoint.hpp"
#include "mc_msgs/msg/fault_event.hpp"
#include "mc_msgs/msg/baro_altitude.hpp"
#include "mc_msgs/msg/optical_flow.hpp"
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
// FCC-009: adds /mc/fcc/offboard_setpoint forwarding (SDD §4.1a/§5.3a). Same
// zero-business-logic contract — no rate enforcement, no mode-entry
// sequencing, no staleness detection; those stay mission_manager_node's
// (ME-003) and health_monitor_node's (HM-004) jobs respectively.
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
  void on_offboard_setpoint(const mc_msgs::msg::FccOffboardSetpoint::SharedPtr msg);
  void on_send_command(const mc_msgs::msg::FccCommand::SharedPtr msg);
  mc_msgs::msg::VehicleState to_msg(const VehicleState & state, bool link_alive) const;
  static mc_msgs::msg::OpticalFlow to_flow_msg(
    const OpticalFlowSample & sample,
    rclcpp::Time stamp);
  static MissionItem to_mission_item(const mc_msgs::msg::FccMissionItem & msg);
  void publish_mission_ack(uint8_t request_type, uint32_t upload_id, uint16_t seq, bool success);

  std::unique_ptr<FccBackend> backend_;
  double link_timeout_s_;

  rclcpp::Publisher<mc_msgs::msg::VehicleState>::SharedPtr pub_vehicle_state_;
  rclcpp::Publisher<mc_msgs::msg::FaultEvent>::SharedPtr pub_fault_;
  rclcpp::Publisher<mc_msgs::msg::FccMissionAck>::SharedPtr pub_mission_ack_;
  // HIF-008 relay (ICD §3.3a) — published once per new frame (seq), and
  // only after the first valid one: nothing before it, rather than an
  // all-zero topic that would read as a real (stationary) velocity reading
  // to gnss_sim_node; and nothing after the stream stops, so the stall
  // reaches gnss_sim_node's IGS-005 staleness watchdog instead of being
  // masked by the last sample repeating every tick.
  rclcpp::Publisher<mc_msgs::msg::OpticalFlow>::SharedPtr pub_optical_flow_;
  uint32_t last_flow_seq_{0};
  // Raw baro altitude (ALTITUDE, same channel) — same once-per-frame rule.
  rclcpp::Publisher<mc_msgs::msg::BaroAltitude>::SharedPtr pub_baro_altitude_;
  uint32_t last_baro_seq_{0};
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
  // Own reentrant group, same as sub_send_command_ (command_cb_group_'s doc
  // comment above) — a straight publish with no mission-socket handshake to
  // serialize against, so it must not queue behind in-flight mission
  // traffic either (FCC-009 needs this to sustain >= 2 Hz, ICD §9.1).
  rclcpp::Subscription<mc_msgs::msg::FccOffboardSetpoint>::SharedPtr sub_offboard_setpoint_;

  // Edge-triggers FCC_LINK_LOST — publish once per alive->dead transition,
  // not on every tick the link stays down. Starts false: FccBackend reads
  // "no messages yet" as link-down, so a slow first FCC message at startup
  // (DDS discovery of /fmu/out/* routinely takes > link_timeout_s on the
  // Jetson) is never mistaken for a real link loss — which mission_manager_
  // node would otherwise latch as a terminal FAULT on every boot.
  bool link_was_alive_{false};
};

}  // namespace fcc_bridge
