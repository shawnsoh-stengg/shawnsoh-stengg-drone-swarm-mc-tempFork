// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include "fcc_bridge/fcc_bridge_node.hpp"

#include <chrono>
#include <string>

namespace fcc_bridge
{

namespace
{
constexpr double kDefaultVehicleStateRateHz = 10.0;
constexpr double kDefaultLinkTimeoutS = 1.0;

// MAV_SEVERITY_CRITICAL (MAVLink common.xml) — FR-015 calls FCC link loss a
// "critical fault". The failsafe response (HOLD/RTL) is health_monitor_
// node's decision from the aggregated fault mask (SDD §10.3), not this
// adapter's — failsafe_action is left at ACTION_NONE below.
constexpr uint8_t kSeverityCritical = 2;
}  // namespace

FccBridgeNode::FccBridgeNode(
  const rclcpp::NodeOptions & options, std::unique_ptr<FccBackend> backend)
: Node("fcc_bridge_node", options),
  backend_(backend ? std::move(backend) : make_fcc_backend(this)),
  link_timeout_s_(declare_parameter("link_timeout_s", kDefaultLinkTimeoutS))
{
  const double rate_hz = declare_parameter("vehicle_state_rate_hz", kDefaultVehicleStateRateHz);

  pub_vehicle_state_ =
    create_publisher<mc_msgs::msg::VehicleState>("/mc/fcc/vehicle_state", rclcpp::QoS(10));
  pub_fault_ = create_publisher<mc_msgs::msg::FaultEvent>("/mc/fault", rclcpp::QoS(10));

  // Mutually-exclusive group: periodic publish stays serialized against
  // itself (threading model, SDD §2.1) even once other timers are added.
  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this] {on_publish_timer();},
    timer_cb_group_);

  // FCC-005: mission upload/item-update/command forwarding (SDD §3.4 item 2,
  // ICD §7.1). Own callback groups, per timer_cb_group_'s doc comment above
  // and the group comments on mission_cb_group_/command_cb_group_ in the
  // header — neither group is timer_cb_group_, so mission/command traffic
  // never serializes behind the vehicle_state publish timer.
  mission_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  command_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions mission_opts;
  mission_opts.callback_group = mission_cb_group_;
  sub_mission_upload_ = create_subscription<mc_msgs::msg::FccMission>(
    "/mc/fcc/mission_upload", rclcpp::QoS(10),
    [this](const mc_msgs::msg::FccMission::SharedPtr msg) {on_mission_upload(msg);},
    mission_opts);
  sub_mission_item_update_ = create_subscription<mc_msgs::msg::FccMissionItem>(
    "/mc/fcc/mission_item_update", rclcpp::QoS(10),
    [this](const mc_msgs::msg::FccMissionItem::SharedPtr msg) {on_mission_item_update(msg);},
    mission_opts);

  rclcpp::SubscriptionOptions command_opts;
  command_opts.callback_group = command_cb_group_;
  sub_send_command_ = create_subscription<mc_msgs::msg::FccCommand>(
    "/mc/fcc/send_command", rclcpp::QoS(10),
    [this](const mc_msgs::msg::FccCommand::SharedPtr msg) {on_send_command(msg);},
    command_opts);

  RCLCPP_INFO(
    get_logger(), "fcc_bridge_node started: vehicle_state_rate=%.1f Hz, link_timeout=%.1f s",
    rate_hz, link_timeout_s_);
}

void FccBridgeNode::on_mission_upload(const mc_msgs::msg::FccMission::SharedPtr msg)
{
  std::vector<MissionItem> items;
  items.reserve(msg->items.size());
  for (const auto & item : msg->items) {
    items.push_back(to_mission_item(item));
  }
  backend_->upload_mission(items);
}

void FccBridgeNode::on_mission_item_update(const mc_msgs::msg::FccMissionItem::SharedPtr msg)
{
  backend_->update_mission_item(msg->seq, to_mission_item(*msg));
}

void FccBridgeNode::on_send_command(const mc_msgs::msg::FccCommand::SharedPtr msg)
{
  backend_->send_command(msg->cmd, msg->params);
}

MissionItem FccBridgeNode::to_mission_item(const mc_msgs::msg::FccMissionItem & msg)
{
  MissionItem item;
  item.seq = msg.seq;
  item.frame = msg.frame;
  item.command = msg.command;
  item.lat = msg.lat;
  item.lon = msg.lon;
  item.alt = msg.alt;
  item.acceptance_radius = msg.acceptance_radius;
  item.stamp = msg.header.stamp;
  return item;
}

void FccBridgeNode::on_publish_timer()
{
  const VehicleState state = backend_->get_vehicle_state();
  const bool link_alive = backend_->is_link_alive(link_timeout_s_);

  pub_vehicle_state_->publish(to_msg(state, link_alive));

  if (link_was_alive_ && !link_alive) {
    mc_msgs::msg::FaultEvent fault;
    fault.header.stamp = now();
    fault.fault_id = mc_msgs::msg::FaultEvent::FCC_LINK_LOST;
    fault.severity = kSeverityCritical;
    fault.description =
      "FCC uXRCE-DDS link silent > " + std::to_string(link_timeout_s_) + " s";
    fault.failsafe_action = mc_msgs::msg::FaultEvent::ACTION_NONE;
    pub_fault_->publish(fault);
    RCLCPP_ERROR(get_logger(), "%s", fault.description.c_str());
  }
  link_was_alive_ = link_alive;
}

mc_msgs::msg::VehicleState FccBridgeNode::to_msg(const VehicleState & state, bool link_alive) const
{
  mc_msgs::msg::VehicleState msg;
  msg.header.stamp = state.stamp;
  msg.lat = state.lat;
  msg.lon = state.lon;
  msg.alt_msl = state.alt_msl;
  msg.x = state.x;
  msg.y = state.y;
  msg.z = state.z;
  msg.vx = state.vx;
  msg.vy = state.vy;
  msg.vz = state.vz;
  msg.heading = state.heading;
  msg.q = state.q;
  msg.nav_state = state.nav_state;
  msg.arming_state = state.arming_state;
  msg.battery_pct = state.battery_remaining;
  msg.fcc_link_ok = link_alive;
  // groundspeed, battery_v/battery_warning, and GNSS fix-quality fields are
  // not yet aggregated by FccBackend::VehicleState — left at zero/false
  // pending a backend enhancement, tracked separately from FCC-004.
  return msg;
}

}  // namespace fcc_bridge
