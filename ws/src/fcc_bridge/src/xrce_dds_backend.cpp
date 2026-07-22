// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include "fcc_bridge/xrce_dds_backend.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>

namespace fcc_bridge
{

namespace
{
// Matches the QoS PX4's uXRCE-DDS client uses on /fmu/* topics (best-effort,
// volatile, shallow queue) — required for the DDS<->uORB bridge to see us.
rclcpp::QoS px4_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr ? std::string(v) : fallback;
}
}  // namespace

XrceDdsBackend::XrceDdsBackend(rclcpp::Node * node)
: node_(node), last_fcc_msg_time_(node->now())
{
  const auto qos = px4_qos();

  sub_local_pos_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos,
    [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {on_local_pos(msg);});

  sub_global_pos_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    "/fmu/out/vehicle_global_position", qos,
    [this](px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {on_global_pos(msg);});

  sub_status_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", qos,
    [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {on_vehicle_status(msg);});

  sub_battery_ = node_->create_subscription<px4_msgs::msg::BatteryStatus>(
    "/fmu/out/battery_status", qos,
    [this](px4_msgs::msg::BatteryStatus::SharedPtr msg) {on_battery(msg);});

  sub_attitude_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
    "/fmu/out/vehicle_attitude", qos,
    [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {on_attitude(msg);});

  sub_odometry_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", qos,
    [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {on_odometry(msg);});

  sub_imu_ = node_->create_subscription<px4_msgs::msg::SensorCombined>(
    "/fmu/out/sensor_combined", qos,
    [this](px4_msgs::msg::SensorCombined::SharedPtr msg) {on_imu(msg);});

  pub_vehicle_command_ =
    node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);
  pub_visual_odometry_ =
    node_->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos);
  pub_offboard_control_mode_ =
    node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
  pub_trajectory_setpoint_ =
    node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);

  mission_sock_fd_ = open_mission_socket();

  const std::string fcc_ip = env_or("FCC_ETH_IP", "192.168.12.31");
  const int mission_port = std::stoi(env_or("FCC_MISSION_PORT", "14555"));
  sockaddr_in target_addr{};
  target_addr.sin_family = AF_INET;
  target_addr.sin_port = htons(static_cast<uint16_t>(mission_port));
  if (::inet_pton(AF_INET, fcc_ip.c_str(), &target_addr.sin_addr) != 1) {
    throw std::runtime_error("XrceDdsBackend: invalid FCC_ETH_IP '" + fcc_ip + "'");
  }

  // MC's own MAVLink identity on the mission-protocol channel. DRONE_ID
  // doubles as system_id, matching gcs_mavlink_node's convention (ICD §4.1).
  const uint8_t system_id = static_cast<uint8_t>(std::stoi(env_or("DRONE_ID", "1")));
  constexpr uint8_t kCompIdOnboardComputer = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
  mission_client_ = std::make_unique<MissionProtocolClient>(
    mission_sock_fd_, target_addr, system_id, kCompIdOnboardComputer, node_->get_logger());

  const std::string transport = env_or("FCC_TRANSPORT", "serial");
  RCLCPP_INFO(
    node_->get_logger(),
    "XrceDdsBackend: DDS transport=%s, mission protocol=%s:%d (always ethernet)",
    transport.c_str(), fcc_ip.c_str(), mission_port);
}

XrceDdsBackend::~XrceDdsBackend()
{
  // mission_client_ does not own the socket (header comment,
  // MissionProtocolClient ctor) — closing it here is our responsibility.
  if (mission_sock_fd_ >= 0) {
    ::close(mission_sock_fd_);
  }
}

int XrceDdsBackend::open_mission_socket()
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error("XrceDdsBackend: failed to create mission-protocol UDP socket");
  }
  constexpr int kReuseAddr = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &kReuseAddr, sizeof(kReuseAddr));

  // Bind to the local side of the dedicated MC<->FCC ethernet link
  // (192.168.12.1, HIF-002) so replies route back here regardless of which
  // interface currently carries the DDS primary/secondary transport.
  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(static_cast<uint16_t>(std::stoi(env_or("FCC_MISSION_PORT",
      "14555"))));
  local_addr.sin_addr.s_addr = INADDR_ANY;
  if (::bind(fd, reinterpret_cast<const sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
    ::close(fd);
    throw std::runtime_error("XrceDdsBackend: failed to bind mission-protocol UDP socket");
  }
  return fd;
}

void XrceDdsBackend::touch_link_alive()
{
  std::scoped_lock lock(state_mtx_);
  last_fcc_msg_time_ = node_->now();
}

void XrceDdsBackend::on_local_pos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  std::scoped_lock lock(state_mtx_);
  state_.x = msg->x;
  state_.y = msg->y;
  state_.z = msg->z;
  state_.vx = msg->vx;
  state_.vy = msg->vy;
  state_.vz = msg->vz;
  state_.heading = msg->heading;
  state_.xy_valid = msg->xy_valid;
  state_.z_valid = msg->z_valid;
  state_.stamp = node_->now();
  last_fcc_msg_time_ = state_.stamp;
}

void XrceDdsBackend::on_global_pos(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  std::scoped_lock lock(state_mtx_);
  state_.lat = msg->lat;
  state_.lon = msg->lon;
  state_.alt_msl = msg->alt;
  last_fcc_msg_time_ = node_->now();
}

void XrceDdsBackend::on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::scoped_lock lock(state_mtx_);
  state_.arming_state = msg->arming_state;
  state_.nav_state = msg->nav_state;
  // NOTE: stock PX4 v1.14+ VehicleStatus has no failure_detector_status
  // field (ICD §3.2 describes one; that field does not exist upstream —
  // flagged as a follow-up ICD correction, not fixed here). `failsafe` is
  // the closest available signal in the meantime.
  state_.failure_flags = msg->failsafe ? 1U : 0U;
  last_fcc_msg_time_ = node_->now();
}

void XrceDdsBackend::on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  std::scoped_lock lock(state_mtx_);
  state_.battery_remaining = msg->remaining;
  last_fcc_msg_time_ = node_->now();
}

void XrceDdsBackend::on_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  std::scoped_lock lock(state_mtx_);
  state_.q = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
  last_fcc_msg_time_ = node_->now();
}

void XrceDdsBackend::on_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr /*msg*/)
{
  // Relayed to /mc/fcc/odometry by fcc_bridge_node directly from the uORB
  // topic (SDD §3.4 item 4) — not folded into VehicleState. Still counts
  // as link activity.
  touch_link_alive();
}

void XrceDdsBackend::on_imu(const px4_msgs::msg::SensorCombined::SharedPtr /*msg*/)
{
  // Relayed to /mc/fcc/imu by fcc_bridge_node directly (SDD §3.4 item 4).
  touch_link_alive();
}

bool XrceDdsBackend::upload_mission(const std::vector<MissionItem> & items)
{
  const bool accepted = mission_client_->upload_mission(items);
  if (!accepted) {
    RCLCPP_ERROR(node_->get_logger(),
        "upload_mission: FCC rejected or did not ACK the mission upload");
  }
  return accepted;
}

bool XrceDdsBackend::update_mission_item(uint16_t seq, const MissionItem & item)
{
  const bool accepted = mission_client_->update_mission_item(seq, item);
  if (!accepted) {
    RCLCPP_WARN(
      node_->get_logger(),
        "update_mission_item(seq=%u): no ACK within timeout — will retry next cycle",
      seq);
  }
  return accepted;
}

void XrceDdsBackend::stream_offboard_setpoint(const OffboardSetpoint & sp)
{
  const auto now_us = static_cast<uint64_t>(node_->now().nanoseconds() / 1000);

  px4_msgs::msg::OffboardControlMode mode{};
  mode.timestamp = now_us;
  mode.position = true;
  // velocity/acceleration/attitude/body_rate/actuator left false — position
  // + yaw only, matches mc_msgs/FccOffboardSetpoint (ICD §9.2, FCC-009).
  pub_offboard_control_mode_->publish(mode);

  px4_msgs::msg::TrajectorySetpoint setpoint{};
  setpoint.timestamp = now_us;
  setpoint.position = {sp.x, sp.y, sp.z};
  // NaN, not zero — TrajectorySetpoint.msg: "setting a value to NaN means
  // the state should not be controlled." Zero would tell PX4 to also hold
  // zero velocity/acceleration, fighting the position setpoint.
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  setpoint.velocity = {kNaN, kNaN, kNaN};
  setpoint.acceleration = {kNaN, kNaN, kNaN};
  setpoint.jerk = {kNaN, kNaN, kNaN};
  setpoint.yaw = sp.yaw;
  setpoint.yawspeed = kNaN;
  pub_trajectory_setpoint_->publish(setpoint);
}

void XrceDdsBackend::send_command(uint32_t cmd, const std::array<float, 7> & params)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = static_cast<uint64_t>(node_->now().nanoseconds() / 1000);
  msg.command = cmd;
  msg.param1 = params[0];
  msg.param2 = params[1];
  msg.param3 = params[2];
  msg.param4 = params[3];
  // NOTE: px4_msgs::msg::VehicleCommand::param5/param6 are float64 (PX4
  // needs the extra precision for lat/lon-carrying commands e.g.
  // DO_REPOSITION, SET_HOME). FccBackend::send_command()'s float32[7]
  // signature (inherited from SDD §3.1, predating this ticket) loses that
  // precision. None of the commands currently issued by
  // mission_manager_node/health_monitor_node (ARM_DISARM, DO_SET_MODE,
  // TAKEOFF, RTL, HOLD)
  // use param5/6 as coordinates, so this is latent, not active, and is
  // flagged here rather than silently widening the abstract interface.
  msg.param5 = params[4];
  msg.param6 = params[5];
  msg.param7 = params[6];
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = static_cast<uint8_t>(std::stoi(env_or("DRONE_ID", "1")));
  msg.source_component = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
  msg.from_external = true;
  pub_vehicle_command_->publish(msg);
}

void XrceDdsBackend::publish_visual_odometry(const VisualOdometry & vo)
{
  px4_msgs::msg::VehicleOdometry msg{};
  msg.timestamp = static_cast<uint64_t>(node_->now().nanoseconds() / 1000);
  msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  msg.position = vo.position;
  msg.q = vo.q;
  msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  msg.velocity = vo.velocity;
  msg.position_variance = vo.position_variance;
  pub_visual_odometry_->publish(msg);
}

VehicleState XrceDdsBackend::get_vehicle_state() const
{
  std::scoped_lock lock(state_mtx_);
  return state_;
}

bool XrceDdsBackend::is_link_alive(double timeout_s) const
{
  std::scoped_lock lock(state_mtx_);
  return (node_->now() - last_fcc_msg_time_).seconds() <= timeout_s;
}

}  // namespace fcc_bridge
