// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include "fcc_bridge/mavlink_udp_backend.hpp"
#include "fcc_bridge/mavlink_ids.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

namespace fcc_bridge
{

namespace
{
std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr ? std::string(v) : fallback;
}

constexpr uint8_t kArmingStateDisarmed = 1;  // ICD-MC-001 §10.2
constexpr uint8_t kArmingStateArmed = 2;     // ICD-MC-001 §10.2
}  // namespace

MavlinkUdpBackend::MavlinkUdpBackend(rclcpp::Node * node)
: node_(node), drone_id_(drone_id_from_env()), last_fcc_msg_time_(node->now())
{
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    throw std::runtime_error("MavlinkUdpBackend: failed to create SITL UDP socket");
  }
  constexpr int kReuseAddr = 1;
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &kReuseAddr, sizeof(kReuseAddr));

  const int port = std::stoi(env_or("SITL_FCC_PORT", "14540"));
  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(static_cast<uint16_t>(port));
  local_addr.sin_addr.s_addr = INADDR_ANY;
  if (::bind(sock_fd_, reinterpret_cast<const sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
    ::close(sock_fd_);
    throw std::runtime_error(
            "MavlinkUdpBackend: failed to bind SITL UDP socket on port " +
            std::to_string(port));
  }

  RCLCPP_INFO(
    node_->get_logger(), "MavlinkUdpBackend: listening for PX4 SITL on UDP port %d", port);

  recv_thread_ = std::thread([this] {recv_loop();});
}

MavlinkUdpBackend::~MavlinkUdpBackend()
{
  stop_.store(true);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
  }
}

void MavlinkUdpBackend::recv_loop()
{
  while (!stop_.load()) {
    {
      std::scoped_lock lock(socket_use_mtx_);
      // Non-blocking drain: consume everything currently queued, then
      // release the lock so a waiting mission-protocol call isn't starved
      // (socket_use_mtx_'s doc comment in the header explains why the two
      // consumers can't read concurrently).
      uint8_t raw[512];
      sockaddr_in from{};
      socklen_t fromlen = sizeof(from);
      ssize_t n;
      while ((n = ::recvfrom(
          sock_fd_, raw, sizeof(raw), MSG_DONTWAIT,
          reinterpret_cast<sockaddr *>(&from), &fromlen)) > 0)
      {
        note_peer(from);
        // Distinct channel from MissionProtocolClient's MAVLINK_COMM_0
        // (mission_protocol_client.cpp) — each channel keeps its own parse
        // state machine, so the two consumers' independent recv() calls on
        // the shared fd don't corrupt each other's partial-frame state.
        mavlink_message_t msg;
        mavlink_status_t status;
        for (ssize_t i = 0; i < n; ++i) {
          if (mavlink_parse_char(MAVLINK_COMM_1, raw[i], &msg, &status)) {
            dispatch(msg);
          }
        }
        fromlen = sizeof(from);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MavlinkUdpBackend::dispatch(const mavlink_message_t & msg)
{
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      handle_global_position_int(msg);
      break;
    case MAVLINK_MSG_ID_ATTITUDE:
      handle_attitude(msg);
      break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      handle_battery_status(msg);
      break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
      handle_local_position_ned(msg);
      break;
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_heartbeat(msg);
      break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      handle_gps_raw_int(msg);
      break;
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
    case MAVLINK_MSG_ID_ALTITUDE:
      // HIF-008 flow relay (OpticalFlowRelay's own doc comment) — flow is
      // only ever populated if PX4 SITL's own simulated flow sensor sends
      // these on this connection; ALTITUDE is in PX4's default stream set.
      flow_relay_.handle_frame(msg);
      break;
    default:
      // Includes mission-protocol replies arriving outside an active
      // handshake (nothing is waiting for them) — dropped, matching
      // recv_loop_'s single responsibility of state ingestion.
      break;
  }
}

void MavlinkUdpBackend::note_peer(const sockaddr_in & addr)
{
  std::scoped_lock lock(peer_mtx_);
  peer_addr_ = addr;
  peer_known_ = true;
}

void MavlinkUdpBackend::handle_global_position_int(const mavlink_message_t & msg)
{
  mavlink_global_position_int_t pos{};
  mavlink_msg_global_position_int_decode(&msg, &pos);

  std::scoped_lock lock(state_mtx_);
  state_.lat = static_cast<double>(pos.lat) * 1e-7;
  state_.lon = static_cast<double>(pos.lon) * 1e-7;
  state_.alt_msl = static_cast<float>(pos.alt) * 1e-3F;
  // GLOBAL_POSITION_INT is the only one of the three AC-mandated message
  // types carrying velocity/heading — there is no LOCAL_POSITION_NED
  // equivalent in scope, so VehicleState.x/y/z (local Cartesian) stay at
  // their default; vx/vy/vz below are still NED, just sourced from this
  // geodetic message instead of a local-position one.
  state_.vx = static_cast<float>(pos.vx) * 1e-2F;
  state_.vy = static_cast<float>(pos.vy) * 1e-2F;
  state_.vz = static_cast<float>(pos.vz) * 1e-2F;
  if (pos.hdg != UINT16_MAX) {  // 65535 = unknown (MAVLink common.xml)
    state_.heading = static_cast<float>(pos.hdg) * 0.01F * static_cast<float>(M_PI) / 180.0F;
  }
  state_.xy_valid = true;
  state_.z_valid = true;
  state_.stamp = node_->now();
  last_fcc_msg_time_ = state_.stamp;
  fcc_msg_received_ = true;
}

void MavlinkUdpBackend::handle_attitude(const mavlink_message_t & msg)
{
  mavlink_attitude_t att{};
  mavlink_msg_attitude_decode(&msg, &att);

  // ATTITUDE carries Euler angles, not the quaternion VehicleState.q wants
  // (unlike vehicle_attitude's already-quaternion `q` field XrceDdsBackend
  // reads) — standard aerospace ZYX (yaw-pitch-roll) Euler->quaternion
  // conversion, body FRD -> NED, matching VehicleState.q's documented frame.
  const double cr = std::cos(static_cast<double>(att.roll) * 0.5);
  const double sr = std::sin(static_cast<double>(att.roll) * 0.5);
  const double cp = std::cos(static_cast<double>(att.pitch) * 0.5);
  const double sp = std::sin(static_cast<double>(att.pitch) * 0.5);
  const double cy = std::cos(static_cast<double>(att.yaw) * 0.5);
  const double sy = std::sin(static_cast<double>(att.yaw) * 0.5);

  std::scoped_lock lock(state_mtx_);
  state_.q[0] = static_cast<float>(cr * cp * cy + sr * sp * sy);  // w
  state_.q[1] = static_cast<float>(sr * cp * cy - cr * sp * sy);  // x
  state_.q[2] = static_cast<float>(cr * sp * cy + sr * cp * sy);  // y
  state_.q[3] = static_cast<float>(cr * cp * sy - sr * sp * cy);  // z
  state_.stamp = node_->now();
  last_fcc_msg_time_ = state_.stamp;
  fcc_msg_received_ = true;
}

void MavlinkUdpBackend::handle_battery_status(const mavlink_message_t & msg)
{
  mavlink_battery_status_t batt{};
  mavlink_msg_battery_status_decode(&msg, &batt);

  std::scoped_lock lock(state_mtx_);
  if (batt.battery_remaining >= 0) {
    // MAVLink battery_remaining is a 0-100 percentage (-1 = unknown);
    // VehicleState.battery_remaining is a 0.0-1.0 fraction, matching
    // px4_msgs::BatteryStatus::remaining which XrceDdsBackend copies
    // through unscaled (config/environment's LOW_BATTERY_THRESHOLD=0.20
    // confirms the fraction convention).
    state_.battery_remaining = static_cast<float>(batt.battery_remaining) / 100.0F;
  }
  state_.stamp = node_->now();
  last_fcc_msg_time_ = state_.stamp;
  fcc_msg_received_ = true;
}

void MavlinkUdpBackend::handle_local_position_ned(const mavlink_message_t & msg)
{
  mavlink_local_position_ned_t pos{};
  mavlink_msg_local_position_ned_decode(&msg, &pos);

  std::scoped_lock lock(state_mtx_);
  // Already NED meters, matching VehicleState.x/y/z's own documented frame
  // exactly -- unlike GLOBAL_POSITION_INT's geodetic fields, no scaling.
  state_.x = pos.x;
  state_.y = pos.y;
  state_.z = pos.z;
  state_.xy_valid = true;
  state_.z_valid = true;
  state_.stamp = node_->now();
  last_fcc_msg_time_ = state_.stamp;
  fcc_msg_received_ = true;
}

void MavlinkUdpBackend::handle_heartbeat(const mavlink_message_t & msg)
{
  mavlink_heartbeat_t hb{};
  mavlink_msg_heartbeat_decode(&msg, &hb);

  std::scoped_lock lock(state_mtx_);
  // MAV_MODE_FLAG_SAFETY_ARMED (bit 7, value 128) is standard across every
  // MAVLink-compliant autopilot -- unlike nav_state, arming has a real
  // standard-dialect bit to read instead of needing PX4-specific decode.
  state_.arming_state = ((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0) ?
    kArmingStateArmed : kArmingStateDisarmed;
  state_.stamp = node_->now();
  last_fcc_msg_time_ = state_.stamp;
  fcc_msg_received_ = true;
}

void MavlinkUdpBackend::handle_gps_raw_int(const mavlink_message_t & msg)
{
  mavlink_gps_raw_int_t gps{};
  mavlink_msg_gps_raw_int_decode(&msg, &gps);

  std::scoped_lock lock(state_mtx_);
  state_.fix_type = gps.fix_type;
  // eph/epv are HDOP/VDOP * 100, UINT16_MAX (65535) = unknown (MAVLink
  // common.xml) -- left at 0.0F (already gnss_ok=false's default) rather
  // than a nonsense 655.35 when unknown.
  if (gps.eph != UINT16_MAX) {
    state_.hdop = static_cast<float>(gps.eph) / 100.0F;
  }
  if (gps.epv != UINT16_MAX) {
    state_.vdop = static_cast<float>(gps.epv) / 100.0F;
  }
  state_.gnss_ok = is_gnss_ok(gps.fix_type, state_.hdop);
  state_.stamp = node_->now();
  last_fcc_msg_time_ = state_.stamp;
  fcc_msg_received_ = true;
}

bool MavlinkUdpBackend::ensure_mission_client()
{
  std::scoped_lock lock(peer_mtx_);
  if (!peer_known_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "MavlinkUdpBackend: no SITL peer seen yet — dropping mission request");
    return false;
  }
  if (!mission_client_) {
    mission_client_ = std::make_unique<MissionProtocolClient>(
      sock_fd_, peer_addr_, drone_id_, drone_id_, kMcComponentId,
      node_->get_logger());
  }
  return true;
}

bool MavlinkUdpBackend::upload_mission(const std::vector<MissionItem> & items)
{
  std::scoped_lock socket_lock(socket_use_mtx_);
  if (!ensure_mission_client()) {
    return false;
  }
  const bool accepted = mission_client_->upload_mission(items);
  if (!accepted) {
    RCLCPP_ERROR(
      node_->get_logger(), "upload_mission: SITL FCC rejected or did not ACK the mission upload");
  }
  return accepted;
}

bool MavlinkUdpBackend::update_mission_item(uint16_t seq, const MissionItem & item)
{
  std::scoped_lock socket_lock(socket_use_mtx_);
  if (!ensure_mission_client()) {
    return false;
  }
  const bool accepted = mission_client_->update_mission_item(seq, item);
  if (!accepted) {
    RCLCPP_WARN(
      node_->get_logger(),
      "update_mission_item(seq=%u): no ACK within timeout — will retry next cycle", seq);
  }
  return accepted;
}

void MavlinkUdpBackend::stream_offboard_setpoint(const OffboardSetpoint & sp)
{
  sockaddr_in target{};
  {
    std::scoped_lock lock(peer_mtx_);
    if (!peer_known_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "MavlinkUdpBackend: no SITL peer seen yet — dropping OFFBOARD setpoint");
      return;
    }
    target = peer_addr_;
  }

  // POSITION_TARGET_TYPEMASK: ignore velocity (8|16|32) + acceleration/force
  // (64|128|256) + yaw_rate (2048) — position + yaw only, matching
  // mc_msgs/FccOffboardSetpoint (ICD §9.2) and XrceDdsBackend's NaN'd
  // velocity/acceleration/jerk fields on the TrajectorySetpoint side.
  constexpr uint16_t kTypeMask = 8 | 16 | 32 | 64 | 128 | 256 | 2048;

  mavlink_message_t msg;
  mavlink_msg_set_position_target_local_ned_pack(
    drone_id_, kMcComponentId, &msg,
    static_cast<uint32_t>(node_->now().nanoseconds() / 1000000), drone_id_, kFccComponentId,
    MAV_FRAME_LOCAL_NED, kTypeMask, sp.x, sp.y, sp.z,
    /*vx=*/ 0, /*vy=*/ 0, /*vz=*/ 0, /*afx=*/ 0, /*afy=*/ 0, /*afz=*/ 0, sp.yaw, /*yaw_rate=*/ 0);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&target), sizeof(target));
}

void MavlinkUdpBackend::send_command(uint32_t cmd, const std::array<float, 7> & params)
{
  sockaddr_in target{};
  {
    std::scoped_lock lock(peer_mtx_);
    if (!peer_known_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "MavlinkUdpBackend: no SITL peer seen yet — dropping command %u", cmd);
      return;
    }
    target = peer_addr_;
  }

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    drone_id_, kMcComponentId, &msg, drone_id_, kFccComponentId,
    static_cast<uint16_t>(cmd), /*confirmation=*/ 0, params[0], params[1], params[2], params[3],
    params[4], params[5], params[6]);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&target), sizeof(target));
}

void MavlinkUdpBackend::publish_visual_odometry(const VisualOdometry & /*vo*/)
{
  // Not in FCC-007's scope (AC covers vehicle state/mission/commands only)
  // — visual_localisation_node's GNSS-denied handover has no SITL path yet.
  // A future ticket would translate this into VISION_POSITION_ESTIMATE.
  RCLCPP_WARN_ONCE(
    node_->get_logger(),
    "MavlinkUdpBackend::publish_visual_odometry: not implemented for SITL — dropped");
}

VehicleState MavlinkUdpBackend::get_vehicle_state() const
{
  std::scoped_lock lock(state_mtx_);
  return state_;
}

OpticalFlowSample MavlinkUdpBackend::get_optical_flow() const
{
  return flow_relay_.get_sample();
}

BaroAltitudeSample MavlinkUdpBackend::get_baro_altitude() const
{
  return flow_relay_.get_baro_altitude();
}

bool MavlinkUdpBackend::is_link_alive(double timeout_s) const
{
  std::scoped_lock lock(state_mtx_);
  return fcc_msg_received_ && (node_->now() - last_fcc_msg_time_).seconds() <= timeout_s;
}

}  // namespace fcc_bridge
