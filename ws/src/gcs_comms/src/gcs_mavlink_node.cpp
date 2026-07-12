// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

#include "gcs_comms/gcs_mavlink_node.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace gcs_comms
{

namespace
{
constexpr double kDefaultGcsLinkTimeoutS = 10.0;  // ICD §4.8

std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr ? std::string(v) : fallback;
}

int open_loopback_socket(uint16_t port)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    throw std::runtime_error("GcsMavlinkNode: failed to create Channel B UDP socket");
  }
  constexpr int kReuseAddr = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &kReuseAddr, sizeof(kReuseAddr));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port);
  // Loopback only — this is the `GCS_MAVLINK_NODE` endpoint of
  // mavlink-router-channel-b.conf (ICD §4.2), never reachable off-box.
  local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd, reinterpret_cast<const sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
    ::close(fd);
    throw std::runtime_error(
            "GcsMavlinkNode: failed to bind Channel B UDP socket to port " +
            std::to_string(port));
  }
  return fd;
}
}  // namespace

GcsMavlinkNode::GcsMavlinkNode(const rclcpp::NodeOptions & options)
: Node("gcs_mavlink_node", options),
  gcs_link_timeout_s_(declare_parameter("gcs_link_timeout_s", kDefaultGcsLinkTimeoutS)),
  last_gcs_heartbeat_(now())
{
  system_id_ = static_cast<uint8_t>(std::stoi(env_or("DRONE_ID", "1")));
  const uint16_t loopback_port = static_cast<uint16_t>(
    std::stoi(env_or("GCS_MAVLINK_LOOPBACK_PORT", "14560")));
  sock_fd_ = open_loopback_socket(loopback_port);

  pub_command_ = create_publisher<mc_msgs::msg::GcsCommand>("/mc/gcs/command", rclcpp::QoS(10));
  pub_link_status_ =
    create_publisher<mc_msgs::msg::LinkStatus>("/mc/gcs/link_status", rclcpp::QoS(10));

  timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recv_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    [this] {on_recv_timer();},
    timer_cb_group_);
  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    [this] {on_heartbeat_timer();},
    timer_cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "gcs_mavlink_node started: system_id=%u, Channel B loopback port=%u, gcs_link_timeout=%.1f s",
    system_id_, loopback_port, gcs_link_timeout_s_);
}

GcsMavlinkNode::~GcsMavlinkNode()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
  }
}

void GcsMavlinkNode::on_recv_timer()
{
  uint8_t buf[512];
  sockaddr_in from{};
  socklen_t from_len = sizeof(from);

  while (true) {
    const ssize_t n = ::recvfrom(
      sock_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &from_len);
    if (n <= 0) {
      break;  // EAGAIN/EWOULDBLOCK — nothing more queued this tick
    }

    mavlink_message_t msg;
    mavlink_status_t status;
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        ++rx_packet_count_;
        // Cumulative since channel init, not a per-call delta — overwrite,
        // don't accumulate.
        rx_drop_count_ = status.packet_rx_drop_count;
        handle_message(msg, from);
      }
    }
  }
}

void GcsMavlinkNode::handle_message(const mavlink_message_t & msg, const sockaddr_in & from)
{
  // Learn/refresh mavlink-router's relay endpoint from every frame it
  // forwards us, not just heartbeats, so replies always target the freshest
  // known source (header comment, last_gcs_endpoint_).
  last_gcs_endpoint_ = from;
  has_gcs_endpoint_ = true;

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_heartbeat(msg);
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      handle_command_long(msg, from);
      break;
    default:
      break;
  }
}

void GcsMavlinkNode::handle_heartbeat(const mavlink_message_t & msg)
{
  mavlink_heartbeat_t hb{};
  mavlink_msg_heartbeat_decode(&msg, &hb);

  // ICD §4.3: GCS heartbeat is type=6 (MAV_TYPE_GCS), autopilot=8
  // (MAV_AUTOPILOT_INVALID) — filters out any other stray heartbeat traffic
  // that might reach this loopback (there should be none given Channel B
  // isolation, but this keeps the check meaningful rather than tautological).
  if (hb.type != MAV_TYPE_GCS || hb.autopilot != MAV_AUTOPILOT_INVALID) {
    return;
  }
  has_gcs_heartbeat_ = true;
  last_gcs_heartbeat_ = now();
}

void GcsMavlinkNode::handle_command_long(const mavlink_message_t & msg, const sockaddr_in & from)
{
  mavlink_command_long_t cmd{};
  mavlink_msg_command_long_decode(&msg, &cmd);

  // ICD §4.5: all MC-directed commands target this MC's system_id and
  // MAV_COMP_ID_ONBOARD_COMPUTER (191).
  if (cmd.target_system != system_id_ || cmd.target_component != kComponentId) {
    return;
  }

  mc_msgs::msg::GcsCommand out;
  out.header.stamp = now();
  out.command = cmd.command;
  out.params = {cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6,
    cmd.param7};
  out.target_system = cmd.target_system;
  out.target_component = cmd.target_component;
  out.confirmation = cmd.confirmation;
  pub_command_->publish(out);

  // Acknowledges receipt/decode only — actual command execution and its
  // result belong to whichever node consumes /mc/gcs/command (SDD §11
  // "zero business logic" adapter philosophy, matching fcc_bridge_node).
  send_command_ack(from, cmd.command, MAV_RESULT_ACCEPTED);
}

void GcsMavlinkNode::on_heartbeat_timer()
{
  send_heartbeat();
  publish_link_status();
}

void GcsMavlinkNode::send_heartbeat()
{
  // Nowhere to send until mavlink-router has relayed at least one inbound
  // frame — its Channel B `GCS_MAVLINK_NODE` endpoint is a "Normal" UDP
  // endpoint on an OS-assigned ephemeral port (header comment,
  // last_gcs_endpoint_), not a fixed address we can address up front. This
  // self-resolves within ~1 s of the GCS coming online since it heartbeats
  // at 1 Hz too (ICD §4.3) and mavlink-router relays immediately.
  if (!has_gcs_endpoint_) {
    return;
  }

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    system_id_, kComponentId, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
    /*base_mode=*/ 0, /*custom_mode=*/ 0, MAV_STATE_ACTIVE);
  send_frame(msg, last_gcs_endpoint_);
}

void GcsMavlinkNode::send_command_ack(const sockaddr_in & to, uint16_t command, uint8_t result)
{
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack(
    system_id_, kComponentId, &msg, command, result, /*progress=*/ 255, /*result_param2=*/ 0,
    /*target_system=*/ 255, /*target_component=*/ 0);
  send_frame(msg, to);
}

void GcsMavlinkNode::send_frame(const mavlink_message_t & msg, const sockaddr_in & to)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to));
  ++tx_packet_count_;
}

void GcsMavlinkNode::publish_link_status()
{
  const double age_s = (now() - last_gcs_heartbeat_).seconds();
  const bool link_ok = has_gcs_heartbeat_ && age_s < gcs_link_timeout_s_;

  mc_msgs::msg::LinkStatus msg;
  msg.header.stamp = now();
  msg.gcs_link_ok = link_ok;
  msg.last_heartbeat_us =
    has_gcs_heartbeat_ ? static_cast<uint64_t>(last_gcs_heartbeat_.nanoseconds() / 1000) : 0U;
  // No RSSI available over a UDP/loopback transport (ICD §7.2 marks this
  // RSSI-derived value as "if available") — binary heuristic instead.
  msg.link_quality = link_ok ? 1.0F : 0.0F;
  msg.rx_packet_count = rx_packet_count_;
  msg.tx_packet_count = tx_packet_count_;
  msg.rx_drop_count = rx_drop_count_;
  pub_link_status_->publish(msg);
}

}  // namespace gcs_comms
