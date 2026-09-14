// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include "fcc_bridge/mission_protocol_client.hpp"

#include <sys/socket.h>
#include <sys/time.h>

#include <algorithm>
#include <chrono>

namespace fcc_bridge
{

namespace
{
constexpr uint8_t kTargetSystem = 1;     // PX4 default system ID (ICD §3.3)
constexpr uint8_t kTargetComponent = 1;  // PX4 default component ID
}  // namespace

MissionProtocolClient::MissionProtocolClient(
  int sock_fd, sockaddr_in target_addr, uint8_t system_id, uint8_t component_id,
  rclcpp::Logger logger)
: sock_fd_(sock_fd),
  target_addr_(target_addr),
  system_id_(system_id),
  component_id_(component_id),
  logger_(logger)
{
}

void MissionProtocolClient::send_mission_count(uint16_t count)
{
  mavlink_message_t msg;
  mavlink_msg_mission_count_pack(
    system_id_, component_id_, &msg, kTargetSystem, kTargetComponent, count,
    MAV_MISSION_TYPE_MISSION, 0);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(
    sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&target_addr_),
    sizeof(target_addr_));
}

void MissionProtocolClient::send_mission_item_int(const MissionItem & item)
{
  mavlink_message_t msg;
  // current=0: PX4 does not (re)activate the item on write; a separate
  // MAV_CMD_DO_SET_MISSION_CURRENT / MISSION_START drives activation
  // (mission_manager_node, SDD §7.2). autocontinue=1: proceed to next item.
  mavlink_msg_mission_item_int_pack(
    system_id_, component_id_, &msg, kTargetSystem, kTargetComponent, item.seq, item.frame,
    item.command, /*current=*/ 0, /*autocontinue=*/ 1, /*param1 hold_time=*/ 0.0F,
    /*param2 acceptance_radius=*/ item.acceptance_radius, /*param3 pass_radius=*/ 0.0F,
    /*param4 yaw=*/ NAN, item.lat, item.lon, item.alt, MAV_MISSION_TYPE_MISSION);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(
    sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&target_addr_),
    sizeof(target_addr_));
}

bool MissionProtocolClient::recv_frame(double timeout_s, mavlink_message_t & out_msg)
{
  timeval tv{};
  tv.tv_sec = static_cast<time_t>(timeout_s);
  tv.tv_usec = static_cast<suseconds_t>((timeout_s - static_cast<double>(tv.tv_sec)) * 1e6);
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  uint8_t raw[512];
  const ssize_t n = ::recv(sock_fd_, raw, sizeof(raw), 0);
  if (n <= 0) {
    return false;  // timeout or error
  }

  mavlink_status_t status;
  for (ssize_t i = 0; i < n; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, raw[i], &out_msg, &status)) {
      return true;
    }
  }
  return false;
}

bool MissionProtocolClient::upload_mission(
  const std::vector<MissionItem> & items, double overall_timeout_s)
{
  send_mission_count(static_cast<uint16_t>(items.size()));

  uint16_t next_expected = 0;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(overall_timeout_s));
  // Each recv_frame() wait is capped at whatever's left of the overall
  // budget, not a fixed 0.5 s — otherwise a short overall_timeout_s (as
  // update_mission_item() passes for FCC-010's FORMATION_SLOT re-upload)
  // could block well past its own deadline waiting on a single recv.
  // Never exactly 0.0: on Linux, SO_RCVTIMEO of {0,0} means "block
  // indefinitely", not "return immediately" — a 0.0 here would defeat the
  // overall deadline instead of enforcing it. A 1 ms floor keeps every
  // recv_frame() call bounded; the outer loop's own deadline check is what
  // actually aborts promptly once time runs out.
  const auto per_recv_timeout = [&] {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      return std::max(
        0.001, std::min(
          0.5, std::chrono::duration<double>(remaining).count()));
    };

  while (next_expected < items.size()) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_WARN(
        logger_, "mission upload timed out waiting for MISSION_REQUEST_INT(seq=%u)",
        next_expected);
      return false;
    }

    mavlink_message_t msg;
    if (!recv_frame(per_recv_timeout(), msg) || msg.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
      continue;
    }

    mavlink_mission_request_int_t req{};
    mavlink_msg_mission_request_int_decode(&msg, &req);

    if (req.seq != next_expected) {
      RCLCPP_WARN(
        logger_, "FCC requested out-of-order seq %u, expected %u", req.seq,
        next_expected);
      continue;
    }
    send_mission_item_int(items[req.seq]);
    ++next_expected;
  }

  // Final MISSION_ACK
  mavlink_message_t ack_msg;
  if (!recv_frame(per_recv_timeout(), ack_msg) || ack_msg.msgid != MAVLINK_MSG_ID_MISSION_ACK) {
    RCLCPP_WARN(logger_, "mission upload: no MISSION_ACK received");
    return false;
  }
  mavlink_mission_ack_t ack{};
  mavlink_msg_mission_ack_decode(&ack_msg, &ack);
  const bool accepted = ack.type == MAV_MISSION_ACCEPTED;
  if (accepted) {
    current_mission_ = items;
  }
  return accepted;
}

bool MissionProtocolClient::update_mission_item(
  uint16_t seq, const MissionItem & item, double timeout_s)
{
  // FCC-010: no known PX4 version handles MISSION_WRITE_PARTIAL_LIST, so a
  // single-item partial write can never be ACKed by real firmware. Patch
  // the item into the last successfully uploaded mission and re-run the
  // full handshake instead — bounded to timeout_s, same as the partial
  // write it replaces (SDD §5.3: mission mode has no PX4-side staleness
  // safety net, so this must not block the ≥2 Hz formation-controller tick
  // past timeout_s — the caller retries on its next cycle rather than
  // waiting longer).
  if (seq >= current_mission_.size()) {
    RCLCPP_ERROR(
      logger_,
      "update_mission_item(seq=%u): no cached mission to update (only %zu item(s) known) — "
      "upload_mission() must succeed at least once first",
      seq, current_mission_.size());
    return false;
  }

  std::vector<MissionItem> updated_mission = current_mission_;
  updated_mission[seq] = item;
  return upload_mission(updated_mission, timeout_s);
}

}  // namespace fcc_bridge
