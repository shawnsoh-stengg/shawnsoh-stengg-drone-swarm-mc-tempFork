// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include "fcc_bridge/mission_protocol_client.hpp"

#include <sys/socket.h>
#include <sys/time.h>

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
  // (mission_executor_node, SDD §7.2). autocontinue=1: proceed to next item.
  mavlink_msg_mission_item_int_pack(
    system_id_, component_id_, &msg, kTargetSystem, kTargetComponent, item.seq, item.frame,
    item.command, /*current=*/0, /*autocontinue=*/1, /*param1 hold_time=*/0.0F,
    /*param2 acceptance_radius=*/item.acceptance_radius, /*param3 pass_radius=*/0.0F,
    /*param4 yaw=*/NAN, item.lat, item.lon, item.alt, MAV_MISSION_TYPE_MISSION);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(
    sock_fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&target_addr_),
    sizeof(target_addr_));
}

void MissionProtocolClient::send_mission_write_partial_list(uint16_t start_seq, uint16_t end_seq)
{
  mavlink_message_t msg;
  mavlink_msg_mission_write_partial_list_pack(
    system_id_, component_id_, &msg, kTargetSystem, kTargetComponent,
    static_cast<int16_t>(start_seq), static_cast<int16_t>(end_seq), MAV_MISSION_TYPE_MISSION);

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

bool MissionProtocolClient::upload_mission(const std::vector<MissionItem> & items)
{
  send_mission_count(static_cast<uint16_t>(items.size()));

  uint16_t next_expected = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

  while (next_expected < items.size()) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_WARN(logger_, "mission upload timed out waiting for MISSION_REQUEST_INT(seq=%u)",
          next_expected);
      return false;
    }

    mavlink_message_t msg;
    if (!recv_frame(0.5, msg) || msg.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
      continue;
    }

    mavlink_mission_request_int_t req{};
    mavlink_msg_mission_request_int_decode(&msg, &req);

    if (req.seq != next_expected) {
      RCLCPP_WARN(logger_, "FCC requested out-of-order seq %u, expected %u", req.seq,
          next_expected);
      continue;
    }
    send_mission_item_int(items[req.seq]);
    ++next_expected;
  }

  // Final MISSION_ACK
  mavlink_message_t ack_msg;
  if (!recv_frame(0.5, ack_msg) || ack_msg.msgid != MAVLINK_MSG_ID_MISSION_ACK) {
    RCLCPP_WARN(logger_, "mission upload: no MISSION_ACK received");
    return false;
  }
  mavlink_mission_ack_t ack{};
  mavlink_msg_mission_ack_decode(&ack_msg, &ack);
  return ack.type == MAV_MISSION_ACCEPTED;
}

bool MissionProtocolClient::serve_single_item_request(const MissionItem & item, double timeout_s)
{
  mavlink_message_t req_msg;
  if (!recv_frame(timeout_s, req_msg) || req_msg.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
    return false;
  }

  mavlink_mission_request_int_t req{};
  mavlink_msg_mission_request_int_decode(&req_msg, &req);

  if (req.seq != item.seq) {
    return false;
  }
  send_mission_item_int(item);

  mavlink_message_t ack_msg;
  if (!recv_frame(timeout_s, ack_msg) || ack_msg.msgid != MAVLINK_MSG_ID_MISSION_ACK) {
    return false;
  }
  mavlink_mission_ack_t ack{};
  mavlink_msg_mission_ack_decode(&ack_msg, &ack);
  return ack.type == MAV_MISSION_ACCEPTED;
}

bool MissionProtocolClient::update_mission_item(
  uint16_t seq, const MissionItem & item, double timeout_s)
{
  send_mission_write_partial_list(seq, seq);
  // SDD §5.3: mission mode has no PX4-side staleness safety net, so this
  // must not block the ≥5 Hz formation-controller tick past timeout_s —
  // the caller retries on its next cycle rather than waiting longer.
  return serve_single_item_request(item, timeout_s);
}

}  // namespace fcc_bridge
