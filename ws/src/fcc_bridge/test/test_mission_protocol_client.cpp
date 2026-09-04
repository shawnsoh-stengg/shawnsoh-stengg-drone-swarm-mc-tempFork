// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

// FCC-002: exercises MissionProtocolClient's MAVLink mission-protocol
// handshake against a loopback UDP peer that plays the FCC's side of the
// microservice (ICD §3.3). Real MAVLink wire format via ros-jazzy-mavlink's
// generated headers — not a hand-rolled stand-in.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <atomic>
#include <chrono>
#include <thread>

#include <rclcpp/logging.hpp>

#include "fcc_bridge/mission_protocol_client.hpp"

namespace
{

// A UDP socket bound to an ephemeral loopback port, for either end of the
// test pair.
class UdpEndpoint
{
public:
  UdpEndpoint()
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;  // ask the kernel for a free port
    ::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));

    socklen_t len = sizeof(addr_);
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr_), &len);
  }
  ~UdpEndpoint() {::close(fd_);}

  int fd() const {return fd_;}
  sockaddr_in addr() const {return addr_;}

private:
  int fd_;
  sockaddr_in addr_{};
};

void send_frame(int fd, const sockaddr_in & dest, const mavlink_message_t & msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(fd, buf, len, 0, reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
}

bool recv_frame(int fd, double timeout_s, mavlink_message_t & out)
{
  timeval tv{};
  tv.tv_sec = static_cast<time_t>(timeout_s);
  tv.tv_usec = static_cast<suseconds_t>((timeout_s - static_cast<double>(tv.tv_sec)) * 1e6);
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  uint8_t raw[512];
  const ssize_t n = ::recv(fd, raw, sizeof(raw), 0);
  if (n <= 0) {
    return false;
  }
  mavlink_status_t status;
  for (ssize_t i = 0; i < n; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_1, raw[i], &out, &status)) {
      return true;
    }
  }
  return false;
}

}  // namespace

class MissionProtocolClientTest : public ::testing::Test
{
protected:
  UdpEndpoint mc_;   // the "MC" side — owned by the MissionProtocolClient under test
  UdpEndpoint fcc_;  // the "FCC" side — driven by the test as a mock peer

  fcc_bridge::MissionItem make_item(uint16_t seq)
  {
    fcc_bridge::MissionItem item;
    item.seq = seq;
    item.lat = 10000000 + seq;
    item.lon = 20000000 + seq;
    item.alt = 50.0F;
    item.acceptance_radius = 15.0F;
    return item;
  }
};

TEST_F(MissionProtocolClientTest, UploadMissionCompletesFullHandshake)
{
  fcc_bridge::MissionProtocolClient client(
    mc_.fd(), fcc_.addr(), /*system_id=*/ 1, /*component_id=*/ 191, rclcpp::get_logger("test"));

  const std::vector<fcc_bridge::MissionItem> items = {make_item(0), make_item(1), make_item(2)};

  std::atomic<bool> mock_fcc_ok{false};
  std::thread mock_fcc([&] {
    // 1. Expect MISSION_COUNT
      mavlink_message_t msg;
      if (!recv_frame(fcc_.fd(), 2.0, msg) || msg.msgid != MAVLINK_MSG_ID_MISSION_COUNT) {
        return;
      }
      mavlink_mission_count_t count_msg{};
      mavlink_msg_mission_count_decode(&msg, &count_msg);
      if (count_msg.count != items.size()) {
        return;
      }

    // 2. Request each item in order, verify payload round-trips
      for (uint16_t seq = 0; seq < count_msg.count; ++seq) {
        mavlink_message_t req_msg;
        mavlink_msg_mission_request_int_pack(1, 1, &req_msg, 1, 191, seq, MAV_MISSION_TYPE_MISSION);
        send_frame(fcc_.fd(), mc_.addr(), req_msg);

        mavlink_message_t item_msg;
        if (!recv_frame(fcc_.fd(), 2.0,
        item_msg) || item_msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT)
        {
          return;
        }
        mavlink_mission_item_int_t item{};
        mavlink_msg_mission_item_int_decode(&item_msg, &item);
        if (item.seq != seq || item.x != items[seq].lat || item.y != items[seq].lon) {
          return;
        }
      }

    // 3. Accept
      mavlink_message_t ack_msg;
      mavlink_msg_mission_ack_pack(1, 1, &ack_msg, 1, 191, MAV_MISSION_ACCEPTED,
      MAV_MISSION_TYPE_MISSION, 0);
      send_frame(fcc_.fd(), mc_.addr(), ack_msg);
      mock_fcc_ok = true;
    });

  const bool result = client.upload_mission(items);
  mock_fcc.join();

  EXPECT_TRUE(mock_fcc_ok) << "mock FCC did not see a well-formed handshake";
  EXPECT_TRUE(result);
}

TEST_F(MissionProtocolClientTest, UpdateMissionItemUsesPartialListNotFullUpload)
{
  fcc_bridge::MissionProtocolClient client(
    mc_.fd(), fcc_.addr(), /*system_id=*/ 1, /*component_id=*/ 191, rclcpp::get_logger("test"));

  const auto item = make_item(fcc_bridge::FORMATION_SLOT);

  std::atomic<bool> mock_fcc_ok{false};
  std::thread mock_fcc([&] {
      mavlink_message_t msg;
      if (!recv_frame(fcc_.fd(), 2.0,
      msg) || msg.msgid != MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST)
      {
        return;
      }
      mavlink_mission_write_partial_list_t partial{};
      mavlink_msg_mission_write_partial_list_decode(&msg, &partial);
      if (partial.start_index != fcc_bridge::FORMATION_SLOT ||
      partial.end_index != fcc_bridge::FORMATION_SLOT)
      {
        return;
      }

      mavlink_message_t req_msg;
      mavlink_msg_mission_request_int_pack(
      1, 1, &req_msg, 1, 191, fcc_bridge::FORMATION_SLOT, MAV_MISSION_TYPE_MISSION);
      send_frame(fcc_.fd(), mc_.addr(), req_msg);

      mavlink_message_t item_msg;
      if (!recv_frame(fcc_.fd(), 2.0,
      item_msg) || item_msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT)
      {
        return;
      }

      mavlink_message_t ack_msg;
      mavlink_msg_mission_ack_pack(1, 1, &ack_msg, 1, 191, MAV_MISSION_ACCEPTED,
      MAV_MISSION_TYPE_MISSION, 0);
      send_frame(fcc_.fd(), mc_.addr(), ack_msg);
      mock_fcc_ok = true;
    });

  const bool result =
    client.update_mission_item(fcc_bridge::FORMATION_SLOT, item, /*timeout_s=*/ 2.0);
  mock_fcc.join();

  EXPECT_TRUE(mock_fcc_ok) <<
    "mock FCC did not see MISSION_WRITE_PARTIAL_LIST (client fell back to full upload?)";
  EXPECT_TRUE(result);
}

TEST_F(MissionProtocolClientTest, UpdateMissionItemAbortsOnTimeoutRatherThanBlocking)
{
  fcc_bridge::MissionProtocolClient client(
    mc_.fd(), fcc_.addr(), /*system_id=*/ 1, /*component_id=*/ 191, rclcpp::get_logger("test"));

  // No mock FCC thread at all — nothing ever replies.
  const auto start = std::chrono::steady_clock::now();
  const bool result =
    client.update_mission_item(fcc_bridge::FORMATION_SLOT, make_item(0), /*timeout_s=*/ 0.3);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result);
  // SDD §5.3: must abort near the requested timeout, not hang the ≥5 Hz
  // control loop indefinitely.
  EXPECT_LT(elapsed, std::chrono::seconds(2));
}
