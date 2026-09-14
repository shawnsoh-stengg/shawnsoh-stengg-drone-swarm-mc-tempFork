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
#include <utility>
#include <vector>

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
        if (!recv_frame(
          fcc_.fd(), 2.0,
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
      mavlink_msg_mission_ack_pack(
        1, 1, &ack_msg, 1, 191, MAV_MISSION_ACCEPTED,
        MAV_MISSION_TYPE_MISSION, 0);
      send_frame(fcc_.fd(), mc_.addr(), ack_msg);
      mock_fcc_ok = true;
    });

  const bool result = client.upload_mission(items);
  mock_fcc.join();

  EXPECT_TRUE(mock_fcc_ok) << "mock FCC did not see a well-formed handshake";
  EXPECT_TRUE(result);
}

namespace
{
// Plays the FCC's side of a full MISSION_COUNT -> MISSION_ITEM_INT x N ->
// MISSION_ACK handshake, recording each item's (lat, lon) it was sent.
// Shared by the upload and FCC-010 re-upload tests below.
bool run_mock_fcc_full_handshake(
  int fcc_fd, const sockaddr_in & mc_addr, uint16_t expected_count,
  std::vector<std::pair<int32_t, int32_t>> & received_items)
{
  mavlink_message_t msg;
  if (!recv_frame(fcc_fd, 2.0, msg) || msg.msgid != MAVLINK_MSG_ID_MISSION_COUNT) {
    return false;
  }
  mavlink_mission_count_t count_msg{};
  mavlink_msg_mission_count_decode(&msg, &count_msg);
  if (count_msg.count != expected_count) {
    return false;
  }

  for (uint16_t seq = 0; seq < count_msg.count; ++seq) {
    mavlink_message_t req_msg;
    mavlink_msg_mission_request_int_pack(1, 1, &req_msg, 1, 191, seq, MAV_MISSION_TYPE_MISSION);
    send_frame(fcc_fd, mc_addr, req_msg);

    mavlink_message_t item_msg;
    if (!recv_frame(fcc_fd, 2.0, item_msg) || item_msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT) {
      return false;
    }
    mavlink_mission_item_int_t item{};
    mavlink_msg_mission_item_int_decode(&item_msg, &item);
    if (item.seq != seq) {
      return false;
    }
    received_items.emplace_back(item.x, item.y);
  }

  mavlink_message_t ack_msg;
  mavlink_msg_mission_ack_pack(
    1, 1, &ack_msg, 1, 191, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
  send_frame(fcc_fd, mc_addr, ack_msg);
  return true;
}
}  // namespace

// FCC-010: PX4 has no handler for MISSION_WRITE_PARTIAL_LIST at any known
// version — update_mission_item() must re-run the full upload handshake
// instead of a partial write, patching only the requested seq.
TEST_F(MissionProtocolClientTest, UpdateMissionItemReUploadsFullMissionAfterInitialUpload)
{
  fcc_bridge::MissionProtocolClient client(
    mc_.fd(), fcc_.addr(), /*system_id=*/ 1, /*component_id=*/ 191, rclcpp::get_logger("test"));

  const std::vector<fcc_bridge::MissionItem> items = {make_item(0), make_item(1), make_item(2)};

  // 1. Seed current_mission_ via a normal initial upload.
  {
    std::vector<std::pair<int32_t, int32_t>> received;
    std::thread mock_fcc(
      [&] {run_mock_fcc_full_handshake(fcc_.fd(), mc_.addr(), 3, received);});
    ASSERT_TRUE(client.upload_mission(items));
    mock_fcc.join();
  }

  // 2. Replace seq=1 — expect a full re-upload (MISSION_COUNT(3) again), not
  // MISSION_WRITE_PARTIAL_LIST, with item 1's payload changed and 0/2 intact.
  auto replacement = make_item(1);
  replacement.lat = 99000001;
  replacement.lon = 99000002;

  std::vector<std::pair<int32_t, int32_t>> received;
  std::atomic<bool> mock_fcc_ok{false};
  std::thread mock_fcc([&] {
      mock_fcc_ok = run_mock_fcc_full_handshake(fcc_.fd(), mc_.addr(), 3, received);
    });

  const bool result = client.update_mission_item(1, replacement, /*timeout_s=*/ 2.0);
  mock_fcc.join();

  ASSERT_TRUE(mock_fcc_ok) <<
    "mock FCC did not see a full 3-item re-upload (client sent MISSION_WRITE_PARTIAL_LIST?)";
  EXPECT_TRUE(result);
  ASSERT_EQ(received.size(), 3U);
  EXPECT_EQ(received[0].first, items[0].lat);
  EXPECT_EQ(received[1].first, replacement.lat);
  EXPECT_EQ(received[1].second, replacement.lon);
  EXPECT_EQ(received[2].first, items[2].lat);
}

TEST_F(MissionProtocolClientTest, UpdateMissionItemFailsFastWithoutAPriorUpload)
{
  fcc_bridge::MissionProtocolClient client(
    mc_.fd(), fcc_.addr(), /*system_id=*/ 1, /*component_id=*/ 191, rclcpp::get_logger("test"));

  // No upload_mission() call at all — current_mission_ is empty, so seq 0
  // is out of range and this must fail without ever touching the network
  // (no mock FCC thread here; a stray datagram would fail the test below).
  const bool result = client.update_mission_item(fcc_bridge::FORMATION_SLOT, make_item(0), 0.3);
  EXPECT_FALSE(result);

  mavlink_message_t stray;
  EXPECT_FALSE(recv_frame(fcc_.fd(), 0.1, stray)) <<
    "update_mission_item() sent something despite no prior upload_mission()";
}

TEST_F(MissionProtocolClientTest, UpdateMissionItemAbortsOnTimeoutRatherThanBlocking)
{
  fcc_bridge::MissionProtocolClient client(
    mc_.fd(), fcc_.addr(), /*system_id=*/ 1, /*component_id=*/ 191, rclcpp::get_logger("test"));

  // Seed current_mission_ first so seq=0 is in range and the timeout below
  // is exercised by the re-upload's own handshake, not the range guard.
  {
    std::vector<std::pair<int32_t, int32_t>> received;
    std::thread mock_fcc(
      [&] {run_mock_fcc_full_handshake(fcc_.fd(), mc_.addr(), 1, received);});
    ASSERT_TRUE(client.upload_mission({make_item(0)}));
    mock_fcc.join();
  }

  // No mock FCC thread this time — nothing ever replies to the re-upload.
  const auto start = std::chrono::steady_clock::now();
  const bool result =
    client.update_mission_item(fcc_bridge::FORMATION_SLOT, make_item(0), /*timeout_s=*/ 0.3);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result);
  // SDD §5.3: must abort near the requested timeout, not hang the ≥2 Hz
  // control loop indefinitely.
  EXPECT_LT(elapsed, std::chrono::seconds(2));
}
