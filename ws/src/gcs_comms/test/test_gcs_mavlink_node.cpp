// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

// GCS-001: gcs_mavlink_node integration tests — mock GCS MAVLink peer (a raw
// loopback UDP socket standing in for `mock_gcs_node`, TP-MC-001 §2.4) sends
// HEARTBEAT/COMMAND_LONG frames on Channel B; assert correct GcsCommand
// decode and link_status heartbeat tracking. Same technique as
// test_fcc_bridge_node.cpp/test_mission_protocol_client.cpp: a loopback UDP
// peer rather than launch_testing infrastructure.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/link_status.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gcs_comms/gcs_mavlink_node.hpp"

namespace
{
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

::testing::Environment * const kRclcppEnv =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

// Spins `node` until `predicate()` is true or `timeout` elapses.
template<typename Predicate>
bool spin_until(
  const rclcpp::Node::SharedPtr & node, Predicate predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// Mock GCS MAVLink peer — a plain UDP socket that sends frames straight at
// gcs_mavlink_node's loopback port, standing in for the real
// mavlink-router-channel-b.conf relay (which just forwards bytes unchanged).
class MockGcsPeer
{
public:
  explicit MockGcsPeer(uint16_t target_port)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    target_.sin_family = AF_INET;
    target_.sin_port = htons(target_port);
    target_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    // Explicit bind (ephemeral port) rather than relying on an implicit
    // bind-on-first-sendto: the mission-upload tests spawn a thread that
    // calls recv_frame() before this peer has necessarily sent anything, and
    // recv() on a not-yet-bound socket can't receive.
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = 0;
    local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr));
  }

  ~MockGcsPeer() {::close(fd_);}

  void send_heartbeat()
  {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
      kGcsSysId, kGcsCompId, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    send_frame(msg);
  }

  void send_command_long(
    uint8_t target_system, uint8_t target_component, uint16_t command,
    const std::array<float, 7> & params)
  {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
      kGcsSysId, kGcsCompId, &msg, target_system, target_component, command, /*confirmation=*/ 1,
      params[0], params[1], params[2], params[3], params[4], params[5], params[6]);
    send_frame(msg);
  }

  // GCS-002 mission upload (ICD §4.6/§4.1: target_system=1, target_component
  // =191, mission_type=kMcMissionType).
  void send_mission_count(uint16_t count, uint32_t opaque_id = 0)
  {
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
      kGcsSysId, kGcsCompId, &msg, /*target_system=*/ 1, /*target_component=*/ 191, count,
      kMcMissionType, opaque_id);
    send_frame(msg);
  }

  void send_mission_item_int(const mavlink_mission_item_int_t & item)
  {
    mavlink_message_t msg;
    mavlink_msg_mission_item_int_pack(
      kGcsSysId, kGcsCompId, &msg, /*target_system=*/ 1, /*target_component=*/ 191, item.seq,
      item.frame, item.command, item.current, item.autocontinue, item.param1, item.param2,
      item.param3, item.param4, item.x, item.y, item.z, kMcMissionType);
    send_frame(msg);
  }

  // Blocking receive with timeout — used only by the mission-upload tests,
  // which (unlike the fire-and-forget heartbeat/command tests above) must
  // read the node's MISSION_REQUEST_INT/MISSION_ACK replies. Parses on a
  // different MAVLink channel (COMM_1) than the node's own internal COMM_0
  // parse state, matching test_mission_protocol_client.cpp's convention for
  // the same reason (separate per-channel parser state).
  bool recv_frame(double timeout_s, mavlink_message_t & out)
  {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_s);
    tv.tv_usec = static_cast<suseconds_t>((timeout_s - static_cast<double>(tv.tv_sec)) * 1e6);
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t raw[512];
    const ssize_t n = ::recv(fd_, raw, sizeof(raw), 0);
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

  static constexpr uint8_t kGcsSysId = 255;   // ICD §4.1
  static constexpr uint8_t kGcsCompId = 1;
  static constexpr uint8_t kMcMissionType = 3;  // ICD §4.6

private:
  void send_frame(const mavlink_message_t & msg)
  {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ::sendto(
      fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&target_), sizeof(target_));
  }

  int fd_;
  sockaddr_in target_{};
};
}  // namespace

class GcsMavlinkNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setenv("DRONE_ID", "1", 1);
    setenv("GCS_MAVLINK_LOOPBACK_PORT", "25560", 1);

    rclcpp::NodeOptions options;
    options.append_parameter_override("gcs_link_timeout_s", 0.3);
    // Shrunk from the SDD §11.2 defaults (1 s / 5 s) so the dropped-item
    // retry test below doesn't have to burn a full second of wall-clock.
    options.append_parameter_override("mission_request_retry_s", 0.2);
    options.append_parameter_override("mission_upload_timeout_s", 2.0);
    node_ = std::make_shared<gcs_comms::GcsMavlinkNode>(options);
    peer_ = std::make_shared<MockGcsPeer>(25560);

    sub_command_ = node_->create_subscription<mc_msgs::msg::GcsCommand>(
      "/mc/gcs/command", rclcpp::QoS(10),
      [this](mc_msgs::msg::GcsCommand::SharedPtr msg) {command_msgs_.push_back(msg);});
    sub_link_status_ = node_->create_subscription<mc_msgs::msg::LinkStatus>(
      "/mc/gcs/link_status", rclcpp::QoS(10),
      [this](mc_msgs::msg::LinkStatus::SharedPtr msg) {link_status_msgs_.push_back(msg);});
    sub_mission_upload_ = node_->create_subscription<mc_msgs::msg::MissionUpload>(
      "/mc/gcs/mission_upload", rclcpp::QoS(10),
      [this](mc_msgs::msg::MissionUpload::SharedPtr msg) {mission_upload_msgs_.push_back(msg);});
  }

  void TearDown() override
  {
    peer_.reset();
    node_.reset();
    unsetenv("DRONE_ID");
    unsetenv("GCS_MAVLINK_LOOPBACK_PORT");
  }

  std::shared_ptr<gcs_comms::GcsMavlinkNode> node_;
  std::shared_ptr<MockGcsPeer> peer_;
  rclcpp::Subscription<mc_msgs::msg::GcsCommand>::SharedPtr sub_command_;
  rclcpp::Subscription<mc_msgs::msg::LinkStatus>::SharedPtr sub_link_status_;
  rclcpp::Subscription<mc_msgs::msg::MissionUpload>::SharedPtr sub_mission_upload_;
  std::vector<mc_msgs::msg::GcsCommand::SharedPtr> command_msgs_;
  std::vector<mc_msgs::msg::LinkStatus::SharedPtr> link_status_msgs_;
  std::vector<mc_msgs::msg::MissionUpload::SharedPtr> mission_upload_msgs_;
};

TEST_F(GcsMavlinkNodeTest, ConstructsWithExpectedName)
{
  EXPECT_STREQ(node_->get_name(), "gcs_mavlink_node");
}

TEST_F(GcsMavlinkNodeTest, HeartbeatFromGcsMarksLinkOk)
{
  auto hb_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});

  ASSERT_TRUE(
    spin_until(
      node_,
      [this] {return !link_status_msgs_.empty() && link_status_msgs_.back()->gcs_link_ok;},
      std::chrono::seconds(3)));

  EXPECT_GT(link_status_msgs_.back()->last_heartbeat_us, 0U);
  EXPECT_GT(link_status_msgs_.back()->rx_packet_count, 0U);
}

// gcs_link_timeout_s is overridden to 0.3 s in SetUp() so this stays fast.
TEST_F(GcsMavlinkNodeTest, LinkGoesDownAfterHeartbeatSilence)
{
  auto hb_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});

  ASSERT_TRUE(
    spin_until(
      node_,
      [this] {return !link_status_msgs_.empty() && link_status_msgs_.back()->gcs_link_ok;},
      std::chrono::seconds(3)));

  hb_timer->cancel();

  ASSERT_TRUE(
    spin_until(
      node_, [this] {return !link_status_msgs_.back()->gcs_link_ok;},
      std::chrono::seconds(3)));
}

TEST_F(GcsMavlinkNodeTest, CommandLongDecodesToGcsCommand)
{
  auto cmd_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50),
    [this] {
      // target_system=1 (DRONE_ID), target_component=191
      // (MAV_COMP_ID_ONBOARD_COMPUTER, ICD §4.5).
      peer_->send_command_long(1, 191, 50001, {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F});
    });

  ASSERT_TRUE(
    spin_until(
      node_, [this] {return !command_msgs_.empty();}, std::chrono::seconds(3)));

  const auto & msg = *command_msgs_.back();
  EXPECT_EQ(msg.command, 50001U);
  EXPECT_FLOAT_EQ(msg.params[0], 1.0F);
  EXPECT_FLOAT_EQ(msg.params[6], 7.0F);
  EXPECT_EQ(msg.target_system, 1U);
  EXPECT_EQ(msg.target_component, 191U);
  EXPECT_EQ(msg.confirmation, 1U);
}

TEST_F(GcsMavlinkNodeTest, CommandLongForOtherComponentIsIgnored)
{
  auto cmd_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50),
    [this] {
      // Wrong target_component (not MAV_COMP_ID_ONBOARD_COMPUTER) — this MC
      // must not forward it as a GcsCommand.
      peer_->send_command_long(1, 1, 50001, {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F});
    });

  // Give the (never-arriving) publish every chance to show up before
  // asserting its absence.
  spin_until(node_, [this] {return !command_msgs_.empty();}, std::chrono::milliseconds(500));

  EXPECT_TRUE(command_msgs_.empty());
}

// GCS-002: drives the full MISSION_COUNT -> MISSION_REQUEST_INT ->
// MISSION_ITEM_INT x N -> MISSION_ACK handshake (SDD §11.2, ICD §4.6),
// covering all five MC_MISSION_UPLOAD item kinds, plus one item (the
// formation offset) whose first MISSION_ITEM_INT reply is deliberately
// dropped to exercise the mission_request_retry_s_ re-request path before
// the mission_upload_timeout_s_ abort would fire.
TEST_F(GcsMavlinkNodeTest, MissionUploadAssemblesAllItemKindsWithDroppedRetriedItem)
{
  // ICD §4.6 MC_MISSION_UPLOAD item `command` values.
  constexpr uint16_t kCmdIngressWaypoint = 50200;
  constexpr uint16_t kCmdMusterPoint = 50201;
  constexpr uint16_t kCmdFormationOffset = 50202;
  constexpr uint16_t kCmdSearchAreaVertex = 50203;
  constexpr uint16_t kCmdSwarmPeerEntry = 50204;
  constexpr uint16_t kDropSeq = 2;  // formation offset item

  auto make_item =
    [](uint16_t seq, uint16_t command, float param1, float param2, int32_t x, int32_t y, float z)
    {
      mavlink_mission_item_int_t item{};
      item.seq = seq;
      item.frame = MAV_FRAME_GLOBAL;
      item.command = command;
      item.autocontinue = 1;
      item.param1 = param1;
      item.param2 = param2;
      item.x = x;
      item.y = y;
      item.z = z;
      return item;
    };

  const std::vector<mavlink_mission_item_int_t> items = {
    // seq0: ingress waypoint — lat=1.3, lon=103.8, alt=50 m (speed param1
    // unused: no field for it on mc_msgs/MissionUpload, ICD §4.6 gap note).
    make_item(0, kCmdIngressWaypoint, /*speed=*/ 5.0F, 0.0F, 13000000, 1038000000, 50.0F),
    // seq1: muster point — folded into ingress_waypoints (ICD §4.6 gap note).
    make_item(1, kCmdMusterPoint, 0.0F, 0.0F, 13500000, 1038500000, 60.0F),
    // seq2: formation offset — drone_id=2, N=10.5 m, E=-3.25 m (mm-scaled
    // int32), D=1.0 m (native float).
    make_item(2, kCmdFormationOffset, /*drone_id=*/ 2.0F, 0.0F, 10500, -3250, 1.0F),
    // seq3/4: 2-vertex search polygon.
    make_item(
      3, kCmdSearchAreaVertex, /*vertex_index=*/ 0.0F, /*total=*/ 2.0F, 10000000, 1030000000,
      0.0F),
    make_item(
      4, kCmdSearchAreaVertex, /*vertex_index=*/ 1.0F, /*total=*/ 2.0F, 11000000, 1031000000,
      0.0F),
    // seq5: swarm peer entry — drone_id=3, IP reconstructed as 192.168.10.13.
    make_item(
      5, kCmdSwarmPeerEntry, 0.0F, 0.0F, /*drone_id=*/ 3, /*octet3=*/ 10, /*octet4=*/ 13.0F),
  };

  std::atomic<bool> mock_gcs_ok{false};
  std::thread mock_gcs([&] {
      bool dropped_once = false;
      uint16_t served = 0;
      while (served < items.size()) {
        mavlink_message_t req_msg;
        if (!peer_->recv_frame(3.0, req_msg) ||
        req_msg.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT)
        {
          return;
        }
        mavlink_mission_request_int_t req{};
        mavlink_msg_mission_request_int_decode(&req_msg, &req);
        if (req.seq != served) {
          return;  // node must request strictly in ascending order (SDD §11.2)
        }
        if (req.seq == kDropSeq && !dropped_once) {
          dropped_once = true;
          continue;  // simulate a lost MISSION_ITEM_INT — no reply this round
        }
        peer_->send_mission_item_int(items[req.seq]);
        ++served;
      }

      mavlink_message_t ack_msg;
      if (!peer_->recv_frame(3.0, ack_msg) || ack_msg.msgid != MAVLINK_MSG_ID_MISSION_ACK) {
        return;
      }
      mavlink_mission_ack_t ack{};
      mavlink_msg_mission_ack_decode(&ack_msg, &ack);
      mock_gcs_ok = dropped_once && ack.type == MAV_MISSION_ACCEPTED;
    });

  peer_->send_mission_count(static_cast<uint16_t>(items.size()));

  ASSERT_TRUE(
    spin_until(
      node_, [this] {return !mission_upload_msgs_.empty();}, std::chrono::seconds(5)));
  mock_gcs.join();

  EXPECT_TRUE(mock_gcs_ok) << "mock GCS did not see a well-formed handshake with a retried item";

  const auto & up = *mission_upload_msgs_.back();
  EXPECT_EQ(up.upload_id, 1U);

  ASSERT_EQ(up.ingress_waypoints.size(), 2U);
  EXPECT_NEAR(up.ingress_waypoints[0].pose.position.x, 1.3, 1e-5);
  EXPECT_NEAR(up.ingress_waypoints[0].pose.position.y, 103.8, 1e-5);
  EXPECT_FLOAT_EQ(up.ingress_waypoints[0].pose.position.z, 50.0F);
  EXPECT_NEAR(up.ingress_waypoints[1].pose.position.x, 1.35, 1e-5);

  ASSERT_EQ(up.formation_offsets.size(), 1U);
  EXPECT_EQ(up.formation_offsets[0].drone_id, 2U);
  EXPECT_FLOAT_EQ(up.formation_offsets[0].n, 10.5F);
  EXPECT_FLOAT_EQ(up.formation_offsets[0].e, -3.25F);
  EXPECT_FLOAT_EQ(up.formation_offsets[0].d, 1.0F);

  ASSERT_EQ(up.search_polygon.size(), 2U);
  EXPECT_NEAR(up.search_polygon[0].latitude, 1.0, 1e-5);
  EXPECT_NEAR(up.search_polygon[1].latitude, 1.1, 1e-5);

  ASSERT_EQ(up.swarm_manifest.size(), 1U);
  EXPECT_EQ(up.swarm_manifest[0].drone_id, 3U);
  EXPECT_EQ(up.swarm_manifest[0].ip_address, "192.168.10.13");
}

// No mock GCS response at all — the 2 s mission_upload_timeout_s_ override
// (SetUp()) must fire MISSION_ACK(MAV_MISSION_ERROR) and return to IDLE
// rather than blocking the state machine forever (SDD §11.2 "on timeout").
TEST_F(GcsMavlinkNodeTest, MissionUploadAbortsOnTotalTimeout)
{
  peer_->send_mission_count(1);

  // peer_->recv_frame() blocks the calling thread on its own socket, so
  // node_ needs its own thread spinning it (unlike the spin_until()-based
  // tests above, where only the calling thread ever touches node_).
  std::atomic<bool> stop_spinning{false};
  std::thread spin_thread([&] {
      while (!stop_spinning) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  mavlink_message_t ack_msg{};
  bool got_ack = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (!got_ack && std::chrono::steady_clock::now() < deadline) {
    if (peer_->recv_frame(0.5, ack_msg) && ack_msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
      got_ack = true;
    }
  }

  stop_spinning = true;
  spin_thread.join();

  ASSERT_TRUE(got_ack);
  mavlink_mission_ack_t ack{};
  mavlink_msg_mission_ack_decode(&ack_msg, &ack);
  EXPECT_EQ(ack.type, MAV_MISSION_ERROR);
  EXPECT_TRUE(mission_upload_msgs_.empty());
}
