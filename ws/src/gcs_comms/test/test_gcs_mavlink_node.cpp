// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §4).

// GCS-001/GCS-003: gcs_mavlink_node integration tests — mock GCS MAVLink
// peer (a raw loopback UDP socket standing in for `mock_gcs_node`,
// TP-MC-001 §2.4) sends HEARTBEAT/COMMAND_LONG frames on Channel B; assert
// correct GcsCommand decode and link_status heartbeat tracking. Same
// technique as test_fcc_bridge_node.cpp/test_mission_protocol_client.cpp: a
// loopback UDP peer rather than launch_testing infrastructure. The GCS-003
// tests below (telemetry multiplexer) reuse the same peer but as a
// *receiver* of MC_STATUS/MC_DETECTION/MC_HEALTH, decoded by hand against
// the wire layout gcs_mavlink_node.cpp packs against (mc_mavlink_defs.xml)
// since no generated dialect header exists for these custom messages
// (gcs_mavlink_node.cpp's own comment on this).

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <mc_msgs/msg/detection.hpp>
#include <mc_msgs/msg/detection_array.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/gcs_command_ack.hpp>
#include <mc_msgs/msg/link_status.hpp>
#include <mc_msgs/msg/localisation_mode.hpp>
#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <mc_msgs/msg/status_text.hpp>
#include <mc_msgs/msg/system_health.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gcs_comms/gcs_mavlink_node.hpp"

namespace
{

// GCS-003: msgid + wire-offset constants mirroring
// gcs_comms/mc_mavlink_defs.xml / gcs_mavlink_node.cpp's pack_mc_*()
// helpers — see that file's comment for how these were derived.
constexpr uint32_t kMsgIdMcStatus = 50100;
constexpr uint32_t kMsgIdMcDetection = 50101;
constexpr uint32_t kMsgIdMcHealth = 50103;

// mavlink_parse_char() validates each message's wire checksum against a
// CRC_EXTRA value it looks up by msgid from whichever dialect(s) are
// compiled into this translation unit — only the plain common dialect
// here, which has no entry for 50100/50101/50103. Empirically (verified
// against a real wire-correct frame before writing this): it rejects every
// one of them as a checksum failure rather than returning them with a
// wrong-but-present result, so `peer_`'s existing recv_frame()/
// mavlink_parse_char() path — fine for the standard HEARTBEAT/MISSION_*
// messages the GCS-001/002 tests above use — cannot receive these three at
// all. UDP is datagram-oriented and send_frame() (gcs_mavlink_node.cpp)
// sends exactly one complete MAVLink v2 frame per sendto() call, so a raw
// datagram received here always *is* exactly one frame — decoding the
// fixed v2 header directly sidesteps mavlink_parse_char()'s dialect lookup
// entirely for these three messages.
struct RawMavlinkFrame
{
  uint32_t msgid{0};
  // 55 bytes covers the largest of the three (MC_DETECTION); zero-filled
  // before copying the received bytes over it, in try_decode_raw_frame()
  // — MAVLink v2 trims trailing zero bytes from the wire payload (a
  // bandwidth optimization every compliant implementation sends and
  // receives), so a real dialect-aware decoder would zero-pad back up to
  // the message's known full length before reading anything at/after the
  // trim point; starting from an all-zero buffer here achieves the same
  // thing for free.
  uint8_t payload[55]{};
};

bool try_decode_raw_frame(const uint8_t * buf, ssize_t n, RawMavlinkFrame & out)
{
  constexpr ssize_t kHeaderLen = 10;  // v2 core header: magic..msgid (MAVLINK_CORE_HEADER_LEN+1)
  if (n < kHeaderLen + 2 || buf[0] != MAVLINK_STX) {
    return false;
  }
  const uint8_t len = buf[1];
  if (n < kHeaderLen + len + 2 || len > sizeof(out.payload)) {
    return false;
  }
  out.msgid = static_cast<uint32_t>(buf[7]) | (static_cast<uint32_t>(buf[8]) << 8) |
    (static_cast<uint32_t>(buf[9]) << 16);
  std::memset(out.payload, 0, sizeof(out.payload));
  std::memcpy(out.payload, buf + kHeaderLen, len);
  return true;
}

// Host is always little-endian here (x86_64 dev/CI, aarch64 Jetson target)
// — matches the assumption gcs_mavlink_node.cpp's own _mav_put_*() calls
// already make (their !MAVLINK_NEED_BYTE_SWAP branch is a raw pointer
// cast), so a plain memcpy-based read is wire-correct without needing
// mavlink.h's byte-swap machinery.
template<typename T>
T read_le(const uint8_t * payload, size_t offset)
{
  T value;
  std::memcpy(&value, payload + offset, sizeof(T));
  return value;
}

struct McStatusFields
{
  uint64_t time_usec;
  int32_t lat;
  int32_t lon;
  int32_t alt_msl;
  uint16_t heading;
  uint16_t groundspeed;
  int16_t climb_rate;
  uint16_t fault_mask;
  uint8_t battery_remaining;
  uint8_t mission_phase;
  uint8_t localisation_mode;
  uint8_t peer_count;
  uint8_t isr_role;
};

McStatusFields decode_mc_status(const RawMavlinkFrame & frame)
{
  const uint8_t * p = frame.payload;
  McStatusFields f{};
  f.time_usec = read_le<uint64_t>(p, 0);
  f.lat = read_le<int32_t>(p, 8);
  f.lon = read_le<int32_t>(p, 12);
  f.alt_msl = read_le<int32_t>(p, 16);
  f.heading = read_le<uint16_t>(p, 20);
  f.groundspeed = read_le<uint16_t>(p, 22);
  f.climb_rate = read_le<int16_t>(p, 24);
  f.fault_mask = read_le<uint16_t>(p, 26);
  f.battery_remaining = p[28];
  f.mission_phase = p[29];
  f.localisation_mode = p[30];
  f.peer_count = p[31];
  f.isr_role = p[32];
  return f;
}

struct McDetectionFields
{
  uint64_t time_usec;
  uint32_t detection_id;
  int32_t lat;
  int32_t lon;
  int32_t alt;
  uint16_t class_id;
  uint16_t confidence;
  uint16_t pos_uncertainty;
  uint8_t drone_id;
  std::string class_name;
};

McDetectionFields decode_mc_detection(const RawMavlinkFrame & frame)
{
  const uint8_t * p = frame.payload;
  McDetectionFields f{};
  f.time_usec = read_le<uint64_t>(p, 0);
  f.detection_id = read_le<uint32_t>(p, 8);
  f.lat = read_le<int32_t>(p, 12);
  f.lon = read_le<int32_t>(p, 16);
  f.alt = read_le<int32_t>(p, 20);
  f.class_id = read_le<uint16_t>(p, 24);
  f.confidence = read_le<uint16_t>(p, 26);
  f.pos_uncertainty = read_le<uint16_t>(p, 28);
  f.drone_id = p[30];
  char class_name[25] = {};
  std::memcpy(class_name, p + 31, 24);
  f.class_name = class_name;
  return f;
}

struct McHealthFields
{
  uint64_t time_usec;
  uint8_t cpu_load;
  uint8_t mem_used;
  uint8_t disk_used;
  uint8_t gpu_temp;
  uint8_t cpu_temp;
  uint16_t fault_mask;
  uint16_t node_alive_mask;
  uint32_t uptime_s;
};

McHealthFields decode_mc_health(const RawMavlinkFrame & frame)
{
  const uint8_t * p = frame.payload;
  McHealthFields f{};
  f.time_usec = read_le<uint64_t>(p, 0);
  f.uptime_s = read_le<uint32_t>(p, 8);
  f.fault_mask = read_le<uint16_t>(p, 12);
  f.node_alive_mask = read_le<uint16_t>(p, 14);
  f.cpu_load = p[16];
  f.mem_used = p[17];
  f.disk_used = p[18];
  f.gpu_temp = p[19];
  f.cpu_temp = p[20];
  return f;
}

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

  // Raw datagram receive with timeout, no MAVLink parsing — used by the
  // GCS-003 tests via try_decode_raw_frame() instead of recv_frame(), since
  // mavlink_parse_char() can't validate mc_mavlink_defs.xml's custom
  // messages (see that function's comment for why).
  bool recv_raw(double timeout_s, uint8_t * buf, size_t buf_len, ssize_t & out_len)
  {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_s);
    tv.tv_usec = static_cast<suseconds_t>((timeout_s - static_cast<double>(tv.tv_sec)) * 1e6);
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    const ssize_t n = ::recv(fd_, buf, buf_len, 0);
    if (n <= 0) {
      return false;
    }
    out_len = n;
    return true;
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
    // seq5: swarm peer entry — drone_id=3, IP reconstructed as 192.168.100.13.
    make_item(
      5, kCmdSwarmPeerEntry, 0.0F, 0.0F, /*drone_id=*/ 3, /*octet3=*/ 100, /*octet4=*/ 13.0F),
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
  // Channel B half of the upload_id space (upload_id.hpp), echoed in MISSION_ACK.
  EXPECT_NE(up.upload_id, 0U);
  EXPECT_EQ(up.upload_id >> gcs_comms::kUploadIdCounterBits, 0U);

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
  EXPECT_EQ(up.swarm_manifest[0].ip_address, "192.168.100.13");
}

// MC_MISSION_CONFIG (50205): param1 = MissionUpload.skip_phase_mask. Serves
// `items` to the node's MISSION_REQUEST_INTs and returns the final
// MISSION_ACK type, or -1 if the handshake broke off.
namespace
{
int serve_mission_items(MockGcsPeer & peer, const std::vector<mavlink_mission_item_int_t> & items)
{
  for (uint16_t served = 0; served < items.size(); ) {
    mavlink_message_t msg;
    if (!peer.recv_frame(3.0, msg)) {
      return -1;
    }
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
      mavlink_mission_ack_t ack{};
      mavlink_msg_mission_ack_decode(&msg, &ack);
      return ack.type;  // rejected mid-upload
    }
    if (msg.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
      continue;
    }
    mavlink_mission_request_int_t req{};
    mavlink_msg_mission_request_int_decode(&msg, &req);
    if (req.seq == served) {
      peer.send_mission_item_int(items[served++]);
    }
  }
  mavlink_message_t msg;
  while (peer.recv_frame(3.0, msg)) {
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
      mavlink_mission_ack_t ack{};
      mavlink_msg_mission_ack_decode(&msg, &ack);
      return ack.type;
    }
  }
  return -1;
}

mavlink_mission_item_int_t mission_item(uint16_t seq, uint16_t command, float param1)
{
  mavlink_mission_item_int_t item{};
  item.seq = seq;
  item.frame = MAV_FRAME_GLOBAL;
  item.command = command;
  item.param1 = param1;
  item.x = 13000000;
  item.y = 1038000000;
  item.z = 30.0F;
  return item;
}
}  // namespace

TEST_F(GcsMavlinkNodeTest, MissionUploadCarriesSkipPhaseMaskFromMissionConfigItem)
{
  const std::vector<mavlink_mission_item_int_t> items = {
    mission_item(0, 50200 /*ingress waypoint*/, 0.0F),
    mission_item(
      1, 50205 /*MC_MISSION_CONFIG*/,
      static_cast<float>(
        mc_msgs::msg::MissionUpload::SKIP_SWARM_PHASES | mc_msgs::msg::MissionUpload::SKIP_SEARCH)),
  };
  std::atomic<int> ack_type{-1};
  std::thread mock_gcs([&] {ack_type = serve_mission_items(*peer_, items);});
  peer_->send_mission_count(static_cast<uint16_t>(items.size()));

  ASSERT_TRUE(
    spin_until(
      node_, [this] {return !mission_upload_msgs_.empty();}, std::chrono::seconds(5)));
  mock_gcs.join();
  EXPECT_EQ(ack_type, MAV_MISSION_ACCEPTED);
  EXPECT_EQ(
    mission_upload_msgs_.back()->skip_phase_mask,
    mc_msgs::msg::MissionUpload::SKIP_SWARM_PHASES | mc_msgs::msg::MissionUpload::SKIP_SEARCH);
}

TEST_F(GcsMavlinkNodeTest, MissionUploadRejectsUnflyableSkipPhaseMask)
{
  // MUSTER alone (bit 4) -- not a flyable stage set.
  const std::vector<mavlink_mission_item_int_t> items = {
    mission_item(0, 50200 /*ingress waypoint*/, 0.0F),
    mission_item(1, 50205 /*MC_MISSION_CONFIG*/, 16.0F),
  };
  std::atomic<int> ack_type{-1};
  std::thread mock_gcs([&] {ack_type = serve_mission_items(*peer_, items);});
  peer_->send_mission_count(static_cast<uint16_t>(items.size()));

  spin_until(node_, [&] {return ack_type != -1;}, std::chrono::seconds(5));
  mock_gcs.join();
  EXPECT_EQ(ack_type, MAV_MISSION_INVALID_PARAM1);
  EXPECT_TRUE(mission_upload_msgs_.empty());
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

// GCS-003: telemetry multiplexer (SDD §11.3). Synthetic internal-topic
// traffic, published directly on node_ standing in for
// fcc_bridge_node/mission_manager_node/visual_localisation_node/
// health_monitor_node, must reach the mock GCS peer as MC_STATUS. A
// repeating heartbeat keeps has_gcs_endpoint_ true (send_heartbeat()'s
// guard, shared by every mux send) and gives DDS discovery time to match
// the publishers created below before the actual publish.
TEST_F(GcsMavlinkNodeTest, MultiplexerPublishesMcStatusFromSyntheticTopics)
{
  auto hb_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});

  auto pub_vehicle_state = node_->create_publisher<mc_msgs::msg::VehicleState>(
    "/mc/fcc/vehicle_state", rclcpp::QoS(10));
  auto pub_mission_phase = node_->create_publisher<mc_msgs::msg::MissionPhase>(
    "/mc/mission_phase", rclcpp::QoS(10));
  auto pub_localisation_mode = node_->create_publisher<mc_msgs::msg::LocalisationMode>(
    "/mc/localisation_mode", rclcpp::QoS(10));
  auto pub_health = node_->create_publisher<mc_msgs::msg::SystemHealth>(
    "/mc/health", rclcpp::QoS(10));

  // Not a real predicate wait — just spins node_ for a fixed warm-up
  // window so DDS discovery matches the publishers above before the
  // actual publish below.
  spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));

  mc_msgs::msg::VehicleState vs;
  vs.lat = 1.3;
  vs.lon = 103.8;
  vs.alt_msl = 50.0F;
  vs.heading = static_cast<float>(-M_PI / 2.0);  // -90 deg -> wraps to 270 deg
  vs.groundspeed = 10.0F;
  vs.vz = -3.0F;           // NED down; climb_rate = -vz = 3 m/s up
  vs.battery_pct = 0.75F;
  pub_vehicle_state->publish(vs);

  mc_msgs::msg::MissionPhase phase;
  phase.phase = mc_msgs::msg::MissionPhase::SEARCH;
  pub_mission_phase->publish(phase);

  mc_msgs::msg::LocalisationMode loc_mode;
  loc_mode.mode = mc_msgs::msg::LocalisationMode::MODE_VISUAL_ODOMETRY;
  pub_localisation_mode->publish(loc_mode);

  mc_msgs::msg::SystemHealth health;
  health.fault_mask = 8;  // LOW_BATTERY (ICD §8.8, mc_msgs/FaultEvent)
  pub_health->publish(health);

  uint8_t raw[512];
  ssize_t raw_len = 0;
  RawMavlinkFrame frame;
  bool got_status = false;
  McStatusFields fields{};
  int status_count = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    if (peer_->recv_raw(0.05, raw, sizeof(raw), raw_len) &&
      try_decode_raw_frame(raw, raw_len, frame) && frame.msgid == kMsgIdMcStatus)
    {
      if (!got_status) {
        fields = decode_mc_status(frame);
        got_status = true;
      }
      ++status_count;
    }
  }

  ASSERT_TRUE(got_status);
  EXPECT_EQ(fields.lat, 13000000);
  EXPECT_EQ(fields.lon, 1038000000);
  EXPECT_EQ(fields.alt_msl, 50000);
  EXPECT_EQ(fields.groundspeed, 1000U);
  EXPECT_EQ(fields.climb_rate, 300);
  EXPECT_EQ(fields.battery_remaining, 75U);
  EXPECT_EQ(fields.mission_phase, mc_msgs::msg::MissionPhase::SEARCH);
  EXPECT_EQ(fields.localisation_mode, mc_msgs::msg::LocalisationMode::MODE_VISUAL_ODOMETRY);
  EXPECT_EQ(fields.fault_mask, 8U);

  // Same wrap formula as send_mc_status() — computed here rather than
  // hardcoded so float-precision-induced offsets don't make this brittle.
  double expected_heading_deg = std::fmod(static_cast<double>(vs.heading) * 180.0 / M_PI, 360.0);
  if (expected_heading_deg < 0.0) {
    expected_heading_deg += 360.0;
  }
  EXPECT_EQ(fields.heading, static_cast<uint16_t>(std::lround(expected_heading_deg * 100.0)));

  // SDD §11.3: >= 2 Hz, via a 500 ms WallTimer — over ~3 s that's at least
  // 5 ticks; also an upper bound (not more than ~2x that) so this isn't
  // secretly firing far faster than the documented period.
  EXPECT_GE(status_count, 5);
  EXPECT_LE(status_count, 12);
}

// GCS-003: MC_DETECTION is event-driven (SDD §11.3), not tied to the
// MC_STATUS timer — one message per mc_msgs/Detection element, since
// DetectionArray may carry several per frame but MC_DETECTION (ICD §4.6)
// describes only one target.
TEST_F(GcsMavlinkNodeTest, MultiplexerRelaysDetectionsImmediately)
{
  auto hb_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});
  auto pub_detections = node_->create_publisher<mc_msgs::msg::DetectionArray>(
    "/mc/detections", rclcpp::QoS(10));

  // Not a real predicate wait — just spins node_ for a fixed warm-up
  // window so DDS discovery matches the publishers above before the
  // actual publish below.
  spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));

  mc_msgs::msg::DetectionArray arr;
  mc_msgs::msg::Detection d1;
  d1.detection_id = 42;
  d1.frame_timestamp_us = 123456789ULL;
  d1.class_id = 3;
  d1.class_name = "person";
  d1.confidence = 0.955F;
  d1.lat = 1.31;
  d1.lon = 103.81;
  d1.alt_msl = 45.0F;
  d1.pos_uncertainty_m = 2.5F;
  arr.detections.push_back(d1);

  mc_msgs::msg::Detection d2 = d1;
  d2.detection_id = 43;
  arr.detections.push_back(d2);
  pub_detections->publish(arr);

  std::vector<McDetectionFields> received;
  uint8_t raw[512];
  ssize_t raw_len = 0;
  RawMavlinkFrame frame;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (received.size() < 2 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    if (peer_->recv_raw(0.05, raw, sizeof(raw), raw_len) &&
      try_decode_raw_frame(raw, raw_len, frame) && frame.msgid == kMsgIdMcDetection)
    {
      received.push_back(decode_mc_detection(frame));
    }
  }

  ASSERT_EQ(received.size(), 2U);
  EXPECT_EQ(received[0].detection_id, 42U);
  EXPECT_EQ(received[1].detection_id, 43U);
  EXPECT_EQ(received[0].time_usec, 123456789ULL);
  EXPECT_EQ(received[0].class_id, 3U);
  EXPECT_EQ(received[0].class_name, "person");
  EXPECT_EQ(received[0].confidence, 9550U);
  EXPECT_EQ(received[0].lat, 13100000);
  EXPECT_EQ(received[0].lon, 1038100000);
  EXPECT_EQ(received[0].alt, 45000);
  EXPECT_EQ(received[0].pos_uncertainty, 250U);
  EXPECT_EQ(received[0].drone_id, 1U);  // DRONE_ID (SetUp())
}

// GCS-003: MC_HEALTH is event-driven too — published as soon as /mc/health
// is received, not deferred to the 500 ms MC_STATUS timer (SDD §11.3).
TEST_F(GcsMavlinkNodeTest, MultiplexerRelaysHealthImmediately)
{
  auto hb_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});
  auto pub_health = node_->create_publisher<mc_msgs::msg::SystemHealth>(
    "/mc/health", rclcpp::QoS(10));

  // Not a real predicate wait — just spins node_ for a fixed warm-up
  // window so DDS discovery matches the publishers above before the
  // actual publish below.
  spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));

  mc_msgs::msg::SystemHealth health;
  health.cpu_pct = 55;
  health.mem_pct = 40;
  health.disk_pct = 30;
  health.gpu_temp_c = 60;
  health.cpu_temp_c = 65;
  health.fault_mask = 0x0011;
  health.node_alive_mask = 0x00FF;
  health.uptime_s = 12345;
  pub_health->publish(health);

  uint8_t raw[512];
  ssize_t raw_len = 0;
  RawMavlinkFrame frame;
  bool got_health = false;
  McHealthFields fields{};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!got_health && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    if (peer_->recv_raw(0.05, raw, sizeof(raw), raw_len) &&
      try_decode_raw_frame(raw, raw_len, frame) && frame.msgid == kMsgIdMcHealth)
    {
      fields = decode_mc_health(frame);
      got_health = true;
    }
  }

  ASSERT_TRUE(got_health);
  EXPECT_EQ(fields.cpu_load, 55U);
  EXPECT_EQ(fields.mem_used, 40U);
  EXPECT_EQ(fields.disk_used, 30U);
  EXPECT_EQ(fields.gpu_temp, 60U);
  EXPECT_EQ(fields.cpu_temp, 65U);
  EXPECT_EQ(fields.fault_mask, 0x0011U);
  EXPECT_EQ(fields.node_alive_mask, 0x00FFU);
  EXPECT_EQ(fields.uptime_s, 12345U);
}

// GCS-003 AC: "does not block or degrade the command-decode path (GCS-001)
// under detection-heavy traffic". Floods /mc/detections while a
// COMMAND_LONG is in flight on mux_sub_cb_group_'s reentrant group and
// recv_timer_'s own group respectively; the command must still decode and
// ack well within FR-061's 500 ms budget.
TEST_F(GcsMavlinkNodeTest, DetectionFloodDoesNotDelayCommandDecode)
{
  auto hb_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});
  auto pub_detections = node_->create_publisher<mc_msgs::msg::DetectionArray>(
    "/mc/detections", rclcpp::QoS(50));

  // Not a real predicate wait — just spins node_ for a fixed warm-up
  // window so DDS discovery matches the publishers above before the
  // actual publish below.
  spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));

  mc_msgs::msg::DetectionArray flood;
  for (int i = 0; i < 20; ++i) {
    mc_msgs::msg::Detection d;
    d.detection_id = static_cast<uint32_t>(i);
    d.class_name = "flood";
    flood.detections.push_back(d);
  }
  auto flood_timer = node_->create_wall_timer(
    std::chrono::milliseconds(5), [&] {pub_detections->publish(flood);});

  peer_->send_command_long(1, 191, 50001, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});

  ASSERT_TRUE(
    spin_until(
      node_, [this] {return !command_msgs_.empty();}, std::chrono::milliseconds(500)));
  flood_timer->cancel();

  EXPECT_EQ(command_msgs_.back()->command, 50001U);
}

// TEST-007 finding: a guard-rejected MC_CMD_PHASE_TRANSITION used to get
// the same immediate MAV_RESULT_ACCEPTED as a successful one. It now gets
// IN_PROGRESS on receipt, then mission_manager_node's real outcome
// (/mc/gcs/command_ack) as a final DENIED COMMAND_ACK plus a STATUSTEXT
// carrying the reason.
TEST_F(GcsMavlinkNodeTest, PhaseTransitionRelaysOutcomeAck)
{
  auto pub_ack = node_->create_publisher<mc_msgs::msg::GcsCommandAck>(
    "/mc/gcs/command_ack", rclcpp::QoS(10));
  spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));

  peer_->send_command_long(1, 191, 50001, {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});

  auto recv_until = [this](uint32_t msgid, mavlink_message_t & out) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node_);
        if (peer_->recv_frame(0.05, out) && out.msgid == msgid) {
          return true;
        }
      }
      return false;
    };

  mavlink_message_t frame;
  ASSERT_TRUE(recv_until(MAVLINK_MSG_ID_COMMAND_ACK, frame));
  mavlink_command_ack_t ack{};
  mavlink_msg_command_ack_decode(&frame, &ack);
  EXPECT_EQ(ack.command, 50001U);
  EXPECT_EQ(ack.result, MAV_RESULT_IN_PROGRESS);

  mc_msgs::msg::GcsCommandAck outcome;
  outcome.command = 50001;
  outcome.result = mc_msgs::msg::GcsCommandAck::RESULT_REJECTED;
  outcome.reason = "GCS ARM authorisation not set";
  pub_ack->publish(outcome);

  ASSERT_TRUE(recv_until(MAVLINK_MSG_ID_COMMAND_ACK, frame));
  mavlink_msg_command_ack_decode(&frame, &ack);
  EXPECT_EQ(ack.command, 50001U);
  EXPECT_EQ(ack.result, MAV_RESULT_DENIED);

  ASSERT_TRUE(recv_until(MAVLINK_MSG_ID_STATUSTEXT, frame));
  mavlink_statustext_t text{};
  mavlink_msg_statustext_decode(&frame, &text);
  EXPECT_EQ(text.severity, MAV_SEVERITY_WARNING);
  EXPECT_EQ(std::string(text.text, strnlen(text.text, sizeof(text.text))),
    "GCS ARM authorisation not set");
}

TEST_F(GcsMavlinkNodeTest, OtherCommandsStillAckAcceptedImmediately)
{
  peer_->send_command_long(1, 191, 50002, {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
  mavlink_message_t frame;
  bool got = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!got && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    got = peer_->recv_frame(0.05, frame) && frame.msgid == MAVLINK_MSG_ID_COMMAND_ACK;
  }
  ASSERT_TRUE(got);
  mavlink_command_ack_t ack{};
  mavlink_msg_command_ack_decode(&frame, &ack);
  EXPECT_EQ(ack.command, 50002U);
  EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);
}

TEST_F(GcsMavlinkNodeTest, StatusTextIsRelayedAsStatustext)
{
  auto hb_timer = node_->create_wall_timer(
    std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});
  auto pub_text = node_->create_publisher<mc_msgs::msg::StatusText>(
    "/mc/status_text", rclcpp::QoS(10));
  spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));

  mc_msgs::msg::StatusText msg;
  msg.severity = mc_msgs::msg::StatusText::SEVERITY_ERROR;
  msg.text = "FCC link not alive";
  pub_text->publish(msg);

  mavlink_message_t frame;
  bool got = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!got && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    got = peer_->recv_frame(0.05, frame) && frame.msgid == MAVLINK_MSG_ID_STATUSTEXT;
  }
  ASSERT_TRUE(got);
  mavlink_statustext_t text{};
  mavlink_msg_statustext_decode(&frame, &text);
  EXPECT_EQ(text.severity, MAV_SEVERITY_ERROR);
  EXPECT_EQ(std::string(text.text, strnlen(text.text, sizeof(text.text))), "FCC link not alive");
}

// Texts over STATUSTEXT's 50 chars (gnss_sim_node's anchor confirmation, long
// pre-arm reasons) used to be cut at 50. They now go out as MAVLink v2
// chunks sharing one nonzero id, which QGC and the TEST-007 panel reassemble.
class GcsMavlinkNodeStatustextChunkTest : public GcsMavlinkNodeTest
{
protected:
  std::vector<mavlink_statustext_t> relay(const std::string & text)
  {
    auto hb_timer = node_->create_wall_timer(
      std::chrono::milliseconds(50), [this] {peer_->send_heartbeat();});
    auto pub_text = node_->create_publisher<mc_msgs::msg::StatusText>(
      "/mc/status_text", rclcpp::QoS(10));
    spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));

    mc_msgs::msg::StatusText msg;
    msg.severity = mc_msgs::msg::StatusText::SEVERITY_INFO;
    msg.text = text;
    pub_text->publish(msg);

    std::vector<mavlink_statustext_t> chunks;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      mavlink_message_t frame;
      if (!peer_->recv_frame(0.05, frame) || frame.msgid != MAVLINK_MSG_ID_STATUSTEXT) {
        continue;
      }
      mavlink_statustext_t chunk{};
      mavlink_msg_statustext_decode(&frame, &chunk);
      chunks.push_back(chunk);
      if (strnlen(chunk.text, sizeof(chunk.text)) < sizeof(chunk.text)) {
        break;  // a NUL marks the last chunk
      }
    }
    return chunks;
  }

  static std::string join(const std::vector<mavlink_statustext_t> & chunks)
  {
    std::string out;
    for (const auto & c : chunks) {
      out.append(c.text, strnlen(c.text, sizeof(c.text)));
    }
    return out;
  }
};

TEST_F(GcsMavlinkNodeStatustextChunkTest, LongTextIsSplitIntoChunksSharingOneId)
{
  const std::string text =
    "gnss_sim_node: anchored false origin lat=1.4180151 lon=103.8629088 alt_msl=6.00";
  ASSERT_GT(text.size(), 50U);
  const auto chunks = relay(text);
  ASSERT_EQ(chunks.size(), 2U);
  EXPECT_NE(chunks[0].id, 0);
  EXPECT_EQ(chunks[1].id, chunks[0].id);
  EXPECT_EQ(chunks[0].chunk_seq, 0);
  EXPECT_EQ(chunks[1].chunk_seq, 1);
  EXPECT_EQ(chunks[0].severity, MAV_SEVERITY_INFO);
  EXPECT_EQ(join(chunks), text);
}

TEST_F(GcsMavlinkNodeStatustextChunkTest, TextFillingItsLastChunkGetsAnEmptyTerminator)
{
  const std::string text(100, 'x');
  const auto chunks = relay(text);
  ASSERT_EQ(chunks.size(), 3U);
  EXPECT_EQ(chunks[2].chunk_seq, 2);
  EXPECT_EQ(strnlen(chunks[2].text, sizeof(chunks[2].text)), 0U);
  EXPECT_EQ(join(chunks), text);
}

TEST_F(GcsMavlinkNodeStatustextChunkTest, ShortTextStaysOneFrameWithIdZero)
{
  const auto chunks = relay("FCC link not alive");
  ASSERT_EQ(chunks.size(), 1U);
  EXPECT_EQ(chunks[0].id, 0);
  EXPECT_EQ(join(chunks), "FCC link not alive");
}
