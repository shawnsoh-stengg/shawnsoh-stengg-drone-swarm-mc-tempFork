// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §11, ICD-MC-001 §12).

// GCS-004: mission_orchestrator_node integration tests — a mock Channel C
// peer (a raw loopback UDP socket, same technique as
// test_gcs_mavlink_node.cpp's MockGcsPeer) drives MAVFTP create/list/read/
// delete plus MC_MISSION_PLAN_ACTIVATE against the real node, asserting
// correct on-disk persistence, correct ack correlation, and the
// /mc/gcs/mission_upload publish on activate (ICD §12). FILE_TRANSFER_PROTOCOL
// (110) is a standard common.xml message, so the peer decodes it with the
// generated mavlink_msg_file_transfer_protocol_decode() normally; MC_MISSION_
// PLAN_ACTIVATE/ACK (50104/50105) are hand-packed/hand-decoded for the same
// reason test_gcs_mavlink_node.cpp's RawMavlinkFrame is — mavlink_parse_char()
// cannot validate a custom dialect message with only common.xml compiled in
// (mc_mavlink_defs.xml's header comment).

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gcs_comms/mission_orchestrator_node.hpp"

namespace
{

constexpr uint32_t kMsgIdMcMissionPlanActivate = 50104;
constexpr uint8_t kLenMcMissionPlanActivate = 68;
constexpr uint8_t kCrcExtraMcMissionPlanActivate = 243;
constexpr uint32_t kMsgIdMcMissionPlanAck = 50105;

// MAVFTP header layout mission_orchestrator_node.cpp packs/unpacks — see
// that file's FtpHeader for the authoritative definition; duplicated here
// because it's a private nested type, same as every other test in this
// package re-deriving its target's private wire layout independently.
struct __attribute__((packed)) FtpHeader
{
  uint16_t seq_number;
  uint8_t session;
  uint8_t opcode;
  uint8_t size;
  uint8_t req_opcode;
  uint8_t burst_complete;
  uint8_t padding;
  uint32_t offset;
};

enum FtpOpcode : uint8_t
{
  kOpTerminateSession = 1,
  kOpListDirectory = 3,
  kOpOpenFileRO = 4,
  kOpReadFile = 5,
  kOpCreateFile = 6,
  kOpWriteFile = 7,
  kOpRemoveFile = 8,
  kOpOpenFileWO = 11,
  kOpBurstReadFile = 15,
  kOpAck = 128,
  kOpNak = 129,
};

enum FtpError : uint8_t
{
  kErrEOF = 6,
  kErrFileExists = 8,
  kErrFileProtected = 9,
  kErrFileNotFound = 10,
};

// A raw, dialect-agnostic MAVLink v2 frame — used only for
// MC_MISSION_PLAN_ACK (50105), which mavlink_parse_char() cannot decode in
// this translation unit (only common.xml is compiled in).
struct RawFrame
{
  uint32_t msgid{0};
  uint8_t payload[128]{};  // covers MC_MISSION_PLAN_ACK's 119 bytes
};

bool try_decode_raw_frame(const uint8_t * buf, ssize_t n, RawFrame & out)
{
  constexpr ssize_t kHeaderLen = 10;
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

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

::testing::Environment * const kRclcppEnv =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

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

// Mock Mission Orchestrator MAVLink peer — a plain UDP socket that sends
// frames straight at mission_orchestrator_node's loopback port, standing in
// for the real mavlink-router-channel-c.conf relay (which just forwards
// bytes unchanged).
class MockOrchestratorPeer
{
public:
  explicit MockOrchestratorPeer(uint16_t target_port)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    target_.sin_family = AF_INET;
    target_.sin_port = htons(target_port);
    target_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = 0;
    local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr));
  }

  ~MockOrchestratorPeer() {::close(fd_);}

  void send_ftp(uint8_t opcode, uint32_t offset, const void * data, uint8_t size)
  {
    uint8_t ftp_payload[251] = {};
    FtpHeader hdr{};
    hdr.seq_number = seq_++;
    hdr.session = 0;
    hdr.opcode = opcode;
    hdr.size = size;
    hdr.offset = offset;
    std::memcpy(ftp_payload, &hdr, sizeof(hdr));
    if (data != nullptr && size > 0) {
      std::memcpy(ftp_payload + sizeof(hdr), data, size);
    }

    mavlink_message_t msg;
    mavlink_msg_file_transfer_protocol_pack(
      kSysId, kCompId, &msg, /*target_network=*/ 0, kTargetSysId, kTargetCompId, ftp_payload);
    send_frame(msg);
  }

  void send_create_file(const std::string & path)
  {
    send_ftp(kOpCreateFile, 0, path.data(), static_cast<uint8_t>(path.size()));
  }
  void send_open_file_wo(const std::string & path)
  {
    send_ftp(kOpOpenFileWO, 0, path.data(), static_cast<uint8_t>(path.size()));
  }
  void send_open_file_ro(const std::string & path)
  {
    send_ftp(kOpOpenFileRO, 0, path.data(), static_cast<uint8_t>(path.size()));
  }
  void send_write_file(uint32_t offset, const std::string & data)
  {
    send_ftp(kOpWriteFile, offset, data.data(), static_cast<uint8_t>(data.size()));
  }
  void send_read_file(uint32_t offset, uint8_t size) {send_ftp(kOpReadFile, offset, nullptr, size);}
  void send_burst_read_file(uint32_t offset) {send_ftp(kOpBurstReadFile, offset, nullptr, 0);}
  void send_list_directory(uint32_t entry_offset)
  {
    send_ftp(kOpListDirectory, entry_offset, nullptr, 0);
  }
  void send_remove_file(const std::string & path)
  {
    send_ftp(kOpRemoveFile, 0, path.data(), static_cast<uint8_t>(path.size()));
  }
  void send_terminate_session() {send_ftp(kOpTerminateSession, 0, nullptr, 0);}

  struct FtpReply
  {
    uint8_t opcode{0};
    uint8_t size{0};
    uint8_t req_opcode{0};
    uint8_t burst_complete{0};
    uint32_t offset{0};
    std::array<uint8_t, 239> data{};
  };

  // FILE_TRANSFER_PROTOCOL (110) is standard common.xml — decoded normally.
  bool recv_ftp_reply(double timeout_s, FtpReply & out)
  {
    mavlink_message_t msg;
    if (!recv_frame(timeout_s, msg) || msg.msgid != MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
      return false;
    }
    mavlink_file_transfer_protocol_t ftp{};
    mavlink_msg_file_transfer_protocol_decode(&msg, &ftp);
    FtpHeader hdr{};
    std::memcpy(&hdr, ftp.payload, sizeof(hdr));
    out.opcode = hdr.opcode;
    out.size = hdr.size;
    out.req_opcode = hdr.req_opcode;
    out.burst_complete = hdr.burst_complete;
    out.offset = hdr.offset;
    std::memcpy(out.data.data(), ftp.payload + sizeof(hdr), out.data.size());
    return true;
  }

  void send_activate(uint32_t client_request_id, const std::string & filename)
  {
    char buf[kLenMcMissionPlanActivate];
    _mav_put_uint32_t(buf, 0, client_request_id);
    char fname[64] = {};
    std::strncpy(fname, filename.c_str(), sizeof(fname) - 1);
    _mav_put_char_array(buf, 4, fname, 64);

    mavlink_message_t msg;
    std::memcpy(_MAV_PAYLOAD_NON_CONST(&msg), buf, sizeof(buf));
    msg.msgid = kMsgIdMcMissionPlanActivate;
    mavlink_finalize_message(
      &msg, kSysId, kCompId, kLenMcMissionPlanActivate, kLenMcMissionPlanActivate,
      kCrcExtraMcMissionPlanActivate);
    send_frame(msg);
  }

  struct AckFields
  {
    uint32_t client_request_id{0};
    std::string filename;
    uint8_t result{0};
    std::string error_message;
  };

  // MC_MISSION_PLAN_ACK (50105) is custom-dialect — hand-decoded, same
  // reasoning as this file's header comment.
  bool recv_activate_ack(double timeout_s, AckFields & out)
  {
    RawFrame frame;
    if (!recv_raw(timeout_s, frame) || frame.msgid != kMsgIdMcMissionPlanAck) {
      return false;
    }
    std::memcpy(&out.client_request_id, frame.payload + 0, sizeof(out.client_request_id));
    char fname[65] = {};
    std::memcpy(fname, frame.payload + 4, 64);
    out.filename = fname;
    out.result = frame.payload[68];
    char err[51] = {};
    std::memcpy(err, frame.payload + 69, 50);
    out.error_message = err;
    return true;
  }

private:
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

  bool recv_raw(double timeout_s, RawFrame & out)
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
    return try_decode_raw_frame(raw, n, out);
  }

  void send_frame(const mavlink_message_t & msg)
  {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ::sendto(fd_, buf, len, 0, reinterpret_cast<const sockaddr *>(&target_), sizeof(target_));
  }

  static constexpr uint8_t kSysId = 255;         // ICD §12.1 Orchestrator system_id
  static constexpr uint8_t kCompId = 190;        // MAV_COMP_ID_MISSIONPLANNER
  static constexpr uint8_t kTargetSysId = 1;     // this MC's drone ID (DRONE_ID=1 in tests)
  static constexpr uint8_t kTargetCompId = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER

  int fd_;
  sockaddr_in target_{};
  uint16_t seq_{0};
};

}  // namespace

class MissionOrchestratorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setenv("DRONE_ID", "1", 1);
    setenv("ORCHESTRATOR_MAVLINK_LOOPBACK_PORT", "25580", 1);

    plans_dir_ = (std::filesystem::temp_directory_path() /
      ("mission_orchestrator_node_test_" + std::to_string(::getpid()))).string();
    std::filesystem::remove_all(plans_dir_);

    rclcpp::NodeOptions options;
    options.append_parameter_override("plans_dir", plans_dir_);
    node_ = std::make_shared<gcs_comms::MissionOrchestratorNode>(options);
    peer_ = std::make_shared<MockOrchestratorPeer>(25580);

    sub_mission_upload_ = node_->create_subscription<mc_msgs::msg::MissionUpload>(
      "/mc/gcs/mission_upload", rclcpp::QoS(10),
      [this](mc_msgs::msg::MissionUpload::SharedPtr msg) {mission_upload_msgs_.push_back(msg);});
    pub_mission_phase_ = node_->create_publisher<mc_msgs::msg::MissionPhase>(
      "/mc/mission_phase", rclcpp::QoS(10));
  }

  void TearDown() override
  {
    peer_.reset();
    node_.reset();
    std::filesystem::remove_all(plans_dir_);
    unsetenv("DRONE_ID");
    unsetenv("ORCHESTRATOR_MAVLINK_LOOPBACK_PORT");
  }

  bool spin_until_reply(MockOrchestratorPeer::FtpReply & out)
  {
    bool got = false;
    spin_until(
      node_, [this, &out, &got] {
        if (peer_->recv_ftp_reply(0.05, out)) {
          got = true;
          return true;
        }
        return false;
      },
      std::chrono::seconds(3));
    return got;
  }

  bool spin_until_ack(MockOrchestratorPeer::AckFields & out)
  {
    bool got = false;
    spin_until(
      node_, [this, &out, &got] {
        if (peer_->recv_activate_ack(0.05, out)) {
          got = true;
          return true;
        }
        return false;
      },
      std::chrono::seconds(3));
    return got;
  }

  // No accessor exists for the node's private latest_mission_phase_, so this
  // just publishes and spins long enough for on_mission_phase() to have run
  // — every test using this only checks the resulting ack, not the instant
  // the subscription callback fires.
  void set_mission_phase(uint8_t phase)
  {
    mc_msgs::msg::MissionPhase msg;
    msg.phase = phase;
    pub_mission_phase_->publish(msg);
    spin_until(node_, [] {return false;}, std::chrono::milliseconds(200));
  }

  std::string plans_dir_;
  std::shared_ptr<gcs_comms::MissionOrchestratorNode> node_;
  std::shared_ptr<MockOrchestratorPeer> peer_;
  rclcpp::Subscription<mc_msgs::msg::MissionUpload>::SharedPtr sub_mission_upload_;
  std::vector<mc_msgs::msg::MissionUpload::SharedPtr> mission_upload_msgs_;
  rclcpp::Publisher<mc_msgs::msg::MissionPhase>::SharedPtr pub_mission_phase_;
};

TEST_F(MissionOrchestratorNodeTest, ConstructsWithExpectedName)
{
  EXPECT_STREQ(node_->get_name(), "mission_orchestrator_node");
}

TEST_F(MissionOrchestratorNodeTest, CreateWriteTerminatePersistsFileToDisk)
{
  const std::string content = R"({"name":"recon_alpha","arm_authorised":true})";

  peer_->send_create_file("/recon_alpha.json");
  MockOrchestratorPeer::FtpReply reply;
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
  EXPECT_EQ(reply.req_opcode, kOpCreateFile);

  peer_->send_write_file(0, content);
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
  EXPECT_EQ(reply.req_opcode, kOpWriteFile);

  peer_->send_terminate_session();
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
  EXPECT_EQ(reply.req_opcode, kOpTerminateSession);

  std::ifstream file(plans_dir_ + "/recon_alpha.json", std::ios::binary);
  ASSERT_TRUE(file.good());
  std::string on_disk((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(on_disk, content);
}

TEST_F(MissionOrchestratorNodeTest, CreateExistingFileNaksFileExists)
{
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json");
    f << "{}";
  }

  peer_->send_create_file("/recon_alpha.json");
  MockOrchestratorPeer::FtpReply reply;
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpNak);
  EXPECT_EQ(reply.data[0], kErrFileExists);
}

TEST_F(MissionOrchestratorNodeTest, ListDirectoryReturnsStoredPlan)
{
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json");
    f << R"({"name":"recon_alpha"})";
  }

  peer_->send_list_directory(0);
  MockOrchestratorPeer::FtpReply reply;
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
  const std::string entries(reinterpret_cast<const char *>(reply.data.data()), reply.size);
  EXPECT_NE(entries.find("Frecon_alpha.json\t"), std::string::npos);
}

// TEST-004 regression: ftp_list_directory()'s skip-phase used to advance
// readdir() by `entry_offset` raw entries instead of `entry_offset`
// DT_REG-filtered ones, so it undercounted "."/".." against the pagination
// continuation call's own count of already-returned real entries. A real
// MAVFTP client (pymavlink's mavftp.py included) always re-issues
// ListDirectory with offset = however many entries it has consumed so
// far, expecting Nak(EOF) once there's nothing left — found by exercising
// mission-orchestrator-sim (tools/mission-orchestrator-sim) against a real
// build of this node.
TEST_F(MissionOrchestratorNodeTest, ListDirectoryPaginationDoesNotRepeatEntries)
{
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json");
    f << R"({"name":"recon_alpha"})";
  }
  {
    std::ofstream f(plans_dir_ + "/muster_bravo.json");
    f << R"({"name":"muster_bravo"})";
  }

  peer_->send_list_directory(0);
  MockOrchestratorPeer::FtpReply first_reply;
  ASSERT_TRUE(spin_until_reply(first_reply));
  ASSERT_EQ(first_reply.opcode, kOpAck);
  const std::string first_entries(
    reinterpret_cast<const char *>(first_reply.data.data()), first_reply.size);
  uint32_t returned_count = 0;
  for (size_t pos = 0; pos < first_entries.size();
    pos += std::strlen(first_entries.c_str() + pos) + 1)
  {
    ++returned_count;
  }
  ASSERT_EQ(returned_count, 2U) << "both plan files should fit in one Ack for this test's sizes";

  // The continuation call a real client makes next: offset = every entry
  // already consumed. Must be EOF, not a re-delivery of an entry already
  // returned above.
  peer_->send_list_directory(returned_count);
  MockOrchestratorPeer::FtpReply second_reply;
  ASSERT_TRUE(spin_until_reply(second_reply));
  EXPECT_EQ(second_reply.opcode, kOpNak);
  ASSERT_EQ(second_reply.size, 1);
  EXPECT_EQ(second_reply.data[0], kErrEOF);
}

TEST_F(MissionOrchestratorNodeTest, ReadFileReturnsWrittenContent)
{
  const std::string content = R"({"name":"recon_alpha"})";
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json", std::ios::binary);
    f << content;
  }

  peer_->send_open_file_ro("/recon_alpha.json");
  MockOrchestratorPeer::FtpReply reply;
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);

  peer_->send_read_file(0, static_cast<uint8_t>(content.size()));
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
  EXPECT_EQ(reply.size, content.size());
  EXPECT_EQ(
    std::string(reinterpret_cast<const char *>(reply.data.data()), reply.size), content);

  peer_->send_terminate_session();
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
}

TEST_F(MissionOrchestratorNodeTest, BurstReadFileStreamsWrittenContentThenEof)
{
  const std::string content = R"({"name":"recon_alpha"})";
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json", std::ios::binary);
    f << content;
  }

  peer_->send_open_file_ro("/recon_alpha.json");
  MockOrchestratorPeer::FtpReply reply;
  ASSERT_TRUE(spin_until_reply(reply));
  ASSERT_EQ(reply.opcode, kOpAck);

  peer_->send_burst_read_file(0);
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
  EXPECT_EQ(reply.req_opcode, kOpBurstReadFile);
  EXPECT_EQ(reply.size, content.size());
  EXPECT_EQ(
    std::string(reinterpret_cast<const char *>(reply.data.data()), reply.size), content);

  // Small file fits in one chunk — the very next reply is the terminating
  // Nak(EOF) (handle_burst_read()'s loop, ICD §12.3).
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpNak);
  EXPECT_EQ(reply.data[0], kErrEOF);
}

TEST_F(MissionOrchestratorNodeTest, RemoveNonActivePlanSucceeds)
{
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json");
    f << "{}";
  }

  peer_->send_remove_file("/recon_alpha.json");
  MockOrchestratorPeer::FtpReply reply;
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpAck);
  EXPECT_FALSE(std::filesystem::exists(plans_dir_ + "/recon_alpha.json"));
}

TEST_F(MissionOrchestratorNodeTest, ActivateAcceptsValidPlanAndPublishesMissionUpload)
{
  const std::string content =
    R"({"name":"recon_alpha","arm_authorised":true,"low_battery_threshold":0.25,)"
    R"("formation_offsets":[{"drone_id":2,"north_m":1.5,"east_m":-2.5,"down_m":0.0}]})";
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json", std::ios::binary);
    f << content;
  }

  peer_->send_activate(42, "recon_alpha.json");

  MockOrchestratorPeer::AckFields ack;
  ASSERT_TRUE(spin_until_ack(ack));
  EXPECT_EQ(ack.client_request_id, 42U);
  EXPECT_EQ(ack.filename, "recon_alpha.json");
  EXPECT_EQ(ack.result, 0);  // ACCEPTED (ICD §12.2)

  ASSERT_TRUE(
    spin_until(
      node_,
      [this] {return !mission_upload_msgs_.empty();}, std::chrono::seconds(1)));
  const auto & upload = *mission_upload_msgs_.back();
  EXPECT_TRUE(upload.arm_authorised);
  EXPECT_FLOAT_EQ(upload.low_battery_threshold, 0.25F);
  ASSERT_EQ(upload.formation_offsets.size(), 1U);
  EXPECT_EQ(upload.formation_offsets[0].drone_id, 2);
  EXPECT_FLOAT_EQ(upload.formation_offsets[0].n, 1.5F);
}

TEST_F(MissionOrchestratorNodeTest, ActivateNotFoundIsRejected)
{
  peer_->send_activate(7, "does_not_exist.json");
  MockOrchestratorPeer::AckFields ack;
  ASSERT_TRUE(spin_until_ack(ack));
  EXPECT_EQ(ack.client_request_id, 7U);
  EXPECT_EQ(ack.result, 2);  // REJECTED_NOT_FOUND (ICD §12.2)
}

TEST_F(MissionOrchestratorNodeTest, ActivateInvalidJsonIsRejected)
{
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/broken.json", std::ios::binary);
    f << "{not valid json";
  }

  peer_->send_activate(9, "broken.json");
  MockOrchestratorPeer::AckFields ack;
  ASSERT_TRUE(spin_until_ack(ack));
  EXPECT_EQ(ack.result, 1);  // REJECTED_INVALID_JSON (ICD §12.2)
}

TEST_F(MissionOrchestratorNodeTest, ActivateOutsidePreflightIsRejected)
{
  const std::string content = R"({"name":"recon_alpha"})";
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json", std::ios::binary);
    f << content;
  }

  set_mission_phase(mc_msgs::msg::MissionPhase::SEARCH);

  peer_->send_activate(11, "recon_alpha.json");
  MockOrchestratorPeer::AckFields ack;
  ASSERT_TRUE(spin_until_ack(ack));
  EXPECT_EQ(ack.result, 3);  // REJECTED_PHASE (ICD §12.2)
  EXPECT_TRUE(mission_upload_msgs_.empty());
}

TEST_F(MissionOrchestratorNodeTest, ActivateAllowedAgainAfterReturningToPreflight)
{
  const std::string content = R"({"name":"recon_alpha"})";
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json", std::ios::binary);
    f << content;
  }

  set_mission_phase(mc_msgs::msg::MissionPhase::SEARCH);
  peer_->send_activate(12, "recon_alpha.json");
  MockOrchestratorPeer::AckFields rejected;
  ASSERT_TRUE(spin_until_ack(rejected));
  ASSERT_EQ(rejected.result, 3);  // REJECTED_PHASE

  set_mission_phase(mc_msgs::msg::MissionPhase::PREFLIGHT);
  peer_->send_activate(13, "recon_alpha.json");
  MockOrchestratorPeer::AckFields accepted;
  ASSERT_TRUE(spin_until_ack(accepted));
  EXPECT_EQ(accepted.result, 0);  // ACCEPTED
}

TEST_F(MissionOrchestratorNodeTest, RemoveActivePlanIsProtected)
{
  const std::string content = R"({"name":"recon_alpha"})";
  std::filesystem::create_directories(plans_dir_);
  {
    std::ofstream f(plans_dir_ + "/recon_alpha.json", std::ios::binary);
    f << content;
  }

  peer_->send_activate(1, "recon_alpha.json");
  MockOrchestratorPeer::AckFields ack;
  ASSERT_TRUE(spin_until_ack(ack));
  ASSERT_EQ(ack.result, 0);

  peer_->send_remove_file("/recon_alpha.json");
  MockOrchestratorPeer::FtpReply reply;
  ASSERT_TRUE(spin_until_reply(reply));
  EXPECT_EQ(reply.opcode, kOpNak);
  EXPECT_EQ(reply.data[0], kErrFileProtected);
  EXPECT_TRUE(std::filesystem::exists(plans_dir_ + "/recon_alpha.json"));
}
