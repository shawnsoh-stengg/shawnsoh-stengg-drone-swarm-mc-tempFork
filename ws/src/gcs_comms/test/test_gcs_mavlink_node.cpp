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
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/link_status.hpp>
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

  static constexpr uint8_t kGcsSysId = 255;   // ICD §4.1
  static constexpr uint8_t kGcsCompId = 1;

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
    node_ = std::make_shared<gcs_comms::GcsMavlinkNode>(options);
    peer_ = std::make_shared<MockGcsPeer>(25560);

    sub_command_ = node_->create_subscription<mc_msgs::msg::GcsCommand>(
      "/mc/gcs/command", rclcpp::QoS(10),
      [this](mc_msgs::msg::GcsCommand::SharedPtr msg) {command_msgs_.push_back(msg);});
    sub_link_status_ = node_->create_subscription<mc_msgs::msg::LinkStatus>(
      "/mc/gcs/link_status", rclcpp::QoS(10),
      [this](mc_msgs::msg::LinkStatus::SharedPtr msg) {link_status_msgs_.push_back(msg);});
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
  std::vector<mc_msgs::msg::GcsCommand::SharedPtr> command_msgs_;
  std::vector<mc_msgs::msg::LinkStatus::SharedPtr> link_status_msgs_;
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
