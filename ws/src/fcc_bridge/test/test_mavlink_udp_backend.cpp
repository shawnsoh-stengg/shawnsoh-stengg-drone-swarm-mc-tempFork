// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

// FCC-007: MavlinkUdpBackend state aggregation and mission-protocol
// round-trips against a mock PX4 SITL peer (plain UDP socket in the test,
// no live PX4/Gazebo — that's TP-ST level and explicitly out of scope,
// see FCC-MISSION-CHANNEL-BACKLOG.md).

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fcc_bridge/mavlink_udp_backend.hpp"

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

// A UDP socket bound to an ephemeral loopback port — plays the mock PX4
// SITL peer's side of the connection (mirrors test_mission_protocol_client
// .cpp's UdpEndpoint).
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
    if (mavlink_parse_char(MAVLINK_COMM_2, raw[i], &out, &status)) {
      return true;
    }
  }
  return false;
}

template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

constexpr uint16_t kTestPort = 25540;
}  // namespace

class MavlinkUdpBackendTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setenv("SITL_FCC_PORT", std::to_string(kTestPort).c_str(), 1);
    setenv("DRONE_ID", "1", 1);

    node_ = std::make_shared<rclcpp::Node>("mavlink_udp_backend_test");
    backend_ = std::make_unique<fcc_bridge::MavlinkUdpBackend>(node_.get());

    backend_addr_.sin_family = AF_INET;
    backend_addr_.sin_port = htons(kTestPort);
    ::inet_pton(AF_INET, "127.0.0.1", &backend_addr_.sin_addr);
  }

  void TearDown() override
  {
    backend_.reset();
    unsetenv("SITL_FCC_PORT");
    unsetenv("DRONE_ID");
  }

  // Sends exactly one throwaway datagram from the mock peer so
  // MavlinkUdpBackend learns its address (peer_known_) before a test
  // exercises a path that requires it (send_command, upload_mission,
  // update_mission_item). Deliberately sent once, not resent inside the
  // poll loop: resending risks a second copy still sitting undrained in
  // recv_loop_'s queue right as a later handshake call grabs
  // socket_use_mtx_ — MissionProtocolClient's single-shot reads (no
  // retry-on-mismatch, unlike upload_mission()'s request-wait loop) would
  // then consume that stale datagram instead of the real protocol reply
  // and fail immediately. Waits on xy_valid rather than is_link_alive():
  // last_fcc_msg_time_ is seeded to construction time, so a generous
  // is_link_alive(N) timeout reads "alive" immediately regardless of
  // whether this datagram was actually processed. The trailing sleep gives
  // recv_loop_ one more drain pass so its queue is confirmed empty before
  // the caller starts a mission-protocol call.
  void establish_peer()
  {
    mavlink_message_t msg;
    mavlink_global_position_int_t pos{};
    mavlink_msg_global_position_int_encode(1, 1, &msg, &pos);
    send_frame(fcc_.fd(), backend_addr_, msg);
    ASSERT_TRUE(
      wait_until(
        [&] {return backend_->get_vehicle_state().xy_valid;}, std::chrono::seconds(3)));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<fcc_bridge::MavlinkUdpBackend> backend_;
  sockaddr_in backend_addr_{};
  UdpEndpoint fcc_;  // mock PX4 SITL peer
};

TEST_F(MavlinkUdpBackendTest, AggregatesGlobalPositionIntIntoVehicleState)
{
  mavlink_message_t msg;
  mavlink_global_position_int_t pos{};
  pos.lat = 13521000;         // 1.3521 deg * 1e7
  pos.lon = 1038198000;       // 103.8198 deg * 1e7
  pos.alt = 55000;            // 55 m MSL, mm
  pos.vx = 300;                // 3.0 m/s, cm/s
  pos.vy = -150;               // -1.5 m/s
  pos.vz = 0;
  pos.hdg = 9000;               // 90 deg, cdeg
  mavlink_msg_global_position_int_encode(1, 1, &msg, &pos);

  ASSERT_TRUE(
    wait_until(
      [&] {
        send_frame(fcc_.fd(), backend_addr_, msg);
        return backend_->get_vehicle_state().xy_valid;
      }, std::chrono::seconds(3)));

  const auto state = backend_->get_vehicle_state();
  EXPECT_NEAR(state.lat, 1.3521, 1e-6);
  EXPECT_NEAR(state.lon, 103.8198, 1e-6);
  EXPECT_FLOAT_EQ(state.alt_msl, 55.0F);
  EXPECT_FLOAT_EQ(state.vx, 3.0F);
  EXPECT_FLOAT_EQ(state.vy, -1.5F);
  EXPECT_NEAR(state.heading, M_PI / 2.0, 1e-3);
}

TEST_F(MavlinkUdpBackendTest, AggregatesAttitudeEulerAsQuaternion)
{
  mavlink_message_t msg;
  mavlink_attitude_t att{};
  att.roll = 0.0F;
  att.pitch = 0.0F;
  att.yaw = static_cast<float>(M_PI / 2.0);  // 90 deg about Z
  mavlink_msg_attitude_encode(1, 1, &msg, &att);

  ASSERT_TRUE(
    wait_until(
      [&] {
        send_frame(fcc_.fd(), backend_addr_, msg);
        const auto q = backend_->get_vehicle_state().q;
        return std::abs(q[0] - 1.0F) > 1e-3F || std::abs(q[3]) > 1e-3F;
      }, std::chrono::seconds(3)));

  const auto q = backend_->get_vehicle_state().q;
  // Pure 90 deg yaw: q = [cos(45deg), 0, 0, sin(45deg)].
  EXPECT_NEAR(q[0], std::cos(M_PI / 4.0), 1e-3);
  EXPECT_NEAR(q[1], 0.0, 1e-3);
  EXPECT_NEAR(q[2], 0.0, 1e-3);
  EXPECT_NEAR(q[3], std::sin(M_PI / 4.0), 1e-3);
}

TEST_F(MavlinkUdpBackendTest, AggregatesBatteryRemainingAsFraction)
{
  mavlink_message_t msg;
  mavlink_battery_status_t batt{};
  batt.battery_remaining = 42;  // percent
  mavlink_msg_battery_status_encode(1, 1, &msg, &batt);

  ASSERT_TRUE(
    wait_until(
      [&] {
        send_frame(fcc_.fd(), backend_addr_, msg);
        return backend_->get_vehicle_state().battery_remaining > 0.0F;
      }, std::chrono::seconds(3)));

  EXPECT_FLOAT_EQ(backend_->get_vehicle_state().battery_remaining, 0.42F);
}

TEST_F(MavlinkUdpBackendTest, IsLinkAliveFalseBeforeAnyMessageWithZeroTimeout)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_FALSE(backend_->is_link_alive(0.0));
}

TEST_F(MavlinkUdpBackendTest, IsLinkAliveTrueAfterRecentMessage)
{
  establish_peer();
  EXPECT_TRUE(backend_->is_link_alive(5.0));
}

TEST_F(MavlinkUdpBackendTest, UpdateMissionItemReturnsFalseWithNoPeerYet)
{
  // No datagram has been sent by the mock at all — MavlinkUdpBackend has
  // nowhere to send a mission request until PX4 SITL has spoken first.
  EXPECT_FALSE(
    backend_->update_mission_item(fcc_bridge::FORMATION_SLOT, fcc_bridge::MissionItem{}));
}

TEST_F(MavlinkUdpBackendTest, SendCommandEmitsCommandLongWithAllParams)
{
  establish_peer();

  std::array<float, 7> params{1, 2, 3, 4, 5, 6, 7};
  backend_->send_command(400, params);

  mavlink_message_t msg;
  ASSERT_TRUE(recv_frame(fcc_.fd(), 2.0, msg));
  ASSERT_EQ(msg.msgid, static_cast<uint32_t>(MAVLINK_MSG_ID_COMMAND_LONG));

  mavlink_command_long_t cmd{};
  mavlink_msg_command_long_decode(&msg, &cmd);
  EXPECT_EQ(cmd.command, 400U);
  EXPECT_FLOAT_EQ(cmd.param1, 1.0F);
  EXPECT_FLOAT_EQ(cmd.param7, 7.0F);
}

// FCC-009: no peer known yet — same "nowhere to send" convention
// send_command() already uses (drop + warn, no throw), verified here by
// confirming the mock peer never receives anything within a short window.
TEST_F(MavlinkUdpBackendTest, StreamOffboardSetpointDropsSilentlyWithNoPeerYet)
{
  fcc_bridge::OffboardSetpoint sp;
  sp.x = 1.0F;
  backend_->stream_offboard_setpoint(sp);

  mavlink_message_t msg;
  EXPECT_FALSE(recv_frame(fcc_.fd(), 0.3, msg));
}

TEST_F(MavlinkUdpBackendTest, StreamOffboardSetpointEmitsSetPositionTargetLocalNed)
{
  establish_peer();

  fcc_bridge::OffboardSetpoint sp;
  sp.x = 1.5F;
  sp.y = -2.5F;
  sp.z = -10.0F;
  sp.yaw = 0.75F;
  backend_->stream_offboard_setpoint(sp);

  mavlink_message_t msg;
  ASSERT_TRUE(recv_frame(fcc_.fd(), 2.0, msg));
  ASSERT_EQ(msg.msgid, static_cast<uint32_t>(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED));

  mavlink_set_position_target_local_ned_t target{};
  mavlink_msg_set_position_target_local_ned_decode(&msg, &target);
  EXPECT_EQ(target.coordinate_frame, MAV_FRAME_LOCAL_NED);
  EXPECT_FLOAT_EQ(target.x, 1.5F);
  EXPECT_FLOAT_EQ(target.y, -2.5F);
  EXPECT_FLOAT_EQ(target.z, -10.0F);
  EXPECT_FLOAT_EQ(target.yaw, 0.75F);
  // Velocity/acceleration/yaw_rate ignored (position + yaw only) — same
  // POSITION_TARGET_TYPEMASK bits the backend sets.
  constexpr uint16_t kExpectedTypeMask = 8 | 16 | 32 | 64 | 128 | 256 | 2048;
  EXPECT_EQ(target.type_mask, kExpectedTypeMask);
}

TEST_F(MavlinkUdpBackendTest, UploadMissionCompletesFullHandshakeOverSharedSocket)
{
  establish_peer();

  std::vector<fcc_bridge::MissionItem> items;
  for (uint16_t seq = 0; seq < 3; ++seq) {
    fcc_bridge::MissionItem item;
    item.seq = seq;
    item.lat = 10000000 + seq;
    item.lon = 20000000 + seq;
    item.alt = 50.0F;
    items.push_back(item);
  }

  std::atomic<bool> mock_fcc_ok{false};
  std::thread mock_fcc([&] {
      mavlink_message_t msg;
      if (!recv_frame(fcc_.fd(), 2.0, msg) || msg.msgid != MAVLINK_MSG_ID_MISSION_COUNT) {
        return;
      }
      mavlink_mission_count_t count_msg{};
      mavlink_msg_mission_count_decode(&msg, &count_msg);
      if (count_msg.count != items.size()) {
        return;
      }

      for (uint16_t seq = 0; seq < count_msg.count; ++seq) {
        mavlink_message_t req_msg;
        mavlink_msg_mission_request_int_pack(
          1, 1, &req_msg, 1, 191, seq, MAV_MISSION_TYPE_MISSION);
        send_frame(fcc_.fd(), backend_addr_, req_msg);

        mavlink_message_t item_msg;
        if (!recv_frame(
          fcc_.fd(), 2.0,
          item_msg) || item_msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT)
        {
          return;
        }
      }

      mavlink_message_t ack_msg;
      mavlink_msg_mission_ack_pack(
        1, 1, &ack_msg, 1, 191, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
      send_frame(fcc_.fd(), backend_addr_, ack_msg);
      mock_fcc_ok = true;
    });

  const bool result = backend_->upload_mission(items);
  mock_fcc.join();

  EXPECT_TRUE(mock_fcc_ok) << "mock SITL peer did not see a well-formed handshake";
  EXPECT_TRUE(result);
}

TEST_F(MavlinkUdpBackendTest, UpdateMissionItemUsesPartialListOverSharedSocket)
{
  establish_peer();

  const auto item = [] {
      fcc_bridge::MissionItem i;
      i.seq = fcc_bridge::FORMATION_SLOT;
      i.lat = 10000000;
      i.lon = 20000000;
      i.alt = 50.0F;
      return i;
    } ();

  // update_mission_item()'s window for the FCC's MISSION_REQUEST_INT reply
  // is a tight 0.5s (MissionProtocolClient's production default — not
  // overridable through the FccBackend interface, correctly matching how
  // mission_manager_node/formation_controller_node call it for real). A
  // freshly-spawned mock thread can lose that whole budget to OS scheduling
  // latency before it even calls its first recv_frame(), so synchronize on
  // mock_ready to confirm it's already blocked in recv() before triggering
  // the handshake — this narrows the real critical path to a few syscalls,
  // comfortably inside 0.5s.
  std::atomic<bool> mock_ready{false};
  std::atomic<bool> mock_fcc_ok{false};
  std::thread mock_fcc([&] {
      mock_ready = true;
      mavlink_message_t msg;
      if (!recv_frame(
        fcc_.fd(), 2.0,
        msg) || msg.msgid != MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST)
      {
        return;
      }

      mavlink_message_t req_msg;
      mavlink_msg_mission_request_int_pack(
        1, 1, &req_msg, 1, 191, fcc_bridge::FORMATION_SLOT, MAV_MISSION_TYPE_MISSION);
      send_frame(fcc_.fd(), backend_addr_, req_msg);

      mavlink_message_t item_msg;
      if (!recv_frame(
        fcc_.fd(), 2.0,
        item_msg) || item_msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT)
      {
        return;
      }

      mavlink_message_t ack_msg;
      mavlink_msg_mission_ack_pack(
        1, 1, &ack_msg, 1, 191, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
      send_frame(fcc_.fd(), backend_addr_, ack_msg);
      mock_fcc_ok = true;
    });

  while (!mock_ready.load()) {
    std::this_thread::yield();
  }
  const bool result =
    backend_->update_mission_item(fcc_bridge::FORMATION_SLOT, item);
  mock_fcc.join();

  EXPECT_TRUE(mock_fcc_ok) << "mock SITL peer did not see MISSION_WRITE_PARTIAL_LIST";
  EXPECT_TRUE(result);
}
