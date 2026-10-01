// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md, IGS-004).

// IGS-004 node-integration test: exercises GnssSimNode::send_hil_gps() end to
// end -- known GcsCommand/VehicleState/OpticalFlow inputs (the same test
// seams IGS-001/003 already use) drive a known IGS-003 WGS-84 fix, then the
// resulting HIL_GPS frame is received on a loopback UDP mock FCC and decoded
// with the real generated MAVLink headers, same technique
// test_mission_protocol_client.cpp (fcc_bridge) already uses one layer down.
//
// Firmware-verification note (Open Item 2, INDOOR-GNSS-SIMULATION-BACKLOG.md):
// PX4's mavlink_receiver.cpp has no handler at all for
// MAVLINK_MSG_ID_GPS_INPUT -- confirmed against the real firmware at both
// the v1.14.4 tag originally and the v1.17.0 tag this project's target
// firmware later moved to. The message it actually accepts as a fake GPS
// receiver is HIL_GPS (handle_message_hil_gps(), gated by MAV_USEHILGPS=1
// and a matching sysid), so that is what send_hil_gps() constructs and
// what this test decodes -- see gnss_sim_node.hpp's own class-level doc
// comment for the full citation.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

#include <mc_msgs/msg/baro_altitude.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/indoor_gnss_sim_status.hpp>
#include <mc_msgs/msg/optical_flow.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "indoor_nav/gnss_sim_node.hpp"

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

rclcpp::Time stamp_at(double t_s)
{
  return rclcpp::Time(static_cast<int64_t>(t_s * 1e9), RCL_ROS_TIME);
}

// A UDP socket bound to an ephemeral loopback port, playing the FCC's side
// of this one-way channel -- same helper shape as
// test_mission_protocol_client.cpp's UdpEndpoint (fcc_bridge), trimmed to
// receive-only since send_hil_gps() never expects a reply.
class MockFcc
{
public:
  MockFcc()
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
  ~MockFcc() {::close(fd_);}

  uint16_t port() const {return ntohs(addr_.sin_port);}

  // Returns false if nothing arrived within timeout_s (used to prove
  // send_hil_gps() stays silent before an anchor).
  bool recv(double timeout_s, mavlink_message_t & out)
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
      if (mavlink_parse_char(MAVLINK_COMM_3, raw[i], &out, &status)) {
        return true;
      }
    }
    return false;
  }

private:
  int fd_;
  sockaddr_in addr_{};
};
}  // namespace

class HilGpsSenderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ::setenv("FCC_ETH_IP", "127.0.0.1", 1);
    ::setenv("FCC_GPS_INPUT_PORT", std::to_string(fcc_.port()).c_str(), 1);
    ::setenv("DRONE_ID", "7", 1);
    node_ = std::make_shared<indoor_nav::GnssSimNode>();
  }

  void TearDown() override
  {
    node_.reset();
    ::unsetenv("FCC_ETH_IP");
    ::unsetenv("FCC_GPS_INPUT_PORT");
    ::unsetenv("DRONE_ID");
  }

  MockFcc fcc_;
  std::shared_ptr<indoor_nav::GnssSimNode> node_;
};

TEST_F(HilGpsSenderTest, SendsNothingBeforeAnchor)
{
  node_->send_hil_gps();

  mavlink_message_t msg;
  EXPECT_FALSE(fcc_.recv(0.2, msg))
    << "send_hil_gps() must stay silent until an operator anchor makes wgs84_fix() valid "
    "(Safety Considerations #5) -- there is no honest fix to report before that.";
}

TEST_F(HilGpsSenderTest, SendsCorrectHilGpsFrameFromKnownFix)
{
  constexpr double kLatDeg = 1.5;
  constexpr double kLonDeg = 103.8;
  constexpr float kAltMsl0 = 50.0F;
  constexpr float kAltMsl1 = 45.0F;  // descended 5 m since anchor

  // IGS-005: real_gps_ok_ starts true (fail-safe default) until a real
  // /mc/fcc/vehicle_state sample proves otherwise -- vs0's default
  // fix_type=0 does that, so it must arrive before the anchor attempt or
  // reanchor() rejects it (indoor-only interlock).
  mc_msgs::msg::VehicleState vs0;
  vs0.header.stamp = stamp_at(1000.0);
  vs0.q = {1.0F, 0.0F, 0.0F, 0.0F};  // identity: body frame == local NED frame
  node_->on_vehicle_state(vs0);

  mc_msgs::msg::BaroAltitude baro0;
  baro0.header.stamp = stamp_at(1000.0);
  baro0.alt_m = kAltMsl0;
  node_->on_baro_altitude(baro0);

  mc_msgs::msg::GcsCommand anchor;
  anchor.header.stamp = stamp_at(1000.0);
  anchor.command = indoor_nav::kMcCmdIndoorGnssSimAnchor;
  anchor.params[0] = static_cast<float>(kLatDeg);
  anchor.params[1] = static_cast<float>(kLonDeg);
  anchor.params[2] = kAltMsl0;
  node_->on_gcs_command(anchor);

  mc_msgs::msg::OpticalFlow flow0;
  flow0.header.stamp = stamp_at(1000.0);
  flow0.vx = 0.0F;
  flow0.vy = 0.0F;
  node_->on_optical_flow(flow0);  // primes last_flow_stamp_, no integration yet

  mc_msgs::msg::OpticalFlow flow1;
  flow1.header.stamp = stamp_at(1001.0);
  flow1.vx = 2.0F;  // body FRD forward == local north at this identity attitude
  flow1.vy = 0.0F;
  node_->on_optical_flow(flow1);  // integrates x_ += 2.0 * 1.0 s = 2.0 m north

  mc_msgs::msg::BaroAltitude baro1;
  baro1.header.stamp = stamp_at(1001.0);
  baro1.alt_m = kAltMsl1;  // -> z_ = 5.0 m down, vz_ low-passed toward 5.0 m/s
  node_->on_baro_altitude(baro1);

  const indoor_nav::GnssSimNode::WgsFix fix = node_->wgs84_fix();
  ASSERT_TRUE(fix.valid);
  const mc_msgs::msg::IndoorGnssSimStatus status = node_->gnss_sim_status();
  const nav_msgs::msg::Odometry estimate = node_->local_estimate();

  node_->send_hil_gps();

  mavlink_message_t msg;
  ASSERT_TRUE(fcc_.recv(2.0, msg));
  ASSERT_EQ(msg.msgid, MAVLINK_MSG_ID_HIL_GPS);
  EXPECT_EQ(msg.sysid, 7);  // must equal DRONE_ID -- PX4's own sysid gate (see this file's header)

  mavlink_hil_gps_t hil_gps{};
  mavlink_msg_hil_gps_decode(&msg, &hil_gps);

  EXPECT_EQ(hil_gps.fix_type, 3);
  EXPECT_EQ(hil_gps.lat, static_cast<int32_t>(std::llround(fix.lat_deg * 1e7)));
  EXPECT_EQ(hil_gps.lon, static_cast<int32_t>(std::llround(fix.lon_deg * 1e7)));
  EXPECT_EQ(
    hil_gps.alt,
    static_cast<int32_t>(std::llround(static_cast<double>(fix.alt_msl) * 1000.0)));

  // Same truncating cm-conversion send_hil_gps() itself applies -- an exact
  // match here is the point (this test asserts correct encoding, not IGS-003's
  // WGS-84/uncertainty math, which has its own test coverage).
  EXPECT_EQ(hil_gps.eph, static_cast<uint16_t>(status.eph * 100.0F));
  EXPECT_EQ(hil_gps.epv, static_cast<uint16_t>(status.epv * 100.0F));

  constexpr double kEphAtAnchorM = 1.0;      // gnss_sim_node.hpp default
  constexpr double kEkf2ReqEphDefaultM = 3.0;  // PX4 EKF2_REQ_EPH default (v1.14.4 and v1.17.0)
  constexpr double kSatsAtAnchor = 10.0;     // gnss_sim_node.hpp default
  constexpr double kMinSats = 4.0;           // gnss_sim_node.hpp default
  const double eph_frac = std::clamp(
    (static_cast<double>(status.eph) - kEphAtAnchorM) / (kEkf2ReqEphDefaultM - kEphAtAnchorM),
    0.0, 1.0);
  const uint8_t expected_sats = static_cast<uint8_t>(
    std::lround(kSatsAtAnchor - eph_frac * (kSatsAtAnchor - kMinSats)));
  EXPECT_EQ(hil_gps.satellites_visible, expected_sats);

  const float vx = static_cast<float>(estimate.twist.twist.linear.x);
  const float vy = static_cast<float>(estimate.twist.twist.linear.y);
  const float vz = static_cast<float>(estimate.twist.twist.linear.z);
  EXPECT_EQ(hil_gps.vn, static_cast<int16_t>(vx * 100.0F));
  EXPECT_EQ(hil_gps.ve, static_cast<int16_t>(vy * 100.0F));
  EXPECT_EQ(hil_gps.vd, static_cast<int16_t>(vz * 100.0F));
  const float ground_speed = std::sqrt(vx * vx + vy * vy);
  EXPECT_EQ(hil_gps.vel, static_cast<uint16_t>(ground_speed * 100.0F));
  const double cog_deg = std::fmod(
    std::atan2(static_cast<double>(vy), static_cast<double>(vx)) * 180.0 / M_PI + 360.0, 360.0);
  EXPECT_EQ(hil_gps.cog, static_cast<uint16_t>(std::lround(cog_deg * 100.0)));
}
