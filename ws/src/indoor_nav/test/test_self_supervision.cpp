// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md, IGS-005).

// IGS-005 node-integration tests: real-GPS lockout, staleness/drift
// watchdog, and waypoint-deviation sanity bound. Drives GnssSimNode's test
// seams directly (on_vehicle_state/on_optical_flow/on_gcs_command/
// on_mission_upload/check_self_supervision/send_hil_gps), same technique
// test_hil_gps_sender.cpp (IGS-004) already uses for its own loopback UDP
// mock FCC -- reused here to prove "stream stopped" means no HIL_GPS frame
// arrives, not just that some internal flag is set.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <cmath>
#include <cstdlib>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mc_msgs/msg/baro_altitude.hpp>
#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
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

// Same loopback-UDP mock-FCC shape as test_hil_gps_sender.cpp's MockFcc,
// trimmed to just "did anything arrive" -- these tests care whether
// send_hil_gps() stays silent, not about decoding the frame's fields
// (already covered by IGS-004's own tests).
class MockFcc
{
public:
  MockFcc()
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));

    socklen_t len = sizeof(addr_);
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr_), &len);
  }
  ~MockFcc() {::close(fd_);}

  uint16_t port() const {return ntohs(addr_.sin_port);}

  bool received_anything(double timeout_s)
  {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_s);
    tv.tv_usec = static_cast<suseconds_t>((timeout_s - static_cast<double>(tv.tv_sec)) * 1e6);
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t raw[512];
    return ::recv(fd_, raw, sizeof(raw), 0) > 0;
  }

private:
  int fd_;
  sockaddr_in addr_{};
};
}  // namespace

class SelfSupervisionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ::setenv("FCC_ETH_IP", "127.0.0.1", 1);
    ::setenv("FCC_GPS_INPUT_PORT", std::to_string(fcc_.port()).c_str(), 1);
    ::setenv("DRONE_ID", "1", 1);
    node_ = std::make_shared<indoor_nav::GnssSimNode>();
  }

  void TearDown() override
  {
    node_.reset();
    ::unsetenv("FCC_ETH_IP");
    ::unsetenv("FCC_GPS_INPUT_PORT");
    ::unsetenv("DRONE_ID");
  }

  // A "no real fix" sample -- fix_type/hdop default-constructed to 0/0.0,
  // which is deliberately NOT gnss_ok() (fix_type >= 3 required), so this
  // also clears the real-GPS lockout's fail-safe-locked startup default.
  mc_msgs::msg::VehicleState no_real_fix_vehicle_state(double t_s)
  {
    mc_msgs::msg::VehicleState msg;
    msg.header.stamp = stamp_at(t_s);
    msg.q = {1.0F, 0.0F, 0.0F, 0.0F};
    return msg;
  }

  mc_msgs::msg::BaroAltitude baro(double t_s, float alt_m = 50.0F)
  {
    mc_msgs::msg::BaroAltitude msg;
    msg.header.stamp = stamp_at(t_s);
    msg.alt_m = alt_m;
    return msg;
  }

  // Default anchor is near Singapore -- a longitude well over the 50 m
  // deviation bound when misread as metres (IGS-006).
  mc_msgs::msg::GcsCommand anchor_command(
    double t_s, float lat_deg = kAnchorLatDeg, float lon_deg = kAnchorLonDeg)
  {
    mc_msgs::msg::GcsCommand msg;
    msg.header.stamp = stamp_at(t_s);
    msg.command = indoor_nav::kMcCmdIndoorGnssSimAnchor;
    msg.params[0] = lat_deg;
    msg.params[1] = lon_deg;
    msg.params[2] = 50.0F;
    return msg;
  }

  // Anchors the node at t_s using a preceding no-real-fix vehicle_state
  // and baro sample -- the common "get to a streaming baseline" setup every
  // test below starts from.
  void anchor_at(double t_s, float lat_deg = kAnchorLatDeg, float lon_deg = kAnchorLonDeg)
  {
    node_->on_vehicle_state(no_real_fix_vehicle_state(t_s));
    node_->on_baro_altitude(baro(t_s));
    node_->on_gcs_command(anchor_command(t_s, lat_deg, lon_deg));
    ASSERT_TRUE(node_->wgs84_fix().valid);
  }

  // Fresh vehicle_state/baro/optical_flow samples at t_s, so a
  // check_self_supervision(t_s) call can only trip on deviation/drift.
  void feed_fresh_inputs(double t_s)
  {
    node_->on_vehicle_state(no_real_fix_vehicle_state(t_s));
    node_->on_baro_altitude(baro(t_s));
    mc_msgs::msg::OpticalFlow flow;
    flow.header.stamp = stamp_at(t_s);
    node_->on_optical_flow(flow);
  }

  // A one-waypoint plan. x/y are lat/lon (deg), z metres above home --
  // how both GCS decoders fill ingress_waypoints (IGS-006).
  static mc_msgs::msg::MissionUpload one_waypoint_plan(
    double t_s, double lat_deg, double lon_deg, double alt_m = 10.0)
  {
    mc_msgs::msg::MissionUpload mission;
    mission.header.stamp = stamp_at(t_s);
    geometry_msgs::msg::PoseStamped wp;
    wp.pose.position.x = lat_deg;
    wp.pose.position.y = lon_deg;
    wp.pose.position.z = alt_m;
    mission.ingress_waypoints.push_back(wp);
    return mission;
  }

  static constexpr float kAnchorLatDeg = 1.5F;
  static constexpr float kAnchorLonDeg = 103.8F;

  MockFcc fcc_;
  std::shared_ptr<indoor_nav::GnssSimNode> node_;
};

// --- Real-GPS lockout (AC 1) ---------------------------------------------

TEST_F(SelfSupervisionTest, RefusesAnchorWhileRealGpsPresent)
{
  mc_msgs::msg::VehicleState real_fix;
  real_fix.header.stamp = stamp_at(0.0);
  real_fix.fix_type = 3;
  real_fix.hdop = 1.0F;
  node_->on_vehicle_state(real_fix);
  ASSERT_TRUE(node_->real_gps_ok());

  node_->on_gcs_command(anchor_command(0.0));

  EXPECT_FALSE(node_->wgs84_fix().valid)
    << "reanchor() must reject an anchor attempt while a real GPS fix (fix_type>=3, hdop<2.0) "
    "is present -- indoor-only interlock (Safety Considerations #3)";
}

TEST_F(SelfSupervisionTest, StopsStreamingWhenRealGpsFixAppears)
{
  anchor_at(1000.0);
  node_->send_hil_gps();
  ASSERT_TRUE(fcc_.received_anything(2.0)) << "sanity check: streaming must work before the fix";

  mc_msgs::msg::VehicleState real_fix;
  real_fix.header.stamp = stamp_at(1001.0);
  real_fix.fix_type = 3;
  real_fix.hdop = 1.0F;
  node_->on_vehicle_state(real_fix);
  ASSERT_TRUE(node_->real_gps_ok());

  node_->send_hil_gps();
  EXPECT_FALSE(fcc_.received_anything(0.3))
    << "send_hil_gps() must stay silent once a real GPS fix is present, even though the node "
    "is still anchored -- \"stops streaming\", not a one-time refusal";
}

// --- Staleness watchdog (AC 2) -------------------------------------------

TEST_F(SelfSupervisionTest, ForcedStalenessPublishesFaultAndStopsStream)
{
  anchor_at(2000.0);
  mc_msgs::msg::OpticalFlow flow0;
  flow0.header.stamp = stamp_at(2000.0);
  node_->on_optical_flow(flow0);  // primes last_flow_stamp_ -- input WAS flowing
  ASSERT_FALSE(node_->gps_sim_lost());
  node_->send_hil_gps();
  ASSERT_TRUE(fcc_.received_anything(2.0)) << "sanity check: streaming must work before staleness";

  // input_staleness_timeout_s_ defaults to 1.0 s -- nothing arrives after
  // t=2000.0, so evaluating at t=2005.0 is well past it.
  node_->check_self_supervision(stamp_at(2005.0));

  EXPECT_TRUE(node_->gps_sim_lost());
  const auto fault = node_->latest_fault();
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->fault_id, mc_msgs::msg::FaultEvent::INDOOR_GNSS_SIM_LOST);
  EXPECT_NE(fault->description.find("stale"), std::string::npos);

  node_->send_hil_gps();
  EXPECT_FALSE(fcc_.received_anything(0.3))
    << "staleness must stop the stream entirely, not degrade it while still live";

  // Does not silently self-heal -- re-evaluating later while still stale
  // must not clear the latch.
  node_->check_self_supervision(stamp_at(2010.0));
  EXPECT_TRUE(node_->gps_sim_lost());

  // The only recovery path is an explicit re-anchor.
  node_->on_vehicle_state(no_real_fix_vehicle_state(2010.0));
  node_->on_gcs_command(anchor_command(2010.0));
  EXPECT_FALSE(node_->gps_sim_lost()) << "reanchor() must clear a latched staleness fault";
}

// FT-006: the vertical input (raw baro) is supervised like the others -- a
// frozen altitude must stop the stream, not hold HIL_GPS altitude constant.
TEST_F(SelfSupervisionTest, StaleBaroAloneFaults)
{
  anchor_at(4000.0);
  for (double t = 4000.0; t <= 4003.0; t += 0.5) {
    mc_msgs::msg::OpticalFlow flow;
    flow.header.stamp = stamp_at(t);
    node_->on_optical_flow(flow);
    node_->on_vehicle_state(no_real_fix_vehicle_state(t));
  }

  node_->check_self_supervision(stamp_at(4003.0));

  EXPECT_TRUE(node_->gps_sim_lost());
  const auto fault = node_->latest_fault();
  ASSERT_TRUE(fault.has_value());
  EXPECT_NE(fault->description.find("/mc/sensors/baro_altitude"), std::string::npos)
    << fault->description;
}

// --- Waypoint-deviation sanity bound (AC 3) -------------------------------

TEST_F(SelfSupervisionTest, ForcedLargeWaypointDeviationFaultsWithoutChangingEstimate)
{
  anchor_at(3000.0);

  // A single ingress waypoint right at the false origin the anchor above
  // just declared, given as lat/lon like a real plan (IGS-006).
  node_->on_mission_upload(
    one_waypoint_plan(
      3000.0, static_cast<double>(kAnchorLatDeg), static_cast<double>(kAnchorLonDeg)));

  // Drive the local estimate far north -- well past max_waypoint_
  // deviation_m_'s default (50 m) -- via two optical_flow samples, the same
  // technique IGS-001's own tests use. Kept within input_staleness_
  // timeout_s_ and eph's own bound so deviation is isolated as the sole
  // trigger.
  mc_msgs::msg::OpticalFlow flow0;
  flow0.header.stamp = stamp_at(3000.0);
  node_->on_optical_flow(flow0);  // primes last_flow_stamp_

  mc_msgs::msg::OpticalFlow flow1;
  flow1.header.stamp = stamp_at(3000.3);  // dt=0.3s, comfortably under max_flow_dt_s_ (0.5s)
  flow1.vx = 400.0F;  // -> x_ += 400 * 0.3 = 120 m north
  node_->on_optical_flow(flow1);

  EXPECT_NEAR(node_->compute_waypoint_deviation(), 120.0F, 0.01F);

  const nav_msgs::msg::Odometry estimate_before = node_->local_estimate();

  node_->check_self_supervision(stamp_at(3000.3));

  EXPECT_TRUE(node_->gps_sim_lost());
  const auto fault = node_->latest_fault();
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->fault_id, mc_msgs::msg::FaultEvent::INDOOR_GNSS_SIM_LOST);
  EXPECT_NE(fault->description.find("deviation"), std::string::npos);

  const nav_msgs::msg::Odometry estimate_after = node_->local_estimate();
  EXPECT_EQ(estimate_before.pose.pose.position.x, estimate_after.pose.pose.position.x);
  EXPECT_EQ(estimate_before.pose.pose.position.y, estimate_after.pose.pose.position.y);
  EXPECT_EQ(estimate_before.pose.pose.position.z, estimate_after.pose.pose.position.z);
}

// IGS-006 regression: a real plan whose waypoint sits at the anchor must
// not fault. Before the fix, its raw lat/lon degrees were subtracted from
// local metres -- ~104 "m" at this longitude, over the 50 m bound -- so the
// first tick after any real upload latched INDOOR_GNSS_SIM_LOST.
TEST_F(SelfSupervisionTest, RealPlanAtAnchorDoesNotFault)
{
  anchor_at(4000.0);
  node_->on_mission_upload(
    one_waypoint_plan(
      4000.0, static_cast<double>(kAnchorLatDeg), static_cast<double>(kAnchorLonDeg)));
  feed_fresh_inputs(4000.0);

  EXPECT_NEAR(node_->compute_waypoint_deviation(), 0.0F, 0.01F);
  node_->check_self_supervision(stamp_at(4000.0));
  EXPECT_FALSE(node_->gps_sim_lost());
}

// Waypoint z is above *home*, the estimate's z is from the baro anchor --
// unrelated datums, so the bound is horizontal only.
TEST_F(SelfSupervisionTest, WaypointAltitudeIgnored)
{
  anchor_at(4100.0);
  node_->on_mission_upload(
    one_waypoint_plan(
      4100.0, static_cast<double>(kAnchorLatDeg), static_cast<double>(kAnchorLonDeg), 500.0));
  feed_fresh_inputs(4100.0);

  EXPECT_NEAR(node_->compute_waypoint_deviation(), 0.0F, 0.01F);
  node_->check_self_supervision(stamp_at(4100.0));
  EXPECT_FALSE(node_->gps_sim_lost());
}

// No false origin yet -> no local frame to project a waypoint into.
TEST_F(SelfSupervisionTest, NoDeviationBeforeAnchor)
{
  node_->on_mission_upload(one_waypoint_plan(4200.0, 45.0, 90.0));
  EXPECT_EQ(node_->compute_waypoint_deviation(), 0.0F);
}

// Anchor just west of +180 deg, waypoint 0.0003 deg further east, written
// on the -180 side: ~33 m, not ~40000 km.
TEST_F(SelfSupervisionTest, DeviationWrapsAtAntimeridian)
{
  constexpr float kLonDeg = 179.9998F;
  anchor_at(4300.0, 0.0F, kLonDeg);
  node_->on_mission_upload(
    one_waypoint_plan(4300.0, 0.0, static_cast<double>(kLonDeg) + 0.0003 - 360.0));

  const double expected_m = 0.0003 * M_PI / 180.0 * 6371000.0;
  EXPECT_NEAR(node_->compute_waypoint_deviation(), expected_m, 0.01);
}

// The waypoint projection is the inverse of wgs84_fix()'s: a waypoint at
// the fix the node itself would send sits on the estimate.
TEST_F(SelfSupervisionTest, WaypointAtOwnFixRoundTrips)
{
  anchor_at(4400.0);
  node_->on_mission_upload(
    one_waypoint_plan(
      4400.0, static_cast<double>(kAnchorLatDeg), static_cast<double>(kAnchorLonDeg)));

  mc_msgs::msg::OpticalFlow flow0;
  flow0.header.stamp = stamp_at(4400.0);
  node_->on_optical_flow(flow0);
  mc_msgs::msg::OpticalFlow flow1;
  flow1.header.stamp = stamp_at(4400.3);
  flow1.vx = 100.0F;   // 30 m north
  flow1.vy = -200.0F;  // 60 m west
  node_->on_optical_flow(flow1);

  const nav_msgs::msg::Odometry estimate = node_->local_estimate();
  const double expected_m =
    std::hypot(estimate.pose.pose.position.x, estimate.pose.pose.position.y);
  ASSERT_GT(expected_m, 50.0);
  EXPECT_NEAR(node_->compute_waypoint_deviation(), expected_m, 0.01);

  const auto fix = node_->wgs84_fix();
  node_->on_mission_upload(one_waypoint_plan(4400.3, fix.lat_deg, fix.lon_deg));
  EXPECT_NEAR(node_->compute_waypoint_deviation(), 0.0F, 0.01F);
}
