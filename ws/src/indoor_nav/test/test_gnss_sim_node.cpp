// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md, IGS-001).

// IGS-001 node-integration test: a harness node publishes synthetic
// /mc/fcc/vehicle_state and /mc/sensors/optical_flow sequences directly
// (standing in for fcc_bridge_node / the not-yet-built HIF-008 driver,
// same technique test_fcc_bridge_node.cpp uses one layer down for
// XrceDdsBackend), then asserts the resulting /mc/indoor/local_estimate
// trajectory is within tolerance of a known ground truth. Message
// timestamps (not wall-clock arrival time) drive gnss_sim_node's dt
// computation, so the trajectory math is exact regardless of test
// scheduling jitter.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <vector>

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

template<typename Predicate>
bool spin_until(
  std::initializer_list<rclcpp::Node::SharedPtr> nodes, Predicate predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    for (const auto & n : nodes) {
      rclcpp::spin_some(n);
    }
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// [w,x,y,z] NED-to-body quaternion for a pure yaw rotation `psi_rad`,
// matching mc_msgs/VehicleState.msg's documented convention.
std::array<float, 4> yaw_quaternion(double psi_rad)
{
  return {
    static_cast<float>(std::cos(psi_rad / 2.0)), 0.0F, 0.0F,
    static_cast<float>(std::sin(psi_rad / 2.0))
  };
}

rclcpp::Time stamp_at(double t_s)
{
  return rclcpp::Time(static_cast<int64_t>(t_s * 1e9), RCL_ROS_TIME);
}
}  // namespace

class GnssSimNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<indoor_nav::GnssSimNode>();
    harness_ = std::make_shared<rclcpp::Node>("gnss_sim_node_test_harness");

    pub_vehicle_state_ = harness_->create_publisher<mc_msgs::msg::VehicleState>(
      "/mc/fcc/vehicle_state", rclcpp::QoS(10));
    pub_optical_flow_ = harness_->create_publisher<mc_msgs::msg::OpticalFlow>(
      "/mc/sensors/optical_flow", rclcpp::QoS(10));
    pub_baro_altitude_ = harness_->create_publisher<mc_msgs::msg::BaroAltitude>(
      "/mc/sensors/baro_altitude", rclcpp::QoS(10));
    pub_gcs_command_ = harness_->create_publisher<mc_msgs::msg::GcsCommand>(
      "/mc/gcs/command", rclcpp::QoS(10));
    sub_local_estimate_ = harness_->create_subscription<nav_msgs::msg::Odometry>(
      "/mc/indoor/local_estimate", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {estimates_.push_back(msg);});
    sub_gnss_sim_status_ = harness_->create_subscription<mc_msgs::msg::IndoorGnssSimStatus>(
      "/mc/indoor/gnss_sim_status", rclcpp::QoS(10),
      [this](mc_msgs::msg::IndoorGnssSimStatus::SharedPtr msg) {statuses_.push_back(msg);});
  }

  void TearDown() override
  {
    node_.reset();
    harness_.reset();
  }

  mc_msgs::msg::VehicleState make_vehicle_state(
    double t_s, float alt_msl, const std::array<float, 4> & q)
  {
    mc_msgs::msg::VehicleState msg;
    msg.header.stamp = stamp_at(t_s);
    msg.alt_msl = alt_msl;
    msg.q = q;
    return msg;
  }

  mc_msgs::msg::BaroAltitude make_baro(double t_s, float alt_m)
  {
    mc_msgs::msg::BaroAltitude msg;
    msg.header.stamp = stamp_at(t_s);
    msg.alt_m = alt_m;
    return msg;
  }

  mc_msgs::msg::OpticalFlow make_flow(double t_s, float vx, float vy)
  {
    mc_msgs::msg::OpticalFlow msg;
    msg.header.stamp = stamp_at(t_s);
    msg.vx = vx;
    msg.vy = vy;
    msg.ground_distance = 1.5F;
    msg.quality = 200;
    return msg;
  }

  mc_msgs::msg::GcsCommand make_anchor_command(
    double t_s, float lat_deg, float lon_deg, float alt_msl)
  {
    mc_msgs::msg::GcsCommand msg;
    msg.header.stamp = stamp_at(t_s);
    msg.command = indoor_nav::kMcCmdIndoorGnssSimAnchor;
    msg.params = {lat_deg, lon_deg, alt_msl, 0.0F, 0.0F, 0.0F, 0.0F};
    return msg;
  }

  std::shared_ptr<indoor_nav::GnssSimNode> node_;
  rclcpp::Node::SharedPtr harness_;
  rclcpp::Publisher<mc_msgs::msg::VehicleState>::SharedPtr pub_vehicle_state_;
  rclcpp::Publisher<mc_msgs::msg::OpticalFlow>::SharedPtr pub_optical_flow_;
  rclcpp::Publisher<mc_msgs::msg::BaroAltitude>::SharedPtr pub_baro_altitude_;
  rclcpp::Publisher<mc_msgs::msg::GcsCommand>::SharedPtr pub_gcs_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_local_estimate_;
  rclcpp::Subscription<mc_msgs::msg::IndoorGnssSimStatus>::SharedPtr sub_gnss_sim_status_;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> estimates_;
  std::vector<mc_msgs::msg::IndoorGnssSimStatus::SharedPtr> statuses_;
};

// Straight-line flight, level attitude (identity q): body-forward ==
// local-north. 10 flow samples at 10 Hz, 2 m/s forward, should integrate
// to ~1.8 m of north travel (the first sample only seeds last_flow_stamp_,
// per gnss_sim_node's own documented "need a prior sample for dt" rule --
// 9 integrated intervals of 0.1 s each at 2 m/s = 1.8 m).
TEST_F(GnssSimNodeTest, StraightLineLevelAttitudeIntegratesToExpectedDistance)
{
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 100.0F, level_q));

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  for (int i = 1; i <= 10; ++i) {
    pub_optical_flow_->publish(make_flow(i * 0.1, 2.0F, 0.0F));
  }

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 11U;}, std::chrono::seconds(3)));

  const auto & last = *estimates_.back();
  EXPECT_NEAR(last.pose.pose.position.x, 1.8, 0.05);
  EXPECT_NEAR(last.pose.pose.position.y, 0.0, 0.05);
  EXPECT_NEAR(last.twist.twist.linear.x, 2.0, 0.05);
}

// Same forward flow, but yawed 90 deg (nose pointing east) -- body-forward
// should integrate to local-east distance instead of north.
TEST_F(GnssSimNodeTest, YawedAttitudeRotatesFlowIntoLocalFrame)
{
  const std::array<float, 4> east_facing_q = yaw_quaternion(M_PI / 2.0);
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 100.0F, east_facing_q));

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  for (int i = 1; i <= 10; ++i) {
    pub_optical_flow_->publish(make_flow(i * 0.1, 2.0F, 0.0F));
  }

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 11U;}, std::chrono::seconds(3)));

  const auto & last = *estimates_.back();
  EXPECT_NEAR(last.pose.pose.position.x, 0.0, 0.05);
  EXPECT_NEAR(last.pose.pose.position.y, 1.8, 0.05);
}

// Baro-derived vertical: a 1 m altitude gain (climb) over 1 s should read
// as z decreasing by 1 m (NED down) and vz of about -1 m/s.
TEST_F(GnssSimNodeTest, BaroAltitudeChangeUpdatesVerticalPositionAndRate)
{
  pub_baro_altitude_->publish(make_baro(0.0, 100.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_baro_altitude_->publish(make_baro(1.0, 101.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 2U;}, std::chrono::seconds(3)));

  const auto & last = *estimates_.back();
  EXPECT_NEAR(last.pose.pose.position.z, -1.0, 0.01);
  EXPECT_NEAR(last.twist.twist.linear.z, -1.0, 0.01);
}

// FT-006 regression: VehicleState.alt_msl is EKF2 output, which fuses this
// node's own HIL_GPS altitude -- it must not move the vertical estimate, or
// the two close a loop (PX4's altitude ran ~42 km off on the bench).
TEST_F(GnssSimNodeTest, VehicleStateAltMslDoesNotDriveVerticalEstimate)
{
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_baro_altitude_->publish(make_baro(0.0, 100.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_vehicle_state_->publish(make_vehicle_state(1.0, -41975.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 2U;}, std::chrono::seconds(3)));

  const auto & last = *estimates_.back();
  EXPECT_NEAR(last.pose.pose.position.z, 0.0, 1e-6);
  EXPECT_NEAR(last.twist.twist.linear.z, 0.0, 1e-6);
}

// A flow sample gap wider than max_flow_dt_s_ (default 0.5 s) must not be
// integrated across -- the position must stay put rather than absorb a
// spurious jump.
TEST_F(GnssSimNodeTest, StaleFlowGapIsNotIntegrated)
{
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 100.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_optical_flow_->publish(make_flow(0.1, 2.0F, 0.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 2U;}, std::chrono::seconds(3)));

  // 2 s later -- far beyond max_flow_dt_s_ -- must be dropped, not
  // integrated as a 2 s * 2 m/s displacement.
  pub_optical_flow_->publish(make_flow(2.1, 2.0F, 0.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 3U;}, std::chrono::seconds(3)));

  const auto & last = *estimates_.back();
  EXPECT_NEAR(last.pose.pose.position.x, 0.0, 0.05);
}

// IGS-003 -----------------------------------------------------------------

// Before any anchor command, the node must not report a trustworthy fix or
// active status -- Safety Considerations #5: bootstrapping the false origin
// is an operator action, never automatic/default.
TEST_F(GnssSimNodeTest, InactiveBeforeAnyAnchorCommand)
{
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 50.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !statuses_.empty();}, std::chrono::seconds(3)));

  EXPECT_FALSE(statuses_.back()->active);
  EXPECT_FALSE(node_->wgs84_fix().valid);
}

// Known false origin + known north/altitude trajectory -> expected WGS-84
// lat/alt within tolerance. Reuses IGS-001's own StraightLineLevelAttitude
// trajectory numbers (9 integrated 0.1 s intervals at 2 m/s = 1.8 m north)
// so the expected displacement is already independently verified by that
// test; this test only adds the local -> WGS-84 conversion on top.
TEST_F(GnssSimNodeTest, AnchorThenKnownNorthTrajectoryConvertsToExpectedWgs84)
{
  constexpr double kEarthRadiusM = 6371000.0;  // must match gnss_sim_node.cpp's own constant
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};

  pub_vehicle_state_->publish(make_vehicle_state(0.0, 50.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_gcs_command_->publish(make_anchor_command(0.05, 1.300000F, 103.800000F, 50.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !statuses_.empty() && statuses_.back()->active;},
      std::chrono::seconds(3)));

  // GcsCommand.params is float32[7] (ICD-wide, not this ticket's choice), so
  // the anchor's own lat/lon round-trip through float32 before reanchor()
  // ever sees them -- fix_at_anchor is therefore only accurate to ~1e-5 deg
  // (observed ~3e-6 deg on lon at this magnitude), not double precision.
  // Documented in gnss_sim_node.hpp; checked loosely here as a sanity bound,
  // not as the thing this test is actually verifying.
  const auto fix_at_anchor = node_->wgs84_fix();
  ASSERT_TRUE(fix_at_anchor.valid);
  EXPECT_NEAR(fix_at_anchor.lat_deg, 1.300000, 1e-4);
  EXPECT_NEAR(fix_at_anchor.lon_deg, 103.800000, 1e-4);
  EXPECT_NEAR(fix_at_anchor.alt_msl, 50.0, 1e-3);

  for (int i = 1; i <= 10; ++i) {
    pub_optical_flow_->publish(make_flow(i * 0.1, 2.0F, 0.0F));
  }
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 11U;}, std::chrono::seconds(3)));

  // The actual thing under test: the local -> WGS-84 *delta* since anchor.
  // Diffing against fix_at_anchor (not the double literal above) cancels
  // out the float32 anchor-rounding exactly, isolating the conversion math.
  const double expected_dlat_deg = (1.8 / kEarthRadiusM) * (180.0 / M_PI);
  const auto fix = node_->wgs84_fix();
  ASSERT_TRUE(fix.valid);
  EXPECT_NEAR(fix.lat_deg - fix_at_anchor.lat_deg, expected_dlat_deg, 1e-7);
  EXPECT_NEAR(fix.lon_deg, fix_at_anchor.lon_deg, 1e-9);  // no east motion
  EXPECT_NEAR(fix.alt_msl, fix_at_anchor.alt_msl, 1e-3);  // no altitude change since anchor
}

// Same trajectory shape, yawed 90 deg east (mirrors IGS-001's own
// YawedAttitudeRotatesFlowIntoLocalFrame) -- proves the longitude formula's
// cos(origin latitude) term is applied, not just the latitude formula.
TEST_F(GnssSimNodeTest, AnchorThenKnownEastTrajectoryConvertsToExpectedLongitude)
{
  constexpr double kEarthRadiusM = 6371000.0;  // must match gnss_sim_node.cpp's own constant
  const std::array<float, 4> east_facing_q = yaw_quaternion(M_PI / 2.0);

  pub_vehicle_state_->publish(make_vehicle_state(0.0, 50.0F, east_facing_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_gcs_command_->publish(make_anchor_command(0.05, 1.300000F, 103.800000F, 50.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !statuses_.empty() && statuses_.back()->active;},
      std::chrono::seconds(3)));

  // See AnchorThenKnownNorthTrajectoryConvertsToExpectedWgs84's own comment:
  // capture the at-anchor fix so the delta check below cancels out the
  // anchor command's float32-params rounding instead of being swamped by it.
  const auto fix_at_anchor = node_->wgs84_fix();
  ASSERT_TRUE(fix_at_anchor.valid);

  for (int i = 1; i <= 10; ++i) {
    pub_optical_flow_->publish(make_flow(i * 0.1, 2.0F, 0.0F));
  }
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 11U;}, std::chrono::seconds(3)));

  const double origin_lat_rad = fix_at_anchor.lat_deg * M_PI / 180.0;
  const double expected_dlon_deg =
    (1.8 / (kEarthRadiusM * std::cos(origin_lat_rad))) * (180.0 / M_PI);
  const auto fix = node_->wgs84_fix();
  ASSERT_TRUE(fix.valid);
  EXPECT_NEAR(fix.lat_deg, fix_at_anchor.lat_deg, 1e-9);  // no north motion
  EXPECT_NEAR(fix.lon_deg - fix_at_anchor.lon_deg, expected_dlon_deg, 1e-7);
}

// eph/epv must grow monotonically with time-since-anchor, matching the
// documented model (gnss_sim_node.hpp's own eph_at_anchor_m_/
// epv_at_anchor_m_/eph_growth_rate_m_per_s_/epv_growth_rate_m_per_s_
// defaults: eph(t) = 1.0 + 0.02*t, epv(t) = 1.5 + 0.005*t). Deliberately
// checked against those literal defaults, not re-derived from the node --
// if a future change alters the model or its defaults without updating
// this test, that mismatch should fail loudly here, not slip through.
TEST_F(GnssSimNodeTest, EphEpvGrowMonotonicallyMatchingDocumentedModel)
{
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 50.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_gcs_command_->publish(make_anchor_command(0.0, 1.0F, 2.0F, 10.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !statuses_.empty() && statuses_.back()->active;},
      std::chrono::seconds(3)));
  EXPECT_NEAR(statuses_.back()->time_since_anchor_s, 0.0, 1e-6);
  EXPECT_NEAR(statuses_.back()->eph, 1.0, 1e-6);
  EXPECT_NEAR(statuses_.back()->epv, 1.5, 1e-6);

  pub_vehicle_state_->publish(make_vehicle_state(2.0, 50.0F, level_q));
  // statuses_ so far: [bootstrap, anchor] -- this vehicle_state adds a 3rd.
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return statuses_.size() >= 3U;}, std::chrono::seconds(3)));
  EXPECT_NEAR(statuses_.back()->time_since_anchor_s, 2.0, 0.01);
  EXPECT_NEAR(statuses_.back()->eph, 1.0 + 0.02 * 2.0, 1e-3);
  EXPECT_NEAR(statuses_.back()->epv, 1.5 + 0.005 * 2.0, 1e-3);

  pub_vehicle_state_->publish(make_vehicle_state(5.0, 50.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return statuses_.size() >= 4U;}, std::chrono::seconds(3)));
  EXPECT_NEAR(statuses_.back()->time_since_anchor_s, 5.0, 0.01);
  EXPECT_NEAR(statuses_.back()->eph, 1.0 + 0.02 * 5.0, 1e-3);
  EXPECT_NEAR(statuses_.back()->epv, 1.5 + 0.005 * 5.0, 1e-3);

  // Monotonic non-decreasing across the whole recorded sequence, not just
  // the three checkpoints above.
  for (size_t i = 1; i < statuses_.size(); ++i) {
    EXPECT_GE(statuses_[i]->eph, statuses_[i - 1]->eph);
    EXPECT_GE(statuses_[i]->epv, statuses_[i - 1]->epv);
    EXPECT_GE(statuses_[i]->time_since_anchor_s, statuses_[i - 1]->time_since_anchor_s);
  }
}

// An explicit re-anchor must: (a) reset time_since_anchor_s/eph/epv to
// their at-anchor floor, not merely stop growing from wherever they were,
// and (b) re-zero the running local integrator so the reported fix reflects
// only the new origin, never the previous origin's accumulated drift on top
// of it. Both are logged, per SFR-006's command-logging precedent.
TEST_F(GnssSimNodeTest, ReanchorResetsTimeSinceAnchorUncertaintyAndLocalIntegrator)
{
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 10.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  // First anchor ("origin A").
  pub_gcs_command_->publish(make_anchor_command(0.05, 1.0F, 2.0F, 10.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !statuses_.empty() && statuses_.back()->active;},
      std::chrono::seconds(3)));

  // Accumulate some real drift against origin A, and let time-since-anchor
  // grow past the at-anchor floor -- proving the reset below actually reset
  // something, rather than the fields having simply never moved.
  for (int i = 1; i <= 5; ++i) {
    pub_optical_flow_->publish(make_flow(i * 0.1, 2.0F, 0.0F));
  }
  // statuses_ so far: [bootstrap, anchor] -- 5 flow samples add up to 5 more.
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return statuses_.size() >= 7U;}, std::chrono::seconds(3)));
  ASSERT_GT(statuses_.back()->time_since_anchor_s, 0.0F);
  ASSERT_GT(statuses_.back()->eph, 1.0F);
  ASSERT_NE(node_->wgs84_fix().lat_deg, 1.0);  // drifted away from origin A

  // Re-anchor ("origin B") -- a later, unrelated timestamp and a different
  // false origin entirely.
  pub_gcs_command_->publish(make_anchor_command(5.0, 3.0F, 4.0F, 20.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {
        return statuses_.back()->time_since_anchor_s < 0.01F && statuses_.back()->active;
      }, std::chrono::seconds(3)));

  EXPECT_NEAR(statuses_.back()->eph, 1.0, 1e-6);
  EXPECT_NEAR(statuses_.back()->epv, 1.5, 1e-6);

  const auto fix = node_->wgs84_fix();
  ASSERT_TRUE(fix.valid);
  EXPECT_NEAR(fix.lat_deg, 3.0, 1e-9);
  EXPECT_NEAR(fix.lon_deg, 4.0, 1e-9);
  EXPECT_NEAR(fix.alt_msl, 20.0, 1e-3);
}

// Out-of-range coordinates must be rejected (logged, no state change), not
// silently clamped or accepted -- an operator typo should not silently
// anchor a session at a nonsensical false origin.
TEST_F(GnssSimNodeTest, RejectsOutOfRangeAnchorCoordinatesWithoutChangingState)
{
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 10.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_gcs_command_->publish(make_anchor_command(0.05, 1.0F, 2.0F, 10.0F));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !statuses_.empty() && statuses_.back()->active;},
      std::chrono::seconds(3)));

  pub_gcs_command_->publish(make_anchor_command(1.0, 999.0F, 2.0F, 10.0F));
  // No new /mc/indoor/gnss_sim_status is published for a rejected command
  // (reanchor() returns before publish_gnss_sim_status()), so there is no
  // message to spin_until on; briefly spin to let the rejection path run,
  // then assert the previous anchor is untouched.
  ASSERT_FALSE(
    spin_until(
      {node_, harness_}, [this] {return statuses_.back()->time_since_anchor_s > 0.5F;},
      std::chrono::milliseconds(200)));

  const auto fix = node_->wgs84_fix();
  ASSERT_TRUE(fix.valid);
  EXPECT_NEAR(fix.lat_deg, 1.0, 1e-9);
  EXPECT_NEAR(fix.lon_deg, 2.0, 1e-9);
}
