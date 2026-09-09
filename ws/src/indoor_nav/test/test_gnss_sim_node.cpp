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
    sub_local_estimate_ = harness_->create_subscription<nav_msgs::msg::Odometry>(
      "/mc/indoor/local_estimate", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {estimates_.push_back(msg);});
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

  std::shared_ptr<indoor_nav::GnssSimNode> node_;
  rclcpp::Node::SharedPtr harness_;
  rclcpp::Publisher<mc_msgs::msg::VehicleState>::SharedPtr pub_vehicle_state_;
  rclcpp::Publisher<mc_msgs::msg::OpticalFlow>::SharedPtr pub_optical_flow_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_local_estimate_;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> estimates_;
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
  const std::array<float, 4> level_q{1.0F, 0.0F, 0.0F, 0.0F};
  pub_vehicle_state_->publish(make_vehicle_state(0.0, 100.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !estimates_.empty();}, std::chrono::seconds(3)));

  pub_vehicle_state_->publish(make_vehicle_state(1.0, 101.0F, level_q));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return estimates_.size() >= 2U;}, std::chrono::seconds(3)));

  const auto & last = *estimates_.back();
  EXPECT_NEAR(last.pose.pose.position.z, -1.0, 0.01);
  EXPECT_NEAR(last.twist.twist.linear.z, -1.0, 0.01);
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
