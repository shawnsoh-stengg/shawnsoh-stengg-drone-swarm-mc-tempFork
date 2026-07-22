// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

// FCC-002: XrceDdsBackend state aggregation, TP-NI style (SDD §3.2) — a
// mock uORB publisher stands in for the Micro XRCE-DDS Agent. No live FCC
// or mission-protocol traffic is exercised here (see
// test_mission_protocol_client.cpp for that); this only checks the DDS
// subscribe -> VehicleState aggregation path and vehicle_command/visual
// odometry publishing.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fcc_bridge/xrce_dds_backend.hpp"

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

rclcpp::QoS px4_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

// Spins `node` until `predicate()` is true or `timeout` elapses. Returns
// whether the predicate was satisfied — used instead of a fixed sleep since
// DDS discovery/delivery is asynchronous.
template<typename Predicate>
bool spin_until(
  rclcpp::Node::SharedPtr node, Predicate predicate,
  std::chrono::milliseconds timeout)
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
}  // namespace

class XrceDdsBackendTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Isolated from the real FCC subnet — mission-protocol socket just
    // needs to bind locally; nothing in this test suite sends to it.
    setenv("FCC_ETH_IP", "127.0.0.1", 1);
    setenv("FCC_MISSION_PORT", "14555", 1);

    node_ = std::make_shared<rclcpp::Node>("xrce_dds_backend_test");
    backend_ = std::make_unique<fcc_bridge::XrceDdsBackend>(node_.get());

    pub_local_pos_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos());
    pub_global_pos_ = node_->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", px4_qos());
    pub_status_ = node_->create_publisher<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos());
    pub_battery_ = node_->create_publisher<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", px4_qos());

    sub_vehicle_command_ = node_->create_subscription<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos(),
      [this](px4_msgs::msg::VehicleCommand::SharedPtr msg) {last_command_ = msg;});
    sub_offboard_control_mode_ = node_->create_subscription<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos(),
      [this](px4_msgs::msg::OffboardControlMode::SharedPtr msg) {last_offboard_control_mode_ = msg;});
    sub_trajectory_setpoint_ = node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos(),
      [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {last_trajectory_setpoint_ = msg;});
  }

  void TearDown() override
  {
    backend_.reset();
    unsetenv("FCC_ETH_IP");
    unsetenv("FCC_MISSION_PORT");
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<fcc_bridge::XrceDdsBackend> backend_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pub_local_pos_;
  rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr pub_global_pos_;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<px4_msgs::msg::BatteryStatus>::SharedPtr pub_battery_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr sub_vehicle_command_;
  px4_msgs::msg::VehicleCommand::SharedPtr last_command_;
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr sub_offboard_control_mode_;
  px4_msgs::msg::OffboardControlMode::SharedPtr last_offboard_control_mode_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_trajectory_setpoint_;
  px4_msgs::msg::TrajectorySetpoint::SharedPtr last_trajectory_setpoint_;
};

TEST_F(XrceDdsBackendTest, AggregatesLocalPositionIntoVehicleState)
{
  px4_msgs::msg::VehicleLocalPosition msg{};
  msg.x = 1.5F;
  msg.y = -2.5F;
  msg.z = -10.0F;
  msg.vx = 3.0F;
  msg.xy_valid = true;
  msg.z_valid = true;

  ASSERT_TRUE(spin_until(
    node_, [&] {pub_local_pos_->publish(msg); return backend_->get_vehicle_state().xy_valid;},
    std::chrono::seconds(3)));

  const auto state = backend_->get_vehicle_state();
  EXPECT_FLOAT_EQ(state.x, 1.5F);
  EXPECT_FLOAT_EQ(state.y, -2.5F);
  EXPECT_FLOAT_EQ(state.z, -10.0F);
  EXPECT_FLOAT_EQ(state.vx, 3.0F);
  EXPECT_TRUE(state.z_valid);
}

TEST_F(XrceDdsBackendTest, AggregatesGlobalPositionIntoVehicleState)
{
  px4_msgs::msg::VehicleGlobalPosition msg{};
  msg.lat = 1.3521;
  msg.lon = 103.8198;
  msg.alt = 55.0F;

  ASSERT_TRUE(spin_until(
    node_, [&] {pub_global_pos_->publish(msg); return backend_->get_vehicle_state().lat != 0.0;},
    std::chrono::seconds(3)));

  const auto state = backend_->get_vehicle_state();
  EXPECT_DOUBLE_EQ(state.lat, 1.3521);
  EXPECT_DOUBLE_EQ(state.lon, 103.8198);
  EXPECT_FLOAT_EQ(state.alt_msl, 55.0F);
}

TEST_F(XrceDdsBackendTest, AggregatesArmingAndNavStateFromVehicleStatus)
{
  px4_msgs::msg::VehicleStatus msg{};
  msg.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  msg.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION;

  ASSERT_TRUE(spin_until(
    node_,
      [&] {
        pub_status_->publish(msg);
        return backend_->get_vehicle_state().arming_state ==
               px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    },
    std::chrono::seconds(3)));

  EXPECT_EQ(backend_->get_vehicle_state().nav_state,
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION);
}

TEST_F(XrceDdsBackendTest, AggregatesBatteryRemaining)
{
  px4_msgs::msg::BatteryStatus msg{};
  msg.remaining = 0.42F;

  ASSERT_TRUE(spin_until(
      node_, [&] {
        pub_battery_->publish(msg);
        return backend_->get_vehicle_state().battery_remaining > 0.0F;
      },
      std::chrono::seconds(3)));

  EXPECT_FLOAT_EQ(backend_->get_vehicle_state().battery_remaining, 0.42F);
}

TEST_F(XrceDdsBackendTest, IsLinkAliveFalseBeforeAnyMessageWithZeroTimeout)
{
  // No uORB traffic published yet in this test — is_link_alive(0) should be
  // false immediately after construction (last_fcc_msg_time_ == now(), and
  // any elapsed time exceeds a 0s timeout).
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_FALSE(backend_->is_link_alive(0.0));
}

TEST_F(XrceDdsBackendTest, IsLinkAliveTrueAfterRecentMessage)
{
  px4_msgs::msg::VehicleLocalPosition msg{};
  ASSERT_TRUE(spin_until(
    node_, [&] {pub_local_pos_->publish(msg); return backend_->is_link_alive(5.0);},
    std::chrono::seconds(3)));
}

// FCC-006: XrceDdsBackend must forward MissionProtocolClient's bool result
// rather than swallow it — no live FCC peer replies here, so the underlying
// client's own timeout path (test_mission_protocol_client.cpp covers its
// wire-level correctness) is what should surface as `false` here.
TEST_F(XrceDdsBackendTest, UpdateMissionItemReturnsFalseWhenFccDoesNotAck)
{
  EXPECT_FALSE(
    backend_->update_mission_item(fcc_bridge::FORMATION_SLOT, fcc_bridge::MissionItem{}));
}

TEST_F(XrceDdsBackendTest, SendCommandPublishesVehicleCommandWithAllParams)
{
  std::array<float, 7> params{1, 2, 3, 4, 5, 6, 7};

  ASSERT_TRUE(spin_until(
    node_, [&] {backend_->send_command(400, params); return last_command_ != nullptr;},
    std::chrono::seconds(3)));

  EXPECT_EQ(last_command_->command, 400U);
  EXPECT_FLOAT_EQ(last_command_->param1, 1.0F);
  EXPECT_FLOAT_EQ(last_command_->param7, 7.0F);
  EXPECT_TRUE(last_command_->from_external);
}

// FCC-009: stream_offboard_setpoint() publishes OffboardControlMode
// (position=true, all else false) and TrajectorySetpoint (position+yaw
// set, velocity/acceleration/jerk/yawspeed NaN'd out per that message's own
// "NaN means not controlled" convention).
TEST_F(XrceDdsBackendTest, StreamOffboardSetpointPublishesControlModeAndTrajectorySetpoint)
{
  fcc_bridge::OffboardSetpoint sp;
  sp.x = 1.5F;
  sp.y = -2.5F;
  sp.z = -10.0F;
  sp.yaw = 0.75F;

  ASSERT_TRUE(spin_until(
    node_,
      [&] {
        backend_->stream_offboard_setpoint(sp);
        return last_offboard_control_mode_ != nullptr && last_trajectory_setpoint_ != nullptr;
    },
    std::chrono::seconds(3)));

  EXPECT_TRUE(last_offboard_control_mode_->position);
  EXPECT_FALSE(last_offboard_control_mode_->velocity);
  EXPECT_FALSE(last_offboard_control_mode_->acceleration);
  EXPECT_FALSE(last_offboard_control_mode_->attitude);
  EXPECT_FALSE(last_offboard_control_mode_->body_rate);

  EXPECT_FLOAT_EQ(last_trajectory_setpoint_->position[0], 1.5F);
  EXPECT_FLOAT_EQ(last_trajectory_setpoint_->position[1], -2.5F);
  EXPECT_FLOAT_EQ(last_trajectory_setpoint_->position[2], -10.0F);
  EXPECT_FLOAT_EQ(last_trajectory_setpoint_->yaw, 0.75F);
  EXPECT_TRUE(std::isnan(last_trajectory_setpoint_->velocity[0]));
  EXPECT_TRUE(std::isnan(last_trajectory_setpoint_->yawspeed));
}
