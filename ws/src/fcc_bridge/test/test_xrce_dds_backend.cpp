// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

// FCC-002: XrceDdsBackend state aggregation, TP-NI style (SDD §3.2) — a
// mock uORB publisher stands in for the Micro XRCE-DDS Agent. No live FCC
// or mission-protocol traffic is exercised here (see
// test_mission_protocol_client.cpp for that); this only checks the DDS
// subscribe -> VehicleState aggregation path and vehicle_command/visual
// odometry publishing.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <chrono>
#include <cmath>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
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
    setenv("FCC_FLOW_PORT", "14655", 1);

    node_ = std::make_shared<rclcpp::Node>("xrce_dds_backend_test");
    backend_ = std::make_unique<fcc_bridge::XrceDdsBackend>(node_.get());

    pub_local_pos_ = node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos());
    pub_global_pos_ = node_->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", px4_qos());
    pub_status_ = node_->create_publisher<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4_qos());
    pub_battery_ = node_->create_publisher<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status_v1", px4_qos());
    pub_gps_ = node_->create_publisher<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", px4_qos());

    sub_vehicle_command_ = node_->create_subscription<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos(),
      [this](px4_msgs::msg::VehicleCommand::SharedPtr msg) {last_command_ = msg;});
    sub_offboard_control_mode_ = node_->create_subscription<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos(),
      [this](px4_msgs::msg::OffboardControlMode::SharedPtr msg) {
        last_offboard_control_mode_ = msg;
      });
    sub_trajectory_setpoint_ = node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos(),
      [this](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {last_trajectory_setpoint_ = msg;});
  }

  void TearDown() override
  {
    backend_.reset();
    unsetenv("FCC_ETH_IP");
    unsetenv("FCC_MISSION_PORT");
    unsetenv("FCC_FLOW_PORT");
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<fcc_bridge::XrceDdsBackend> backend_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pub_local_pos_;
  rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr pub_global_pos_;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<px4_msgs::msg::BatteryStatus>::SharedPtr pub_battery_;
  rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr pub_gps_;
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

  ASSERT_TRUE(
    spin_until(
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

  ASSERT_TRUE(
    spin_until(
      node_, [&] {pub_global_pos_->publish(msg); return backend_->get_vehicle_state().lat != 0.0;},
      std::chrono::seconds(3)));

  const auto state = backend_->get_vehicle_state();
  EXPECT_DOUBLE_EQ(state.lat, 1.3521);
  EXPECT_DOUBLE_EQ(state.lon, 103.8198);
  EXPECT_FLOAT_EQ(state.alt_msl, 55.0F);
}

// device_id packing per PX4 v1.17.0's device::Device::DeviceId: bus_type
// bits 0-2, bus 3-7, address (MAVLink sysid for HIL_GPS) 8-15, devtype 16-23.
constexpr uint32_t kRealUbloxDeviceId = (0xA5U << 16) | (1U << 3) | 5U;  // UBX, SERIAL bus 1
constexpr uint32_t kHilGpsDeviceId = (0xAFU << 16) | (4U << 8) | (1U << 3) | 6U;  // sysid 4

TEST_F(XrceDdsBackendTest, AggregatesRealGpsFixIntoVehicleState)
{
  px4_msgs::msg::SensorGps msg{};
  msg.device_id = kRealUbloxDeviceId;
  msg.fix_type = 3;
  msg.hdop = 0.9F;
  msg.vdop = 1.4F;

  ASSERT_TRUE(
    spin_until(
      node_, [&] {pub_gps_->publish(msg); return backend_->get_vehicle_state().fix_type == 3;},
      std::chrono::seconds(3)));

  const auto state = backend_->get_vehicle_state();
  EXPECT_FLOAT_EQ(state.hdop, 0.9F);
  EXPECT_FLOAT_EQ(state.vdop, 1.4F);
  EXPECT_TRUE(state.gnss_ok);
}

TEST_F(XrceDdsBackendTest, RealGpsFixAboveHdopThresholdIsNotGnssOk)
{
  px4_msgs::msg::SensorGps msg{};
  msg.device_id = kRealUbloxDeviceId;
  msg.fix_type = 3;
  msg.hdop = 2.5F;

  ASSERT_TRUE(
    spin_until(
      node_, [&] {pub_gps_->publish(msg); return backend_->get_vehicle_state().fix_type == 3;},
      std::chrono::seconds(3)));

  EXPECT_FALSE(backend_->get_vehicle_state().gnss_ok);
}

// FT-006 finding: PX4 fills hdop=0 on a HIL_GPS-sourced fix, so counting it
// would report gnss_ok=true and trip gnss_sim_node's real-GPS lockout on
// its own injected stream.
TEST_F(XrceDdsBackendTest, HilGpsSourcedFixIsReportedAsNoRealFix)
{
  px4_msgs::msg::SensorGps real{};
  real.device_id = kRealUbloxDeviceId;
  real.fix_type = 3;
  real.hdop = 0.9F;
  ASSERT_TRUE(
    spin_until(
      node_, [&] {pub_gps_->publish(real); return backend_->get_vehicle_state().gnss_ok;},
      std::chrono::seconds(3)));

  px4_msgs::msg::SensorGps hil{};
  hil.device_id = kHilGpsDeviceId;
  hil.fix_type = 3;
  hil.hdop = 0.0F;
  ASSERT_TRUE(
    spin_until(
      node_, [&] {pub_gps_->publish(hil); return !backend_->get_vehicle_state().gnss_ok;},
      std::chrono::seconds(3)));

  const auto state = backend_->get_vehicle_state();
  EXPECT_EQ(state.fix_type, 0);
  EXPECT_FLOAT_EQ(state.hdop, 0.0F);
}

TEST_F(XrceDdsBackendTest, AggregatesArmingAndNavStateFromVehicleStatus)
{
  px4_msgs::msg::VehicleStatus msg{};
  msg.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  msg.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION;

  ASSERT_TRUE(
    spin_until(
      node_,
      [&] {
        pub_status_->publish(msg);
        return backend_->get_vehicle_state().arming_state ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
      },
      std::chrono::seconds(3)));

  EXPECT_EQ(
    backend_->get_vehicle_state().nav_state,
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION);
}

TEST_F(XrceDdsBackendTest, AggregatesFailureDetectorStatusFromVehicleStatus)
{
  // DEP-009: failure_detector_status is a real px4_msgs release/1.17 field
  // (ICD §3.2) — no longer approximated from `failsafe`.
  px4_msgs::msg::VehicleStatus msg{};
  msg.failure_detector_status = px4_msgs::msg::VehicleStatus::FAILURE_BATTERY |
    px4_msgs::msg::VehicleStatus::FAILURE_MOTOR;

  ASSERT_TRUE(
    spin_until(
      node_,
      [&] {
        pub_status_->publish(msg);
        return backend_->get_vehicle_state().failure_flags != 0U;
      },
      std::chrono::seconds(3)));

  EXPECT_EQ(
    backend_->get_vehicle_state().failure_flags,
    px4_msgs::msg::VehicleStatus::FAILURE_BATTERY |
    px4_msgs::msg::VehicleStatus::FAILURE_MOTOR);
}

TEST_F(XrceDdsBackendTest, AggregatesBatteryRemaining)
{
  px4_msgs::msg::BatteryStatus msg{};
  msg.remaining = 0.42F;

  ASSERT_TRUE(
    spin_until(
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
  ASSERT_TRUE(
    spin_until(
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

  ASSERT_TRUE(
    spin_until(
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

  ASSERT_TRUE(
    spin_until(
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

// IGS-002: HIF-008 flow relay — a mock H-Flow-relay peer sends a raw MAVLink
// datagram straight to FCC_FLOW_PORT (14655 here), independent of the DDS
// subscriptions above. OpticalFlowRelay's own wire-decode/velocity math is
// covered by test_optical_flow_relay.cpp; this only checks the socket/thread
// plumbing that gets a real datagram into get_optical_flow().
TEST_F(XrceDdsBackendTest, GetOpticalFlowReflectsFrameSentToDedicatedFlowPort)
{
  EXPECT_FALSE(backend_->get_optical_flow().valid) << "nothing sent yet";

  const int peer_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(14655);
  ::inet_pton(AF_INET, "127.0.0.1", &dest.sin_addr);

  mavlink_message_t msg;
  mavlink_msg_optical_flow_rad_pack(
    1, 1, &msg, 0, 0, /*integration_time_us=*/ 100000, /*integrated_x=*/ 0.0F,
    /*integrated_y=*/ 0.0F, 0.0F, 0.0F, 0.0F, 0, /*quality=*/ 150, 0, /*distance=*/ 2.5F);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  ASSERT_TRUE(
    spin_until(
      node_,
      [&] {
        ::sendto(peer_fd, buf, len, 0, reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
        return backend_->get_optical_flow().valid;
      },
      std::chrono::seconds(3)));

  ::close(peer_fd);

  const auto sample = backend_->get_optical_flow();
  EXPECT_FLOAT_EQ(sample.ground_distance, 2.5F);
  EXPECT_EQ(sample.quality, 150);
}
