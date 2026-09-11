// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

// IGS-002: OpticalFlowRelay's OPTICAL_FLOW_RAD/DISTANCE_SENSOR decode and
// velocity-scaling math, exercised directly against packed mavlink_message_t
// frames — no socket, no PX4 hardware. The velocity formula itself is
// replicated from PX4's real VehicleOpticalFlow.cpp (confirmed against
// v1.14.4 originally, re-confirmed against v1.17.0 — see
// optical_flow_relay.cpp's handle_optical_flow_rad() doc comment); these
// test values are hand-computed against that same formula, not against a
// real H-Flow unit (no physical sensor exists in this environment — see
// test-results/IGS-002-hif-008-bench-validation.md for that separate,
// still-open bench-validation gap).

#include <gtest/gtest.h>

#include <mavlink/v2.0/common/mavlink.h>

#include "fcc_bridge/optical_flow_relay.hpp"

namespace
{
mavlink_message_t pack_optical_flow_rad(
  uint32_t integration_time_us, float integrated_x, float integrated_y,
  float integrated_xgyro, float integrated_ygyro, uint8_t quality, float distance)
{
  mavlink_message_t msg;
  mavlink_msg_optical_flow_rad_pack(
    1, 1, &msg, /*time_usec=*/ 0, /*sensor_id=*/ 0, integration_time_us, integrated_x,
    integrated_y, integrated_xgyro, integrated_ygyro, /*integrated_zgyro=*/ 0.0F,
    /*temperature=*/ 0, quality, /*time_delta_distance_us=*/ 0, distance);
  return msg;
}

mavlink_message_t pack_distance_sensor(uint16_t current_distance_cm)
{
  mavlink_message_t msg;
  mavlink_msg_distance_sensor_pack(
    1, 1, &msg, /*time_boot_ms=*/ 0, /*min_distance=*/ 0, /*max_distance=*/ 3000,
    current_distance_cm, MAV_DISTANCE_SENSOR_LASER, /*id=*/ 0,
    MAV_SENSOR_ROTATION_PITCH_270, /*covariance=*/ 0, /*horizontal_fov=*/ 0.0F,
    /*vertical_fov=*/ 0.0F, /*quaternion=*/ nullptr, /*signal_quality=*/ 0);
  return msg;
}
}  // namespace

TEST(OpticalFlowRelay, InvalidBeforeAnyFrame)
{
  fcc_bridge::OpticalFlowRelay relay;
  EXPECT_FALSE(relay.get_sample().valid);
}

// Hand-computed against the exact PX4 formula (unchanged v1.14.4/v1.17.0,
// see optical_flow_relay.cpp's doc comment): flow_dt=0.1s, range=2.0m, no
// gyro rotation (delta angle 0).
// flow_x_rad=-integrated_x=-0.05, flow_y_rad=-integrated_y=0.02 (no gyro to
// subtract) -> vx = -range*flow_comp_y/dt = -2*0.02/0.1 = -0.4,
// vy = range*flow_comp_x/dt = 2*(-0.05)/0.1 = -1.0.
TEST(OpticalFlowRelay, ComputesBodyVelocityFromSelfPairedDistance)
{
  fcc_bridge::OpticalFlowRelay relay;
  const auto msg = pack_optical_flow_rad(
    /*integration_time_us=*/ 100000, /*integrated_x=*/ 0.05F, /*integrated_y=*/ -0.02F,
    /*integrated_xgyro=*/ 0.0F, /*integrated_ygyro=*/ 0.0F, /*quality=*/ 200,
    /*distance=*/ 2.0F);

  relay.handle_frame(msg);

  const auto sample = relay.get_sample();
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.vx, -0.4F, 1e-4);
  EXPECT_NEAR(sample.vy, -1.0F, 1e-4);
  EXPECT_FLOAT_EQ(sample.ground_distance, 2.0F);
  EXPECT_EQ(sample.quality, 200);
}

// Gyro-compensation term: same flow as above, but with a nonzero gyro
// integral that must be subtracted before scaling by range/dt.
// flow_x_rad=-0.05, gyro_x=-integrated_xgyro=-0.01 -> flow_comp_x=-0.05-(-0.01)=-0.04
// flow_y_rad=0.02, gyro_y=-integrated_ygyro=0.0 -> flow_comp_y=0.02
// vx = -2*0.02/0.1 = -0.4 (unchanged, gyro only affected the X component)
// vy = 2*(-0.04)/0.1 = -0.8
TEST(OpticalFlowRelay, SubtractsGyroCompensationBeforeScaling)
{
  fcc_bridge::OpticalFlowRelay relay;
  const auto msg = pack_optical_flow_rad(
    /*integration_time_us=*/ 100000, /*integrated_x=*/ 0.05F, /*integrated_y=*/ -0.02F,
    /*integrated_xgyro=*/ 0.01F, /*integrated_ygyro=*/ 0.0F, /*quality=*/ 200,
    /*distance=*/ 2.0F);

  relay.handle_frame(msg);

  const auto sample = relay.get_sample();
  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.vx, -0.4F, 1e-4);
  EXPECT_NEAR(sample.vy, -0.8F, 1e-4);
}

TEST(OpticalFlowRelay, FallsBackToStandaloneDistanceSensorWhenFlowDistanceUnavailable)
{
  fcc_bridge::OpticalFlowRelay relay;

  // No height reference yet -- frame should be dropped, not produce a
  // divide-by-zero or a bogus zero-range velocity.
  const auto flow_no_distance = pack_optical_flow_rad(
    100000, 0.05F, -0.02F, 0.0F, 0.0F, 200, /*distance=*/ -1.0F);
  relay.handle_frame(flow_no_distance);
  EXPECT_FALSE(relay.get_sample().valid);

  relay.handle_frame(pack_distance_sensor(/*current_distance_cm=*/ 150));  // 1.5 m

  relay.handle_frame(flow_no_distance);
  const auto sample = relay.get_sample();
  ASSERT_TRUE(sample.valid);
  EXPECT_FLOAT_EQ(sample.ground_distance, 1.5F);
  // range=1.5: vx = -1.5*0.02/0.1 = -0.3, vy = 1.5*(-0.05)/0.1 = -0.75
  EXPECT_NEAR(sample.vx, -0.3F, 1e-4);
  EXPECT_NEAR(sample.vy, -0.75F, 1e-4);
}

TEST(OpticalFlowRelay, IgnoresZeroIntegrationTimeToAvoidDivideByZero)
{
  fcc_bridge::OpticalFlowRelay relay;
  const auto msg = pack_optical_flow_rad(
    /*integration_time_us=*/ 0, 0.05F, -0.02F, 0.0F, 0.0F, 200, 2.0F);

  relay.handle_frame(msg);

  EXPECT_FALSE(relay.get_sample().valid);
}

TEST(OpticalFlowRelay, IgnoresUnrelatedMessageTypes)
{
  fcc_bridge::OpticalFlowRelay relay;
  mavlink_message_t heartbeat;
  mavlink_heartbeat_t hb{};
  mavlink_msg_heartbeat_encode(1, 1, &heartbeat, &hb);

  relay.handle_frame(heartbeat);

  EXPECT_FALSE(relay.get_sample().valid);
}
