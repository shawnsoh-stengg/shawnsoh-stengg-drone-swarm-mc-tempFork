// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md, IGS-001).

#pragma once

#include <array>
#include <optional>

#include <mc_msgs/msg/optical_flow.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace indoor_nav
{

// IGS-001: dead-reckoning fusion core. Integrates optical-flow-derived
// body-frame horizontal velocity (rotated into a local nav frame using FCC
// attitude), corrects the vertical axis directly from FCC baro-derived
// altitude, and publishes the result on /mc/indoor/local_estimate for bench
// validation. No camera dependency; no WGS-84 conversion (IGS-003) or
// GPS_INPUT injection (IGS-004) here.
//
// Open Item 3 resolution (attitude/baro source, INDOOR-GNSS-SIMULATION-
// BACKLOG.md): /mc/fcc/vehicle_state already carries both signals this
// fusion core needs and nothing else does --
//   - attitude: VehicleState.q (PX4's attitude estimator output: gyro +
//     accel + mag, no GPS dependency, so it stays valid indoors) -- used
//     here to rotate flow-derived body-frame velocity into the local frame.
//   - vertical: VehicleState.alt_msl. Not a raw-baro topic (it is PX4's
//     global-position estimate, which *can* blend GPS altitude in when a
//     fix exists), but indoors there is no GPS altitude to blend -- baro is
//     the dominant, effectively only, vertical aiding source EKF2 has
//     available, so alt_msl is a faithful proxy in exactly the regime this
//     module targets. A new fcc_bridge raw-baro relay (vehicle_air_data)
//     would be more precise but is not needed for this ticket's fusion
//     approach, which never differentiates/integrates IMU accelerometer
//     data -- flagged here rather than silently assumed, per this ticket's
//     own AC.
// This node therefore does not add any new fcc_bridge topic. It does still
// need *a* horizontal-velocity input: HIF-008's optical-flow sensor
// (IGS-002) doesn't exist yet, so it subscribes to a new, explicitly
// provisional mc_msgs/OpticalFlow message on /mc/sensors/optical_flow (see
// that message's own header comment) to develop and test the fusion core
// against ahead of IGS-002 landing, per this ticket's relationships note.
class GnssSimNode : public rclcpp::Node
{
public:
  explicit GnssSimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Test seams: drive the fusion core directly with a known message,
  // bypassing pub/sub timing so a test can assert on a specific sequence
  // deterministically. Production code only reaches these via the
  // subscription callbacks wired in the constructor.
  void on_vehicle_state(const mc_msgs::msg::VehicleState & msg);
  void on_optical_flow(const mc_msgs::msg::OpticalFlow & msg);

  // Test seam: the last message this node published on
  // /mc/indoor/local_estimate, without needing a subscriber of its own.
  nav_msgs::msg::Odometry local_estimate() const {return latest_estimate_;}

private:
  void publish_estimate(const rclcpp::Time & stamp);

  rclcpp::Subscription<mc_msgs::msg::VehicleState>::SharedPtr sub_vehicle_state_;
  rclcpp::Subscription<mc_msgs::msg::OpticalFlow>::SharedPtr sub_optical_flow_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_local_estimate_;

  // A flow sample that arrives more than this many seconds after the
  // previous one is dropped from integration (still updates
  // last_flow_stamp_) rather than integrated over a stale dt -- a gap this
  // long means the sensor/link glitched, and integrating blindly across it
  // would inject a large, silent position error. Parameterized so a test
  // can shrink it; IGS-005 owns the real staleness *fault* on top of this.
  double max_flow_dt_s_{0.5};

  // Latest FCC attitude ("NED-to-body" quaternion, VehicleState.q's own
  // documented convention). Required before the first flow sample can be
  // integrated -- there is no safe default attitude to assume.
  std::array<float, 4> attitude_q_{1.0F, 0.0F, 0.0F, 0.0F};
  bool has_attitude_{false};

  // Vertical channel: alt_msl is applied directly (never integrated), so
  // z_ always reflects the latest baro sample, anchored at the first one
  // received so the estimate starts at z=0 (IGS-003 anchors this to a real
  // WGS-84 false origin later).
  std::optional<float> alt_msl_anchor_;
  std::optional<rclcpp::Time> last_vehicle_state_stamp_;
  float z_{0.0F};
  float vz_{0.0F};

  // Horizontal channel: integrated from consecutive optical-flow samples.
  std::optional<rclcpp::Time> last_flow_stamp_;
  float x_{0.0F};
  float y_{0.0F};
  float vx_{0.0F};
  float vy_{0.0F};

  nav_msgs::msg::Odometry latest_estimate_;
};

}  // namespace indoor_nav
