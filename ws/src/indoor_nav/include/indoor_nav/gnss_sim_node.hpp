// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md).

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/indoor_gnss_sim_status.hpp>
#include <mc_msgs/msg/optical_flow.hpp>
#include <mc_msgs/msg/status_text.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace indoor_nav
{

// IGS-003, ICD-MC-001.md §4.5/§10.5: new MAV_CMD user-defined ID, next free
// slot after the existing MC_CMD_* block (50001-50004, mission_manager_
// node.cpp) and the mission-item-upload block (50200-50204, gcs_mavlink_
// node.cpp). param1/2/3 = false-origin lat_deg/lon_deg/alt_msl_m. Owned here
// (not mission_manager_node) because gnss_sim_node is the only consumer --
// gcs_mavlink_node forwards any COMMAND_LONG regardless of ID (SDD §11
// "zero business logic" adapter), so no change is needed there.
constexpr uint32_t kMcCmdIndoorGnssSimAnchor = 50005;

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
//
// IGS-003 extends this same class (not a new node) with: an operator-
// commanded false-origin anchor (kMcCmdIndoorGnssSimAnchor), a local (x, y,
// z) -> WGS-84 conversion anchored at that origin, and an eph/epv model
// that grows monotonically with time-since-anchor. Local frame is NED (x
// north, y east, z down -- matching this file's own IGS-001 code and the
// project-wide convention VehicleState.msg etc. already use), not the ENU
// this ticket's own prose says -- INDOOR-GNSS-SIMULATION-BACKLOG.md's
// IGS-003 description says "Local-tangent-plane (ENU)" but IGS-001's actual
// x_/y_/z_ are NED (see z_'s own "NED down" comment below); flagged rather
// than silently reconciled, same as this project's other doc-vs-code
// corrections (e.g. HIF-008's).
class GnssSimNode : public rclcpp::Node
{
public:
  // A single WGS-84 fix derived from the current local estimate and false
  // origin. `valid` is false until at least one anchor command has been
  // accepted -- there is no safe default false origin (Safety
  // Considerations #5: bootstrapping the origin is an operator action, not
  // automatic). This is IGS-004's own consumption point (GPS_INPUT sender,
  // not built yet) as well as this ticket's test seam -- deliberately not a
  // ROS topic itself, see IndoorGnssSimStatus.msg's own header comment.
  struct WgsFix
  {
    bool valid{false};
    double lat_deg{0.0};
    double lon_deg{0.0};
    float alt_msl{0.0F};
  };

  explicit GnssSimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Test seams: drive the fusion core directly with a known message,
  // bypassing pub/sub timing so a test can assert on a specific sequence
  // deterministically. Production code only reaches these via the
  // subscription callbacks wired in the constructor.
  void on_vehicle_state(const mc_msgs::msg::VehicleState & msg);
  void on_optical_flow(const mc_msgs::msg::OpticalFlow & msg);
  void on_gcs_command(const mc_msgs::msg::GcsCommand & msg);

  // Test seam: the last message this node published on
  // /mc/indoor/local_estimate, without needing a subscriber of its own.
  nav_msgs::msg::Odometry local_estimate() const {return latest_estimate_;}

  // Test seam: the last message this node published on
  // /mc/indoor/gnss_sim_status.
  mc_msgs::msg::IndoorGnssSimStatus gnss_sim_status() const {return latest_gnss_sim_status_;}

  // Test seam / IGS-004 consumption point: the current WGS-84 fix derived
  // from the local estimate + false origin. See WgsFix's own doc comment.
  WgsFix wgs84_fix() const;

private:
  void publish_estimate(const rclcpp::Time & stamp);
  void publish_gnss_sim_status(const rclcpp::Time & stamp);

  // Re-anchors the false origin at (lat_deg, lon_deg, alt_msl) as of
  // `stamp`: resets time_since_anchor_s (and therefore eph/epv) to their
  // at-anchor floor, and re-zeroes the running local (x, y, z) integrator
  // so "local (0,0,0)" always means "the most recently declared false
  // origin", never a stale earlier one plus whatever drift accumulated on
  // top of it. Logged via /mc/status_text either way (SFR-006's command-
  // logging precedent) -- rejects (logged as an error, no state change) if
  // lat/lon are outside valid WGS-84 ranges.
  void reanchor(double lat_deg, double lon_deg, float alt_msl, const rclcpp::Time & stamp);

  rclcpp::Subscription<mc_msgs::msg::VehicleState>::SharedPtr sub_vehicle_state_;
  rclcpp::Subscription<mc_msgs::msg::OpticalFlow>::SharedPtr sub_optical_flow_;
  rclcpp::Subscription<mc_msgs::msg::GcsCommand>::SharedPtr sub_gcs_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_local_estimate_;
  rclcpp::Publisher<mc_msgs::msg::IndoorGnssSimStatus>::SharedPtr pub_gnss_sim_status_;
  rclcpp::Publisher<mc_msgs::msg::StatusText>::SharedPtr pub_status_text_;

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
  // received so the estimate starts at z=0 -- and re-anchored again on each
  // IGS-003 reanchor() call, see that method's own doc comment.
  std::optional<float> alt_msl_anchor_;
  std::optional<float> last_alt_msl_;  // rebaseline target for reanchor()
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

  // IGS-003: false-origin anchor state. false_origin_* are only meaningful
  // once active_ is true (WgsFix::valid mirrors active_).
  bool active_{false};
  double false_origin_lat_deg_{0.0};
  double false_origin_lon_deg_{0.0};
  float false_origin_alt_msl_{0.0F};
  std::optional<rclcpp::Time> anchor_stamp_;

  // Uncertainty model (documented in reanchor()/publish_gnss_sim_status()):
  // eph(t) = eph_at_anchor_m_ + eph_growth_rate_m_per_s_ * time_since_anchor_s
  // epv(t) = epv_at_anchor_m_ + epv_growth_rate_m_per_s_ * time_since_anchor_s
  // Defaults: horizontal grows faster than vertical because it is purely
  // integrated (optical-flow dead reckoning, unbounded drift), while the
  // vertical channel is a direct baro reading each cycle (bounded, slower-
  // drifting bias) -- not a claim about any specific sensor's datasheet,
  // just the right qualitative shape; tune against real bench data once
  // IGS-002's physical validation (test-results/IGS-002-hif-008-bench-
  // validation.md) lands.
  double eph_at_anchor_m_{1.0};
  double epv_at_anchor_m_{1.5};
  double eph_growth_rate_m_per_s_{0.02};
  double epv_growth_rate_m_per_s_{0.005};

  mc_msgs::msg::IndoorGnssSimStatus latest_gnss_sim_status_;
};

}  // namespace indoor_nav
