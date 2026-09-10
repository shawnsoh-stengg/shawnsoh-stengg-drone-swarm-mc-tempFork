// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md).

#include "indoor_nav/gnss_sim_node.hpp"

#include <cmath>
#include <cstdio>

namespace indoor_nav
{

namespace
{
// Mean Earth radius (m), matched to PX4 v1.14.4's own CONSTANTS_RADIUS_OF_
// EARTH (src/lib/geo/geo.h) rather than the WGS-84 semi-major axis
// (6378137 m) -- since IGS-004's GPS_INPUT ultimately feeds PX4's own EKF2,
// using the same constant PX4's map_projection already uses avoids a second,
// independently-wrong small discrepancy on top of this module's own
// equirectangular approximation. That approximation (vs. PX4's azimuthal-
// equidistant map_projection) is deliberately simpler: both agree to well
// under 1% over the tens-of-meters displacements this indoor-cage-scoped
// module ever sees (Safety Considerations #4), so the extra complexity of
// matching PX4's exact projection isn't worth it here.
constexpr double kEarthRadiusM = 6371000.0;

// Local NED (north_m, east_m, down_m) -> WGS-84, anchored at
// (origin_lat_deg, origin_lon_deg, origin_alt_msl). Small-displacement
// equirectangular projection: exact at the origin, first-order accurate
// away from it.
GnssSimNode::WgsFix local_to_wgs84(
  double origin_lat_deg, double origin_lon_deg, float origin_alt_msl,
  float north_m, float east_m, float down_m)
{
  const double origin_lat_rad = origin_lat_deg * M_PI / 180.0;
  const double dlat_rad = static_cast<double>(north_m) / kEarthRadiusM;
  const double dlon_rad =
    static_cast<double>(east_m) / (kEarthRadiusM * std::cos(origin_lat_rad));

  GnssSimNode::WgsFix fix;
  fix.valid = true;
  fix.lat_deg = origin_lat_deg + dlat_rad * 180.0 / M_PI;
  fix.lon_deg = origin_lon_deg + dlon_rad * 180.0 / M_PI;
  fix.alt_msl = origin_alt_msl - down_m;
  return fix;
}
}  // namespace

namespace
{
// Rotates a body-frame vector into this node's local nav frame using `q`
// (VehicleState.q's documented "[w,x,y,z] NED-to-body" convention),
// verified against the standard aerospace NED-to-body DCM: applying the
// Hamilton quaternion sandwich rotate(q, v_body) directly (no conjugation)
// recovers the same result as R_nb^T @ v_body -- i.e. it is already the
// body->NED transform, not its inverse. Equivalent closed form (avoids a
// second quaternion multiply):
//   v' = v + 2w(qv x v) + 2 qv x (qv x v)
std::array<float, 3> rotate_body_to_local(
  const std::array<float, 4> & q, const std::array<float, 3> & v_body)
{
  const float w = q[0];
  const std::array<float, 3> qv{q[1], q[2], q[3]};

  auto cross = [](const std::array<float, 3> & a, const std::array<float, 3> & b) {
      return std::array<float, 3>{
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]};
    };

  const std::array<float, 3> t = cross(qv, v_body);
  const std::array<float, 3> t2 = cross(qv, t);
  return {
    v_body[0] + 2.0F * w * t[0] + 2.0F * t2[0],
    v_body[1] + 2.0F * w * t[1] + 2.0F * t2[1],
    v_body[2] + 2.0F * w * t[2] + 2.0F * t2[2]
  };
}
}  // namespace

GnssSimNode::GnssSimNode(const rclcpp::NodeOptions & options)
: Node("gnss_sim_node", options)
{
  max_flow_dt_s_ = declare_parameter("max_flow_dt_s", max_flow_dt_s_);
  eph_at_anchor_m_ = declare_parameter("eph_at_anchor_m", eph_at_anchor_m_);
  epv_at_anchor_m_ = declare_parameter("epv_at_anchor_m", epv_at_anchor_m_);
  eph_growth_rate_m_per_s_ =
    declare_parameter("eph_growth_rate_m_per_s", eph_growth_rate_m_per_s_);
  epv_growth_rate_m_per_s_ =
    declare_parameter("epv_growth_rate_m_per_s", epv_growth_rate_m_per_s_);

  pub_local_estimate_ = create_publisher<nav_msgs::msg::Odometry>(
    "/mc/indoor/local_estimate", rclcpp::QoS(10));
  pub_gnss_sim_status_ = create_publisher<mc_msgs::msg::IndoorGnssSimStatus>(
    "/mc/indoor/gnss_sim_status", rclcpp::QoS(10));
  pub_status_text_ =
    create_publisher<mc_msgs::msg::StatusText>("/mc/status_text", rclcpp::QoS(10));

  sub_vehicle_state_ = create_subscription<mc_msgs::msg::VehicleState>(
    "/mc/fcc/vehicle_state", rclcpp::QoS(10),
    [this](mc_msgs::msg::VehicleState::SharedPtr msg) {on_vehicle_state(*msg);});
  sub_optical_flow_ = create_subscription<mc_msgs::msg::OpticalFlow>(
    "/mc/sensors/optical_flow", rclcpp::QoS(10),
    [this](mc_msgs::msg::OpticalFlow::SharedPtr msg) {on_optical_flow(*msg);});
  sub_gcs_command_ = create_subscription<mc_msgs::msg::GcsCommand>(
    "/mc/gcs/command", rclcpp::QoS(10),
    [this](mc_msgs::msg::GcsCommand::SharedPtr msg) {on_gcs_command(*msg);});

  RCLCPP_INFO(get_logger(), "gnss_sim_node started");
}

void GnssSimNode::on_vehicle_state(const mc_msgs::msg::VehicleState & msg)
{
  attitude_q_ = msg.q;
  has_attitude_ = true;
  last_alt_msl_ = msg.alt_msl;

  const rclcpp::Time stamp(msg.header.stamp);

  if (!alt_msl_anchor_.has_value()) {
    alt_msl_anchor_ = msg.alt_msl;
    z_ = 0.0F;
    last_vehicle_state_stamp_ = stamp;
    publish_estimate(stamp);
    return;
  }

  // NED down: gaining altitude (alt_msl increasing) means z should
  // decrease, matching VehicleState.z/alt_msl's own documented frames.
  const float new_z = -(msg.alt_msl - *alt_msl_anchor_);
  const double dt = (stamp - *last_vehicle_state_stamp_).seconds();
  if (dt > 0.0) {
    vz_ = static_cast<float>((new_z - z_) / dt);
  }
  z_ = new_z;
  last_vehicle_state_stamp_ = stamp;

  publish_estimate(stamp);
}

void GnssSimNode::on_gcs_command(const mc_msgs::msg::GcsCommand & msg)
{
  if (msg.command != kMcCmdIndoorGnssSimAnchor) {
    return;
  }
  reanchor(
    static_cast<double>(msg.params[0]), static_cast<double>(msg.params[1]), msg.params[2],
    rclcpp::Time(msg.header.stamp));
}

void GnssSimNode::reanchor(
  double lat_deg, double lon_deg, float alt_msl, const rclcpp::Time & stamp)
{
  mc_msgs::msg::StatusText status;
  status.header.stamp = stamp;

  if (std::abs(lat_deg) > 90.0 || std::abs(lon_deg) > 180.0) {
    status.severity = mc_msgs::msg::StatusText::SEVERITY_ERROR;
    status.text = "gnss_sim_node: rejected false-origin anchor, lat/lon out of range";
    RCLCPP_ERROR(
      get_logger(),
      "reanchor: rejected out-of-range false origin (lat=%.7f, lon=%.7f) -- "
      "previous anchor (if any) is unchanged",
      lat_deg, lon_deg);
    pub_status_text_->publish(status);
    return;
  }

  // Re-zero the running local integrator so "local (0,0,0)" always means
  // "the most recently declared false origin" -- otherwise a re-anchor
  // mid-session would double-count whatever drift had already accumulated
  // against the *previous* origin on top of the newly declared one.
  x_ = 0.0F;
  y_ = 0.0F;
  z_ = 0.0F;
  if (last_alt_msl_.has_value()) {
    alt_msl_anchor_ = *last_alt_msl_;
  }

  const bool was_active = active_;
  false_origin_lat_deg_ = lat_deg;
  false_origin_lon_deg_ = lon_deg;
  false_origin_alt_msl_ = alt_msl;
  anchor_stamp_ = stamp;
  active_ = true;

  char text_buf[128];
  std::snprintf(
    text_buf, sizeof(text_buf), "gnss_sim_node: %s false origin lat=%.7f lon=%.7f alt_msl=%.2f",
    (was_active ? "re-anchored" : "anchored"), lat_deg, lon_deg, static_cast<double>(alt_msl));
  status.severity = mc_msgs::msg::StatusText::SEVERITY_INFO;
  status.text = text_buf;
  RCLCPP_INFO(get_logger(), "%s", status.text.c_str());
  pub_status_text_->publish(status);

  publish_gnss_sim_status(stamp);
}

GnssSimNode::WgsFix GnssSimNode::wgs84_fix() const
{
  if (!active_) {
    return WgsFix{};
  }
  return local_to_wgs84(
    false_origin_lat_deg_, false_origin_lon_deg_, false_origin_alt_msl_, x_, y_, z_);
}

void GnssSimNode::publish_gnss_sim_status(const rclcpp::Time & stamp)
{
  const double time_since_anchor_s =
    active_ ? (stamp - *anchor_stamp_).seconds() : 0.0;

  mc_msgs::msg::IndoorGnssSimStatus msg;
  msg.header.stamp = stamp;
  msg.active = active_;
  msg.eph = static_cast<float>(eph_at_anchor_m_ + eph_growth_rate_m_per_s_ * time_since_anchor_s);
  msg.epv = static_cast<float>(epv_at_anchor_m_ + epv_growth_rate_m_per_s_ * time_since_anchor_s);
  msg.time_since_anchor_s = static_cast<float>(time_since_anchor_s);
  // IGS-005's own concern (waypoint-deviation sanity bound) -- not
  // implemented yet, see IndoorGnssSimStatus.msg's own header comment.
  msg.waypoint_deviation_m = 0.0F;

  latest_gnss_sim_status_ = msg;
  pub_gnss_sim_status_->publish(msg);
}

void GnssSimNode::on_optical_flow(const mc_msgs::msg::OpticalFlow & msg)
{
  const rclcpp::Time stamp(msg.header.stamp);

  if (!has_attitude_) {
    RCLCPP_WARN(
      get_logger(),
      "on_optical_flow: no FCC attitude received yet -- dropping flow sample "
      "(cannot rotate body-frame velocity into the local frame)");
    last_flow_stamp_ = stamp;
    return;
  }

  if (last_flow_stamp_.has_value()) {
    const double dt = (stamp - *last_flow_stamp_).seconds();
    if (dt > 0.0 && dt <= max_flow_dt_s_) {
      const std::array<float, 3> v_local =
        rotate_body_to_local(attitude_q_, {msg.vx, msg.vy, 0.0F});
      x_ += v_local[0] * static_cast<float>(dt);
      y_ += v_local[1] * static_cast<float>(dt);
      vx_ = v_local[0];
      vy_ = v_local[1];
    } else if (dt > max_flow_dt_s_) {
      RCLCPP_WARN(
        get_logger(),
        "on_optical_flow: gap of %.3f s exceeds max_flow_dt_s_ (%.3f s) -- "
        "sample dropped from integration, not integrated across the gap",
        dt, max_flow_dt_s_);
    }
  }
  last_flow_stamp_ = stamp;

  publish_estimate(stamp);
}

void GnssSimNode::publish_estimate(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "indoor_nav_local";
  msg.child_frame_id = "base_link";
  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = z_;
  msg.pose.pose.orientation.w = attitude_q_[0];
  msg.pose.pose.orientation.x = attitude_q_[1];
  msg.pose.pose.orientation.y = attitude_q_[2];
  msg.pose.pose.orientation.z = attitude_q_[3];
  msg.twist.twist.linear.x = vx_;
  msg.twist.twist.linear.y = vy_;
  msg.twist.twist.linear.z = vz_;

  latest_estimate_ = msg;
  pub_local_estimate_->publish(msg);

  // Republished alongside the local estimate (same message-timestamp-driven
  // cadence, no wall timer) so eph/epv's growth-with-time-since-anchor is
  // observable on /mc/indoor/gnss_sim_status as new sensor samples arrive,
  // not just at anchor time.
  publish_gnss_sim_status(stamp);
}

}  // namespace indoor_nav
