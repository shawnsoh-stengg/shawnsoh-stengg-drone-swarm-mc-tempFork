// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md, IGS-001).

#include "indoor_nav/gnss_sim_node.hpp"

namespace indoor_nav
{

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

  pub_local_estimate_ = create_publisher<nav_msgs::msg::Odometry>(
    "/mc/indoor/local_estimate", rclcpp::QoS(10));

  sub_vehicle_state_ = create_subscription<mc_msgs::msg::VehicleState>(
    "/mc/fcc/vehicle_state", rclcpp::QoS(10),
    [this](mc_msgs::msg::VehicleState::SharedPtr msg) {on_vehicle_state(*msg);});
  sub_optical_flow_ = create_subscription<mc_msgs::msg::OpticalFlow>(
    "/mc/sensors/optical_flow", rclcpp::QoS(10),
    [this](mc_msgs::msg::OpticalFlow::SharedPtr msg) {on_optical_flow(*msg);});

  RCLCPP_INFO(get_logger(), "gnss_sim_node started");
}

void GnssSimNode::on_vehicle_state(const mc_msgs::msg::VehicleState & msg)
{
  attitude_q_ = msg.q;
  has_attitude_ = true;

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
}

}  // namespace indoor_nav
