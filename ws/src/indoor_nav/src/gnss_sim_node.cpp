// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md).

#include "indoor_nav/gnss_sim_node.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace indoor_nav
{

namespace
{
std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr ? std::string(v) : fallback;
}
}  // namespace

namespace
{
// Mean Earth radius (m), matched to PX4's own CONSTANTS_RADIUS_OF_
// EARTH (src/lib/geo/geo.h, confirmed unchanged at v1.14.4 and v1.17.0)
// rather than the WGS-84 semi-major axis
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
  hil_gps_rate_hz_ = declare_parameter("hil_gps_rate_hz", hil_gps_rate_hz_);
  hil_gps_sats_at_anchor_ =
    declare_parameter("hil_gps_sats_at_anchor", hil_gps_sats_at_anchor_);
  hil_gps_min_sats_ = declare_parameter("hil_gps_min_sats", hil_gps_min_sats_);

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

  // IGS-004: HIL_GPS sender (ICD-MC-001.md §3.3b). Same env-var convention
  // as fcc_bridge's XrceDdsBackend (FCC_ETH_IP, DRONE_ID) -- this is the
  // same physical MC<->FCC ethernet link (HIF-002), just a fourth MC<->FCC
  // MAVLink exception on top of the mission-protocol/optical-flow-relay
  // pair fcc_bridge already owns. Deliberately not routed through
  // fcc_bridge_node/FccBackend -- gnss_sim_node (indoor_nav) owns this
  // socket directly, since it is the only producer and this is a one-way
  // stream (this ticket's own AC: "a direct point-to-point UDP send if
  // routing isn't needed").
  const std::string fcc_ip = env_or("FCC_ETH_IP", "192.168.12.31");
  const int hil_gps_port = std::stoi(env_or("FCC_GPS_INPUT_PORT", "14557"));
  hil_gps_target_addr_.sin_family = AF_INET;
  hil_gps_target_addr_.sin_port = htons(static_cast<uint16_t>(hil_gps_port));
  if (::inet_pton(AF_INET, fcc_ip.c_str(), &hil_gps_target_addr_.sin_addr) != 1) {
    throw std::runtime_error("GnssSimNode: invalid FCC_ETH_IP '" + fcc_ip + "'");
  }
  // mav_system_id_ must equal the FCC's own MAV_SYS_ID (see this member's
  // own .hpp doc comment) -- DRONE_ID is this project's existing convention
  // for that value (MissionProtocolClient's system_id_, same env var).
  mav_system_id_ = static_cast<uint8_t>(std::stoi(env_or("DRONE_ID", "1")));
  hil_gps_sock_fd_ = open_hil_gps_socket();

  const auto hil_gps_period =
    std::chrono::duration<double>(1.0 / std::max(hil_gps_rate_hz_, 0.1));
  hil_gps_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(hil_gps_period),
    [this] {send_hil_gps();});

  RCLCPP_INFO(
    get_logger(), "gnss_sim_node started, HIL_GPS -> %s:%d @ %.1f Hz (sysid=%u)",
    fcc_ip.c_str(), hil_gps_port, hil_gps_rate_hz_, static_cast<unsigned>(mav_system_id_));
}

GnssSimNode::~GnssSimNode()
{
  if (hil_gps_sock_fd_ >= 0) {
    ::close(hil_gps_sock_fd_);
  }
}

int GnssSimNode::open_hil_gps_socket()
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error("GnssSimNode: failed to create HIL_GPS UDP socket");
  }
  // Send-only: no bind() -- there is nothing for the FCC to reply to on
  // this channel (contrast XrceDdsBackend::open_mission_socket(), which
  // binds so MISSION_REQUEST_INT replies have somewhere fixed to land).
  return fd;
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

// IGS-004. Sent at hil_gps_rate_hz_ (default 5 Hz) via hil_gps_timer_ --
// comfortably inside both of PX4 EKF2's own GNSS timing bounds, confirmed
// against the real v1.17.0 src/modules/ekf2/EKF/common.h (unchanged path
// and value from v1.14.4): GNSS_MAX_INTERVAL = 500 ms (>=2 Hz avoids the
// "intermittent" classification in gps_control.cpp -- at v1.17.0 this
// file moved to src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp,
// same logic) and 2x that (1 s) before EKF2 calls the feed stale outright.
// 5 Hz is not this ticket's own invention -- it is exactly the PX4-typical
// GPS receiver rate this proposal's own text guessed, now verified
// compatible against the actual firmware bound rather than assumed.
void GnssSimNode::send_hil_gps()
{
  const WgsFix fix = wgs84_fix();
  if (!fix.valid) {
    // No false origin anchored yet -- there is no honest fix to report
    // (Safety Considerations #5, INDOOR-GNSS-SIMULATION-BACKLOG.md).
    // reanchor() already logs anchor state via /mc/status_text, so this
    // silence is not itself un-observable.
    return;
  }

  const mc_msgs::msg::IndoorGnssSimStatus status = gnss_sim_status();

  // eph/epv: HIL_GPS's wire units are centimeters -- PX4's own
  // handle_message_hil_gps() decodes `hil_gps.eph * 1e-2f` -> meters
  // (confirmed against the real mavlink_receiver.cpp at both v1.14.4 and
  // v1.17.0), so this is the exact inverse of that conversion, not an
  // assumed unit.
  const uint16_t eph_cm =
    static_cast<uint16_t>(std::clamp(status.eph * 100.0F, 0.0F, 65535.0F));
  const uint16_t epv_cm =
    static_cast<uint16_t>(std::clamp(status.epv * 100.0F, 0.0F, 65535.0F));

  // satellites_visible: an honest, degrading value (this ticket's own AC:
  // "never a fixed 'great accuracy' placeholder"), tied to the same eph
  // PX4's own EKF2 GPS-quality gate uses -- degrades linearly from
  // hil_gps_sats_at_anchor_ down to hil_gps_min_sats_ as eph grows from
  // eph_at_anchor_m_ toward EKF2_REQ_EPH's default (3.0 m, confirmed
  // against the real v1.17.0 src/modules/ekf2/params_gnss.yaml --
  // v1.14.4's equivalent was src/modules/ekf2/ekf2_params.c, before PX4
  // migrated EKF2 params to YAML) -- satellite count and EKF2's own
  // accuracy-gate rejection degrade on the same honest timescale, not two
  // independently-tuned numbers.
  constexpr double kEkf2ReqEphDefaultM = 3.0;
  const double eph_span = std::max(kEkf2ReqEphDefaultM - eph_at_anchor_m_, 1e-3);
  const double eph_frac =
    std::clamp((static_cast<double>(status.eph) - eph_at_anchor_m_) / eph_span, 0.0, 1.0);
  const uint8_t satellites_visible = static_cast<uint8_t>(std::lround(
      hil_gps_sats_at_anchor_ - eph_frac * (hil_gps_sats_at_anchor_ - hil_gps_min_sats_)));

  // vn/ve/vd: x_/y_/z_ and their derivatives are already this node's local
  // NED frame (this file's own IGS-001 comments) -- no rotation needed.
  const int16_t vn_cms = static_cast<int16_t>(std::clamp(vx_ * 100.0F, -32768.0F, 32767.0F));
  const int16_t ve_cms = static_cast<int16_t>(std::clamp(vy_ * 100.0F, -32768.0F, 32767.0F));
  const int16_t vd_cms = static_cast<int16_t>(std::clamp(vz_ * 100.0F, -32768.0F, 32767.0F));
  const float ground_speed_m_s = std::sqrt(vx_ * vx_ + vy_ * vy_);
  const uint16_t vel_cms =
    static_cast<uint16_t>(std::clamp(ground_speed_m_s * 100.0F, 0.0F, 65535.0F));
  const double cog_deg = std::fmod(
    std::atan2(static_cast<double>(vy_), static_cast<double>(vx_)) * 180.0 / M_PI + 360.0, 360.0);
  const uint16_t cog_cdeg = static_cast<uint16_t>(std::lround(cog_deg * 100.0));

  mavlink_message_t msg;
  mavlink_msg_hil_gps_pack(
    mav_system_id_, mav_component_id_, &msg,
    static_cast<uint64_t>(now().nanoseconds() / 1000),
    /*fix_type=*/ 3,  // 3D fix -- only ever sent once wgs84_fix() is valid
    /*lat=*/ static_cast<int32_t>(std::llround(fix.lat_deg * 1e7)),
    /*lon=*/ static_cast<int32_t>(std::llround(fix.lon_deg * 1e7)),
    /*alt=*/ static_cast<int32_t>(std::llround(static_cast<double>(fix.alt_msl) * 1000.0)),
    eph_cm, epv_cm, vel_cms, vn_cms, ve_cms, vd_cms, cog_cdeg, satellites_visible,
    /*id=*/ 0, /*yaw=*/ 0);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ::sendto(
    hil_gps_sock_fd_, buf, len, 0,
    reinterpret_cast<const sockaddr *>(&hil_gps_target_addr_), sizeof(hil_gps_target_addr_));
}

}  // namespace indoor_nav
