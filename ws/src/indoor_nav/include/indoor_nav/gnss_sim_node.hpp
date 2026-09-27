// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (INDOOR-GNSS-SIMULATION-BACKLOG.md).

#pragma once

#include <netinet/in.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <mc_msgs/msg/baro_altitude.hpp>
#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/indoor_gnss_sim_status.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
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
// attitude), corrects the vertical axis directly from FCC raw baro
// altitude, and publishes the result on /mc/indoor/local_estimate for bench
// validation. No camera dependency. WGS-84 conversion (IGS-003) and
// HIL_GPS injection (IGS-004) are later extensions of this same class,
// documented further down.
//
// Open Item 3 resolution (attitude/baro source, INDOOR-GNSS-SIMULATION-
// BACKLOG.md):
//   - attitude: VehicleState.q (PX4's attitude estimator output: gyro +
//     accel + mag, no GPS dependency, so it stays valid indoors) -- used
//     here to rotate flow-derived body-frame velocity into the local frame.
//   - vertical: /mc/sensors/baro_altitude (mc_msgs/BaroAltitude), raw baro
//     relayed by fcc_bridge from MAVLink ALTITUDE.altitude_monotonic.
//     Correction (FT-006, 2026-09-26): this used VehicleState.alt_msl, on
//     the premise that indoors EKF2 has no GPS altitude to blend. That
//     premise is false once PX4 accepts this node's own HIL_GPS: EKF2 fuses
//     the injected altitude back into alt_msl, which fed the next HIL_GPS
//     altitude -- a closed loop that ran PX4's altitude estimate ~42 km off
//     on the orin-nx bench. The vertical input must be independent of the
//     estimator this node feeds.
// It does still need *a* horizontal-velocity input: HIF-008's optical-flow sensor
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
//
// IGS-004 extends this same class again with a periodic sender that
// streams wgs84_fix() to the FCC over a new dedicated MC->FCC UDP channel
// (ICD-MC-001.md §3.3b) so PX4 accepts it as a live GPS receiver. Firmware
// correction (Open Item 2, confirmed against the real PX4 v1.14.4 tag
// originally, re-confirmed against v1.17.0 once this project's target
// firmware moved, same verification discipline HIF-008 and the mission
// protocol already used): `mavlink_receiver.cpp` has no handler at all
// for `GPS_INPUT` (`MAVLINK_MSG_ID_GPS_INPUT` does not appear anywhere in
// `src/`, at either tag) -- this proposal's own wording was never checked
// against firmware. The message PX4 actually accepts as a fake GPS receiver over
// MAVLink is `HIL_GPS`, gated by `handle_message_hil_gps()` running only
// when `MAV_USEHILGPS=1` (default 0) *and* the frame's `sysid` equals the
// FCC's own `MAV_SYS_ID` -- full-HIL mode is deliberately not used here
// (see send_hil_gps()'s own .cpp doc comment). Every "GPS_INPUT" in this
// ticket's own AC/deliverables text refers to this HIL_GPS mechanism.
//
// IGS-005 extends this same class a final time with self-supervision, so
// this module fails safely instead of compounding a bad estimate into a
// worse one (Safety Considerations, INDOOR-GNSS-SIMULATION-BACKLOG.md):
//   - Real-GPS lockout (real_gps_ok_): continuously re-evaluated from every
//     /mc/fcc/vehicle_state sample using the same gnss_ok() threshold
//     VIS-002/SDD §8.3 defines (fix_type >= 3, hdop < 2.0) -- auto-clears
//     the moment a real fix goes away again, since this isn't a fault of
//     this module, just "doesn't apply right now."
//   - Staleness/drift/waypoint-deviation watchdog (gps_sim_lost_): checked
//     every hil_gps_timer_ tick (check_self_supervision()) so a total input
//     stall is still caught even though nothing message-driven would fire
//     otherwise. Unlike real_gps_ok_, this LATCHES -- once tripped, only an
//     explicit reanchor() clears it (Safety Considerations #1/#2: a
//     compromised estimate must never silently resume streaming on its
//     own). Publishes /mc/fault (INDOOR_GNSS_SIM_LOST) -- health_monitor_
//     node does not own this fault (this module publishes it directly,
//     same non-duplication precedent OFFBOARD_STREAM_STALE already sets,
//     OFFBOARD-MODE-SWITCHING-BACKLOG.md) -- mission_manager_node's
//     existing generic on_fault()/guard_any_fault() (severity-keyed, not a
//     fault_id allowlist) is what actually reacts.
//   - Waypoint-deviation (compute_waypoint_deviation()) is read-only by
//     construction -- a const method that only ever reads x_/y_/z_ and
//     latest_mission_upload_, never writes them (Safety Considerations #2:
//     a sanity-bound fault trigger, never a position correction). Compares
//     against the nearest /mc/gcs/mission_upload ingress_waypoints entry,
//     not real leg-progress tracking (no such signal reaches this node) --
//     a coarse "is the estimate anywhere near the planned route" bound,
//     not track-following; documented simplification, not silently
//     assumed. Also assumes the operator's false-origin anchor is placed
//     at/near the FCC's own local-position origin, since ingress_waypoints
//     are local Cartesian, not WGS-84 -- unverified elsewhere in this
//     repo, flagged here rather than silently relied upon.
class GnssSimNode : public rclcpp::Node
{
public:
  // A single WGS-84 fix derived from the current local estimate and false
  // origin. `valid` is false until at least one anchor command has been
  // accepted -- there is no safe default false origin (Safety
  // Considerations #5: bootstrapping the origin is an operator action, not
  // automatic). This is IGS-004's own consumption point (send_hil_gps(),
  // below) as well as this ticket's test seam -- deliberately not a ROS
  // topic itself, see IndoorGnssSimStatus.msg's own header comment.
  struct WgsFix
  {
    bool valid{false};
    double lat_deg{0.0};
    double lon_deg{0.0};
    float alt_msl{0.0F};
  };

  explicit GnssSimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GnssSimNode() override;

  // Test seams: drive the fusion core directly with a known message,
  // bypassing pub/sub timing so a test can assert on a specific sequence
  // deterministically. Production code only reaches these via the
  // subscription callbacks wired in the constructor.
  void on_vehicle_state(const mc_msgs::msg::VehicleState & msg);
  void on_baro_altitude(const mc_msgs::msg::BaroAltitude & msg);
  void on_optical_flow(const mc_msgs::msg::OpticalFlow & msg);
  void on_gcs_command(const mc_msgs::msg::GcsCommand & msg);
  // IGS-005: caches the latest uploaded mission for
  // compute_waypoint_deviation() -- see this class's own doc comment.
  void on_mission_upload(const mc_msgs::msg::MissionUpload & msg);

  // Test seam: the last message this node published on
  // /mc/indoor/local_estimate, without needing a subscriber of its own.
  nav_msgs::msg::Odometry local_estimate() const {return latest_estimate_;}

  // Test seam: the last message this node published on
  // /mc/indoor/gnss_sim_status.
  mc_msgs::msg::IndoorGnssSimStatus gnss_sim_status() const {return latest_gnss_sim_status_;}

  // Test seam / IGS-004 consumption point: the current WGS-84 fix derived
  // from the local estimate + false origin. See WgsFix's own doc comment.
  WgsFix wgs84_fix() const;

  // IGS-004 test seam: builds one HIL_GPS frame (ICD-MC-001.md §3.3b) from
  // the current wgs84_fix() + gnss_sim_status() and sends it to the FCC,
  // bypassing hil_gps_timer_ so a test can assert on a specific,
  // deterministic frame. No-op (sends nothing) while wgs84_fix().valid is
  // false -- there is no honest fix to report before the first anchor
  // (Safety Considerations #5, INDOOR-GNSS-SIMULATION-BACKLOG.md). See the
  // .cpp for the full field-by-field derivation and firmware citations.
  void send_hil_gps();

  // IGS-005 test seam: runs the staleness/drift/waypoint-deviation watchdog
  // once, as of `stamp`, bypassing hil_gps_timer_ so a test can assert
  // deterministically. Production code reaches this from the same timer
  // callback that drives send_hil_gps(), immediately before it -- see this
  // class's own doc comment for the full design.
  void check_self_supervision(const rclcpp::Time & stamp);

  // Test seam: true once check_self_supervision() has latched the
  // staleness/drift/waypoint-deviation fault (cleared only by reanchor()).
  bool gps_sim_lost() const {return gps_sim_lost_;}

  // Test seam: true when the last /mc/fcc/vehicle_state sample reported a
  // usable real GPS fix (gnss_ok() threshold) -- the real-GPS lockout gate.
  bool real_gps_ok() const {return real_gps_ok_;}

  // Test seam: the last FaultEvent this node published on /mc/fault, if
  // any -- avoids requiring a test to spin a real subscription, same
  // rationale as local_estimate()/gnss_sim_status() above.
  std::optional<mc_msgs::msg::FaultEvent> latest_fault() const {return latest_fault_;}

private:
  void publish_estimate(const rclcpp::Time & stamp);
  void publish_gnss_sim_status(const rclcpp::Time & stamp);
  int open_hil_gps_socket();

  // IGS-005: read-only by construction -- only ever reads x_/y_/z_ and
  // latest_mission_upload_, never writes them (Safety Considerations #2).
  // Returns 0.0 if no mission (or no ingress_waypoints) has been uploaded
  // yet -- nothing to compare against, not a fabricated "zero deviation".
  float compute_waypoint_deviation() const;

  // IGS-005: publishes INDOOR_GNSS_SIM_LOST on /mc/fault (edge-triggered --
  // check_self_supervision() only calls this once per latch, matching
  // HEALTH-003's own "does not repeatedly re-fire" precedent).
  void publish_gps_sim_lost_fault(const rclcpp::Time & stamp, const std::string & reason);

  // Re-anchors the false origin at (lat_deg, lon_deg, alt_msl) as of
  // `stamp`: resets time_since_anchor_s (and therefore eph/epv) to their
  // at-anchor floor, and re-zeroes the running local (x, y, z) integrator
  // so "local (0,0,0)" always means "the most recently declared false
  // origin", never a stale earlier one plus whatever drift accumulated on
  // top of it. Logged via /mc/status_text either way (SFR-006's command-
  // logging precedent) -- rejects (logged as an error, no state change) if
  // lat/lon are outside valid WGS-84 ranges, or (IGS-005) if a real GPS fix
  // is currently present (indoor-only interlock, Safety Considerations #3).
  // On success, also clears gps_sim_lost_ -- re-anchoring is this module's
  // only recovery path from a latched staleness/drift/deviation fault.
  void reanchor(double lat_deg, double lon_deg, float alt_msl, const rclcpp::Time & stamp);

  // IGS-005: updates real_gps_ok_ from a fresh /mc/fcc/vehicle_state
  // sample, logging (edge-triggered, not every sample) via /mc/status_text
  // when it changes -- "logged, not silent" (this ticket's own AC).
  void update_real_gps_lockout(const mc_msgs::msg::VehicleState & msg, const rclcpp::Time & stamp);

  rclcpp::Subscription<mc_msgs::msg::VehicleState>::SharedPtr sub_vehicle_state_;
  rclcpp::Subscription<mc_msgs::msg::BaroAltitude>::SharedPtr sub_baro_altitude_;
  rclcpp::Subscription<mc_msgs::msg::OpticalFlow>::SharedPtr sub_optical_flow_;
  rclcpp::Subscription<mc_msgs::msg::GcsCommand>::SharedPtr sub_gcs_command_;
  rclcpp::Subscription<mc_msgs::msg::MissionUpload>::SharedPtr sub_mission_upload_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_local_estimate_;
  rclcpp::Publisher<mc_msgs::msg::IndoorGnssSimStatus>::SharedPtr pub_gnss_sim_status_;
  rclcpp::Publisher<mc_msgs::msg::StatusText>::SharedPtr pub_status_text_;
  rclcpp::Publisher<mc_msgs::msg::FaultEvent>::SharedPtr pub_fault_;

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

  // Vertical channel: raw baro altitude is applied directly (never
  // integrated), so z_ always reflects the latest baro sample, anchored at
  // the first one received so the estimate starts at z=0 -- and re-anchored
  // again on each IGS-003 reanchor() call, see that method's own doc
  // comment. Only differences matter (pressure altitude, no QNH).
  std::optional<float> baro_alt_anchor_;
  std::optional<float> last_baro_alt_;  // rebaseline target for reanchor()
  std::optional<rclcpp::Time> last_baro_stamp_;
  // Attitude + real-GPS lockout input; IGS-005 staleness still covers it.
  std::optional<rclcpp::Time> last_vehicle_state_stamp_;
  float z_{0.0F};
  // Low-pass filtered baro rate (on_baro_altitude()), not the raw sample
  // difference -- it goes out as HIL_GPS vd, which PX4 always treats as
  // valid (handle_message_hil_gps() hardcodes vel_ned_valid = true), so
  // this node is the only place the noise can be removed.
  float vz_{0.0F};
  // First-order time constant for vz_. 0.5 s cuts stationary noise ~10x
  // (from ~1 m/s at 0.1 m baro jitter) while lagging a real climb rate by
  // well under a second. <= 0 disables the filter (raw difference).
  double baro_vz_filter_tau_s_{0.5};

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

  // IGS-004: HIL_GPS sender (ICD-MC-001.md §3.3b). Send-only, unbound UDP
  // socket -- this is a one-way MC->FCC stream, so unlike
  // XrceDdsBackend::open_mission_socket() (fcc_bridge/xrce_dds_backend.cpp)
  // there is no reply to bind a local port for.
  int hil_gps_sock_fd_{-1};
  sockaddr_in hil_gps_target_addr_{};

  // This MC's MAVLink identity on the HIL_GPS channel. mav_system_id_ is
  // not just an addressing label here (contrast MissionProtocolClient's
  // system_id_, fcc_bridge/mission_protocol_client.hpp) -- PX4's
  // handle_message_hil_gps() gate additionally requires it to equal the
  // FCC's own configured MAV_SYS_ID (mavlink_receiver.cpp's
  // `msg->sysid == mavlink_system.sysid` check), so this must track
  // whatever DRONE_ID/MAV_SYS_ID this drone is actually configured with,
  // not merely be internally consistent.
  uint8_t mav_system_id_{1};
  uint8_t mav_component_id_{191};  // MAV_COMP_ID_ONBOARD_COMPUTER

  rclcpp::TimerBase::SharedPtr hil_gps_timer_;
  double hil_gps_rate_hz_{5.0};

  // Honest satellites_visible model (send_hil_gps()'s own doc comment has
  // the full derivation): degrades linearly from hil_gps_sats_at_anchor_ to
  // hil_gps_min_sats_ as eph grows from eph_at_anchor_m_ toward PX4's own
  // EKF2_REQ_EPH default (3.0 m, confirmed against the real v1.17.0
  // src/modules/ekf2/params_gnss.yaml -- v1.14.4's equivalent was
  // src/modules/ekf2/ekf2_params.c, before PX4 migrated EKF2 params to
  // YAML) -- never a fixed "great fix" count.
  double hil_gps_sats_at_anchor_{10.0};
  double hil_gps_min_sats_{4.0};

  // IGS-005: real-GPS lockout (Safety Considerations #3). Matches VIS-002/
  // SDD §8.3's own gnss_ok() threshold exactly, not an independently-tuned
  // number. Starts true (fail-safe default: assume a real fix may be
  // present/unknown until the first /mc/fcc/vehicle_state sample proves
  // otherwise) -- same "no safe default, prove it first" philosophy as
  // WgsFix::valid starting false.
  double gnss_ok_hdop_threshold_{2.0};
  bool real_gps_ok_{true};

  // IGS-005: staleness/drift/waypoint-deviation watchdog. gps_sim_lost_
  // latches (see check_self_supervision()'s own .cpp doc comment for why);
  // cleared only by a successful reanchor().
  double input_staleness_timeout_s_{1.0};
  double max_eph_before_fault_m_{5.0};
  double max_waypoint_deviation_m_{50.0};
  bool gps_sim_lost_{false};

  mc_msgs::msg::MissionUpload::SharedPtr latest_mission_upload_;
  std::optional<mc_msgs::msg::FaultEvent> latest_fault_;
};

}  // namespace indoor_nav
