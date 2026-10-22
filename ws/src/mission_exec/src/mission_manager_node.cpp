// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#include "mission_exec/mission_manager_node.hpp"

#include <algorithm>
#include <cmath>

#include <mc_msgs/msg/swarm_peer.hpp>

namespace mission_exec
{

namespace
{
// ICD-MC-001 §10.1/§10.2
constexpr uint8_t kArmingStateArmed = 2;
constexpr uint8_t kNavStateAutoMission = 3;
constexpr uint8_t kNavStateAutoLoiter = 4;

// ICD-MC-001 §10.5
constexpr uint32_t kMavCmdNavReturnToLaunch = 20;
// MAV_CMD_NAV_LAND — standard MAVLink common.xml ID 21, not itself listed in
// ICD §10.5's abridged catalogue (a documentation gap in the same vein as
// the "gap note"s already present elsewhere in this ICD, e.g. IsrRole.msg).
constexpr uint32_t kMavCmdNavLand = 21;
constexpr uint32_t kMavCmdNavTakeoff = 22;
constexpr uint32_t kMavCmdMissionStart = 300;
constexpr uint32_t kMavCmdComponentArmDisarm = 400;
constexpr uint32_t kMcCmdPhaseTransition = 50001;
constexpr uint32_t kMcCmdArmAuthorise = 50002;
constexpr uint32_t kMcCmdEndMission = 50003;
constexpr uint32_t kMcCmdGcsOverride = 50004;

// MAV_SEVERITY (MAVLink common.xml): 0=EMERGENCY..2=CRITICAL..7=DEBUG.
constexpr uint8_t kSeverityCritical = 2;

// MAVLink MAV_FRAME / MAV_CMD values for FccMissionItem.
constexpr uint8_t kMavFrameGlobalInt = 5;
constexpr uint8_t kMavFrameGlobalRelativeAltInt = 6;
constexpr uint16_t kMavCmdNavWaypoint = 16;

// check_mission_start(): resend MISSION_START this often, at most this many
// times in total, until vehicle_state reports AUTO_MISSION.
constexpr double kMissionStartRetryPeriodS = 1.0;
constexpr int kMissionStartMaxAttempts = 3;

// Equirectangular approximation -- plenty for acceptance radii of a few
// metres. lon difference is wrapped so the antimeridian doesn't break it.
double horizontal_distance_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg)
{
  constexpr double kEarthRadiusM = 6371008.8;
  constexpr double kDegToRad = M_PI / 180.0;
  const double dlat = (lat2_deg - lat1_deg) * kDegToRad;
  const double dlon = std::remainder(lon2_deg - lon1_deg, 360.0) * kDegToRad *
    std::cos(0.5 * (lat1_deg + lat2_deg) * kDegToRad);
  return kEarthRadiusM * std::hypot(dlat, dlon);
}

// FaultEvent.fault_id -> its constant name, for the FAULT cause report.
std::string fault_name(uint16_t fault_id)
{
  using F = mc_msgs::msg::FaultEvent;
  switch (fault_id) {
    case F::FCC_LINK_LOST: return "FCC_LINK_LOST";
    case F::GCS_LINK_LOST: return "GCS_LINK_LOST";
    case F::MANAGER_DEAD: return "MANAGER_DEAD";
    case F::LOW_BATTERY: return "LOW_BATTERY";
    case F::PEER_LOST: return "PEER_LOST";
    case F::GPU_OVERTEMP: return "GPU_OVERTEMP";
    case F::DISK_FULL: return "DISK_FULL";
    case F::CAMERA_FAIL: return "CAMERA_FAIL";
    case F::GNSS_DEGRADED: return "GNSS_DEGRADED";
    case F::VISUAL_ODOM_FAIL: return "VISUAL_ODOM_FAIL";
    case F::VISION_MEM_HIGH: return "VISION_MEM_HIGH";
    case F::INDOOR_GNSS_SIM_LOST: return "INDOOR_GNSS_SIM_LOST";
    default: return "fault_id " + std::to_string(fault_id);
  }
}
}  // namespace

MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions & options)
: Node("mission_manager_node", options)
{
  control_loop_period_s_ = declare_parameter("control_loop_period_s", control_loop_period_s_);
  mission_ack_timeout_s_ = declare_parameter("mission_ack_timeout_s", mission_ack_timeout_s_);
  mission_ack_max_retries_ =
    declare_parameter("mission_ack_max_retries", mission_ack_max_retries_);
  muster_quorum_count_ = declare_parameter("muster_quorum_count", muster_quorum_count_);
  takeoff_altitude_m_ = declare_parameter("takeoff_altitude_m", takeoff_altitude_m_);
  takeoff_altitude_tolerance_m_ =
    declare_parameter("takeoff_altitude_tolerance_m", takeoff_altitude_tolerance_m_);
  waypoint_acceptance_radius_m_ =
    declare_parameter("waypoint_acceptance_radius_m", waypoint_acceptance_radius_m_);
  home_position_tolerance_m_ =
    declare_parameter("home_position_tolerance_m", home_position_tolerance_m_);
  low_battery_margin_ = declare_parameter("low_battery_margin", low_battery_margin_);
  local_pose_stale_timeout_s_ =
    declare_parameter("local_pose_stale_timeout_s", local_pose_stale_timeout_s_);
  mission_upload_path_ = declare_parameter("mission_upload_path", mission_upload_path_);
  landed_max_vertical_speed_mps_ =
    declare_parameter("landed_max_vertical_speed_mps", landed_max_vertical_speed_mps_);

  build_transition_table();

  // EXEC-004 / SDD §4.5: reload whichever mission upload was most recently
  // persisted, so a watchdog-triggered restart (FR-074) doesn't require a
  // fresh GCS upload. arm_authorised_ is re-derived the same way
  // on_gcs_mission_upload() does it, so a reload behaves identically to a
  // fresh live upload for every downstream guard/check.
  if (auto loaded = load_mission_upload(mission_upload_path_)) {
    mission_upload_ =
      std::make_shared<mc_msgs::msg::MissionUpload>(std::move(*loaded));
    arm_authorised_ = mission_upload_->arm_authorised;
    RCLCPP_INFO(
      get_logger(),
      "reloaded persisted mission upload from %s (upload_id=%u, originally received at %d.%09u)",
      mission_upload_path_.c_str(), mission_upload_->upload_id,
      mission_upload_->header.stamp.sec, mission_upload_->header.stamp.nanosec);
  }

  phase_entered_at_ = now();
  last_phase_publish_at_ = now();
  pending_ack_sent_at_ = now();
  last_local_pose_at_ = now();

  pub_mission_phase_ =
    create_publisher<mc_msgs::msg::MissionPhase>("/mc/mission_phase", rclcpp::QoS(10));
  pub_heartbeat_ = create_publisher<std_msgs::msg::Header>(
    "/mc/manager/heartbeat",
    rclcpp::QoS(10));
  pub_send_command_ =
    create_publisher<mc_msgs::msg::FccCommand>("/mc/fcc/send_command", rclcpp::QoS(10));
  pub_mission_upload_ =
    create_publisher<mc_msgs::msg::FccMission>("/mc/fcc/mission_upload", rclcpp::QoS(10));
  pub_status_text_ =
    create_publisher<mc_msgs::msg::StatusText>("/mc/status_text", rclcpp::QoS(10));
  pub_command_ack_ =
    create_publisher<mc_msgs::msg::GcsCommandAck>("/mc/gcs/command_ack", rclcpp::QoS(10));

  // See the state_cb_group_ doc comment in the header for why every
  // subscription and the control timer share this one group.
  state_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cb_group_;

  sub_vehicle_state_ = create_subscription<mc_msgs::msg::VehicleState>(
    "/mc/fcc/vehicle_state", rclcpp::QoS(10),
    [this](mc_msgs::msg::VehicleState::SharedPtr msg) {on_vehicle_state(msg);}, sub_opts);
  sub_mission_ack_ = create_subscription<mc_msgs::msg::FccMissionAck>(
    "/mc/fcc/mission_ack", rclcpp::QoS(10),
    [this](mc_msgs::msg::FccMissionAck::SharedPtr msg) {on_mission_ack(msg);}, sub_opts);
  sub_gcs_command_ = create_subscription<mc_msgs::msg::GcsCommand>(
    "/mc/gcs/command", rclcpp::QoS(10),
    [this](mc_msgs::msg::GcsCommand::SharedPtr msg) {on_gcs_command(msg);}, sub_opts);
  sub_gcs_mission_upload_ = create_subscription<mc_msgs::msg::MissionUpload>(
    "/mc/gcs/mission_upload", rclcpp::QoS(10),
    [this](mc_msgs::msg::MissionUpload::SharedPtr msg) {on_gcs_mission_upload(msg);}, sub_opts);
  sub_peer_states_ = create_subscription<mc_msgs::msg::SwarmPeerArray>(
    "/mc/swarm/peer_states", rclcpp::QoS(10),
    [this](mc_msgs::msg::SwarmPeerArray::SharedPtr msg) {on_peer_states(msg);}, sub_opts);
  sub_isr_role_ = create_subscription<mc_msgs::msg::IsrRole>(
    "/mc/swarm/isr_role", rclcpp::QoS(10),
    [this](mc_msgs::msg::IsrRole::SharedPtr msg) {on_isr_role(msg);}, sub_opts);
  sub_voronoi_cell_ = create_subscription<mc_msgs::msg::VoronoiCell>(
    "/mc/swarm/voronoi_cell", rclcpp::QoS(10),
    [this](mc_msgs::msg::VoronoiCell::SharedPtr msg) {on_voronoi_cell(msg);}, sub_opts);
  sub_coverage_path_ = create_subscription<nav_msgs::msg::Path>(
    "/mc/search/coverage_path", rclcpp::QoS(10),
    [this](nav_msgs::msg::Path::SharedPtr msg) {on_coverage_path(msg);}, sub_opts);
  sub_fault_ = create_subscription<mc_msgs::msg::FaultEvent>(
    "/mc/fault", rclcpp::QoS(10),
    [this](mc_msgs::msg::FaultEvent::SharedPtr msg) {on_fault(msg);}, sub_opts);
  sub_local_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/mc/local_pose", rclcpp::QoS(10),
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      last_local_pose_at_ = msg->header.stamp;
      local_pose_ever_received_ = true;
    },
    sub_opts);

  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_loop_period_s_)),
    [this] {on_control_timer();}, state_cb_group_);

  RCLCPP_INFO(get_logger(), "mission_manager_node started in PREFLIGHT");
}

// ---------------------------------------------------------------------------
// Transition table (SDD §4.1)
// ---------------------------------------------------------------------------

void MissionManagerNode::build_transition_table()
{
  transition_table_[Phase::PREFLIGHT] = {
    {Phase::ARMING, [this] {return guard_preflight_to_arming();},
      [this] {action_enter_arming();}},
  };
  transition_table_[Phase::ARMING] = {
    {Phase::TAKEOFF, [this] {return guard_arming_to_takeoff();},
      [this] {action_send_takeoff();}},
  };
  transition_table_[Phase::TAKEOFF] = {
    {Phase::INGRESS, [this] {return guard_takeoff_to_ingress();},
      [this] {action_begin_ingress_route_upload();}},
  };
  transition_table_[Phase::INGRESS] = {
    {Phase::MUSTER, [this] {return guard_ingress_to_muster();},
      [this] {action_disable_formation_ctrl();}},
  };
  transition_table_[Phase::MUSTER] = {
    {Phase::ISR_SELECTION, [this] {return guard_muster_to_isr_selection();},
      [this] {action_notify_swarm();}},
  };
  transition_table_[Phase::ISR_SELECTION] = {
    {Phase::VORONOI_PARTITION, [this] {return guard_isr_selection_to_voronoi_partition();},
      [this] {action_notify_swarm();}},
  };
  transition_table_[Phase::VORONOI_PARTITION] = {
    {Phase::SEARCH, [this] {return guard_voronoi_partition_to_search();},
      [this] {action_begin_coverage_path_upload();}},
  };
  transition_table_[Phase::SEARCH] = {
    {Phase::RETURN, [this] {return guard_search_to_return();},
      [this] {action_disable_detection();}},
  };
  transition_table_[Phase::RETURN] = {
    {Phase::LAND, [this] {return guard_return_to_land();}, [this] {action_send_land();}},
  };
  // EXEC-006: a normal end recycles to PREFLIGHT for the next mission.
  transition_table_[Phase::LAND] = {
    {Phase::PREFLIGHT, [this] {return guard_land_to_preflight();},
      [this] {action_recycle_to_preflight();}},
  };
  // EXEC-006: FAULT is terminal — no rows, so every phase-transition request
  // is rejected (FR-002) and only a restart leaves it. check_fault_ended()
  // tells the GCS once the drone is safely down.
  transition_table_[Phase::FAULT] = {};

  any_transitions_ = {
    {Phase::RETURN, [this] {return guard_any_return();},
      [this] {action_cancel_active_mission();}},
    {Phase::FAULT, [this] {return guard_any_fault();}, [this] {action_command_rtl();}},
  };
}

// ---------------------------------------------------------------------------
// Guards
// ---------------------------------------------------------------------------

PreArmCheckInputs MissionManagerNode::build_pre_arm_check_inputs() const
{
  PreArmCheckInputs in;
  in.vehicle_state = vehicle_state_;
  in.mission_upload = mission_upload_;
  in.peer_states = peer_states_;
  in.arm_authorised = arm_authorised_;
  in.low_battery_margin = low_battery_margin_;
  in.visual_localisation_alive = local_pose_ever_received_ &&
    (now() - last_local_pose_at_).seconds() <= local_pose_stale_timeout_s_;
  return in;
}

PreArmCheckReport MissionManagerNode::pre_arm_check_report() const
{
  return evaluate_pre_arm_checks(build_pre_arm_check_inputs());
}

void MissionManagerNode::publish_pre_arm_status()
{
  const auto report = pre_arm_check_report();
  if (report.failed_checks == last_reported_pre_arm_failures_) {
    return;
  }
  last_reported_pre_arm_failures_ = report.failed_checks;
  for (const auto & reason : report.failed_checks) {
    RCLCPP_WARN(get_logger(), "pre-arm check failed: %s", reason.c_str());
    mc_msgs::msg::StatusText msg;
    msg.header.stamp = now();
    msg.severity = mc_msgs::msg::StatusText::SEVERITY_WARNING;
    msg.text = reason;
    pub_status_text_->publish(msg);
  }
}

bool MissionManagerNode::guard_preflight_to_arming() const
{
  return arm_authorised_ && pre_arm_check_report().passed;
}

bool MissionManagerNode::guard_arming_to_takeoff() const
{
  return vehicle_state_ && vehicle_state_->arming_state == kArmingStateArmed;
}

bool MissionManagerNode::guard_takeoff_to_ingress() const
{
  // Both conjuncts of SDD §4.1's "FCC at target altitude AND GCS INGRESS
  // command" — the second is only ever true for the single evaluation pass
  // triggered by that exact GCS command (see gcs_phase_request_'s doc
  // comment), so this transition never fires from the autonomous sweep.
  if (gcs_phase_request_ != Phase::INGRESS) {
    return false;
  }
  if (!vehicle_state_) {
    return false;
  }
  // Nothing to upload as the INGRESS mission otherwise. Pre-arm checks
  // already require it, so this only matters for a malformed upload.
  if (!flown_mission_upload_ || flown_mission_upload_->ingress_waypoints.empty()) {
    return false;
  }
  const double altitude_agl = -static_cast<double>(vehicle_state_->z);
  return std::abs(altitude_agl - takeoff_altitude_m_) <= takeoff_altitude_tolerance_m_;
}

bool MissionManagerNode::guard_ingress_to_muster() const
{
  if (!vehicle_state_ || !flown_mission_upload_ ||
    flown_mission_upload_->ingress_waypoints.empty())
  {
    return false;
  }
  // ingress_waypoints carry lat/lon in degrees in position.x/.y (both
  // gcs_mavlink_node and mission_orchestrator_node decode them that way),
  // so compare against the vehicle's own lat/lon, not its local NED x/y.
  const auto & muster_wp = flown_mission_upload_->ingress_waypoints.back().pose.position;
  return horizontal_distance_m(
    vehicle_state_->lat, vehicle_state_->lon, muster_wp.x, muster_wp.y) <=
         waypoint_acceptance_radius_m_;
}

bool MissionManagerNode::guard_muster_to_isr_selection() const
{
  int alive_in_muster = 0;
  if (peer_states_) {
    for (const auto & peer : peer_states_->peers) {
      if (peer.peer_status != mc_msgs::msg::SwarmPeer::STATUS_ALIVE) {
        continue;
      }
      if (peer.mission_phase != static_cast<uint8_t>(Phase::MUSTER)) {
        return false;  // an ALIVE peer is not (yet) in MUSTER
      }
      ++alive_in_muster;
    }
  }
  const int quorum = alive_in_muster + 1;  // +1 for self, already in MUSTER
  return quorum >= muster_quorum_count_;
}

bool MissionManagerNode::guard_isr_selection_to_voronoi_partition() const
{
  return isr_role_ && isr_role_->all_peers_acked;
}

bool MissionManagerNode::guard_voronoi_partition_to_search() const
{
  if (!voronoi_cell_ || !coverage_path_ || coverage_path_->poses.empty()) {
    return false;
  }
  if (mission_upload_ && voronoi_cell_->upload_id != mission_upload_->upload_id) {
    return false;  // stale cell from a previous mission upload
  }
  return true;
}

bool MissionManagerNode::guard_search_to_return() const
{
  // GCS END_MISSION is handled by the Any -> RETURN overlay (guard_any_
  // return()), which applies from every phase including SEARCH — this
  // guard covers only the phase-specific "FCC mission complete" half of
  // SDD §4.1's "FCC mission complete OR GCS END_MISSION" row.
  return vehicle_state_ && vehicle_state_->nav_state == kNavStateAutoLoiter;
}

bool MissionManagerNode::guard_return_to_land() const
{
  if (!vehicle_state_) {
    return false;
  }
  // PX4's local NED origin is set at EKF initialisation (~home position),
  // so distance to (0,0) approximates "at home waypoint" without a
  // separate home-position topic.
  const double dx = static_cast<double>(vehicle_state_->x);
  const double dy = static_cast<double>(vehicle_state_->y);
  return std::sqrt(dx * dx + dy * dy) <= home_position_tolerance_m_;
}

bool MissionManagerNode::fcc_landed_and_disarmed() const
{
  // DEP-009: PX4 v1.17 arming_state is DISARMED=1/ARMED=2 only (ICD §10.2).
  constexpr uint8_t kArmingStateDisarmed = 1;
  return vehicle_state_ && vehicle_state_->arming_state == kArmingStateDisarmed &&
         std::abs(static_cast<double>(vehicle_state_->vz)) <= landed_max_vertical_speed_mps_;
}

bool MissionManagerNode::guard_land_to_preflight() const
{
  return fcc_landed_and_disarmed();
}

bool MissionManagerNode::guard_any_return() const
{
  return return_requested_latched_ && current_phase_ != Phase::FAULT;
}

bool MissionManagerNode::guard_any_fault() const
{
  return critical_fault_latched_;
}

// ---------------------------------------------------------------------------
// Entry actions
// ---------------------------------------------------------------------------

void MissionManagerNode::action_enter_arming()
{
  // SDD §4.1's abridged entry action text is just "Log entry; notify
  // swarm" — but FR-010 ("MC shall arm the FCC only after GCS ARM
  // authorisation ... and pre-arm checks pass") and FR-012 ("mission_
  // manager_node is the sole node authorised to issue a flight-mode
  // command") both put actually arming the FCC squarely in this node's
  // job, and guard_preflight_to_arming() has already confirmed both
  // conditions — so this fills what looks like an omission in the
  // abridged table (flagged as a follow-up in the ticket summary) rather
  // than leaving ARMING unreachable in practice.
  mc_msgs::msg::FccCommand cmd;
  cmd.header.stamp = now();
  cmd.cmd = kMavCmdComponentArmDisarm;
  cmd.params = {1.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F};
  pub_send_command_->publish(cmd);
  flown_mission_upload_ = mission_upload_;
  action_notify_swarm();
}

void MissionManagerNode::action_notify_swarm()
{
  // The mission_phase publish that follows every commit_transition() call
  // *is* the notification other nodes (swarm_coordinator, formation_
  // controller, gcs_mavlink) key off of (ICD §9.1) — nothing further to do.
  RCLCPP_INFO(get_logger(), "phase entry: notifying swarm via /mc/mission_phase");
}

void MissionManagerNode::action_send_takeoff()
{
  mc_msgs::msg::FccCommand cmd;
  cmd.header.stamp = now();
  cmd.cmd = kMavCmdNavTakeoff;
  // param5/6 (lat/lon) must be NaN, not 0: PX4 v1.17 navigator_main.cpp
  // only takes off in place when both are non-finite -- a finite (0, 0) is
  // an explicit target at Null Island, and on the bench the vehicle armed,
  // made no thrust, and auto-disarmed ~11 s later.
  //
  // param7 is AMSL to PX4, but takeoff_altitude_m_ means "above home" --
  // the same frame guard_takeoff_to_ingress() checks it in (-z). NEAR-730:
  // sending it raw meant PX4 SITL (home ~490 m AMSL) never climbed
  // ("Already higher than takeoff altitude"). alt_msl + z is the local
  // NED origin's own AMSL altitude (alt_msl = origin_amsl - z), so this
  // target lands exactly where the guard's -z reaches takeoff_altitude_m_,
  // whether or not the vehicle is still sitting at z = 0. vehicle_state_ is
  // always set here -- guard_arming_to_takeoff() requires it.
  const double origin_amsl =
    static_cast<double>(vehicle_state_->alt_msl) + static_cast<double>(vehicle_state_->z);
  cmd.params = {0.F, 0.F, 0.F, NAN, NAN, NAN, static_cast<float>(origin_amsl + takeoff_altitude_m_)};
  pub_send_command_->publish(cmd);
}

void MissionManagerNode::send_ingress_route_upload()
{
  // The plan's ingress_waypoints become the FCC mission for INGRESS, flown
  // in MISSION mode once send_mission_start() switches PX4 into it. This
  // replaces the old one-item FORMATION_SLOT placeholder at the vehicle's
  // own position, which formation_controller_node (SC-001) was meant to
  // overwrite but never does -- so the vehicle just held position. SC-001's
  // follower behaviour is still to come; until then every drone flies the
  // plan's route itself.
  //
  // position.x/.y are lat/lon (deg), as both GCS decoders store them.
  // position.z is metres above home, the same frame as takeoff_altitude_m_
  // (NEAR-730), so it goes up as MAV_FRAME_GLOBAL_RELATIVE_ALT_INT rather
  // than AMSL -- sent as AMSL, a 50 m waypoint at a ~490 m AMSL SITL home
  // would be a target underground.
  mc_msgs::msg::FccMission mission;
  mission.header.stamp = now();
  mission.upload_id = pending_upload_id_;

  uint16_t seq = 0;
  for (const auto & wp : flown_mission_upload_->ingress_waypoints) {
    mc_msgs::msg::FccMissionItem item;
    item.header.stamp = now();
    item.seq = seq++;
    item.frame = kMavFrameGlobalRelativeAltInt;
    item.command = kMavCmdNavWaypoint;
    item.lat = static_cast<int32_t>(std::lround(wp.pose.position.x * 1e7));
    item.lon = static_cast<int32_t>(std::lround(wp.pose.position.y * 1e7));
    item.alt = static_cast<float>(wp.pose.position.z);
    item.acceptance_radius = static_cast<float>(waypoint_acceptance_radius_m_);
    mission.items.push_back(item);
  }
  pub_mission_upload_->publish(mission);

  pending_ack_kind_ = AckKind::INGRESS_ROUTE_UPLOAD;
  pending_ack_sent_at_ = now();
}

void MissionManagerNode::action_begin_ingress_route_upload()
{
  pending_target_phase_ = Phase::INGRESS;
  pending_source_ = mc_msgs::msg::MissionPhase::SOURCE_GCS_COMMAND;
  pending_ack_retries_ = 0;
  pending_upload_id_ = next_upload_id_++;
  send_ingress_route_upload();
}

void MissionManagerNode::send_mission_start()
{
  mc_msgs::msg::FccCommand cmd;
  cmd.header.stamp = now();
  cmd.cmd = kMavCmdMissionStart;
  // param1 = first item; only param1 is read. PX4 v1.17's Commander
  // accepts -1..seq_total-1 and switches to AUTO_MISSION; its navigator
  // then jumps to that item if it's >= 0. So the first send starts at item
  // 0 and resends use -1, which leaves the current item alone -- a resend
  // that lands after PX4 did switch (the ack or vehicle_state was just
  // slow) must not rewind the route.
  const float first_item = mission_start_attempts_ == 0 ? 0.F : -1.F;
  cmd.params = {first_item, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F};
  pub_send_command_->publish(cmd);
  ++mission_start_attempts_;
  mission_start_sent_at_ = now();
}

void MissionManagerNode::check_mission_start()
{
  if (!mission_start_pending_) {
    return;
  }
  if (current_phase_ != Phase::INGRESS) {
    mission_start_pending_ = false;
    return;
  }
  if (vehicle_state_ && vehicle_state_->nav_state == kNavStateAutoMission) {
    RCLCPP_INFO(get_logger(), "INGRESS: FCC in MISSION mode, flying ingress route");
    mission_start_pending_ = false;
    return;
  }
  if ((now() - mission_start_sent_at_).seconds() < kMissionStartRetryPeriodS) {
    return;
  }
  if (mission_start_attempts_ >= kMissionStartMaxAttempts) {
    // Not a FAULT: the vehicle is holding safely, and the GCS can still
    // start the mission or abort. Just make sure the operator knows.
    mission_start_pending_ = false;
    RCLCPP_ERROR(
      get_logger(), "INGRESS: FCC not in MISSION mode after %d MISSION_START attempts",
      mission_start_attempts_);
    mc_msgs::msg::StatusText msg;
    msg.header.stamp = now();
    msg.severity = mc_msgs::msg::StatusText::SEVERITY_CRITICAL;
    // ICD §4.7: the MAVLink STATUSTEXT relay truncates to 50 chars.
    msg.text = "INGRESS: FCC rejected MISSION_START, holding";
    pub_status_text_->publish(msg);
    return;
  }
  RCLCPP_WARN(
    get_logger(), "INGRESS: FCC not in MISSION mode yet, resending MISSION_START (%d/%d)",
    mission_start_attempts_ + 1, kMissionStartMaxAttempts);
  send_mission_start();
}

void MissionManagerNode::action_disable_formation_ctrl()
{
  RCLCPP_INFO(get_logger(), "phase entry: disabling formation control, notifying GCS");
}

void MissionManagerNode::action_disable_detection()
{
  RCLCPP_INFO(get_logger(), "phase entry: disabling detection pipeline");
}

void MissionManagerNode::send_coverage_path_upload()
{
  mc_msgs::msg::FccMission mission;
  mission.header.stamp = now();
  mission.upload_id = pending_upload_id_;

  // SDD §7.1 step 7 / §7.3: search_planner_node back-projects to geodetic
  // before publishing /mc/search/coverage_path, so poses arrive already in
  // degrees — but nav_msgs/Path's Pose has no dedicated lat/lon fields, and
  // neither SDD nor ICD documents which of x/y holds which. Assumed here
  // (ENU-style convention): position.x = lon (deg), position.y = lat (deg),
  // position.z = alt_msl (m). Flagged for ICD-owner ratification, same as
  // this repo's other locally-authored "gap note" message definitions.
  uint16_t seq = 0;
  for (const auto & pose : coverage_path_->poses) {
    mc_msgs::msg::FccMissionItem item;
    item.header.stamp = now();
    item.seq = seq++;
    item.frame = kMavFrameGlobalInt;
    item.command = kMavCmdNavWaypoint;
    item.lat = static_cast<int32_t>(pose.pose.position.y * 1e7);
    item.lon = static_cast<int32_t>(pose.pose.position.x * 1e7);
    item.alt = static_cast<float>(pose.pose.position.z);
    item.acceptance_radius = static_cast<float>(waypoint_acceptance_radius_m_);
    mission.items.push_back(item);
  }
  pub_mission_upload_->publish(mission);

  pending_ack_kind_ = AckKind::COVERAGE_PATH_UPLOAD;
  pending_ack_sent_at_ = now();
}

void MissionManagerNode::action_begin_coverage_path_upload()
{
  pending_target_phase_ = Phase::SEARCH;
  pending_source_ = mc_msgs::msg::MissionPhase::SOURCE_AUTONOMOUS;
  pending_ack_retries_ = 0;
  pending_upload_id_ = next_upload_id_++;
  send_coverage_path_upload();
  RCLCPP_INFO(get_logger(), "phase entry: uploading coverage path, enabling detection");
}

void MissionManagerNode::action_send_land()
{
  mc_msgs::msg::FccCommand cmd;
  cmd.header.stamp = now();
  cmd.cmd = kMavCmdNavLand;
  // NaN lat/lon = "current position" per MAVLink. PX4 v1.17 ignores LAND's
  // param5/6 anyway, but (0, 0) would mean Null Island to anything that
  // doesn't -- same trap as action_send_takeoff().
  cmd.params = {0.F, 0.F, 0.F, NAN, NAN, NAN, 0.F};
  pub_send_command_->publish(cmd);
}

void MissionManagerNode::action_log_mission_complete()
{
  RCLCPP_INFO(get_logger(), "mission complete: FCC disarmed and on ground");
}

void MissionManagerNode::action_recycle_to_preflight()
{
  action_log_mission_complete();
  reset_mission_state();
}

void MissionManagerNode::reset_mission_state()
{
  // Only drop the upload that was actually flown. One received mid-mission
  // (a different message, so a different pointer) is already the next
  // mission's, and its own arm_authorised is as current as FR-010 requires.
  if (mission_upload_ && mission_upload_ == flown_mission_upload_) {
    mission_upload_.reset();
    if (!remove_mission_upload(mission_upload_path_)) {
      RCLCPP_WARN(
        get_logger(), "failed to remove persisted mission upload %s",
        mission_upload_path_.c_str());
    }
  }
  flown_mission_upload_.reset();
  arm_authorised_ = mission_upload_ && mission_upload_->arm_authorised;

  return_requested_latched_ = false;
  critical_fault_latched_ = false;
  pending_ack_kind_ = AckKind::NONE;
  pending_ack_retries_ = 0;
  mission_start_pending_ = false;
  isr_role_.reset();
  voronoi_cell_.reset();
  coverage_path_.reset();
  // Re-report every pre-arm failure for the new mission, even if the set is
  // unchanged from the last PREFLIGHT.
  last_reported_pre_arm_failures_.clear();
  RCLCPP_INFO(
    get_logger(), "mission state reset for next mission (%s)",
    mission_upload_ ? "next upload already received" : "awaiting new upload");
}

void MissionManagerNode::check_fault_ended()
{
  if (fault_restart_notice_sent_ || !fcc_landed_and_disarmed()) {
    return;
  }
  fault_restart_notice_sent_ = true;
  RCLCPP_ERROR(
    get_logger(), "fault-ended mission: FCC landed and disarmed — restart required");
  mc_msgs::msg::StatusText msg;
  msg.header.stamp = now();
  msg.severity = mc_msgs::msg::StatusText::SEVERITY_CRITICAL;
  // ICD §4.7: the MAVLink STATUSTEXT relay truncates to 50 chars.
  msg.text = "FAULT: landed+disarmed, restart required";
  pub_status_text_->publish(msg);
}

void MissionManagerNode::send_rtl_command()
{
  mc_msgs::msg::FccCommand cmd;
  cmd.header.stamp = now();
  cmd.cmd = kMavCmdNavReturnToLaunch;
  cmd.params = {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F};
  pub_send_command_->publish(cmd);
}

void MissionManagerNode::action_cancel_active_mission()
{
  send_rtl_command();
  return_requested_latched_ = false;
}

void MissionManagerNode::action_command_rtl()
{
  send_rtl_command();
  critical_fault_latched_ = false;
}

// ---------------------------------------------------------------------------
// Transition engine
// ---------------------------------------------------------------------------

bool MissionManagerNode::evaluate_any_transitions()
{
  for (auto & t : any_transitions_) {
    if (current_phase_ == t.target) {
      continue;
    }
    if (t.guard()) {
      t.action();
      commit_transition(t.target, mc_msgs::msg::MissionPhase::SOURCE_FAILSAFE);
      return true;
    }
  }
  return false;
}

namespace
{
const char * phase_name(Phase p)
{
  switch (p) {
    case Phase::PREFLIGHT: return "PREFLIGHT";
    case Phase::ARMING: return "ARMING";
    case Phase::TAKEOFF: return "TAKEOFF";
    case Phase::INGRESS: return "INGRESS";
    case Phase::MUSTER: return "MUSTER";
    case Phase::ISR_SELECTION: return "ISR_SELECTION";
    case Phase::VORONOI_PARTITION: return "VORONOI_PARTITION";
    case Phase::SEARCH: return "SEARCH";
    case Phase::RETURN: return "RETURN";
    case Phase::LAND: return "LAND";
    case Phase::FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}
}  // namespace

MissionManagerNode::TransitionResult MissionManagerNode::try_transition(
  Phase target, bool from_gcs_command)
{
  if (evaluate_any_transitions()) {
    return TransitionResult::COMMITTED;
  }

  last_reject_reason_ = std::string("no transition ") + phase_name(current_phase_) + " -> " +
    phase_name(target);
  const auto it = transition_table_.find(current_phase_);
  if (it == transition_table_.end()) {
    return TransitionResult::NO_MATCH;
  }
  const auto row = std::find_if(
    it->second.begin(), it->second.end(),
    [target](const Transition & t) {return t.target == target;});
  if (row == it->second.end()) {
    // FR-002: undefined transitions are rejected outright, phase unchanged.
    RCLCPP_WARN(
      get_logger(), "rejected phase transition request: no row %u -> %u in table",
      static_cast<unsigned>(current_phase_), static_cast<unsigned>(target));
    return TransitionResult::REJECTED;
  }

  if (from_gcs_command) {
    gcs_phase_request_ = target;
  }
  const bool guard_ok = row->guard();
  gcs_phase_request_.reset();

  if (!guard_ok) {
    // PREFLIGHT -> ARMING's guard is the pre-arm check set; name the first
    // failing check so the GCS operator sees why (TEST-007 finding).
    const auto report = current_phase_ == Phase::PREFLIGHT && target == Phase::ARMING ?
      pre_arm_check_report() : PreArmCheckReport{};
    last_reject_reason_ = !report.failed_checks.empty() ? report.failed_checks.front() :
      std::string("guard failed ") + phase_name(current_phase_) + " -> " + phase_name(target);
    RCLCPP_WARN(
      get_logger(), "rejected phase transition request: guard failed for %u -> %u",
      static_cast<unsigned>(current_phase_), static_cast<unsigned>(target));
    return TransitionResult::REJECTED;
  }

  row->action();
  if (pending_ack_kind_ != AckKind::NONE) {
    return TransitionResult::PENDING_ACK;
  }
  commit_transition(
    target,
    from_gcs_command ? mc_msgs::msg::MissionPhase::SOURCE_GCS_COMMAND :
    mc_msgs::msg::MissionPhase::SOURCE_AUTONOMOUS);
  return TransitionResult::COMMITTED;
}

void MissionManagerNode::evaluate_autonomous_transitions()
{
  if (evaluate_any_transitions()) {
    return;
  }
  if (pending_ack_kind_ != AckKind::NONE) {
    return;  // mid-flight on a mission-channel request; wait for its ack
  }
  const auto it = transition_table_.find(current_phase_);
  if (it == transition_table_.end()) {
    return;
  }
  for (const auto & row : it->second) {
    if (row.guard()) {
      row.action();
      if (pending_ack_kind_ != AckKind::NONE) {
        return;  // action itself started an ack-blocking request
      }
      commit_transition(row.target, mc_msgs::msg::MissionPhase::SOURCE_AUTONOMOUS);
      return;
    }
  }
}

void MissionManagerNode::commit_transition(Phase target, uint8_t source)
{
  // An Any -> RETURN/FAULT overlay transition can preempt a phase with a
  // mission-channel request still in flight (TAKEOFF, VORONOI_PARTITION).
  // Drop that bookkeeping here rather than leaving it live — otherwise a
  // late FccMissionAck for the now-abandoned request would later hijack
  // on_mission_channel_ack_success() into committing back to the
  // preempted request's target phase, overriding the fault/abort.
  if (pending_ack_kind_ != AckKind::NONE && target != pending_target_phase_) {
    RCLCPP_WARN(
      get_logger(), "phase transition to %u preempted an in-flight mission-channel request",
      static_cast<unsigned>(target));
    pending_ack_kind_ = AckKind::NONE;
  }

  RCLCPP_INFO(
    get_logger(), "phase transition: %u -> %u (source=%u)",
    static_cast<unsigned>(current_phase_), static_cast<unsigned>(target), source);
  current_phase_ = target;
  phase_entered_at_ = now();
  publish_phase(/*force=*/ true);
}

void MissionManagerNode::publish_phase(bool force)
{
  const auto elapsed = (now() - last_phase_publish_at_).seconds();
  if (!force && phase_published_once_ && elapsed < 1.0) {
    return;
  }
  mc_msgs::msg::MissionPhase msg;
  msg.header.stamp = now();
  msg.phase = static_cast<uint8_t>(current_phase_);
  msg.phase_name = phase_name(current_phase_);
  msg.phase_entered_us =
    static_cast<uint64_t>(phase_entered_at_.nanoseconds() / 1000);
  pub_mission_phase_->publish(msg);
  last_phase_publish_at_ = now();
  phase_published_once_ = true;
}

// ---------------------------------------------------------------------------
// Mission-channel ack-blocking (FCC-006)
// ---------------------------------------------------------------------------

void MissionManagerNode::on_mission_channel_ack_success()
{
  const Phase target = pending_target_phase_;
  const uint8_t source = pending_source_;
  const AckKind kind = pending_ack_kind_;
  pending_ack_kind_ = AckKind::NONE;
  commit_transition(target, source);
  if (kind == AckKind::INGRESS_ROUTE_UPLOAD) {
    mission_start_attempts_ = 0;
    mission_start_pending_ = true;
    send_mission_start();
  }
}

void MissionManagerNode::on_mission_channel_ack_failure()
{
  ++pending_ack_retries_;
  if (pending_ack_retries_ > mission_ack_max_retries_) {
    RCLCPP_ERROR(
      get_logger(), "mission-channel request failed after %d retries; escalating to FAULT",
      pending_ack_retries_ - 1);
    pending_ack_kind_ = AckKind::NONE;
    if (current_phase_ != Phase::FAULT) {
      announce_fault("FCC mission channel request failed");
    }
    critical_fault_latched_ = true;
    evaluate_any_transitions();
    return;
  }
  RCLCPP_WARN(
    get_logger(), "mission-channel request failed, retry %d/%d", pending_ack_retries_,
    mission_ack_max_retries_);
  if (pending_ack_kind_ == AckKind::INGRESS_ROUTE_UPLOAD) {
    send_ingress_route_upload();
  } else if (pending_ack_kind_ == AckKind::COVERAGE_PATH_UPLOAD) {
    send_coverage_path_upload();
  }
}

void MissionManagerNode::check_ack_timeout()
{
  if (pending_ack_kind_ == AckKind::NONE) {
    return;
  }
  const auto elapsed = (now() - pending_ack_sent_at_).seconds();
  if (elapsed > mission_ack_timeout_s_) {
    on_mission_channel_ack_failure();
  }
}

// ---------------------------------------------------------------------------
// Topic callbacks
// ---------------------------------------------------------------------------

void MissionManagerNode::on_vehicle_state(mc_msgs::msg::VehicleState::SharedPtr msg)
{
  vehicle_state_ = msg;
}

void MissionManagerNode::on_mission_ack(mc_msgs::msg::FccMissionAck::SharedPtr msg)
{
  if (pending_ack_kind_ == AckKind::NONE) {
    return;
  }
  // FCC-010: both AckKind values this node tracks are full mission uploads
  // at the wire level (see the enum's own comment) — pending_ack_kind_ only
  // picks which local send_*()/retry function to call on failure, not which
  // wire-level ack shape to expect.
  if (msg->request_type != mc_msgs::msg::FccMissionAck::UPLOAD ||
    msg->upload_id != pending_upload_id_)
  {
    return;
  }

  if (msg->success) {
    on_mission_channel_ack_success();
  } else {
    on_mission_channel_ack_failure();
  }
}

void MissionManagerNode::on_gcs_mission_upload(mc_msgs::msg::MissionUpload::SharedPtr msg)
{
  mission_upload_ = msg;
  arm_authorised_ = msg->arm_authorised;

  // EXEC-004 / SDD §4.5: persist every accepted upload so a later watchdog
  // restart can reload it (see the constructor's load_mission_upload() call)
  // instead of requiring a fresh GCS upload. Best-effort: a write failure
  // (e.g. disk full) doesn't block accepting the upload for the current
  // session, it just means a restart during this session would lose it.
  if (!save_mission_upload(mission_upload_path_, *msg)) {
    RCLCPP_WARN(
      get_logger(), "failed to persist mission upload to %s", mission_upload_path_.c_str());
  }
}

void MissionManagerNode::on_peer_states(mc_msgs::msg::SwarmPeerArray::SharedPtr msg)
{
  peer_states_ = msg;
}

void MissionManagerNode::on_isr_role(mc_msgs::msg::IsrRole::SharedPtr msg)
{
  isr_role_ = msg;
}

void MissionManagerNode::on_voronoi_cell(mc_msgs::msg::VoronoiCell::SharedPtr msg)
{
  voronoi_cell_ = msg;
}

void MissionManagerNode::on_coverage_path(nav_msgs::msg::Path::SharedPtr msg)
{
  coverage_path_ = msg;
}

bool MissionManagerNode::is_critical_fault(const mc_msgs::msg::FaultEvent & fault) const
{
  // Deliberately keyed off /mc/fault content only — never re-derives link
  // health from /mc/fcc/vehicle_state.fcc_link_ok, per this ticket's own
  // acceptance criterion (health_monitor_node owns fault detection; this
  // node only reacts to what it publishes, SDD §10.3 ownership split).
  if (fault.fault_id == mc_msgs::msg::FaultEvent::FCC_LINK_LOST) {
    return true;
  }
  return fault.severity <= kSeverityCritical;
}

void MissionManagerNode::announce_fault(const std::string & cause)
{
  // Says why the phase is about to become FAULT -- the transition itself
  // and check_fault_ended()'s restart notice don't. gcs_mavlink_node
  // chunks texts over 50 chars, so the fault's description goes out whole.
  RCLCPP_ERROR(get_logger(), "entering FAULT: %s", cause.c_str());
  mc_msgs::msg::StatusText msg;
  msg.header.stamp = now();
  msg.severity = mc_msgs::msg::StatusText::SEVERITY_CRITICAL;
  msg.text = "FAULT: " + cause;
  pub_status_text_->publish(msg);
}

void MissionManagerNode::on_fault(mc_msgs::msg::FaultEvent::SharedPtr msg)
{
  if (current_phase_ == Phase::FAULT) {
    return;
  }
  if (is_critical_fault(*msg)) {
    std::string cause = fault_name(msg->fault_id);
    if (!msg->description.empty()) {
      cause += ": " + msg->description;
    }
    announce_fault(cause);
    critical_fault_latched_ = true;
    evaluate_any_transitions();
  }
}

void MissionManagerNode::publish_command_ack(uint32_t command, TransitionResult result)
{
  mc_msgs::msg::GcsCommandAck ack;
  ack.header.stamp = now();
  ack.command = command;
  if (result == TransitionResult::REJECTED || result == TransitionResult::NO_MATCH) {
    ack.result = mc_msgs::msg::GcsCommandAck::RESULT_REJECTED;
    ack.reason = last_reject_reason_;
  } else {
    // PENDING_ACK just means the FCC-side mission-channel follow-through
    // (FCC-006) is still in flight — the GCS's own request was accepted.
    ack.result = mc_msgs::msg::GcsCommandAck::RESULT_ACCEPTED;
  }
  pub_command_ack_->publish(ack);
}

void MissionManagerNode::handle_gcs_command(const GcsCommandEnvelope & cmd)
{
  // SDD §4.3 priority 1 / EXEC-003: emergency commands bypass the
  // transition table and execute immediately. TP-UT-012 confirms this also
  // forces the phase to FAULT (not just a raw pass-through) — the same
  // forward-and-fault behaviour now covers every command classify_command_
  // tier() calls EMERGENCY (DISARM, RTL, and — new in EXEC-003 — MANUAL/
  // ALTCTL/POSCTL/STABILIZED via MAV_CMD_DO_SET_MODE), not just DISARM/RTL.
  if (classify_command_tier(cmd.command, cmd.params) == CommandTier::EMERGENCY) {
    mc_msgs::msg::FccCommand fwd;
    fwd.header.stamp = now();
    fwd.cmd = cmd.command;
    fwd.params = cmd.params;
    pub_send_command_->publish(fwd);
    if (current_phase_ != Phase::FAULT) {
      announce_fault("GCS emergency command " + std::to_string(cmd.command));
      commit_transition(Phase::FAULT, mc_msgs::msg::MissionPhase::SOURCE_GCS_COMMAND);
    }
    return;
  }

  switch (cmd.command) {
    case kMcCmdPhaseTransition: {
        const auto target = static_cast<Phase>(
          static_cast<uint8_t>(std::lround(cmd.params[0])));
        const auto result = try_transition(target, /*from_gcs_command=*/ true);
        publish_command_ack(cmd.command, result);
        break;
      }
    case kMcCmdArmAuthorise:
      arm_authorised_ = cmd.params[0] != 0.0F;
      break;
    case kMcCmdEndMission:
      // Satisfies both SDD §4.1 rows that name a GCS command: SEARCH's own
      // "GCS END_MISSION" and the Any -> RETURN "GCS ABORT command" — the
      // command catalogue (ICD §10.5) has no separate raw ID for "ABORT",
      // and both describe the same GCS-initiated, phase-independent stop.
      return_requested_latched_ = true;
      evaluate_any_transitions();
      break;
    case kMcCmdGcsOverride:
      // Out of scope here: OVERRIDE_LOW_BATTERY_RTL/OVERRIDE_GCS_LINK_HOLD
      // gate health_monitor_node's own failsafe decision tree (SDD §10.3),
      // not this node's transition table.
      RCLCPP_INFO(get_logger(), "MC_CMD_GCS_OVERRIDE received (health_monitor's concern)");
      break;
    default:
      RCLCPP_WARN(get_logger(), "unrecognised /mc/gcs/command cmd=%u", cmd.command);
      break;
  }
}

void MissionManagerNode::on_gcs_command(mc_msgs::msg::GcsCommand::SharedPtr msg)
{
  // Enqueue only — see this method's own doc comment in the header for why
  // draining is deferred to the control timer rather than handled here.
  command_arbiter_.enqueue({msg->command, msg->params});
}

void MissionManagerNode::drain_gcs_command_queue()
{
  while (const auto cmd = command_arbiter_.pop_next()) {
    handle_gcs_command(*cmd);
  }
}

void MissionManagerNode::on_control_timer()
{
  drain_gcs_command_queue();
  check_ack_timeout();
  check_mission_start();
  if (current_phase_ == Phase::PREFLIGHT) {
    publish_pre_arm_status();
  }
  // SDD §4.3 priority 3: autonomous transitions are evaluated only when no
  // GCS command is pending. drain_gcs_command_queue() above already empties
  // command_arbiter_ every tick, so this is always true by construction —
  // kept explicit anyway so the third priority tier is visible in code, not
  // merely implied by call order.
  if (command_arbiter_.autonomous_transitions_allowed()) {
    evaluate_autonomous_transitions();
  }

  if (current_phase_ == Phase::FAULT) {
    check_fault_ended();
  }

  publish_phase(/*force=*/ false);

  // Heartbeat is the last thing published each cycle (SDD §4.4): if it
  // publishes, everything above it this cycle completed successfully.
  std_msgs::msg::Header hb;
  hb.stamp = now();
  hb.frame_id = "mission_manager_node";
  pub_heartbeat_->publish(hb);
}

}  // namespace mission_exec
