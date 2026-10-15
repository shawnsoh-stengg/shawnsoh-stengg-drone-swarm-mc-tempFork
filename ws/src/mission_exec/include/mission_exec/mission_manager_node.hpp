// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/fcc_command.hpp>
#include <mc_msgs/msg/fcc_mission.hpp>
#include <mc_msgs/msg/fcc_mission_ack.hpp>
#include <mc_msgs/msg/fcc_mission_item.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/gcs_command_ack.hpp>
#include <mc_msgs/msg/isr_role.hpp>
#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <mc_msgs/msg/status_text.hpp>
#include <mc_msgs/msg/swarm_peer_array.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <mc_msgs/msg/voronoi_cell.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "mission_exec/command_arbiter.hpp"
#include "mission_exec/mission_persistence.hpp"
#include "mission_exec/pre_arm_checks.hpp"

namespace mission_exec
{

// EXEC-001: mission_manager_node — 11-phase mission state machine
// (SDD-MC-001 §4.1, FR-001-005/010-016/060-064). Table-driven: guards and
// entry actions live as data in transition_table_, not a switch statement,
// so the table is inspectable/testable independent of the dispatch
// mechanism (the ticket's own framing).
//
// Deliberately excludes SDD §4.1a's OFFBOARD/MISSION hybrid-mode
// arbitration (a separate, later addition layered on top of this same
// table — OFFBOARD-MODE-SWITCHING-BACKLOG.md) and RETURN/LAND phase
// detail beyond the abridged table's own two rows — both out of scope
// per this ticket's own "Explicitly out of scope" section.
//
// EXEC-006: LAND recycles to PREFLIGHT once the FCC is landed and disarmed,
// ready for the next mission. FAULT stays terminal: a faulted drone needs
// inspection and a restart (SRS FR-006/FR-007).
enum class Phase : uint8_t
{
  PREFLIGHT = 0,
  ARMING = 1,
  TAKEOFF = 2,
  INGRESS = 3,
  MUSTER = 4,
  ISR_SELECTION = 5,
  VORONOI_PARTITION = 6,
  SEARCH = 7,
  RETURN = 8,
  LAND = 9,
  FAULT = 255,
};

struct Transition
{
  Phase target;
  std::function<bool()> guard;
  std::function<void()> action;
};

using TransitionTable = std::vector<Transition>;

class MissionManagerNode : public rclcpp::Node
{
public:
  explicit MissionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Test seam: current phase and a few derived predicates, exercised
  // directly by TP-UT-010/011/012-style unit tests without needing a full
  // topic round-trip.
  Phase current_phase() const {return current_phase_;}
  // EXEC-002: runs all 7 pre_arm_checks.hpp checks against this node's own
  // latest received state — no short-circuit, so every failing condition
  // is reported at once, not just the first.
  PreArmCheckReport pre_arm_check_report() const;

private:
  // --- transition table construction ---
  void build_transition_table();

  // --- EXEC-002: pre-arm checks ---
  PreArmCheckInputs build_pre_arm_check_inputs() const;
  // Called every control-loop tick while in PREFLIGHT (i.e. at the 10 Hz
  // control_loop_period_s_ rate) — re-evaluates the full check suite and,
  // whenever the set of failing checks changes, republishes each reason as
  // an /mc/status_text StatusText (severity WARNING) and logs it. Not
  // throttled beyond "only on change" — a persistent failure is reported
  // once, not every tick, so it doesn't spam the operator.
  void publish_pre_arm_status();

  // --- guards (SDD §4.1 abridged table) ---
  bool guard_preflight_to_arming() const;
  bool guard_arming_to_takeoff() const;
  bool guard_takeoff_to_ingress() const;
  bool guard_ingress_to_muster() const;
  bool guard_muster_to_isr_selection() const;
  bool guard_isr_selection_to_voronoi_partition() const;
  bool guard_voronoi_partition_to_search() const;
  bool guard_search_to_return() const;
  bool guard_return_to_land() const;
  bool guard_land_to_preflight() const;
  // EXEC-006: VehicleState has no land-detector flag, so "landed" is
  // disarmed with |vz| under landed_max_vertical_speed_mps_. PX4 only
  // auto-disarms once its own land detector fires (COM_DISARM_LAND); the vz
  // check covers a disarm in the air (RC kill) so it doesn't count as
  // landed until the vehicle has stopped moving.
  bool fcc_landed_and_disarmed() const;
  // "Any -> ..." overlay guards (SDD §4.1's two Phase-less rows).
  bool guard_any_return() const;
  bool guard_any_fault() const;

  // --- entry actions ---
  void action_enter_arming();
  void action_notify_swarm();
  void action_send_takeoff();
  void action_begin_ingress_route_upload();
  void action_disable_formation_ctrl();
  void action_disable_detection();
  void action_begin_coverage_path_upload();
  void action_send_land();
  void action_log_mission_complete();
  // EXEC-006: LAND -> PREFLIGHT entry action — logs mission complete, then
  // reset_mission_state().
  void action_recycle_to_preflight();
  // EXEC-006: clears every per-mission member so a recycled PREFLIGHT
  // behaves like a fresh boot with no upload: FR-010's ARM authorisation
  // must come from a new upload, not the one just flown. An upload that
  // arrived *during* the mission (Channel B accepts uploads in any phase)
  // is kept, since that is already the next mission's.
  void reset_mission_state();
  // EXEC-006: FAULT is terminal. Once the FCC reports landed and disarmed
  // while in FAULT, tells the GCS once that a restart is needed.
  void check_fault_ended();
  void action_cancel_active_mission();
  void action_command_rtl();
  void send_rtl_command();

  // --- topic callbacks ---
  void on_vehicle_state(mc_msgs::msg::VehicleState::SharedPtr msg);
  void on_mission_ack(mc_msgs::msg::FccMissionAck::SharedPtr msg);
  void on_gcs_command(mc_msgs::msg::GcsCommand::SharedPtr msg);
  void on_gcs_mission_upload(mc_msgs::msg::MissionUpload::SharedPtr msg);
  void on_peer_states(mc_msgs::msg::SwarmPeerArray::SharedPtr msg);
  void on_isr_role(mc_msgs::msg::IsrRole::SharedPtr msg);
  void on_voronoi_cell(mc_msgs::msg::VoronoiCell::SharedPtr msg);
  void on_coverage_path(nav_msgs::msg::Path::SharedPtr msg);
  void on_fault(mc_msgs::msg::FaultEvent::SharedPtr msg);
  void on_control_timer();

  // --- transition engine ---
  // try_transition looks up `target` in the *current* phase's own table
  // (an explicit GCS PHASE_TRANSITION request, FR-002) — `from_gcs_command`
  // set true lets a guard that requires "GCS command received" (TAKEOFF ->
  // INGRESS) observe gcs_phase_request_. evaluate_autonomous_transitions()
  // instead walks the current phase's table looking for the first row
  // whose guard is already true with no GCS request involved (the periodic
  // sweep). Both consult any_transitions_ first (the two "Any -> ..." rows
  // — SDD §4.1 can't key these by Phase since "Any" isn't a Phase value).
  enum class TransitionResult { COMMITTED, PENDING_ACK, REJECTED, NO_MATCH };
  TransitionResult try_transition(Phase target, bool from_gcs_command);
  // EXEC-003: publishes the SDD §4.3 priority-2 outcome ack for a
  // PHASE_TRANSITION request — ACCEPTED for {COMMITTED, PENDING_ACK} (the
  // guard passed; PENDING_ACK just means the FCC-side follow-through is
  // still in flight), REJECTED for {REJECTED, NO_MATCH}.
  void publish_command_ack(uint32_t command, TransitionResult result);
  void evaluate_autonomous_transitions();
  bool evaluate_any_transitions();
  void commit_transition(Phase target, uint8_t source);
  void publish_phase(bool force);

  // --- mission-channel ack-blocking (FCC-006) ---
  // Both kinds are full mission uploads at the wire level (FCC-010: PX4 has
  // no MISSION_WRITE_PARTIAL_LIST handler at any known version, so every
  // mission change goes through fcc_bridge's upload_mission()/
  // `/mc/fcc/mission_upload`) — this enum only disambiguates which local
  // send_*()/retry function is pending, not the ack's wire type.
  enum class AckKind { NONE, INGRESS_ROUTE_UPLOAD, COVERAGE_PATH_UPLOAD };
  void send_ingress_route_upload();
  void send_coverage_path_upload();
  // An uploaded mission doesn't fly itself: PX4 stays in whatever mode it
  // was in (AUTO_LOITER after TAKEOFF) until MAV_CMD_MISSION_START switches
  // it to AUTO_MISSION. Sent once each upload acks; check_mission_start()
  // resends it a bounded number of times until vehicle_state reports
  // AUTO_MISSION, then stops for good, so a later GCS/pilot mode change
  // (FR-003 override authority) is never fought. A backend that doesn't
  // decode nav_state (MavlinkUdpBackend/SITL) just gets every resend, which
  // is harmless -- see send_mission_start().
  void send_mission_start();
  void check_mission_start();
  void check_ack_timeout();
  void on_mission_channel_ack_success();
  void on_mission_channel_ack_failure();

  // --- GCS command dispatch (SDD §4.3 / EXEC-003) ---
  // on_gcs_command() only enqueues (fast, non-blocking); drain_gcs_command_
  // queue() — called from on_control_timer(), before evaluate_autonomous_
  // transitions() — pops and executes every queued command in
  // command_arbiter_'s tier-priority order. Deferring execution to the
  // timer, rather than handling inline in the subscription callback, is
  // what lets a later-arriving EMERGENCY command still jump ahead of an
  // earlier-arriving, not-yet-processed PHASE command: within a single
  // rclcpp callback dispatch there is no way to "look ahead" at a second
  // already-queued message before deciding processing order, so reordering
  // requires batching at least one control-loop tick's worth of arrivals
  // before draining. The default 0.1 s control_loop_period_s_ leaves ample
  // headroom under both this ticket's 200 ms (Emergency) and 500 ms (Phase
  // ack) bounds.
  void drain_gcs_command_queue();
  void handle_gcs_command(const GcsCommandEnvelope & cmd);

  // --- fault handling (does not re-derive link health from vehicle_state) ---
  bool is_critical_fault(const mc_msgs::msg::FaultEvent & fault) const;

  // Params
  double control_loop_period_s_{0.1};
  double mission_ack_timeout_s_{6.0};
  int mission_ack_max_retries_{3};
  int muster_quorum_count_{1};
  double takeoff_altitude_m_{30.0};
  double takeoff_altitude_tolerance_m_{2.0};
  double waypoint_acceptance_radius_m_{5.0};
  double home_position_tolerance_m_{5.0};
  double low_battery_margin_{0.10};
  double local_pose_stale_timeout_s_{2.0};
  double landed_max_vertical_speed_mps_{0.5};
  // EXEC-004: SDD §4.5 mission upload disk persistence path.
  std::string mission_upload_path_{"/var/drone-mc/mission_upload.yaml"};

  // Transition table (SDD §4.1: std::unordered_map<Phase, TransitionTable>).
  std::unordered_map<Phase, TransitionTable> transition_table_;
  // Global "Any -> ..." overlay rows, checked before the per-phase table.
  std::vector<Transition> any_transitions_;

  Phase current_phase_{Phase::PREFLIGHT};
  rclcpp::Time phase_entered_at_;
  rclcpp::Time last_phase_publish_at_;
  bool phase_published_once_{false};

  // Latest received state, kept for guard evaluation.
  mc_msgs::msg::VehicleState::SharedPtr vehicle_state_;
  mc_msgs::msg::MissionUpload::SharedPtr mission_upload_;
  // EXEC-006: the upload in effect when PREFLIGHT -> ARMING committed, i.e.
  // the one being flown. reset_mission_state() compares against it to tell
  // a flown upload from one received mid-mission for the next mission.
  mc_msgs::msg::MissionUpload::SharedPtr flown_mission_upload_;
  mc_msgs::msg::SwarmPeerArray::SharedPtr peer_states_;
  mc_msgs::msg::IsrRole::SharedPtr isr_role_;
  mc_msgs::msg::VoronoiCell::SharedPtr voronoi_cell_;
  nav_msgs::msg::Path::SharedPtr coverage_path_;
  rclcpp::Time last_local_pose_at_;
  bool local_pose_ever_received_{false};

  // EXEC-002: last set of pre-arm failure reasons reported via
  // /mc/status_text — publish_pre_arm_status() only re-publishes when this
  // changes, per its own doc comment.
  std::vector<std::string> last_reported_pre_arm_failures_;

  // EXEC-003: SDD §4.3 three-tier command priority — see drain_gcs_
  // command_queue()'s own doc comment for why draining is deferred to the
  // control timer rather than handled inline in on_gcs_command().
  CommandArbiter command_arbiter_;

  // Transient flag: set for the duration of a single explicit GCS
  // PHASE_TRANSITION(target) evaluation so guards that require "GCS
  // command received" (TAKEOFF -> INGRESS) can observe it; cleared
  // immediately after that one evaluation.
  std::optional<Phase> gcs_phase_request_;

  // arm_authorised_ mirrors mission_upload_->arm_authorised (FR-010) but
  // can also be flipped live by MC_CMD_ARM_AUTHORISE (ICD §10.5, 50002)
  // without requiring a full mission re-upload.
  bool arm_authorised_{false};

  // Mission-channel ack-blocking bookkeeping (FCC-006). Only one mission-
  // channel request is ever in flight at a time — both transitions that
  // use one (TAKEOFF -> INGRESS, VORONOI_PARTITION -> SEARCH) are
  // themselves serialized by the state machine only ever being in one
  // phase at a time.
  AckKind pending_ack_kind_{AckKind::NONE};
  uint32_t pending_upload_id_{0};
  Phase pending_target_phase_{Phase::PREFLIGHT};
  uint8_t pending_source_{0};
  rclcpp::Time pending_ack_sent_at_;
  int pending_ack_retries_{0};

  uint32_t next_upload_id_{1};

  // send_mission_start()/check_mission_start() bookkeeping. pending_ is
  // true from the upload ack until AUTO_MISSION is observed, attempts run
  // out, or the phase changes.
  bool mission_start_pending_{false};
  int mission_start_attempts_{0};
  rclcpp::Time mission_start_sent_at_;

  // Latched by on_fault()/handle_gcs_command() respectively (event-driven,
  // not re-derived from other topics — see is_critical_fault()'s own doc
  // comment on why /mc/fcc/vehicle_state.fcc_link_ok is deliberately not
  // consulted here) and consumed by the any_transitions_ overlay both
  // immediately (from those same callbacks) and again each control-loop
  // tick as a safety net.
  bool critical_fault_latched_{false};
  bool return_requested_latched_{false};
  // EXEC-006: check_fault_ended()'s one-shot restart-required notice.
  bool fault_restart_notice_sent_{false};

  // Every callback that touches state-machine state (current_phase_,
  // pending_ack_*, the various latest-message members) shares one
  // mutually-exclusive group — unlike fcc_bridge_node's split
  // (mission/command/timer each independent), this node's guards and
  // actions all read/write the *same* shared state machine, so splitting
  // them across groups would just reintroduce the data race a single
  // group avoids, per the same reasoning fcc_bridge_node's own
  // mission_cb_group_ comment gives for its narrower case. Every callback
  // body here is non-blocking (guard checks, publishes; mission-ack
  // waiting is polled from the timer tick, never blocked on), so
  // serializing everything still leaves GCS-command handling well within
  // the 500 ms bound (FR-003).
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Publisher<mc_msgs::msg::MissionPhase>::SharedPtr pub_mission_phase_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<mc_msgs::msg::FccCommand>::SharedPtr pub_send_command_;
  rclcpp::Publisher<mc_msgs::msg::FccMission>::SharedPtr pub_mission_upload_;
  rclcpp::Publisher<mc_msgs::msg::StatusText>::SharedPtr pub_status_text_;
  rclcpp::Publisher<mc_msgs::msg::GcsCommandAck>::SharedPtr pub_command_ack_;

  rclcpp::Subscription<mc_msgs::msg::VehicleState>::SharedPtr sub_vehicle_state_;
  rclcpp::Subscription<mc_msgs::msg::FccMissionAck>::SharedPtr sub_mission_ack_;
  rclcpp::Subscription<mc_msgs::msg::GcsCommand>::SharedPtr sub_gcs_command_;
  rclcpp::Subscription<mc_msgs::msg::MissionUpload>::SharedPtr sub_gcs_mission_upload_;
  rclcpp::Subscription<mc_msgs::msg::SwarmPeerArray>::SharedPtr sub_peer_states_;
  rclcpp::Subscription<mc_msgs::msg::IsrRole>::SharedPtr sub_isr_role_;
  rclcpp::Subscription<mc_msgs::msg::VoronoiCell>::SharedPtr sub_voronoi_cell_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_coverage_path_;
  rclcpp::Subscription<mc_msgs::msg::FaultEvent>::SharedPtr sub_fault_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_local_pose_;
};

}  // namespace mission_exec
