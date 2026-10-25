// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <mc_msgs/msg/mission_upload.hpp>
#include <mc_msgs/msg/swarm_peer_array.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>

namespace mission_exec
{

// EXEC-002: Pre-arm check suite (SDD-MC-001 §4.2, FR-010) — the 7 conditions
// gating PREFLIGHT -> ARMING. Deliberately framework-free (no rclcpp::Node,
// no ROS clock): every check is a plain function over plain data, so each
// one is independently callable/unit-testable without spinning a node, per
// this ticket's own acceptance criteria. Time-based staleness (visual
// localisation freshness) is folded in by the caller as a single bool
// (mirrors mission_manager_node's own local-pose staleness check) rather
// than duplicating clock-reading logic in here.
struct PreArmCheckInputs
{
  mc_msgs::msg::VehicleState::ConstSharedPtr vehicle_state;
  mc_msgs::msg::MissionUpload::ConstSharedPtr mission_upload;
  // nullptr => no peer data received yet; check_swarm_peers_preflight()
  // passes vacuously in that case (nothing reported => nothing to fail).
  mc_msgs::msg::SwarmPeerArray::ConstSharedPtr peer_states;
  bool arm_authorised{false};
  double low_battery_margin{0.10};
  // Only consulted when vehicle_state->gnss_ok is false.
  bool visual_localisation_alive{false};
  // Only consulted when vehicle_state->gnss_ok is false: gnss_sim_node is
  // anchored, its status is fresh and its eph is within the limit. Separate
  // from gnss_ok because fcc_bridge reports a HIL_GPS fix as "no fix" (that
  // field means a real receiver -- gnss_sim_node's real-GPS lockout reads it).
  bool indoor_gnss_sim_ok{false};
};

struct CheckResult
{
  bool passed{true};
  std::string reason;  // empty iff passed
};

// MissionUpload.skip_phase_mask: only full (0) and transit (SKIP_SWARM_PHASES
// | SKIP_SEARCH) are flyable -- any other combination, e.g. skipping MUSTER
// alone, would leave a phase waiting on swarm state that never arrives.
// Solo (SKIP_SWARM_PHASES) is refused until search_planner_node publishes
// /mc/search/coverage_path: without it INGRESS would never end.
bool is_valid_skip_phase_mask(uint16_t mask);
// The upload skips every phase in `bits`. Null upload => skips nothing.
bool skips_phases(const mc_msgs::msg::MissionUpload * upload, uint16_t bits);
// drone_id is in the upload's swarm_manifest. Null upload or empty manifest
// => true (no manifest to filter by, so every peer counts).
bool in_swarm_manifest(const mc_msgs::msg::MissionUpload * upload, uint8_t drone_id);

// One function per SDD §4.2 bullet. Each guards against its own missing
// inputs (rather than assuming an earlier check already rejected a null
// vehicle_state/mission_upload) so every check is safe to call standalone.
CheckResult check_fcc_link_alive(const PreArmCheckInputs & in);
CheckResult check_fcc_disarmed(const PreArmCheckInputs & in);
CheckResult check_gcs_arm_authorised(const PreArmCheckInputs & in);
CheckResult check_battery_margin(const PreArmCheckInputs & in);
// Passes vacuously for a transit upload (SKIP_SWARM_PHASES): peers in
// other phases don't affect a drone that never musters with them. Peers
// outside the upload's swarm_manifest are ignored likewise.
CheckResult check_swarm_peers_preflight(const PreArmCheckInputs & in);
// Also rejects an invalid skip_phase_mask, and only requires the fields the
// upload's unskipped phases use: formation_offsets/swarm_manifest for the
// swarm phases, search_polygon for SEARCH.
CheckResult check_mission_upload_complete(const PreArmCheckInputs & in);
CheckResult check_visual_localisation_if_gnss_degraded(const PreArmCheckInputs & in);

struct NamedCheck
{
  const char * name;
  CheckResult (* fn)(const PreArmCheckInputs &);
};

// Table-driven, matching mission_manager_node's own transition_table_
// convention: the 7 checks live as data rather than a hand-rolled sequence
// of if-statements, so the set is inspectable/iterable independently of
// evaluate_pre_arm_checks()'s own aggregation logic.
const std::vector<NamedCheck> & all_pre_arm_checks();

struct PreArmCheckReport
{
  bool passed{true};
  std::vector<std::string> failed_checks;
};

// Runs every check in all_pre_arm_checks() — no short-circuit — so a single
// evaluation surfaces every failing condition at once. SDD §4.2 ("the node
// does not arm without all checks passing — it does not attempt partial
// arm") governs the arm decision, not how many reasons get reported for it.
PreArmCheckReport evaluate_pre_arm_checks(const PreArmCheckInputs & in);

}  // namespace mission_exec
