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
};

struct CheckResult
{
  bool passed{true};
  std::string reason;  // empty iff passed
};

// One function per SDD §4.2 bullet. Each guards against its own missing
// inputs (rather than assuming an earlier check already rejected a null
// vehicle_state/mission_upload) so every check is safe to call standalone.
CheckResult check_fcc_link_alive(const PreArmCheckInputs & in);
CheckResult check_fcc_disarmed(const PreArmCheckInputs & in);
CheckResult check_gcs_arm_authorised(const PreArmCheckInputs & in);
CheckResult check_battery_margin(const PreArmCheckInputs & in);
CheckResult check_swarm_peers_preflight(const PreArmCheckInputs & in);
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
