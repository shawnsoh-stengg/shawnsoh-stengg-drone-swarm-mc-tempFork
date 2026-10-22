// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#include "mission_exec/pre_arm_checks.hpp"

#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/swarm_peer.hpp>

namespace mission_exec
{

namespace
{
// DEP-009: ICD-MC-001 §10.2's v1.14-era STANDBY/STANDBY_ERROR states no
// longer exist on the wire — PX4 v1.17 arming_state is DISARMED=1/ARMED=2
// only, so any value other than DISARMED fails this check.
constexpr uint8_t kArmingStateDisarmed = 1;
}  // namespace

CheckResult check_fcc_link_alive(const PreArmCheckInputs & in)
{
  if (!in.vehicle_state) {
    return {false, "no vehicle_state received yet (cannot verify FCC link)"};
  }
  if (!in.vehicle_state->fcc_link_ok) {
    return {false, "FCC link not alive"};
  }
  return {true, ""};
}

CheckResult check_fcc_disarmed(const PreArmCheckInputs & in)
{
  if (!in.vehicle_state) {
    return {false, "no vehicle_state received yet (cannot verify FCC arming state)"};
  }
  if (in.vehicle_state->arming_state != kArmingStateDisarmed) {
    return {false, "FCC arming_state != DISARMED"};
  }
  return {true, ""};
}

CheckResult check_gcs_arm_authorised(const PreArmCheckInputs & in)
{
  if (!in.arm_authorised) {
    return {false, "GCS ARM authorisation not set"};
  }
  return {true, ""};
}

CheckResult check_battery_margin(const PreArmCheckInputs & in)
{
  if (!in.vehicle_state) {
    return {false, "no vehicle_state received yet (cannot verify battery)"};
  }
  const double low_battery_threshold =
    (in.mission_upload && in.mission_upload->low_battery_threshold > 0.0F) ?
    static_cast<double>(in.mission_upload->low_battery_threshold) : 0.20;
  const double required = low_battery_threshold + in.low_battery_margin;
  if (in.vehicle_state->battery_pct < static_cast<float>(required)) {
    return {false, "battery remaining below low-battery threshold + margin"};
  }
  return {true, ""};
}

CheckResult check_swarm_peers_preflight(const PreArmCheckInputs & in)
{
  if (!in.peer_states) {
    return {true, ""};
  }
  for (const auto & peer : in.peer_states->peers) {
    const bool alive = peer.peer_status == mc_msgs::msg::SwarmPeer::STATUS_ALIVE;
    const bool preflight_or_arming =
      peer.mission_phase == mc_msgs::msg::MissionPhase::PREFLIGHT ||
      peer.mission_phase == mc_msgs::msg::MissionPhase::ARMING;
    if (alive && !preflight_or_arming) {
      return {false, "a swarm peer is not in PREFLIGHT/ARMING"};
    }
  }
  return {true, ""};
}

CheckResult check_mission_upload_complete(const PreArmCheckInputs & in)
{
  if (!in.mission_upload) {
    return {false, "no mission upload received"};
  }
  if (in.mission_upload->ingress_waypoints.empty() ||
    in.mission_upload->formation_offsets.empty() ||
    in.mission_upload->search_polygon.empty() ||
    in.mission_upload->swarm_manifest.empty())
  {
    return {false, "mission upload incomplete (waypoints/offsets/polygon/manifest)"};
  }
  return {true, ""};
}

CheckResult check_visual_localisation_if_gnss_degraded(const PreArmCheckInputs & in)
{
  if (!in.vehicle_state) {
    return {false, "no vehicle_state received yet (cannot verify GNSS quality)"};
  }
  if (!in.vehicle_state->gnss_ok && !in.visual_localisation_alive && !in.indoor_gnss_sim_ok) {
    return {false, "GNSS degraded, visual localisation and indoor GNSS sim not alive"};
  }
  return {true, ""};
}

const std::vector<NamedCheck> & all_pre_arm_checks()
{
  static const std::vector<NamedCheck> kChecks = {
    {"fcc_link_alive", &check_fcc_link_alive},
    {"fcc_disarmed", &check_fcc_disarmed},
    {"gcs_arm_authorised", &check_gcs_arm_authorised},
    {"battery_margin", &check_battery_margin},
    {"swarm_peers_preflight", &check_swarm_peers_preflight},
    {"mission_upload_complete", &check_mission_upload_complete},
    {"visual_localisation_if_gnss_degraded", &check_visual_localisation_if_gnss_degraded},
  };
  return kChecks;
}

PreArmCheckReport evaluate_pre_arm_checks(const PreArmCheckInputs & in)
{
  PreArmCheckReport report;
  for (const auto & check : all_pre_arm_checks()) {
    const CheckResult result = check.fn(in);
    if (!result.passed) {
      report.passed = false;
      report.failed_checks.push_back(result.reason);
    }
  }
  return report;
}

}  // namespace mission_exec
