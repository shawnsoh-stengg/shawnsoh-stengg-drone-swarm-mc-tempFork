// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

// EXEC-002: one test per SDD §4.2 pre-arm-check bullet, plus aggregation
// behaviour (evaluate_pre_arm_checks() must not short-circuit). Pure gtest —
// no rclcpp::init required, since pre_arm_checks.hpp/.cpp take no
// dependency on rclcpp::Node or ROS time (see its own header doc comment).

#include <gtest/gtest.h>

#include <memory>

#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mc_msgs/msg/formation_offset.hpp>
#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <mc_msgs/msg/swarm_peer.hpp>
#include <mc_msgs/msg/swarm_peer_array.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>

#include "mission_exec/pre_arm_checks.hpp"

namespace mission_exec
{
namespace
{

constexpr uint8_t kArmingStateStandby = 1;
constexpr uint8_t kArmingStateArmed = 2;
constexpr uint8_t kArmingStateStandbyError = 3;

mc_msgs::msg::VehicleState::SharedPtr good_vehicle_state()
{
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>();
  vs->arming_state = kArmingStateStandby;
  vs->battery_pct = 1.0F;
  vs->gnss_ok = true;
  vs->fcc_link_ok = true;
  return vs;
}

mc_msgs::msg::MissionUpload::SharedPtr full_mission_upload()
{
  auto up = std::make_shared<mc_msgs::msg::MissionUpload>();
  up->ingress_waypoints = {geometry_msgs::msg::PoseStamped()};
  up->formation_offsets = {mc_msgs::msg::FormationOffset()};
  up->search_polygon = {geographic_msgs::msg::GeoPoint(), geographic_msgs::msg::GeoPoint()};
  up->swarm_manifest = {mc_msgs::msg::SwarmPeer()};
  up->arm_authorised = true;
  up->low_battery_threshold = 0.20F;
  return up;
}

PreArmCheckInputs good_inputs()
{
  PreArmCheckInputs in;
  in.vehicle_state = good_vehicle_state();
  in.mission_upload = full_mission_upload();
  in.arm_authorised = true;
  in.low_battery_margin = 0.10;
  in.visual_localisation_alive = false;  // fine: good_vehicle_state() has gnss_ok == true
  return in;
}

}  // namespace

TEST(PreArmChecksTest, AllPreArmChecksTableHasSevenEntries)
{
  EXPECT_EQ(all_pre_arm_checks().size(), 7U);
}

TEST(PreArmChecksTest, AllPassOnFullyGoodInputs)
{
  const auto report = evaluate_pre_arm_checks(good_inputs());
  EXPECT_TRUE(report.passed);
  EXPECT_TRUE(report.failed_checks.empty());
}

// --- 1. FCC link alive ---

TEST(PreArmChecksTest, FccLinkAliveFailsWithoutVehicleState)
{
  auto in = good_inputs();
  in.vehicle_state = nullptr;
  EXPECT_FALSE(check_fcc_link_alive(in).passed);
}

TEST(PreArmChecksTest, FccLinkAliveFailsWhenLinkDown)
{
  auto in = good_inputs();
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->fcc_link_ok = false;
  in.vehicle_state = vs;
  EXPECT_FALSE(check_fcc_link_alive(in).passed);
  // Independence: a failing link doesn't drag down an unrelated check.
  EXPECT_TRUE(check_fcc_standby(in).passed);
}

// --- 2. FCC STANDBY (not STANDBY_ERROR) ---

TEST(PreArmChecksTest, FccStandbyPassesOnStandby)
{
  EXPECT_TRUE(check_fcc_standby(good_inputs()).passed);
}

TEST(PreArmChecksTest, FccStandbyFailsOnArmed)
{
  auto in = good_inputs();
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->arming_state = kArmingStateArmed;
  in.vehicle_state = vs;
  EXPECT_FALSE(check_fcc_standby(in).passed);
}

TEST(PreArmChecksTest, FccStandbyFailsOnStandbyError)
{
  auto in = good_inputs();
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->arming_state = kArmingStateStandbyError;
  in.vehicle_state = vs;
  EXPECT_FALSE(check_fcc_standby(in).passed);
}

// --- 3. GCS ARM authorisation ---

TEST(PreArmChecksTest, GcsArmAuthorisedPassesWhenSet)
{
  EXPECT_TRUE(check_gcs_arm_authorised(good_inputs()).passed);
}

TEST(PreArmChecksTest, GcsArmAuthorisedFailsWhenNotSet)
{
  auto in = good_inputs();
  in.arm_authorised = false;
  EXPECT_FALSE(check_gcs_arm_authorised(in).passed);
}

// --- 4. Battery > threshold + 10% margin ---

TEST(PreArmChecksTest, BatteryMarginPassesAboveThresholdPlusMargin)
{
  auto in = good_inputs();
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->battery_pct = 0.35F;  // threshold 0.20 + margin 0.10 = 0.30 required
  in.vehicle_state = vs;
  EXPECT_TRUE(check_battery_margin(in).passed);
}

TEST(PreArmChecksTest, BatteryMarginFailsBelowThresholdPlusMargin)
{
  auto in = good_inputs();
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->battery_pct = 0.25F;
  in.vehicle_state = vs;
  EXPECT_FALSE(check_battery_margin(in).passed);
}

TEST(PreArmChecksTest, BatteryMarginUsesDefaultThresholdWithoutMissionUpload)
{
  auto in = good_inputs();
  in.mission_upload = nullptr;
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->battery_pct = 0.25F;  // default threshold 0.20 + margin 0.10 = 0.30 required
  in.vehicle_state = vs;
  EXPECT_FALSE(check_battery_margin(in).passed);
}

// --- 5. All swarm peers in PREFLIGHT/ARMING ---

TEST(PreArmChecksTest, SwarmPeersPreflightPassesWithNoPeerData)
{
  auto in = good_inputs();
  in.peer_states = nullptr;
  EXPECT_TRUE(check_swarm_peers_preflight(in).passed);
}

TEST(PreArmChecksTest, SwarmPeersPreflightFailsWhenAlivePeerPastPreflight)
{
  auto in = good_inputs();
  auto peers = std::make_shared<mc_msgs::msg::SwarmPeerArray>();
  mc_msgs::msg::SwarmPeer peer;
  peer.peer_status = mc_msgs::msg::SwarmPeer::STATUS_ALIVE;
  peer.mission_phase = mc_msgs::msg::MissionPhase::SEARCH;
  peers->peers = {peer};
  in.peer_states = peers;
  EXPECT_FALSE(check_swarm_peers_preflight(in).passed);
}

TEST(PreArmChecksTest, SwarmPeersPreflightPassesWhenDeadPeerPastPreflight)
{
  auto in = good_inputs();
  auto peers = std::make_shared<mc_msgs::msg::SwarmPeerArray>();
  mc_msgs::msg::SwarmPeer peer;
  peer.peer_status = mc_msgs::msg::SwarmPeer::STATUS_DEAD;
  peer.mission_phase = mc_msgs::msg::MissionPhase::SEARCH;
  peers->peers = {peer};
  in.peer_states = peers;
  EXPECT_TRUE(check_swarm_peers_preflight(in).passed);
}

TEST(PreArmChecksTest, SwarmPeersPreflightPassesWhenAlivePeerInArming)
{
  auto in = good_inputs();
  auto peers = std::make_shared<mc_msgs::msg::SwarmPeerArray>();
  mc_msgs::msg::SwarmPeer peer;
  peer.peer_status = mc_msgs::msg::SwarmPeer::STATUS_ALIVE;
  peer.mission_phase = mc_msgs::msg::MissionPhase::ARMING;
  peers->peers = {peer};
  in.peer_states = peers;
  EXPECT_TRUE(check_swarm_peers_preflight(in).passed);
}

// --- 6. Mission upload complete (all 4 fields) ---

TEST(PreArmChecksTest, MissionUploadCompleteFailsWithoutUpload)
{
  auto in = good_inputs();
  in.mission_upload = nullptr;
  EXPECT_FALSE(check_mission_upload_complete(in).passed);
}

TEST(PreArmChecksTest, MissionUploadCompletePassesWithAllFourFields)
{
  EXPECT_TRUE(check_mission_upload_complete(good_inputs()).passed);
}

TEST(PreArmChecksTest, MissionUploadCompleteFailsWhenIngressWaypointsEmpty)
{
  auto in = good_inputs();
  auto up = std::make_shared<mc_msgs::msg::MissionUpload>(*in.mission_upload);
  up->ingress_waypoints.clear();
  in.mission_upload = up;
  EXPECT_FALSE(check_mission_upload_complete(in).passed);
}

TEST(PreArmChecksTest, MissionUploadCompleteFailsWhenFormationOffsetsEmpty)
{
  auto in = good_inputs();
  auto up = std::make_shared<mc_msgs::msg::MissionUpload>(*in.mission_upload);
  up->formation_offsets.clear();
  in.mission_upload = up;
  EXPECT_FALSE(check_mission_upload_complete(in).passed);
}

TEST(PreArmChecksTest, MissionUploadCompleteFailsWhenSearchPolygonEmpty)
{
  auto in = good_inputs();
  auto up = std::make_shared<mc_msgs::msg::MissionUpload>(*in.mission_upload);
  up->search_polygon.clear();
  in.mission_upload = up;
  EXPECT_FALSE(check_mission_upload_complete(in).passed);
}

TEST(PreArmChecksTest, MissionUploadCompleteFailsWhenSwarmManifestEmpty)
{
  auto in = good_inputs();
  auto up = std::make_shared<mc_msgs::msg::MissionUpload>(*in.mission_upload);
  up->swarm_manifest.clear();
  in.mission_upload = up;
  EXPECT_FALSE(check_mission_upload_complete(in).passed);
}

// --- 7. Visual localisation alive if GNSS degraded ---

TEST(PreArmChecksTest, VisualLocalisationCheckPassesWhenGnssOk)
{
  EXPECT_TRUE(check_visual_localisation_if_gnss_degraded(good_inputs()).passed);
}

TEST(PreArmChecksTest, VisualLocalisationCheckFailsWhenGnssDegradedAndVoDead)
{
  auto in = good_inputs();
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->gnss_ok = false;
  in.vehicle_state = vs;
  in.visual_localisation_alive = false;
  EXPECT_FALSE(check_visual_localisation_if_gnss_degraded(in).passed);
}

TEST(PreArmChecksTest, VisualLocalisationCheckPassesWhenGnssDegradedButVoAlive)
{
  auto in = good_inputs();
  auto vs = std::make_shared<mc_msgs::msg::VehicleState>(*in.vehicle_state);
  vs->gnss_ok = false;
  in.vehicle_state = vs;
  in.visual_localisation_alive = true;
  EXPECT_TRUE(check_visual_localisation_if_gnss_degraded(in).passed);
}

// --- Aggregation: no short-circuit, all failures collected ---

TEST(PreArmChecksTest, EvaluateCollectsAllFailuresWithoutShortCircuiting)
{
  PreArmCheckInputs in;  // everything default: null vehicle_state/mission_upload/peer_states
  const auto report = evaluate_pre_arm_checks(in);
  EXPECT_FALSE(report.passed);
  // Every check except swarm_peers_preflight (vacuous pass on null
  // peer_states) fails when given entirely default/missing inputs.
  EXPECT_EQ(report.failed_checks.size(), all_pre_arm_checks().size() - 1);
}

TEST(PreArmChecksTest, EvaluateReportsEmptyFailedChecksWhenPassing)
{
  const auto report = evaluate_pre_arm_checks(good_inputs());
  EXPECT_TRUE(report.passed);
  EXPECT_EQ(report.failed_checks.size(), 0U);
}

}  // namespace mission_exec
