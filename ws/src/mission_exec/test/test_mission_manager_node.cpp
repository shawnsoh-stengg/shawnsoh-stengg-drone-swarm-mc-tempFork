// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

// EXEC-001: mission_manager_node node-integration tests — one per SDD §4.1
// transition table row (guard false -> no transition, guard true -> commit
// with the correct entry-action side effect), plus FR-003's 500 ms GCS
// override latency bound (TP-UT-012 style) and FCC-006's ack-blocks-
// advancement behaviour. swarm_coordinator_node/search_planner_node are
// mocked here via a harness node publishing directly on their documented
// output topics (TP-MC-001 §2.4 mock strategy), matching test_fcc_bridge_
// node.cpp's own technique of standing in for a peer node with a plain
// rclcpp::Node rather than launch_testing infrastructure.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mc_msgs/msg/fault_event.hpp>
#include <mc_msgs/msg/fcc_command.hpp>
#include <mc_msgs/msg/fcc_mission.hpp>
#include <mc_msgs/msg/fcc_mission_ack.hpp>
#include <mc_msgs/msg/formation_offset.hpp>
#include <mc_msgs/msg/gcs_command.hpp>
#include <mc_msgs/msg/gcs_command_ack.hpp>
#include <mc_msgs/msg/isr_role.hpp>
#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <mc_msgs/msg/status_text.hpp>
#include <mc_msgs/msg/swarm_peer.hpp>
#include <mc_msgs/msg/swarm_peer_array.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <mc_msgs/msg/voronoi_cell.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "mission_exec/mission_manager_node.hpp"

namespace
{
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

::testing::Environment * const kRclcppEnv =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

template<typename Predicate>
bool spin_until(
  std::initializer_list<rclcpp::Node::SharedPtr> nodes, Predicate predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    for (const auto & n : nodes) {
      rclcpp::spin_some(n);
    }
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

constexpr uint8_t kArmingStateDisarmed = 1;
constexpr uint8_t kArmingStateArmed = 2;
constexpr uint8_t kNavStateAutoLoiter = 4;
constexpr uint32_t kMavCmdComponentArmDisarm = 400;
constexpr uint32_t kMavCmdNavReturnToLaunch = 20;
constexpr uint32_t kMavCmdNavTakeoff = 22;
constexpr uint32_t kMavCmdDoSetMode = 176;
constexpr uint32_t kMcCmdEndMission = 50003;
}  // namespace

class MissionManagerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Each test gets its own DDS domain, not just fresh node_/harness_
    // objects. node_/harness_ are destroyed in the previous test's
    // TearDown() before this SetUp() runs, but DDS participant/publisher
    // teardown is asynchronous under the hood -- the destructor returning
    // doesn't guarantee CycloneDDS has announced deletion or flushed
    // in-flight datagrams. On fast hardware that window is negligible; under
    // CI's slower QEMU aarch64 emulation it was wide enough for a stale
    // MissionUpload(arm_authorised=true)/VehicleState from the previous
    // test to be delivered to this test's brand-new node, flipping it out
    // of PREFLIGHT before the test body even ran (observed: current_phase()
    // == ARMING on a just-constructed node in InvalidTransitionRequestIsRejected
    // and PhaseTransitionRejectedGetsAckWithReasonWithin500ms). A separate
    // DDS domain per test makes that delivery structurally impossible
    // instead of merely unlikely.
    static std::atomic<size_t> next_domain_id{100};
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(next_domain_id.fetch_add(1));
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr, init_options);

    // Separate from the DDS leak above: EXEC-004's disk persistence
    // (mission_upload_path_, default /var/drone-mc/mission_upload.yaml) is
    // real filesystem state, not DDS -- a domain-per-test does nothing to
    // isolate it. Every test in this binary was writing/reloading the same
    // real file, so a node from a later test would silently start with a
    // previous test's persisted upload_id=42/arm_authorised=true already
    // "received" (observed: InvalidTransitionRequestIsRejected,
    // PhaseTransitionRejectedGetsAckWithReasonWithin500ms, and
    // PreArmChecks_MultipleSimultaneousFailuresAllReported all failing from
    // pre-satisfied guards). Point each test at its own temp path instead --
    // same pattern as test_mission_manager_node_persistence.cpp's path_.
    mission_upload_path_ = ::testing::TempDir() + "mission_manager_node_test_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".yaml";
    std::remove(mission_upload_path_.c_str());
    std::remove((mission_upload_path_ + ".tmp").c_str());

    rclcpp::NodeOptions opts;
    opts.context(context_);
    opts.parameter_overrides(
    {
      rclcpp::Parameter("control_loop_period_s", 0.02),
      rclcpp::Parameter("mission_ack_timeout_s", 1.0),
      rclcpp::Parameter("mission_ack_max_retries", 1),
      rclcpp::Parameter("muster_quorum_count", 1),
      rclcpp::Parameter("takeoff_altitude_m", 10.0),
      rclcpp::Parameter("takeoff_altitude_tolerance_m", 1.0),
      rclcpp::Parameter("waypoint_acceptance_radius_m", 3.0),
      rclcpp::Parameter("home_position_tolerance_m", 3.0),
      rclcpp::Parameter("mission_upload_path", mission_upload_path_),
    });
    node_ = std::make_shared<mission_exec::MissionManagerNode>(opts);
    harness_ = std::make_shared<rclcpp::Node>(
      "mission_manager_node_test_harness", rclcpp::NodeOptions().context(context_));

    pub_vehicle_state_ = harness_->create_publisher<mc_msgs::msg::VehicleState>(
      "/mc/fcc/vehicle_state", rclcpp::QoS(10));
    pub_mission_upload_ = harness_->create_publisher<mc_msgs::msg::MissionUpload>(
      "/mc/gcs/mission_upload", rclcpp::QoS(10));
    pub_gcs_command_ =
      harness_->create_publisher<mc_msgs::msg::GcsCommand>("/mc/gcs/command", rclcpp::QoS(10));
    pub_peer_states_ = harness_->create_publisher<mc_msgs::msg::SwarmPeerArray>(
      "/mc/swarm/peer_states", rclcpp::QoS(10));
    pub_isr_role_ =
      harness_->create_publisher<mc_msgs::msg::IsrRole>("/mc/swarm/isr_role", rclcpp::QoS(10));
    pub_voronoi_cell_ = harness_->create_publisher<mc_msgs::msg::VoronoiCell>(
      "/mc/swarm/voronoi_cell", rclcpp::QoS(10));
    pub_coverage_path_ =
      harness_->create_publisher<nav_msgs::msg::Path>("/mc/search/coverage_path", rclcpp::QoS(10));
    pub_fault_ = harness_->create_publisher<mc_msgs::msg::FaultEvent>("/mc/fault", rclcpp::QoS(10));
    pub_mission_ack_ = harness_->create_publisher<mc_msgs::msg::FccMissionAck>(
      "/mc/fcc/mission_ack", rclcpp::QoS(10));

    sub_phase_ = harness_->create_subscription<mc_msgs::msg::MissionPhase>(
      "/mc/mission_phase", rclcpp::QoS(10),
      [this](mc_msgs::msg::MissionPhase::SharedPtr msg) {phases_.push_back(msg);});
    sub_heartbeat_ = harness_->create_subscription<std_msgs::msg::Header>(
      "/mc/manager/heartbeat", rclcpp::QoS(10),
      [this](std_msgs::msg::Header::SharedPtr msg) {heartbeats_.push_back(msg);});
    sub_send_command_ = harness_->create_subscription<mc_msgs::msg::FccCommand>(
      "/mc/fcc/send_command", rclcpp::QoS(10),
      [this](mc_msgs::msg::FccCommand::SharedPtr msg) {send_commands_.push_back(msg);});
    sub_fcc_mission_upload_ = harness_->create_subscription<mc_msgs::msg::FccMission>(
      "/mc/fcc/mission_upload", rclcpp::QoS(10),
      [this](mc_msgs::msg::FccMission::SharedPtr msg) {fcc_mission_uploads_.push_back(msg);});
    sub_status_text_ = harness_->create_subscription<mc_msgs::msg::StatusText>(
      "/mc/status_text", rclcpp::QoS(10),
      [this](mc_msgs::msg::StatusText::SharedPtr msg) {status_texts_.push_back(msg);});
    sub_command_ack_ = harness_->create_subscription<mc_msgs::msg::GcsCommandAck>(
      "/mc/gcs/command_ack", rclcpp::QoS(10),
      [this](mc_msgs::msg::GcsCommandAck::SharedPtr msg) {command_acks_.push_back(msg);});

    // Let publishers/subscriptions discover each other before any test body
    // starts asserting on timing-sensitive behaviour.
    spin_until(
      {node_, harness_}, [this] {return sub_send_command_->get_publisher_count() > 0;},
      std::chrono::seconds(2));
  }

  void TearDown() override
  {
    node_.reset();
    harness_.reset();
    context_->shutdown("test teardown");
    context_.reset();
    std::remove(mission_upload_path_.c_str());
    std::remove((mission_upload_path_ + ".tmp").c_str());
  }

  mc_msgs::msg::VehicleState good_vehicle_state() const
  {
    mc_msgs::msg::VehicleState vs;
    vs.header.stamp = harness_->now();
    vs.lat = 1.3;
    vs.lon = 103.8;
    vs.alt_msl = 40.0F;
    vs.x = 0.0F;
    vs.y = 0.0F;
    vs.z = 0.0F;
    vs.arming_state = kArmingStateDisarmed;
    vs.nav_state = 0;
    vs.battery_pct = 1.0F;
    vs.gnss_ok = true;
    vs.fcc_link_ok = true;
    return vs;
  }

  mc_msgs::msg::MissionUpload full_mission_upload(bool arm_authorised) const
  {
    mc_msgs::msg::MissionUpload up;
    up.header.stamp = harness_->now();
    up.upload_id = 42;
    geometry_msgs::msg::PoseStamped wp;
    wp.pose.position.x = 0.0;
    wp.pose.position.y = 0.0;
    wp.pose.position.z = 0.0;
    up.ingress_waypoints = {wp};
    mc_msgs::msg::FormationOffset offset;
    offset.drone_id = 1;
    up.formation_offsets = {offset};
    geographic_msgs::msg::GeoPoint pt;
    up.search_polygon = {pt, pt, pt};
    mc_msgs::msg::SwarmPeer peer;
    peer.drone_id = 1;
    up.swarm_manifest = {peer};
    up.arm_authorised = arm_authorised;
    up.low_battery_threshold = 0.20F;
    return up;
  }

  void send_gcs_command(uint32_t cmd, std::array<float, 7> params)
  {
    mc_msgs::msg::GcsCommand out;
    out.header.stamp = harness_->now();
    out.command = cmd;
    out.params = params;
    pub_gcs_command_->publish(out);
  }

  // Drives the node from PREFLIGHT through ARMING, TAKEOFF, and INGRESS
  // (acking the FORMATION_SLOT reservation along the way — a full one-item
  // mission upload, not a mission-item update; FCC-010) so later-phase
  // tests can start from a realistic precondition instead of re-deriving
  // every earlier phase's setup — mirrors TP-ST-001's own full-mission
  // structure.
  void advance_to_ingress()
  {
    pub_mission_upload_->publish(full_mission_upload(/*arm_authorised=*/ true));
    pub_vehicle_state_->publish(good_vehicle_state());
    ASSERT_TRUE(
      spin_until(
        {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
        std::chrono::seconds(2)));

    auto vs = good_vehicle_state();
    vs.arming_state = kArmingStateArmed;
    pub_vehicle_state_->publish(vs);
    ASSERT_TRUE(
      spin_until(
        {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
        std::chrono::seconds(2)));

    vs.z = -10.0F;  // NED down => 10 m AGL, matches takeoff_altitude_m_ override
    pub_vehicle_state_->publish(vs);
    wait_for_ticks();  // ensure the updated altitude lands before the GCS command below
    send_gcs_command(50001 /*MC_CMD_PHASE_TRANSITION*/, {3.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
    ASSERT_TRUE(
      spin_until(
        {node_, harness_}, [this] {return !fcc_mission_uploads_.empty();},
        std::chrono::seconds(2)));
    mc_msgs::msg::FccMissionAck ack;
    ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
    ack.upload_id = fcc_mission_uploads_.back()->upload_id;
    ack.success = true;
    pub_mission_ack_->publish(ack);
    ASSERT_TRUE(
      spin_until(
        {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::INGRESS);},
        std::chrono::seconds(2)));
  }

  // EXEC-006: advance_to_ingress(), then GCS END_MISSION -> RETURN -> LAND
  // with the FCC still armed and at home.
  void advance_to_land()
  {
    advance_to_ingress();
    send_gcs_command(kMcCmdEndMission, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
    ASSERT_TRUE(
      spin_until(
        {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::RETURN);},
        std::chrono::seconds(2)));
    auto vs = good_vehicle_state();
    vs.arming_state = kArmingStateArmed;
    pub_vehicle_state_->publish(vs);
    ASSERT_TRUE(
      spin_until(
        {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::LAND);},
        std::chrono::seconds(2)));
  }

  // EXEC-006: FCC reports landed and disarmed; waits for the recycle.
  void land_and_disarm_to_preflight()
  {
    pub_vehicle_state_->publish(good_vehicle_state());
    ASSERT_TRUE(
      spin_until(
        {node_, harness_},
        [this] {return phase_reached(mc_msgs::msg::MissionPhase::PREFLIGHT);},
        std::chrono::seconds(2)));
  }

  size_t restart_notice_count() const
  {
    return static_cast<size_t>(
      std::count_if(
        status_texts_.begin(), status_texts_.end(),
        [](const mc_msgs::msg::StatusText::SharedPtr & msg) {
          return msg->text.find("restart required") != std::string::npos;
        }));
  }

  bool phase_reached(uint8_t phase) const
  {
    return !phases_.empty() && phases_.back()->phase == phase;
  }

  // Blocks until at least `ticks` further control-loop cycles have
  // completed (observed via the heartbeat published at the end of every
  // cycle, SDD §4.4) — guarantees a just-published message the node
  // subscribes to has actually been processed before the caller proceeds,
  // avoiding a race between an async publish and a synchronous follow-up
  // action like a GCS command that depends on it.
  void wait_for_ticks(int ticks = 2)
  {
    const size_t baseline = heartbeats_.size();
    const auto reached = [this, baseline, ticks] {
      return heartbeats_.size() >= baseline + static_cast<size_t>(ticks);
    };
    ASSERT_TRUE(spin_until({node_, harness_}, reached, std::chrono::seconds(2)));
  }

  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<mission_exec::MissionManagerNode> node_;
  rclcpp::Node::SharedPtr harness_;
  std::string mission_upload_path_;

  rclcpp::Publisher<mc_msgs::msg::VehicleState>::SharedPtr pub_vehicle_state_;
  rclcpp::Publisher<mc_msgs::msg::MissionUpload>::SharedPtr pub_mission_upload_;
  rclcpp::Publisher<mc_msgs::msg::GcsCommand>::SharedPtr pub_gcs_command_;
  rclcpp::Publisher<mc_msgs::msg::SwarmPeerArray>::SharedPtr pub_peer_states_;
  rclcpp::Publisher<mc_msgs::msg::IsrRole>::SharedPtr pub_isr_role_;
  rclcpp::Publisher<mc_msgs::msg::VoronoiCell>::SharedPtr pub_voronoi_cell_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_coverage_path_;
  rclcpp::Publisher<mc_msgs::msg::FaultEvent>::SharedPtr pub_fault_;
  rclcpp::Publisher<mc_msgs::msg::FccMissionAck>::SharedPtr pub_mission_ack_;

  rclcpp::Subscription<mc_msgs::msg::MissionPhase>::SharedPtr sub_phase_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_heartbeat_;
  rclcpp::Subscription<mc_msgs::msg::FccCommand>::SharedPtr sub_send_command_;
  rclcpp::Subscription<mc_msgs::msg::FccMission>::SharedPtr sub_fcc_mission_upload_;
  rclcpp::Subscription<mc_msgs::msg::StatusText>::SharedPtr sub_status_text_;
  rclcpp::Subscription<mc_msgs::msg::GcsCommandAck>::SharedPtr sub_command_ack_;

  std::vector<mc_msgs::msg::MissionPhase::SharedPtr> phases_;
  std::vector<std_msgs::msg::Header::SharedPtr> heartbeats_;
  std::vector<mc_msgs::msg::FccCommand::SharedPtr> send_commands_;
  std::vector<mc_msgs::msg::FccMission::SharedPtr> fcc_mission_uploads_;
  std::vector<mc_msgs::msg::StatusText::SharedPtr> status_texts_;
  std::vector<mc_msgs::msg::GcsCommandAck::SharedPtr> command_acks_;
};

namespace
{
bool any_status_text_contains(
  const std::vector<mc_msgs::msg::StatusText::SharedPtr> & texts, const std::string & substr)
{
  return std::any_of(
    texts.begin(), texts.end(),
    [&](const mc_msgs::msg::StatusText::SharedPtr & msg) {
      return msg->text.find(substr) != std::string::npos;
    });
}
}  // namespace

TEST_F(MissionManagerNodeTest, ConstructsWithExpectedNameAndStartsInPreflight)
{
  EXPECT_STREQ(node_->get_name(), "mission_manager_node");
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
}

// --- PREFLIGHT -> ARMING ---

TEST_F(MissionManagerNodeTest, PreflightToArming_GuardFalseWithoutArmAuthorisation)
{
  pub_vehicle_state_->publish(good_vehicle_state());
  // No mission upload published -> arm_authorised_ stays false.
  spin_until({node_, harness_}, [this] {return !heartbeats_.empty();}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
  EXPECT_TRUE(send_commands_.empty());
}

TEST_F(MissionManagerNodeTest, PreflightToArming_GuardTrueCommandsArmAndTransitions)
{
  pub_mission_upload_->publish(full_mission_upload(/*arm_authorised=*/ true));
  pub_vehicle_state_->publish(good_vehicle_state());

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, kMavCmdComponentArmDisarm);
  EXPECT_FLOAT_EQ(send_commands_.back()->params[0], 1.0F);
}

// --- ARMING -> TAKEOFF ---

TEST_F(MissionManagerNodeTest, ArmingToTakeoff_GuardFalseUntilFccReportsArmed)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));

  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 3;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::ARMING);
}

TEST_F(MissionManagerNodeTest, ArmingToTakeoff_GuardTrueSendsTakeoffCommand)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));

  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  pub_vehicle_state_->publish(vs);

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
      std::chrono::seconds(2)));
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, kMavCmdNavTakeoff);
  // Regression: finite lat/lon (the old 0, 0) sends PX4 to Null Island
  // instead of climbing in place.
  EXPECT_TRUE(std::isnan(send_commands_.back()->params[4]));
  EXPECT_TRUE(std::isnan(send_commands_.back()->params[5]));
  EXPECT_FLOAT_EQ(send_commands_.back()->params[6], 10.0F);
}

// --- TAKEOFF -> INGRESS (mission-channel, ack-blocking) ---

TEST_F(MissionManagerNodeTest, TakeoffToIngress_StaysAtTakeoffWithoutGcsIngressCommand)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
      std::chrono::seconds(2)));

  // Altitude reached but no GCS PHASE_TRANSITION(INGRESS) yet -> guard false.
  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 5;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::TAKEOFF);
  EXPECT_TRUE(fcc_mission_uploads_.empty());
}

TEST_F(MissionManagerNodeTest, TakeoffToIngress_BlocksOnAckThenCommitsOnSuccess)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
      std::chrono::seconds(2)));

  send_gcs_command(50001, {3.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});  // PHASE_TRANSITION(INGRESS)
  // FCC-010: the FORMATION_SLOT reservation is now a full (one-item)
  // mission upload, not a mission-item update.
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !fcc_mission_uploads_.empty();},
      std::chrono::seconds(2)));
  ASSERT_EQ(fcc_mission_uploads_.back()->items.size(), 1U);
  EXPECT_EQ(fcc_mission_uploads_.back()->items.back().seq, 0);

  // No ack yet -> the state machine must not have optimistically advanced.
  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 5;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::TAKEOFF);

  mc_msgs::msg::FccMissionAck ack;
  ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
  ack.upload_id = fcc_mission_uploads_.back()->upload_id;
  ack.success = true;
  pub_mission_ack_->publish(ack);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::INGRESS);},
      std::chrono::seconds(2)));
}

TEST_F(MissionManagerNodeTest, TakeoffToIngress_FailedAckRetriesThenEscalatesToFault)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
      std::chrono::seconds(2)));

  send_gcs_command(50001, {3.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !fcc_mission_uploads_.empty();},
      std::chrono::seconds(2)));

  mc_msgs::msg::FccMissionAck ack;
  ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
  ack.upload_id = fcc_mission_uploads_.back()->upload_id;
  ack.success = false;
  pub_mission_ack_->publish(ack);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return fcc_mission_uploads_.size() >= 2U;},
      std::chrono::seconds(2)))
    << "expected a retry resend after the first failed ack";
  ack.upload_id = fcc_mission_uploads_.back()->upload_id;
  pub_mission_ack_->publish(ack);  // mission_ack_max_retries_ == 1 -> this 2nd failure escalates

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::seconds(3)))
    << "expected escalation to FAULT after exhausting ack retries";
}

// --- INGRESS -> MUSTER ---

TEST_F(MissionManagerNodeTest, IngressToMuster_GuardFalseUntilMusterWaypointReached)
{
  advance_to_ingress();
  // Still far from the mission upload's single (0,0) muster waypoint.
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  vs.x = 100.0F;
  vs.y = 100.0F;
  pub_vehicle_state_->publish(vs);
  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 5;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::INGRESS);
}

TEST_F(MissionManagerNodeTest, IngressToMuster_GuardTrueOnReachingMusterWaypoint)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  vs.x = 0.0F;
  vs.y = 0.0F;  // matches ingress_waypoints' single (0,0) waypoint
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::MUSTER);},
      std::chrono::seconds(2)));
}

// --- MUSTER -> ISR_SELECTION ---

TEST_F(MissionManagerNodeTest, MusterToIsrSelection_GuardTrueWithSoloQuorum)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::MUSTER);},
      std::chrono::seconds(2)));

  // muster_quorum_count_ override == 1 (solo drone) -> no peer_states needed.
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ISR_SELECTION);},
      std::chrono::seconds(2)));
}

// --- ISR_SELECTION -> VORONOI_PARTITION ---

TEST_F(MissionManagerNodeTest, IsrSelectionToVoronoiPartition_GuardFalseUntilAllPeersAcked)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ISR_SELECTION);},
      std::chrono::seconds(2)));

  mc_msgs::msg::IsrRole role;
  role.role = mc_msgs::msg::IsrRole::ISR;
  role.all_peers_acked = false;
  pub_isr_role_->publish(role);
  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 5;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::ISR_SELECTION);
}

TEST_F(MissionManagerNodeTest, IsrSelectionToVoronoiPartition_GuardTrueOnAllPeersAcked)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ISR_SELECTION);},
      std::chrono::seconds(2)));

  mc_msgs::msg::IsrRole role;
  role.role = mc_msgs::msg::IsrRole::ISR;
  role.all_peers_acked = true;
  pub_isr_role_->publish(role);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this] {return phase_reached(mc_msgs::msg::MissionPhase::VORONOI_PARTITION);},
      std::chrono::seconds(2)));
}

// --- VORONOI_PARTITION -> SEARCH (mission-channel, ack-blocking) ---

TEST_F(MissionManagerNodeTest, VoronoiPartitionToSearch_GuardFalseUntilCellAndPathReceived)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  mc_msgs::msg::IsrRole role;
  role.all_peers_acked = true;
  pub_isr_role_->publish(role);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this] {return phase_reached(mc_msgs::msg::MissionPhase::VORONOI_PARTITION);},
      std::chrono::seconds(2)));

  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 5;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::VORONOI_PARTITION);
  // advance_to_ingress() already produced exactly one upload (the
  // FORMATION_SLOT reservation, FCC-010) — no coverage-path upload yet.
  EXPECT_EQ(fcc_mission_uploads_.size(), 1U);
}

TEST_F(MissionManagerNodeTest, VoronoiPartitionToSearch_GuardTrueUploadsCoveragePath)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  mc_msgs::msg::IsrRole role;
  role.all_peers_acked = true;
  pub_isr_role_->publish(role);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this] {return phase_reached(mc_msgs::msg::MissionPhase::VORONOI_PARTITION);},
      std::chrono::seconds(2)));

  mc_msgs::msg::VoronoiCell cell;
  cell.upload_id = 42;  // matches full_mission_upload()'s upload_id
  pub_voronoi_cell_->publish(cell);

  // advance_to_ingress() already produced one upload (FORMATION_SLOT
  // reservation, FCC-010) — wait for a *new* one past that baseline rather
  // than just "non-empty", so `.back()` below is really the coverage path.
  const size_t uploads_before_coverage_path = fcc_mission_uploads_.size();

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1;
  p1.pose.position.x = 103.8;  // lon
  p1.pose.position.y = 1.3;    // lat
  p1.pose.position.z = 50.0;   // alt_msl
  path.poses = {p1};
  pub_coverage_path_->publish(path);

  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this, uploads_before_coverage_path] {
        return fcc_mission_uploads_.size() > uploads_before_coverage_path;
      },
      std::chrono::seconds(2)));
  ASSERT_EQ(fcc_mission_uploads_.back()->items.size(), 1U);

  mc_msgs::msg::FccMissionAck ack;
  ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
  ack.upload_id = fcc_mission_uploads_.back()->upload_id;
  ack.success = true;
  pub_mission_ack_->publish(ack);

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::SEARCH);},
      std::chrono::seconds(2)));
}

// --- SEARCH -> RETURN ---

TEST_F(MissionManagerNodeTest, SearchToReturn_GuardTrueOnFccMissionComplete)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  mc_msgs::msg::IsrRole role;
  role.all_peers_acked = true;
  pub_isr_role_->publish(role);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this] {return phase_reached(mc_msgs::msg::MissionPhase::VORONOI_PARTITION);},
      std::chrono::seconds(2)));

  mc_msgs::msg::VoronoiCell cell;
  cell.upload_id = 42;
  pub_voronoi_cell_->publish(cell);
  // advance_to_ingress() already produced one upload (FORMATION_SLOT
  // reservation, FCC-010) — wait for a *new* one past that baseline.
  const size_t uploads_before_coverage_path = fcc_mission_uploads_.size();
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p1;
  path.poses = {p1};
  pub_coverage_path_->publish(path);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this, uploads_before_coverage_path] {
        return fcc_mission_uploads_.size() > uploads_before_coverage_path;
      },
      std::chrono::seconds(2)));
  mc_msgs::msg::FccMissionAck ack;
  ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
  ack.upload_id = fcc_mission_uploads_.back()->upload_id;
  ack.success = true;
  pub_mission_ack_->publish(ack);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::SEARCH);},
      std::chrono::seconds(2)));

  vs.nav_state = kNavStateAutoLoiter;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::RETURN);},
      std::chrono::seconds(2)));
}

// --- RETURN -> LAND ---

TEST_F(MissionManagerNodeTest, ReturnToLand_GuardTrueAtHomePositionSendsLandCommand)
{
  advance_to_ingress();
  send_gcs_command(kMcCmdEndMission, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::RETURN);},
      std::chrono::seconds(2)));

  auto vs = good_vehicle_state();
  // Still armed: a disarmed FCC would recycle LAND straight on to PREFLIGHT
  // (EXEC-006) before this test could observe LAND.
  vs.arming_state = kArmingStateArmed;
  vs.x = 0.0F;
  vs.y = 0.0F;  // within home_position_tolerance_m_ of the NED origin
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::LAND);},
      std::chrono::seconds(2)));
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, 21U /*MAV_CMD_NAV_LAND*/);
}

// --- Any -> RETURN (GCS END_MISSION / ABORT) ---

TEST_F(MissionManagerNodeTest, AnyToReturn_EndMissionCommandAbortsFromAnyPhase)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));

  send_gcs_command(kMcCmdEndMission, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::RETURN);},
      std::chrono::seconds(2)));
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, kMavCmdNavReturnToLaunch);
}

// --- Any -> FAULT (health_monitor CRITICAL_FAULT event) ---

TEST_F(MissionManagerNodeTest, AnyToFault_FccLinkLostFaultCommandsRtl)
{
  pub_vehicle_state_->publish(good_vehicle_state());
  spin_until({node_, harness_}, [this] {return !heartbeats_.empty();}, std::chrono::seconds(1));

  mc_msgs::msg::FaultEvent fault;
  fault.fault_id = mc_msgs::msg::FaultEvent::FCC_LINK_LOST;
  fault.severity = 2;
  pub_fault_->publish(fault);

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::seconds(2)));
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, kMavCmdNavReturnToLaunch);
  // FaultEvent's own doc comment / is_critical_fault(): this reacts only to
  // the /mc/fault content, never to /mc/fcc/vehicle_state.fcc_link_ok.
}

// --- FR-002: undefined transitions are rejected, phase unchanged ---

TEST_F(MissionManagerNodeTest, InvalidTransitionRequestIsRejected)
{
  pub_vehicle_state_->publish(good_vehicle_state());
  spin_until({node_, harness_}, [this] {return !heartbeats_.empty();}, std::chrono::seconds(1));

  // PREFLIGHT's only row targets ARMING — request SEARCH directly.
  send_gcs_command(50001, {7.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 5;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
}

// --- FR-003 / EXEC-003: GCS emergency command acted on within 200 ms of
// receipt, any phase ---

TEST_F(MissionManagerNodeTest, GcsDisarmCommandActedOnWithin200msFromArmingPhase)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));

  const auto t0 = std::chrono::steady_clock::now();
  send_gcs_command(kMavCmdComponentArmDisarm, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});  // DISARM
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::milliseconds(200)));
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 200);
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, kMavCmdComponentArmDisarm);
  EXPECT_FLOAT_EQ(send_commands_.back()->params[0], 0.0F);
}

// --- EXEC-003: the Emergency tier's new MANUAL/ALTCTL/POSCTL/STABILIZED
// members (via MAV_CMD_DO_SET_MODE), not just DISARM/RTL ---

TEST_F(MissionManagerNodeTest, GcsDoSetModeManualIsTreatedAsEmergency)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));

  send_gcs_command(kMavCmdDoSetMode, {1.F, 1.F, 0.F, 0.F, 0.F, 0.F, 0.F});  // custom_main=MANUAL
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::milliseconds(200)));
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, kMavCmdDoSetMode);
}

// --- EXEC-003: an Emergency command arriving while the state machine is
// mid-autonomous-flow (SEARCH, actively evaluating autonomous transitions
// every tick) still preempts immediately — "GCS commands always supersede
// autonomous transitions" (this ticket's own story), not merely when idle. ---

TEST_F(MissionManagerNodeTest, EmergencyCommandDuringAutonomousSearchPhasePreemptsImmediately)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  mc_msgs::msg::IsrRole role;
  role.all_peers_acked = true;
  pub_isr_role_->publish(role);
  mc_msgs::msg::VoronoiCell cell;
  cell.upload_id = 42;
  pub_voronoi_cell_->publish(cell);
  // advance_to_ingress() already produced one upload (FORMATION_SLOT
  // reservation, FCC-010) — wait for a *new* one past that baseline.
  const size_t uploads_before_coverage_path = fcc_mission_uploads_.size();
  nav_msgs::msg::Path path;
  path.poses = {geometry_msgs::msg::PoseStamped()};
  pub_coverage_path_->publish(path);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this, uploads_before_coverage_path] {
        return fcc_mission_uploads_.size() > uploads_before_coverage_path;
      },
      std::chrono::seconds(2)));
  mc_msgs::msg::FccMissionAck ack;
  ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
  ack.upload_id = fcc_mission_uploads_.back()->upload_id;
  ack.success = true;
  pub_mission_ack_->publish(ack);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::SEARCH);},
      std::chrono::seconds(2)));

  // Node is now autonomously in SEARCH, re-evaluating guard_search_to_
  // return() every control-loop tick with no autonomous guard yet true
  // (nav_state isn't AUTO_LOITER) — an Emergency command must still cut in.
  send_gcs_command(kMavCmdNavReturnToLaunch, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::milliseconds(200)));
}

// --- EXEC-003: a later-arriving Emergency command jumps ahead of an
// earlier-arriving, not-yet-drained Phase-tier command — the behaviour
// command_arbiter's own unit tests exercise directly; this proves the same
// ordering holds through mission_manager_node's actual wiring. ---

TEST_F(MissionManagerNodeTest, EmergencyCommandBypassesAnAlreadyQueuedPhaseCommand)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));

  // Both land in command_arbiter_ before the next control-loop tick drains
  // it — PHASE_TRANSITION(SEARCH), which PREFLIGHT/ARMING's table rejects
  // regardless, enqueued first; DISARM enqueued second.
  send_gcs_command(50001, {7.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  send_gcs_command(kMavCmdComponentArmDisarm, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::milliseconds(200)));
  ASSERT_FALSE(send_commands_.empty());
  EXPECT_EQ(send_commands_.back()->cmd, kMavCmdComponentArmDisarm);
}

// --- EXEC-003: Phase tier ACKed within 500 ms — RESULT_ACCEPTED when the
// guard passes, RESULT_REJECTED (with a reason) when it fails ---

TEST_F(MissionManagerNodeTest, PhaseTransitionAcceptedGetsAckWithin500ms)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));

  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
      std::chrono::seconds(2)));
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  wait_for_ticks();

  const auto t0 = std::chrono::steady_clock::now();
  send_gcs_command(50001, {3.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});  // PHASE_TRANSITION(INGRESS)
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !command_acks_.empty();}, std::chrono::milliseconds(500)));
  EXPECT_LE(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count(),
    500);
  EXPECT_EQ(command_acks_.back()->result, mc_msgs::msg::GcsCommandAck::RESULT_ACCEPTED);
}

TEST_F(MissionManagerNodeTest, PhaseTransitionRejectedGetsAckWithReasonWithin500ms)
{
  pub_vehicle_state_->publish(good_vehicle_state());
  spin_until({node_, harness_}, [this] {return !heartbeats_.empty();}, std::chrono::seconds(1));

  const auto t0 = std::chrono::steady_clock::now();
  // PREFLIGHT's only row targets ARMING — request SEARCH directly.
  send_gcs_command(50001, {7.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !command_acks_.empty();}, std::chrono::milliseconds(500)));
  EXPECT_LE(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count(),
    500);
  EXPECT_EQ(command_acks_.back()->result, mc_msgs::msg::GcsCommandAck::RESULT_REJECTED);
  EXPECT_FALSE(command_acks_.back()->reason.empty());
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
}

// --- FR-074 / TP-NI-011: heartbeat continuity ---

TEST_F(MissionManagerNodeTest, HeartbeatPublishesAtLeastOneHertz)
{
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return heartbeats_.size() >= 20U;}, std::chrono::seconds(2)));
}

// --- EXEC-002: pre-arm check suite, exercised through the node's mock-FCC
// harness rather than pre_arm_checks.hpp's own unit tests (test_pre_arm_
// checks.cpp), so these confirm the full wiring: guard_preflight_to_
// arming() blocks on any single failing condition, multiple simultaneous
// failures are all surfaced (not just the first), and each failure reaches
// /mc/status_text with its specific reason. ---

TEST_F(MissionManagerNodeTest, PreArmChecks_LowBatteryBlocksArmingAndReportsReason)
{
  pub_mission_upload_->publish(full_mission_upload(/*arm_authorised=*/ true));
  auto vs = good_vehicle_state();
  vs.battery_pct = 0.05F;  // below default 0.20 threshold + 0.10 margin
  pub_vehicle_state_->publish(vs);

  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return any_status_text_contains(status_texts_, "battery");},
      std::chrono::seconds(2)));
  spin_until({node_, harness_}, [this] {return heartbeats_.size() > 5;}, std::chrono::seconds(1));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
  EXPECT_TRUE(send_commands_.empty());
}

TEST_F(MissionManagerNodeTest, PreArmChecks_MultipleSimultaneousFailuresAllReported)
{
  // No mission upload at all (arm-auth unset, mission incomplete) and a
  // vehicle_state reporting a dead FCC link and low battery — 4 of the 7
  // checks fail at once; none should suppress reporting of the others.
  auto vs = good_vehicle_state();
  vs.fcc_link_ok = false;
  vs.battery_pct = 0.05F;
  pub_vehicle_state_->publish(vs);

  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this] {
        return any_status_text_contains(status_texts_, "FCC link") &&
        any_status_text_contains(status_texts_, "battery") &&
        any_status_text_contains(status_texts_, "ARM authorisation") &&
        any_status_text_contains(status_texts_, "mission upload");
      },
      std::chrono::seconds(2)));
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
}

TEST_F(MissionManagerNodeTest, PreArmChecks_ReasonClearsOnceConditionIsFixed)
{
  pub_mission_upload_->publish(full_mission_upload(/*arm_authorised=*/ true));
  auto vs = good_vehicle_state();
  vs.battery_pct = 0.05F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return any_status_text_contains(status_texts_, "battery");},
      std::chrono::seconds(2)));

  vs.battery_pct = 1.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
}

// --- EXEC-006: LAND -> PREFLIGHT recycle (normal end) / terminal FAULT ---

TEST_F(MissionManagerNodeTest, FullMissionEndsInPreflightAfterLandAndDisarm)
{
  advance_to_ingress();
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  mc_msgs::msg::IsrRole role;
  role.all_peers_acked = true;
  pub_isr_role_->publish(role);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this] {return phase_reached(mc_msgs::msg::MissionPhase::VORONOI_PARTITION);},
      std::chrono::seconds(2)));

  mc_msgs::msg::VoronoiCell cell;
  cell.upload_id = 42;
  pub_voronoi_cell_->publish(cell);
  const size_t uploads_before_coverage_path = fcc_mission_uploads_.size();
  nav_msgs::msg::Path path;
  path.poses = {geometry_msgs::msg::PoseStamped{}};
  pub_coverage_path_->publish(path);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_},
      [this, uploads_before_coverage_path] {
        return fcc_mission_uploads_.size() > uploads_before_coverage_path;
      },
      std::chrono::seconds(2)));
  mc_msgs::msg::FccMissionAck ack;
  ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
  ack.upload_id = fcc_mission_uploads_.back()->upload_id;
  ack.success = true;
  pub_mission_ack_->publish(ack);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::SEARCH);},
      std::chrono::seconds(2)));

  // FCC mission complete -> RETURN; still armed at home (z irrelevant) -> LAND.
  vs.nav_state = kNavStateAutoLoiter;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::LAND);},
      std::chrono::seconds(2)));

  land_and_disarm_to_preflight();
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
}

TEST_F(MissionManagerNodeTest, GcsAbortEndsInPreflightAfterLandAndDisarm)
{
  advance_to_land();
  land_and_disarm_to_preflight();
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
}

TEST_F(MissionManagerNodeTest, LandStaysUntilDisarmedAndStationary)
{
  advance_to_land();
  auto vs = good_vehicle_state();  // disarmed...
  vs.vz = 2.0F;  // ...but still descending (e.g. an RC kill in the air)
  pub_vehicle_state_->publish(vs);
  wait_for_ticks(5);
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::LAND);

  land_and_disarm_to_preflight();
}

TEST_F(MissionManagerNodeTest, RecycleRefusesRearmUntilNewUploadAndAuthorisation)
{
  advance_to_land();
  std::ifstream persisted_before(mission_upload_path_);
  ASSERT_TRUE(persisted_before.good());
  land_and_disarm_to_preflight();

  // The flown upload (and its ARM authorisation) is gone, in memory and on
  // disk, so the next mission can't arm off it (FR-010).
  const size_t commands_after_recycle = send_commands_.size();
  pub_vehicle_state_->publish(good_vehicle_state());
  wait_for_ticks(5);
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
  EXPECT_EQ(send_commands_.size(), commands_after_recycle);
  const auto report = node_->pre_arm_check_report();
  EXPECT_FALSE(report.passed);
  EXPECT_NE(
    std::find(
      report.failed_checks.begin(), report.failed_checks.end(), "no mission upload received"),
    report.failed_checks.end());
  std::ifstream persisted_after(mission_upload_path_);
  EXPECT_FALSE(persisted_after.good());

  // MC_CMD_ARM_AUTHORISE alone isn't enough without an upload.
  send_gcs_command(50002 /*MC_CMD_ARM_AUTHORISE*/, {1.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  wait_for_ticks(5);
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);

  // A new upload with ARM authorisation arms again.
  pub_mission_upload_->publish(full_mission_upload(/*arm_authorised=*/ true));
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
}

TEST_F(MissionManagerNodeTest, UploadReceivedMidMissionSurvivesRecycle)
{
  advance_to_land();
  // Channel B accepts uploads in any phase: this one is the next mission's.
  auto next = full_mission_upload(/*arm_authorised=*/ false);
  next.upload_id = 43;
  pub_mission_upload_->publish(next);
  wait_for_ticks();
  land_and_disarm_to_preflight();

  const auto report = node_->pre_arm_check_report();
  EXPECT_EQ(
    std::find(
      report.failed_checks.begin(), report.failed_checks.end(), "no mission upload received"),
    report.failed_checks.end());
  EXPECT_NE(
    std::find(
      report.failed_checks.begin(), report.failed_checks.end(),
      "GCS ARM authorisation not set"),
    report.failed_checks.end());
}

TEST_F(MissionManagerNodeTest, LateMissionAckAfterRecycleIsIgnored)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  vs.z = -10.0F;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
      std::chrono::seconds(2)));

  // Start the FORMATION_SLOT upload but never ack it, then abort.
  wait_for_ticks();
  send_gcs_command(50001 /*MC_CMD_PHASE_TRANSITION*/, {3.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return !fcc_mission_uploads_.empty();},
      std::chrono::seconds(2)));
  const uint32_t abandoned_upload_id = fcc_mission_uploads_.back()->upload_id;
  send_gcs_command(kMcCmdEndMission, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::LAND);},
      std::chrono::seconds(2)));
  land_and_disarm_to_preflight();

  mc_msgs::msg::FccMissionAck ack;
  ack.request_type = mc_msgs::msg::FccMissionAck::UPLOAD;
  ack.upload_id = abandoned_upload_id;
  ack.success = true;
  pub_mission_ack_->publish(ack);
  wait_for_ticks(5);
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::PREFLIGHT);
  EXPECT_EQ(phases_.back()->phase, mc_msgs::msg::MissionPhase::PREFLIGHT);
}

TEST_F(MissionManagerNodeTest, HealthMonitorFaultStaysTerminalAndNotifiesRestartOnce)
{
  advance_to_ingress();
  mc_msgs::msg::FaultEvent fault;
  fault.fault_id = mc_msgs::msg::FaultEvent::FCC_LINK_LOST;
  fault.severity = 2;
  pub_fault_->publish(fault);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::seconds(2)));
  wait_for_ticks(3);
  EXPECT_EQ(restart_notice_count(), 0U);  // still armed and flying

  pub_vehicle_state_->publish(good_vehicle_state());  // FCC RTL'd, landed, disarmed
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return restart_notice_count() > 0;},
      std::chrono::seconds(2)));
  EXPECT_EQ(status_texts_.back()->severity, mc_msgs::msg::StatusText::SEVERITY_CRITICAL);
  EXPECT_LE(status_texts_.back()->text.size(), 50U);  // ICD §4.7 relay limit

  // Every way out is refused: a phase request, and a fresh upload.
  const size_t acks_before = command_acks_.size();
  send_gcs_command(50001 /*MC_CMD_PHASE_TRANSITION*/, {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this, acks_before] {return command_acks_.size() > acks_before;},
      std::chrono::seconds(2)));
  EXPECT_EQ(command_acks_.back()->result, mc_msgs::msg::GcsCommandAck::RESULT_REJECTED);
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  wait_for_ticks(10);
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::FAULT);
  EXPECT_EQ(restart_notice_count(), 1U);
}

TEST_F(MissionManagerNodeTest, GcsEmergencyFaultStaysTerminalAndNotifiesRestartOnce)
{
  pub_mission_upload_->publish(full_mission_upload(true));
  pub_vehicle_state_->publish(good_vehicle_state());
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::ARMING);},
      std::chrono::seconds(2)));
  auto vs = good_vehicle_state();
  vs.arming_state = kArmingStateArmed;
  pub_vehicle_state_->publish(vs);
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::TAKEOFF);},
      std::chrono::seconds(2)));

  // Operator takeover (MANUAL) is EMERGENCY-tier: FAULT, by design.
  send_gcs_command(kMavCmdDoSetMode, {1.F, 1.F, 0.F, 0.F, 0.F, 0.F, 0.F});
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return phase_reached(mc_msgs::msg::MissionPhase::FAULT);},
      std::chrono::seconds(2)));

  pub_vehicle_state_->publish(good_vehicle_state());  // pilot landed and disarmed
  ASSERT_TRUE(
    spin_until(
      {node_, harness_}, [this] {return restart_notice_count() > 0;},
      std::chrono::seconds(2)));
  pub_vehicle_state_->publish(good_vehicle_state());
  wait_for_ticks(10);
  EXPECT_EQ(node_->current_phase(), mission_exec::Phase::FAULT);
  EXPECT_EQ(restart_notice_count(), 1U);
}
