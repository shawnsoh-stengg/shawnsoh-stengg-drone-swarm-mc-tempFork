// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

// EXEC-004: node-integration test for SDD §4.5 mission-upload disk
// persistence — proves the actual wiring (on_gcs_mission_upload() saves,
// the constructor reloads), not just the framework-free save/load functions
// covered by test_mission_persistence.cpp. Mirrors test_mission_manager_
// node.cpp's own harness-node technique (TP-MC-001 §2.4) rather than
// launch_testing infrastructure.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mc_msgs/msg/formation_offset.hpp>
#include <mc_msgs/msg/mission_phase.hpp>
#include <mc_msgs/msg/mission_upload.hpp>
#include <mc_msgs/msg/swarm_peer.hpp>
#include <mc_msgs/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_exec/mission_manager_node.hpp"
#include "mission_exec/mission_persistence.hpp"

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

constexpr uint8_t kArmingStateStandby = 1;

mc_msgs::msg::VehicleState good_vehicle_state(const rclcpp::Node & clock_source)
{
  mc_msgs::msg::VehicleState vs;
  vs.header.stamp = clock_source.now();
  vs.lat = 1.3;
  vs.lon = 103.8;
  vs.alt_msl = 40.0F;
  vs.arming_state = kArmingStateStandby;
  vs.battery_pct = 1.0F;
  vs.gnss_ok = true;
  vs.fcc_link_ok = true;
  return vs;
}

mc_msgs::msg::MissionUpload full_mission_upload(const rclcpp::Node & clock_source)
{
  mc_msgs::msg::MissionUpload up;
  up.header.stamp = clock_source.now();
  up.upload_id = 99;
  geometry_msgs::msg::PoseStamped wp;
  up.ingress_waypoints = {wp};
  mc_msgs::msg::FormationOffset offset;
  offset.drone_id = 1;
  up.formation_offsets = {offset};
  geographic_msgs::msg::GeoPoint pt;
  up.search_polygon = {pt, pt, pt};
  mc_msgs::msg::SwarmPeer peer;
  peer.drone_id = 1;
  up.swarm_manifest = {peer};
  up.arm_authorised = true;
  up.low_battery_threshold = 0.20F;
  return up;
}

class MissionManagerNodePersistenceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = ::testing::TempDir() + "mission_manager_node_persistence_test_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".yaml";
    std::remove(path_.c_str());
    std::remove((path_ + ".tmp").c_str());
  }

  void TearDown() override
  {
    std::remove(path_.c_str());
    std::remove((path_ + ".tmp").c_str());
  }

  rclcpp::NodeOptions node_options() const
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(
      {
        rclcpp::Parameter("control_loop_period_s", 0.02),
        rclcpp::Parameter("mission_upload_path", path_),
    });
    return opts;
  }

  std::string path_;
};

TEST_F(MissionManagerNodePersistenceTest, GcsUploadIsPersistedToDisk)
{
  auto node = std::make_shared<mission_exec::MissionManagerNode>(node_options());
  auto harness = std::make_shared<rclcpp::Node>("persistence_test_harness_1");
  auto pub_mission_upload = harness->create_publisher<mc_msgs::msg::MissionUpload>(
    "/mc/gcs/mission_upload", rclcpp::QoS(10));

  ASSERT_TRUE(spin_until(
      {node, harness}, [&] {return pub_mission_upload->get_subscription_count() > 0;},
    std::chrono::seconds(2)));

  pub_mission_upload->publish(full_mission_upload(*harness));

  ASSERT_TRUE(spin_until(
      {node, harness}, [&] {
        std::ifstream f(path_);
        return f.good();
      }, std::chrono::seconds(2)));

  const auto loaded = mission_exec::load_mission_upload(path_);
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->upload_id, 99U);
  EXPECT_TRUE(loaded->arm_authorised);
}

// The core acceptance criterion: a fresh node instance pointed at the same
// path reloads the persisted upload at construction time — proven
// behaviourally (autonomous PREFLIGHT -> ARMING with no live GCS mission
// upload this session) rather than via a private-state getter, since that
// is the actual user-visible guarantee ("a node restart does not require a
// fresh GCS upload").
TEST_F(MissionManagerNodePersistenceTest, RestartedNodeReloadsUploadAndArmsWithoutFreshGcsUpload)
{
  {
    auto node = std::make_shared<mission_exec::MissionManagerNode>(node_options());
    auto harness = std::make_shared<rclcpp::Node>("persistence_test_harness_2a");
    auto pub_mission_upload = harness->create_publisher<mc_msgs::msg::MissionUpload>(
      "/mc/gcs/mission_upload", rclcpp::QoS(10));

    ASSERT_TRUE(spin_until(
        {node, harness}, [&] {return pub_mission_upload->get_subscription_count() > 0;},
      std::chrono::seconds(2)));
    pub_mission_upload->publish(full_mission_upload(*harness));
    ASSERT_TRUE(spin_until(
        {node, harness}, [&] {
          std::ifstream f(path_);
          return f.good();
        }, std::chrono::seconds(2)));
  }
  // node/harness above are destroyed here — simulates the watchdog-restarted
  // process having no in-memory state left at all.

  auto node2 = std::make_shared<mission_exec::MissionManagerNode>(node_options());
  auto harness2 = std::make_shared<rclcpp::Node>("persistence_test_harness_2b");
  auto pub_vehicle_state = harness2->create_publisher<mc_msgs::msg::VehicleState>(
    "/mc/fcc/vehicle_state", rclcpp::QoS(10));
  auto sub_phase = harness2->create_subscription<mc_msgs::msg::MissionPhase>(
    "/mc/mission_phase", rclcpp::QoS(10),
    [](mc_msgs::msg::MissionPhase::SharedPtr) {});

  ASSERT_TRUE(spin_until(
      {node2, harness2}, [&] {return pub_vehicle_state->get_subscription_count() > 0;},
    std::chrono::seconds(2)));

  // Deliberately no /mc/gcs/mission_upload publish to node2 — the only
  // mission upload it ever sees comes from load_mission_upload() in its own
  // constructor, per the SDD §4.5 acceptance criterion.
  pub_vehicle_state->publish(good_vehicle_state(*harness2));

  ASSERT_TRUE(spin_until(
      {node2, harness2},
      [&] {return node2->current_phase() == mission_exec::Phase::ARMING;},
      std::chrono::seconds(2)))
    << "node2 never reached ARMING — reloaded mission_upload_/arm_authorised_ "
       "were not applied at startup";
}

}  // namespace
