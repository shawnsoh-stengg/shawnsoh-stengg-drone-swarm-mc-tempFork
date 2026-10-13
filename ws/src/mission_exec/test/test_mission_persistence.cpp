// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

// EXEC-004: save_mission_upload()/load_mission_upload() unit tests — pure
// gtest, no rclcpp::init required (mission_persistence.hpp/.cpp take no
// dependency on rclcpp::Node or ROS time, matching pre_arm_checks.hpp's own
// "framework-free" precedent).

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mc_msgs/msg/formation_offset.hpp>
#include <mc_msgs/msg/swarm_peer.hpp>

#include "mission_exec/mission_persistence.hpp"

namespace mission_exec
{
namespace
{

class MissionPersistenceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    path_ = ::testing::TempDir() + "mission_upload_test_" + test_info->name() + ".yaml";
    std::remove(path_.c_str());
    std::remove((path_ + ".tmp").c_str());
  }

  void TearDown() override
  {
    std::remove(path_.c_str());
    std::remove((path_ + ".tmp").c_str());
  }

  std::string path_;
};

mc_msgs::msg::MissionUpload sample_upload()
{
  mc_msgs::msg::MissionUpload up;
  up.header.stamp.sec = 1723500000;
  up.header.stamp.nanosec = 123456789;
  up.header.frame_id = "map";
  up.upload_id = 42;

  geometry_msgs::msg::PoseStamped wp1;
  wp1.header.stamp.sec = 1;
  wp1.header.frame_id = "map";
  wp1.pose.position.x = 10.5;
  wp1.pose.position.y = -3.25;
  wp1.pose.position.z = 30.0;
  wp1.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped wp2;
  wp2.pose.position.x = 20.0;
  up.ingress_waypoints = {wp1, wp2};

  mc_msgs::msg::FormationOffset offset;
  offset.drone_id = 3;
  offset.n = 1.0F;
  offset.e = -2.0F;
  offset.d = 0.5F;
  up.formation_offsets = {offset};

  geographic_msgs::msg::GeoPoint pt1;
  pt1.latitude = 1.35;
  pt1.longitude = 103.8;
  pt1.altitude = 50.0;
  geographic_msgs::msg::GeoPoint pt2;
  pt2.latitude = 1.36;
  pt2.longitude = 103.81;
  up.search_polygon = {pt1, pt2};

  mc_msgs::msg::SwarmPeer peer;
  peer.drone_id = 7;
  peer.ip_address = "192.168.100.12";
  peer.mission_phase = 3;
  peer.lat = 1.3;
  peer.lon = 103.8;
  peer.alt_msl = 45.0F;
  peer.vx = 1.0F;
  peer.vy = 2.0F;
  peer.vz = -0.5F;
  peer.battery_pct = 0.85F;
  peer.sensor_flags = 0xAB;
  peer.fault_mask = 0x1234;
  peer.peer_status = mc_msgs::msg::SwarmPeer::STATUS_ALIVE;
  peer.is_leader = true;
  peer.isr_role = false;
  peer.last_heartbeat_us = 1723500000000000ULL;
  up.swarm_manifest = {peer};

  up.arm_authorised = true;
  up.low_battery_threshold = 0.20F;
  up.offboard_phase_mask = 0x0006;
  return up;
}

TEST_F(MissionPersistenceTest, LoadReturnsNulloptWhenFileDoesNotExist)
{
  EXPECT_FALSE(load_mission_upload(path_).has_value());
}

TEST_F(MissionPersistenceTest, LoadReturnsNulloptOnCorruptFile)
{
  std::ofstream f(path_);
  f << "not: [valid, yaml structure for a mission upload\n";
  f.close();
  EXPECT_FALSE(load_mission_upload(path_).has_value());
}

TEST_F(MissionPersistenceTest, SaveThenLoadRoundTripsAllFields)
{
  const auto original = sample_upload();
  ASSERT_TRUE(save_mission_upload(path_, original));

  const auto loaded = load_mission_upload(path_);
  ASSERT_TRUE(loaded.has_value());

  EXPECT_EQ(loaded->header.stamp.sec, original.header.stamp.sec);
  EXPECT_EQ(loaded->header.stamp.nanosec, original.header.stamp.nanosec);
  EXPECT_EQ(loaded->header.frame_id, original.header.frame_id);
  EXPECT_EQ(loaded->upload_id, original.upload_id);

  ASSERT_EQ(loaded->ingress_waypoints.size(), original.ingress_waypoints.size());
  EXPECT_DOUBLE_EQ(
    loaded->ingress_waypoints[0].pose.position.x, original.ingress_waypoints[0].pose.position.x);
  EXPECT_DOUBLE_EQ(
    loaded->ingress_waypoints[0].pose.position.y, original.ingress_waypoints[0].pose.position.y);
  EXPECT_EQ(loaded->ingress_waypoints[0].header.frame_id, "map");

  ASSERT_EQ(loaded->formation_offsets.size(), 1U);
  EXPECT_EQ(loaded->formation_offsets[0].drone_id, 3);
  EXPECT_FLOAT_EQ(loaded->formation_offsets[0].n, 1.0F);
  EXPECT_FLOAT_EQ(loaded->formation_offsets[0].e, -2.0F);
  EXPECT_FLOAT_EQ(loaded->formation_offsets[0].d, 0.5F);

  ASSERT_EQ(loaded->search_polygon.size(), 2U);
  EXPECT_DOUBLE_EQ(loaded->search_polygon[0].latitude, 1.35);
  EXPECT_DOUBLE_EQ(loaded->search_polygon[1].longitude, 103.81);

  ASSERT_EQ(loaded->swarm_manifest.size(), 1U);
  const auto & peer = loaded->swarm_manifest[0];
  EXPECT_EQ(peer.drone_id, 7);
  EXPECT_EQ(peer.ip_address, "192.168.100.12");
  EXPECT_EQ(peer.mission_phase, 3);
  EXPECT_EQ(peer.sensor_flags, 0xAB);
  EXPECT_EQ(peer.fault_mask, 0x1234);
  EXPECT_EQ(peer.peer_status, mc_msgs::msg::SwarmPeer::STATUS_ALIVE);
  EXPECT_TRUE(peer.is_leader);
  EXPECT_FALSE(peer.isr_role);
  EXPECT_EQ(peer.last_heartbeat_us, 1723500000000000ULL);

  EXPECT_TRUE(loaded->arm_authorised);
  EXPECT_FLOAT_EQ(loaded->low_battery_threshold, 0.20F);
  EXPECT_EQ(loaded->offboard_phase_mask, 0x0006);
}

TEST_F(MissionPersistenceTest, SaveDoesNotLeaveTmpFileBehindOnSuccess)
{
  ASSERT_TRUE(save_mission_upload(path_, sample_upload()));
  std::ifstream tmp(path_ + ".tmp");
  EXPECT_FALSE(tmp.good());
}

TEST_F(MissionPersistenceTest, SaveOverwritesPreviousContentAtomically)
{
  auto first = sample_upload();
  first.upload_id = 1;
  ASSERT_TRUE(save_mission_upload(path_, first));

  auto second = sample_upload();
  second.upload_id = 2;
  second.ingress_waypoints.clear();
  ASSERT_TRUE(save_mission_upload(path_, second));

  const auto loaded = load_mission_upload(path_);
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->upload_id, 2U);
  EXPECT_TRUE(loaded->ingress_waypoints.empty());
}

TEST_F(MissionPersistenceTest, SaveFailsGracefullyOnUnwritableDirectory)
{
  EXPECT_FALSE(save_mission_upload("/nonexistent_dir_xyz/mission_upload.yaml", sample_upload()));
}

// --- EXEC-006: remove_mission_upload() ---

TEST_F(MissionPersistenceTest, RemoveDeletesSavedUploadSoLoadReturnsNullopt)
{
  ASSERT_TRUE(save_mission_upload(path_, sample_upload()));
  ASSERT_TRUE(load_mission_upload(path_).has_value());

  EXPECT_TRUE(remove_mission_upload(path_));
  EXPECT_FALSE(load_mission_upload(path_).has_value());
}

TEST_F(MissionPersistenceTest, RemoveSucceedsWhenNothingWasSaved)
{
  EXPECT_TRUE(remove_mission_upload(path_));
}

}  // namespace
}  // namespace mission_exec
