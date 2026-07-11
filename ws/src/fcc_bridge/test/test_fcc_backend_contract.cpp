// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

// FCC-001: FccBackend is a pure abstract interface — this test exercises it
// through a minimal fake implementation to confirm the contract (every
// method reachable via a base-class pointer, defaults are sane) without
// depending on any concrete transport.

#include <gtest/gtest.h>

#include "fcc_bridge/fcc_backend.hpp"

namespace
{

class FakeBackend : public fcc_bridge::FccBackend
{
public:
  void upload_mission(const std::vector<fcc_bridge::MissionItem> & items) override
  {
    last_uploaded_mission_ = items;
  }

  void update_mission_item(uint16_t seq, const fcc_bridge::MissionItem & item) override
  {
    last_updated_seq_ = seq;
    last_updated_item_ = item;
  }

  void send_command(uint32_t cmd, const std::array<float, 7> & params) override
  {
    last_command_ = cmd;
    last_params_ = params;
  }

  void publish_visual_odometry(const fcc_bridge::VisualOdometry & vo) override
  {
    last_vo_ = vo;
  }

  fcc_bridge::VehicleState get_vehicle_state() const override {return state_;}

  bool is_link_alive(double /*timeout_s*/) const override {return link_alive_;}

  fcc_bridge::VehicleState state_{};
  bool link_alive_{true};
  std::vector<fcc_bridge::MissionItem> last_uploaded_mission_;
  uint16_t last_updated_seq_{0xFFFF};
  fcc_bridge::MissionItem last_updated_item_{};
  uint32_t last_command_{0};
  std::array<float, 7> last_params_{};
  fcc_bridge::VisualOdometry last_vo_{};
};

}  // namespace

TEST(FccBackendContract, UploadMissionReachesImplementationThroughBasePointer)
{
  FakeBackend fake;
  fcc_bridge::FccBackend * backend = &fake;

  std::vector<fcc_bridge::MissionItem> items(3);
  items[1].seq = 1;
  items[1].command = fcc_bridge::mav::CMD_NAV_WAYPOINT;
  backend->upload_mission(items);

  ASSERT_EQ(fake.last_uploaded_mission_.size(), 3U);
  EXPECT_EQ(fake.last_uploaded_mission_[1].command, fcc_bridge::mav::CMD_NAV_WAYPOINT);
}

TEST(FccBackendContract, UpdateMissionItemUsesFormationSlotByConvention)
{
  FakeBackend fake;
  fcc_bridge::FccBackend * backend = &fake;

  fcc_bridge::MissionItem item;
  item.lat = 10000000;
  item.lon = 20000000;
  backend->update_mission_item(fcc_bridge::FORMATION_SLOT, item);

  EXPECT_EQ(fake.last_updated_seq_, fcc_bridge::FORMATION_SLOT);
  EXPECT_EQ(fake.last_updated_item_.lat, 10000000);
}

TEST(FccBackendContract, SendCommandForwardsAllSevenParams)
{
  FakeBackend fake;
  fcc_bridge::FccBackend * backend = &fake;

  std::array<float, 7> params{1, 2, 3, 4, 5, 6, 7};
  backend->send_command(400, params);

  EXPECT_EQ(fake.last_command_, 400U);
  EXPECT_EQ(fake.last_params_, params);
}

TEST(FccBackendContract, IsLinkAliveReflectsImplementation)
{
  FakeBackend fake;
  fcc_bridge::FccBackend * backend = &fake;

  fake.link_alive_ = false;
  EXPECT_FALSE(backend->is_link_alive(1.0));

  fake.link_alive_ = true;
  EXPECT_TRUE(backend->is_link_alive(1.0));
}

TEST(MissionItemDefaults, UseGlobalIntFrameAndNavWaypointByDefault)
{
  fcc_bridge::MissionItem item;
  EXPECT_EQ(item.frame, fcc_bridge::mav::FRAME_GLOBAL_INT);
  EXPECT_EQ(item.command, fcc_bridge::mav::CMD_NAV_WAYPOINT);
  EXPECT_EQ(item.seq, 0U);
}

TEST(FormationSlot, IsReservedAsSequenceZero)
{
  EXPECT_EQ(fcc_bridge::FORMATION_SLOT, 0U);
}
