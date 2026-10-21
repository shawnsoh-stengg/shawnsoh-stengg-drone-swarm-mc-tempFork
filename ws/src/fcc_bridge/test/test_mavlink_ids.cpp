// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3.1, ICD-MC-001 §3).

// The FCC's system ID comes only from DRONE_ID: no fallback to PX4's default
// 1, which an FCC with any other MAV_SYS_ID silently ignores.

#include <gtest/gtest.h>

#include <cstdlib>
#include <stdexcept>

#include "fcc_bridge/mavlink_ids.hpp"

using fcc_bridge::drone_id_from_env;

class MavlinkIdsTest : public ::testing::Test
{
protected:
  void TearDown() override {unsetenv("DRONE_ID");}
};

TEST_F(MavlinkIdsTest, ReadsDroneId)
{
  setenv("DRONE_ID", "2", 1);
  EXPECT_EQ(drone_id_from_env(), 2);
  setenv("DRONE_ID", "255", 1);
  EXPECT_EQ(drone_id_from_env(), 255);
}

TEST_F(MavlinkIdsTest, UnsetOrEmptyThrowsInsteadOfDefaultingToOne)
{
  unsetenv("DRONE_ID");
  EXPECT_THROW(drone_id_from_env(), std::runtime_error);
  setenv("DRONE_ID", "", 1);
  EXPECT_THROW(drone_id_from_env(), std::runtime_error);
}

TEST_F(MavlinkIdsTest, OutOfRangeOrNonNumericThrows)
{
  for (const char * bad : {"0", "256", "-1", "abc", "2x", "1.5"}) {
    setenv("DRONE_ID", bad, 1);
    EXPECT_THROW(drone_id_from_env(), std::runtime_error) << "DRONE_ID=" << bad;
  }
}

TEST_F(MavlinkIdsTest, ComponentIdsAreMavlinkNamedConstants)
{
  EXPECT_EQ(fcc_bridge::kFccComponentId, 1);
  EXPECT_EQ(fcc_bridge::kMcComponentId, 191);
}
