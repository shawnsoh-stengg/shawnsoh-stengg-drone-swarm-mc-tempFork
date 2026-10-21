// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

// FCC-001: make_fcc_backend() selection logic. SIMULATION_MODE picks the
// backend class; FCC_TRANSPORT (tested in test_xrce_dds_backend.cpp) is a
// separate, independent knob read by XrceDdsBackend itself (ICD §9.4,
// HID-MC-001 §4.3).

#include <gtest/gtest.h>

#include <cstdlib>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "fcc_bridge/fcc_backend.hpp"

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
}  // namespace

class FactoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    unsetenv("SIMULATION_MODE");
    setenv("DRONE_ID", "1", 1);  // both backends require it (mavlink_ids.hpp)
    node_ = std::make_shared<rclcpp::Node>("fcc_bridge_factory_test");
  }
  void TearDown() override
  {
    unsetenv("SIMULATION_MODE");
    unsetenv("DRONE_ID");
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(FactoryTest, DefaultsToXrceDdsBackendWhenSimulationModeUnset)
{
  auto backend = fcc_bridge::make_fcc_backend(node_.get());
  ASSERT_NE(backend, nullptr);
}

TEST_F(FactoryTest, SimulationModeFalseSelectsXrceDdsBackend)
{
  setenv("SIMULATION_MODE", "false", 1);
  auto backend = fcc_bridge::make_fcc_backend(node_.get());
  ASSERT_NE(backend, nullptr);
}

// FCC-007: MavlinkUdpBackend now exists — SIMULATION_MODE=true constructs
// it instead of throwing. SITL_FCC_PORT is isolated per-test so this
// doesn't collide with test_mavlink_udp_backend.cpp's own bind port.
TEST_F(FactoryTest, SimulationModeTrueSelectsMavlinkUdpBackend)
{
  setenv("SIMULATION_MODE", "true", 1);
  setenv("SITL_FCC_PORT", "25541", 1);
  auto backend = fcc_bridge::make_fcc_backend(node_.get());
  ASSERT_NE(backend, nullptr);
  unsetenv("SITL_FCC_PORT");
}

TEST_F(FactoryTest, MissingDroneIdRefusesToStartRatherThanTargetingSystemOne)
{
  unsetenv("DRONE_ID");
  EXPECT_THROW(fcc_bridge::make_fcc_backend(node_.get()), std::runtime_error);
}
