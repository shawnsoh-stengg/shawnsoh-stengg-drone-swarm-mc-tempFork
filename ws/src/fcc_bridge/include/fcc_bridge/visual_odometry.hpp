// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <array>

#include "rclcpp/time.hpp"

namespace fcc_bridge
{

// Visual-odometry estimate handed to FccBackend::publish_visual_odometry()
// by visual_localisation_node when GNSS quality drops below threshold
// (SDD §3, ICD §3.3 vehicle_visual_odometry). NED frame.
struct VisualOdometry
{
  std::array<float, 3> position{0.0F, 0.0F, 0.0F};
  std::array<float, 4> q{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> velocity{0.0F, 0.0F, 0.0F};
  std::array<float, 3> position_variance{0.0F, 0.0F, 0.0F};
  rclcpp::Time stamp{};
};

}  // namespace fcc_bridge
