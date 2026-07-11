// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <cstdint>

#include "rclcpp/time.hpp"

namespace fcc_bridge
{

// MAVLink MAV_FRAME / MAV_CMD values used by mission items (ICD §3.3).
// Only the subset FccBackend actually emits — not a full MAVLink enum.
namespace mav
{
constexpr uint8_t FRAME_GLOBAL_INT = 5;
constexpr uint16_t CMD_NAV_WAYPOINT = 16;
constexpr uint16_t CMD_NAV_LOITER_UNLIM = 17;
}  // namespace mav

// Reserved mission slot the formation controller repeatedly replaces to
// track the leader during INGRESS (SDD §5, ICD §3.3).
constexpr uint16_t FORMATION_SLOT = 0;

// A single PX4 mission item, exchanged over the MAVLink mission protocol
// (ICD §3.3 "Mission Protocol") — never over uXRCE-DDS (IR-007).
struct MissionItem
{
  uint16_t seq{0};
  uint8_t frame{mav::FRAME_GLOBAL_INT};
  uint16_t command{mav::CMD_NAV_WAYPOINT};
  int32_t lat{0};              // 1e7 deg
  int32_t lon{0};              // 1e7 deg
  float alt{0.0F};             // m MSL
  float acceptance_radius{0.0F};  // m
  rclcpp::Time stamp{};
};

}  // namespace fcc_bridge
