// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

namespace fcc_bridge
{

// Continuous OFFBOARD position setpoint handed to
// FccBackend::stream_offboard_setpoint() by formation_controller_node while
// an OFFBOARD-flagged leg is active (SDD §4.1a/§5.3a, ICD §9.2). NED frame,
// position + yaw only — mirrors the FORMATION_SLOT mission-item-replace
// path this supersedes on OFFBOARD-flagged legs (SDD §5.2).
struct OffboardSetpoint
{
  float x{0.0F};    // N, m
  float y{0.0F};    // E, m
  float z{0.0F};    // D, m
  float yaw{0.0F};  // rad, NED
};

}  // namespace fcc_bridge
