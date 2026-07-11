// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/node.hpp"

#include "fcc_bridge/mission_item.hpp"
#include "fcc_bridge/vehicle_state.hpp"
#include "fcc_bridge/visual_odometry.hpp"

// FCC-001: transport-agnostic MC<->FCC interface (SDD §3.1, ICD §3.4).
// Application nodes never include a concrete backend header — only this one.
namespace fcc_bridge
{

class FccBackend
{
public:
  virtual ~FccBackend() = default;

  // Upload or fully replace the active FCC mission (search paths,
  // ingress/muster legs). MISSION mode only — there is no OFFBOARD setpoint
  // path (SDD §5.3).
  virtual void upload_mission(const std::vector<MissionItem> & items) = 0;

  // Replace a single mission item in place — used for the moving
  // FORMATION_SLOT target during INGRESS (SDD §5.2).
  virtual void update_mission_item(uint16_t seq, const MissionItem & item) = 0;

  // Send a vehicle_command (arm/disarm, mode change, RTL, HOLD, ...).
  virtual void send_command(uint32_t cmd, const std::array<float, 7> & params) = 0;

  // Relay visual-odometry feedback (GNSS-denied localisation).
  virtual void publish_visual_odometry(const VisualOdometry & vo) = 0;

  // Latest aggregated vehicle state (thread-safe).
  virtual VehicleState get_vehicle_state() const = 0;

  // True if the FCC link has produced a message within the last
  // timeout_s seconds.
  virtual bool is_link_alive(double timeout_s = 1.0) const = 0;
};

// Factory: reads the SIMULATION_MODE environment variable and returns the
// appropriate implementation. Logs the selected transport at startup
// (FCC-001 AC). `node` is retained by the returned backend for logging,
// clock access, and topic/socket setup — it must outlive the backend.
std::unique_ptr<FccBackend> make_fcc_backend(rclcpp::Node * node);

}  // namespace fcc_bridge
