// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/node.hpp"

#include "fcc_bridge/baro_altitude_sample.hpp"
#include "fcc_bridge/mission_item.hpp"
#include "fcc_bridge/offboard_setpoint.hpp"
#include "fcc_bridge/optical_flow_sample.hpp"
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
  // ingress/muster legs). Returns true iff MISSION_ACK/MAV_MISSION_ACCEPTED
  // was received before timeout (FCC-006) — false on timeout or rejection.
  virtual bool upload_mission(const std::vector<MissionItem> & items) = 0;

  // Replace a single mission item in place — used for the moving
  // FORMATION_SLOT target during INGRESS when the leg is in MISSION mode
  // (SDD §5.2). Same ack-propagation contract as upload_mission() above.
  virtual bool update_mission_item(uint16_t seq, const MissionItem & item) = 0;

  // Stream a continuous OFFBOARD position setpoint (SDD §4.1a/§5.3a) — used
  // instead of update_mission_item() for a leg the active mission plan
  // flags for OFFBOARD (FR-020/026-029). The backend does not buffer or
  // extrapolate between calls: the caller (formation_controller_node) is
  // responsible for sustaining >= 2 Hz while OFFBOARD is the commanded
  // mode. Does not itself send DO_SET_MODE OFFBOARD — mode commands stay in
  // send_command() below, called exclusively by mission_manager_node
  // (FR-012, FR-026).
  virtual void stream_offboard_setpoint(const OffboardSetpoint & sp) = 0;

  // Send a vehicle_command (arm/disarm, mode change, RTL, HOLD, ...).
  virtual void send_command(uint32_t cmd, const std::array<float, 7> & params) = 0;

  // Relay visual-odometry feedback (GNSS-denied localisation).
  virtual void publish_visual_odometry(const VisualOdometry & vo) = 0;

  // Latest aggregated vehicle state (thread-safe).
  virtual VehicleState get_vehicle_state() const = 0;

  // Latest HIF-008 (Holybro H-Flow) downward optical-flow + rangefinder
  // sample, relayed over the dedicated flow MAVLink channel (ICD §3.3a,
  // HID-MC-001 §10). OpticalFlowSample::valid is false until at least one
  // frame has been received -- MavlinkUdpBackend (SITL) has no simulated
  // H-Flow and always reports invalid.
  virtual OpticalFlowSample get_optical_flow() const = 0;

  // Latest raw barometric altitude (MAVLink ALTITUDE.altitude_monotonic),
  // relayed over the same flow MAVLink channel (ICD §3.3a) -- never EKF2
  // output. BaroAltitudeSample::valid is false until the first frame.
  virtual BaroAltitudeSample get_baro_altitude() const = 0;

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
