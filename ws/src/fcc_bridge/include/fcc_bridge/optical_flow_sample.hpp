// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <cstdint>

namespace fcc_bridge
{

// HIF-008 (Holybro H-Flow via Pixhawk CAN1) downward optical-flow +
// rangefinder sample, aggregated by a backend from the dedicated flow
// MAVLink channel (ICD §3.3a, HID-MC-001 §10). Mirrors mc_msgs/OpticalFlow's
// field shape exactly — fcc_bridge_node does the struct-to-message copy.
struct OpticalFlowSample
{
  float vx{0.0F};               // m/s, body FRD X (forward)
  float vy{0.0F};                // m/s, body FRD Y (right)
  float ground_distance{0.0F};  // m, paired rangefinder height AGL
  uint8_t quality{0};           // 0-255, sensor-reported confidence
  // Increments on every decoded OPTICAL_FLOW_RAD frame, so fcc_bridge_node
  // publishes each frame once instead of re-publishing the last one forever
  // after the stream stops (which would hide the stall from gnss_sim_node's
  // IGS-005 staleness watchdog).
  uint32_t seq{0};
  // False until at least one OPTICAL_FLOW_RAD frame has been decoded.
  // MavlinkUdpBackend (SITL) has no simulated H-Flow and always reports
  // false here.
  bool valid{false};
};

}  // namespace fcc_bridge
