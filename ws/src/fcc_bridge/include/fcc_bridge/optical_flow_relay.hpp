// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <mavlink/v2.0/common/mavlink.h>

#include <mutex>

#include "fcc_bridge/baro_altitude_sample.hpp"
#include "fcc_bridge/optical_flow_sample.hpp"

namespace fcc_bridge
{

// Decodes HIF-008 (Holybro H-Flow)'s two MAVLink messages -- OPTICAL_FLOW_RAD
// and DISTANCE_SENSOR -- into an OpticalFlowSample (ICD §3.3a, HID-MC-001
// §10), plus ALTITUDE's raw-baro altitude_monotonic into a
// BaroAltitudeSample -- gnss_sim_node's vertical input, carried on the same
// channel since it is the same consumer (FT-006 finding). Shared by
// XrceDdsBackend (dedicated flow-relay port, always ethernet) and
// MavlinkUdpBackend (multiplexed over its existing SITL
// socket, if/when PX4 SITL's own simulated flow sensor sends these) — the
// same "one client class, two socket owners" split MissionProtocolClient
// already establishes for the mission protocol.
//
// Not itself a FccBackend — a helper owned by one. Thread-safe: handle_frame()
// and get_sample() may be called from different threads (the backend's own
// recv thread and fcc_bridge_node's publish-timer thread, respectively).
class OpticalFlowRelay
{
public:
  // Call for every parsed MAVLink frame — messages other than
  // OPTICAL_FLOW_RAD/DISTANCE_SENSOR/ALTITUDE are ignored.
  void handle_frame(const mavlink_message_t & msg);

  OpticalFlowSample get_sample() const;
  BaroAltitudeSample get_baro_altitude() const;

private:
  void handle_optical_flow_rad(const mavlink_message_t & msg);
  void handle_distance_sensor(const mavlink_message_t & msg);
  void handle_altitude(const mavlink_message_t & msg);

  mutable std::mutex mtx_;
  OpticalFlowSample sample_;
  BaroAltitudeSample baro_;
  // OPTICAL_FLOW_RAD's own `distance` field is -1 (unavailable) unless the
  // sensor pairs a rangefinder reading with the same integration window
  // (PX4's VehicleOpticalFlow.cpp averages one in when available) -- falls
  // back to the most recent standalone DISTANCE_SENSOR reading otherwise.
  // Only ever touched from handle_frame()'s caller thread (both handlers
  // are dispatched sequentially by whichever backend owns this relay) --
  // no lock needed, unlike sample_ which get_sample() also reads from a
  // different thread.
  float last_distance_sensor_m_{-1.0F};
};

}  // namespace fcc_bridge
