// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <cstdint>

namespace fcc_bridge
{

// Raw barometric altitude from the FCC, decoded from MAVLink ALTITUDE's
// altitude_monotonic on the flow-relay channel (ICD §3.3a). PX4 fills that
// field straight from vehicle_air_data.baro_alt_meter (streams/ALTITUDE.hpp,
// confirmed against the real v1.17.0 tag) -- the barometer alone, never the
// EKF2 estimate, which is the point: gnss_sim_node's vertical channel must
// not be derived from an estimator its own HIL_GPS output feeds (FT-006
// finding). Stock v1.17 bridges no baro topic over uXRCE-DDS. Mirrors
// mc_msgs/BaroAltitude's field shape.
struct BaroAltitudeSample
{
  float alt_m{0.0F};  // m, baro altitude (standard atmosphere, not MSL-corrected)
  // Increments on every decoded frame, so a publisher can tell a fresh
  // sample from a repeat of the last one.
  uint32_t seq{0};
  // False until at least one ALTITUDE frame with a finite
  // altitude_monotonic has been decoded.
  bool valid{false};
};

}  // namespace fcc_bridge
