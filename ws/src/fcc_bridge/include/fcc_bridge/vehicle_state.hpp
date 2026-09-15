// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#pragma once

#include <array>
#include <cstdint>

#include "rclcpp/time.hpp"

namespace fcc_bridge
{

// Aggregated FCC vehicle state (SDD §3.2). Populated by FccBackend
// implementations from whatever transport-specific messages they receive;
// application nodes never see the underlying transport types.
struct VehicleState
{
  // Position/velocity, NED frame (m, m/s) — from vehicle_local_position
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float vx{0.0F};
  float vy{0.0F};
  float vz{0.0F};
  bool xy_valid{false};
  bool z_valid{false};

  // Geodetic position — from vehicle_global_position
  double lat{0.0};
  double lon{0.0};
  float alt_msl{0.0F};

  // Attitude quaternion [w, x, y, z], body FRD -> NED — from vehicle_attitude
  std::array<float, 4> q{1.0F, 0.0F, 0.0F, 0.0F};

  // Heading, rad, from North — from vehicle_local_position
  float heading{0.0F};

  // From battery_status
  float battery_remaining{-1.0F};

  // From vehicle_status
  uint8_t arming_state{0};
  uint8_t nav_state{0};

  // GNSS quality (TEST-002 addition -- see mavlink_udp_backend.hpp's
  // handle_gps_raw_int() doc comment for why this was missing before).
  // fix_type/hdop/vdop default to "no fix"/"unknown" until a real
  // GPS_RAW_INT arrives, matching mc_msgs/VehicleState.msg's own
  // documented gnss_ok semantics (fix_type >= 3 AND hdop < threshold).
  uint8_t fix_type{0};
  float hdop{0.0F};
  float vdop{0.0F};
  bool gnss_ok{false};

  // DEP-009: populated directly from vehicle_status.failure_detector_status
  // (ICD §3.2), the real bitmask field on the px4_msgs release/1.17 schema.
  uint16_t failure_flags{0};

  rclcpp::Time stamp{};
};

}  // namespace fcc_bridge
