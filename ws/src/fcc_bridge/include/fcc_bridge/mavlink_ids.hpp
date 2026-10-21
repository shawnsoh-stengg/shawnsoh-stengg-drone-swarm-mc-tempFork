// MAVLink identities fcc_bridge uses toward the FCC, from configuration
// only. DRONE_ID (/etc/drone-mc/environment) is both this MC's system ID and
// its FCC's: a companion computer shares its vehicle's system ID, and the
// FCC's MAV_SYS_ID must equal DRONE_ID (docs/DEPLOYMENT.md "MAV_SYS_ID /
// DRONE_ID"). PX4 silently drops vehicle commands and mission items
// addressed to any other system ID (v1.17 Commander::handle_command(),
// mavlink_mission.cpp CHECK_SYSID_COMPID_MISSION), so there is no default:
// an unset or invalid DRONE_ID stops fcc_bridge at startup instead of
// quietly addressing system 1.
#pragma once

#include <mavlink/v2.0/common/mavlink.h>

#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace fcc_bridge
{

// PX4's flight stack answers as the autopilot component of its system.
constexpr uint8_t kFccComponentId = MAV_COMP_ID_AUTOPILOT1;
constexpr uint8_t kMcComponentId = MAV_COMP_ID_ONBOARD_COMPUTER;

// Parses DRONE_ID; throws std::runtime_error unless it is an integer 1-255.
inline uint8_t drone_id_from_env()
{
  const char * raw = std::getenv("DRONE_ID");
  if (raw == nullptr || *raw == '\0') {
    throw std::runtime_error(
            "fcc_bridge: DRONE_ID is not set; it must equal the FCC's MAV_SYS_ID "
            "(set it in /etc/drone-mc/environment)");
  }
  char * end = nullptr;
  const long value = std::strtol(raw, &end, 10);
  if (*end != '\0' || value < 1 || value > 255) {
    throw std::runtime_error(
            std::string("fcc_bridge: DRONE_ID='") + raw +
            "' is not a MAVLink system ID (1-255)");
  }
  return static_cast<uint8_t>(value);
}

}  // namespace fcc_bridge
