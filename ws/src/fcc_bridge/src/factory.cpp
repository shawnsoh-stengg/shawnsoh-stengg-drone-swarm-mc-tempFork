// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include <cstdlib>
#include <stdexcept>
#include <string>

#include "fcc_bridge/fcc_backend.hpp"
#include "fcc_bridge/xrce_dds_backend.hpp"

namespace fcc_bridge
{

namespace
{
bool env_true(const char * name)
{
  const char * v = std::getenv(name);
  return v != nullptr && std::string(v) == "true";
}
}  // namespace

std::unique_ptr<FccBackend> make_fcc_backend(rclcpp::Node * node)
{
  // SIMULATION_MODE selects the backend *class* (this ticket, FCC-001);
  // XrceDdsBackend separately reads FCC_TRANSPORT to pick its DDS link
  // (FCC-002). These are independent knobs — see ICD §9.4 and the
  // HID-MC-001 §4.3 correction (the doc previously conflated them into a
  // single three-value FCC_TRANSPORT, which does not match config/environment).
  if (env_true("SIMULATION_MODE")) {
    RCLCPP_INFO(node->get_logger(), "make_fcc_backend: SIMULATION_MODE=true -> MavlinkUdpBackend");
    // MavlinkUdpBackend is not yet implemented (tracked separately, not
    // part of FCC-001/FCC-002) — fail loudly rather than silently falling
    // back to production hardware.
    throw std::runtime_error(
      "make_fcc_backend: MavlinkUdpBackend not yet implemented — "
      "SIMULATION_MODE=true is not usable until that ticket lands");
  }

  RCLCPP_INFO(node->get_logger(), "make_fcc_backend: SIMULATION_MODE=false -> XrceDdsBackend");
  return std::make_unique<XrceDdsBackend>(node);
}

}  // namespace fcc_bridge
