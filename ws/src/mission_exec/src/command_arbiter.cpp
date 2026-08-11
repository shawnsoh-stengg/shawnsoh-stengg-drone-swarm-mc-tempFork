// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#include "mission_exec/command_arbiter.hpp"

namespace mission_exec
{

namespace
{
// ICD-MC-001 §4.5/§10.5.
constexpr uint32_t kMavCmdNavReturnToLaunch = 20;
constexpr uint32_t kMavCmdDoSetMode = 176;
constexpr uint32_t kMavCmdComponentArmDisarm = 400;

// PX4 custom_main mode values (param2 of MAV_CMD_DO_SET_MODE) for the three
// manual-control modes SDD §4.3 groups with DISARM/RTL as Emergency tier —
// see PX4's px4_custom_mode.h. ICD-MC-001.md §4.5 doesn't itself catalogue
// these three (only HOLD/MISSION/RTL/TAKEOFF/OFFBOARD), so they're defined
// here rather than duplicated from a non-existent ICD row.
constexpr float kPx4CustomMainManual = 1.0F;
constexpr float kPx4CustomMainAltctl = 2.0F;
constexpr float kPx4CustomMainPosctl = 3.0F;
constexpr float kPx4CustomMainStabilized = 7.0F;
}  // namespace

CommandTier classify_command_tier(uint32_t command, const std::array<float, 7> & params)
{
  if (command == kMavCmdComponentArmDisarm && params[0] == 0.0F) {
    return CommandTier::EMERGENCY;
  }
  if (command == kMavCmdNavReturnToLaunch) {
    return CommandTier::EMERGENCY;
  }
  if (command == kMavCmdDoSetMode) {
    const float custom_main = params[1];  // param2
    if (custom_main == kPx4CustomMainManual || custom_main == kPx4CustomMainAltctl ||
      custom_main == kPx4CustomMainPosctl || custom_main == kPx4CustomMainStabilized)
    {
      return CommandTier::EMERGENCY;
    }
  }
  return CommandTier::PHASE;
}

void CommandArbiter::enqueue(const GcsCommandEnvelope & cmd)
{
  if (classify_command_tier(cmd.command, cmd.params) == CommandTier::EMERGENCY) {
    emergency_queue_.push_back(cmd);
  } else {
    phase_queue_.push_back(cmd);
  }
}

bool CommandArbiter::empty() const
{
  return emergency_queue_.empty() && phase_queue_.empty();
}

size_t CommandArbiter::size() const
{
  return emergency_queue_.size() + phase_queue_.size();
}

std::optional<GcsCommandEnvelope> CommandArbiter::pop_next()
{
  if (!emergency_queue_.empty()) {
    const auto cmd = emergency_queue_.front();
    emergency_queue_.pop_front();
    return cmd;
  }
  if (!phase_queue_.empty()) {
    const auto cmd = phase_queue_.front();
    phase_queue_.pop_front();
    return cmd;
  }
  return std::nullopt;
}

bool CommandArbiter::autonomous_transitions_allowed() const
{
  return empty();
}

}  // namespace mission_exec
