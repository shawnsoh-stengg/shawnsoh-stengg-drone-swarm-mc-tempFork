// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>

namespace mission_exec
{

// EXEC-003: GCS authority model — SDD §4.3's three-tier command priority
// (Emergency > Phase > Autonomous). Framework-free (no rclcpp), mirroring
// pre_arm_checks.hpp's own EXEC-002 rationale: classification and queue
// ordering are pure logic, independently callable/testable without
// spinning a node. "Autonomous" isn't itself a tier queued here — it is
// mission_manager_node's own periodic guard evaluation, gated by
// autonomous_transitions_allowed() rather than competing for a slot in
// this queue.
enum class CommandTier : uint8_t
{
  EMERGENCY = 0,  // DISARM, RTL, MANUAL, ALTCTL, POSCTL, STABILIZED
  PHASE = 1,      // PHASE_TRANSITION, and every other /mc/gcs/command not
                  // named in this ticket's own Emergency list. ARM_
                  // AUTHORISE/END_MISSION/GCS_OVERRIDE keep their existing,
                  // untouched semantics in mission_manager_node — they are
                  // bucketed here only so classify_command_tier() has a
                  // defined answer for every command, not because this
                  // ticket redefines their handling.
};

// Mirrors mc_msgs/GcsCommand's own command+params fields — deliberately not
// that message type itself, so this header carries no mc_msgs/rclcpp
// dependency (same rationale as PreArmCheckInputs's own plain-data fields
// in pre_arm_checks.hpp, minus the parts of GcsCommand this ticket's
// classification never reads, e.g. header/target_system).
struct GcsCommandEnvelope
{
  uint32_t command{0};
  std::array<float, 7> params{};
};

// Classifies a raw GCS command (MAVLink COMMAND_LONG-style: MAV_CMD id +
// param1-7) per SDD §4.3 priority 1's named list. A disarm (param1 == 0) on
// MAV_CMD_COMPONENT_ARM_DISARM is EMERGENCY; an arm request (param1 == 1) on
// that same MAV_CMD id is not — arming is gated by the normal PREFLIGHT ->
// ARMING guard, not a GCS override.
CommandTier classify_command_tier(uint32_t command, const std::array<float, 7> & params);

// FIFO-within-tier priority queue. pop_next() always drains every queued
// EMERGENCY-tier command before any PHASE-tier command, regardless of
// arrival order — SDD §4.3 priority 1's "bypasses the transition table" is
// implemented here as "jumps ahead of anything already queued behind it",
// not merely "processed with logic that happens to skip the table" once
// its turn comes up naturally in strict arrival order.
class CommandArbiter
{
public:
  void enqueue(const GcsCommandEnvelope & cmd);
  bool empty() const;
  size_t size() const;
  // Removes and returns the highest-priority, oldest-within-that-priority
  // command; std::nullopt if the queue is empty — an empty pop is a
  // defined, safe result to check for, not a precondition to violate.
  std::optional<GcsCommandEnvelope> pop_next();
  // SDD §4.3 priority 3: autonomous transitions may run only when this is
  // true ("evaluated only if no pending GCS command").
  bool autonomous_transitions_allowed() const;

private:
  std::deque<GcsCommandEnvelope> emergency_queue_;
  std::deque<GcsCommandEnvelope> phase_queue_;
};

}  // namespace mission_exec
