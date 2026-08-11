// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §4, ICD-MC-001 §9).

// EXEC-003: classify_command_tier()/CommandArbiter unit tests — pure gtest,
// no rclcpp::init required (command_arbiter.hpp/.cpp take no dependency on
// rclcpp::Node or ROS time, per its own header doc comment).

#include <gtest/gtest.h>

#include <array>

#include "mission_exec/command_arbiter.hpp"

namespace mission_exec
{
namespace
{

constexpr uint32_t kMavCmdNavReturnToLaunch = 20;
constexpr uint32_t kMavCmdDoSetMode = 176;
constexpr uint32_t kMavCmdComponentArmDisarm = 400;
constexpr uint32_t kMcCmdPhaseTransition = 50001;

std::array<float, 7> zero_params()
{
  return {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F};
}

GcsCommandEnvelope make_cmd(uint32_t command, std::array<float, 7> params = zero_params())
{
  GcsCommandEnvelope cmd;
  cmd.command = command;
  cmd.params = params;
  return cmd;
}

}  // namespace

// --- classify_command_tier() ---

TEST(CommandArbiterTest, DisarmClassifiesAsEmergency)
{
  EXPECT_EQ(
    classify_command_tier(kMavCmdComponentArmDisarm, zero_params()), CommandTier::EMERGENCY);
}

TEST(CommandArbiterTest, ArmDoesNotClassifyAsEmergency)
{
  auto params = zero_params();
  params[0] = 1.0F;  // arm, not disarm
  EXPECT_EQ(classify_command_tier(kMavCmdComponentArmDisarm, params), CommandTier::PHASE);
}

TEST(CommandArbiterTest, RtlClassifiesAsEmergency)
{
  EXPECT_EQ(classify_command_tier(kMavCmdNavReturnToLaunch, zero_params()), CommandTier::EMERGENCY);
}

TEST(CommandArbiterTest, DoSetModeManualClassifiesAsEmergency)
{
  auto params = zero_params();
  params[1] = 1.0F;  // custom_main = MANUAL
  EXPECT_EQ(classify_command_tier(kMavCmdDoSetMode, params), CommandTier::EMERGENCY);
}

TEST(CommandArbiterTest, DoSetModeAltctlClassifiesAsEmergency)
{
  auto params = zero_params();
  params[1] = 2.0F;  // custom_main = ALTCTL
  EXPECT_EQ(classify_command_tier(kMavCmdDoSetMode, params), CommandTier::EMERGENCY);
}

TEST(CommandArbiterTest, DoSetModePosctlClassifiesAsEmergency)
{
  auto params = zero_params();
  params[1] = 3.0F;  // custom_main = POSCTL
  EXPECT_EQ(classify_command_tier(kMavCmdDoSetMode, params), CommandTier::EMERGENCY);
}

TEST(CommandArbiterTest, DoSetModeStabilizedClassifiesAsEmergency)
{
  auto params = zero_params();
  params[1] = 7.0F;  // custom_main = STABILIZED
  EXPECT_EQ(classify_command_tier(kMavCmdDoSetMode, params), CommandTier::EMERGENCY);
}

TEST(CommandArbiterTest, DoSetModeMissionDoesNotClassifyAsEmergency)
{
  auto params = zero_params();
  params[1] = 4.0F;  // custom_main = AUTO/MISSION — not in the Emergency list
  EXPECT_EQ(classify_command_tier(kMavCmdDoSetMode, params), CommandTier::PHASE);
}

TEST(CommandArbiterTest, PhaseTransitionClassifiesAsPhase)
{
  EXPECT_EQ(classify_command_tier(kMcCmdPhaseTransition, zero_params()), CommandTier::PHASE);
}

TEST(CommandArbiterTest, UnrecognisedCommandDefaultsToPhaseTier)
{
  EXPECT_EQ(classify_command_tier(999999U, zero_params()), CommandTier::PHASE);
}

// --- CommandArbiter: empty/size/autonomous_transitions_allowed() ---

TEST(CommandArbiterTest, StartsEmptyAndAllowsAutonomousTransitions)
{
  CommandArbiter arbiter;
  EXPECT_TRUE(arbiter.empty());
  EXPECT_EQ(arbiter.size(), 0U);
  EXPECT_TRUE(arbiter.autonomous_transitions_allowed());
}

TEST(CommandArbiterTest, EnqueueMakesQueueNonEmptyAndBlocksAutonomousTransitions)
{
  CommandArbiter arbiter;
  arbiter.enqueue(make_cmd(kMcCmdPhaseTransition));
  EXPECT_FALSE(arbiter.empty());
  EXPECT_EQ(arbiter.size(), 1U);
  EXPECT_FALSE(arbiter.autonomous_transitions_allowed());
}

// --- Invalid rejection: popping with nothing valid to pop is a defined,
// safe no-op — not undefined behaviour. ---

TEST(CommandArbiterTest, PopNextOnEmptyQueueReturnsNulloptNotUndefinedBehaviour)
{
  CommandArbiter arbiter;
  EXPECT_EQ(arbiter.pop_next(), std::nullopt);
  // Repeated pops on an already-empty queue remain safe.
  EXPECT_EQ(arbiter.pop_next(), std::nullopt);
}

// --- FIFO ordering within a single tier ---

TEST(CommandArbiterTest, PhaseTierCommandsPopInFifoOrder)
{
  CommandArbiter arbiter;
  arbiter.enqueue(make_cmd(kMcCmdPhaseTransition, {1.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F}));
  arbiter.enqueue(make_cmd(kMcCmdPhaseTransition, {2.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F}));
  arbiter.enqueue(make_cmd(kMcCmdPhaseTransition, {3.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F}));

  ASSERT_EQ(arbiter.pop_next()->params[0], 1.F);
  ASSERT_EQ(arbiter.pop_next()->params[0], 2.F);
  ASSERT_EQ(arbiter.pop_next()->params[0], 3.F);
  EXPECT_TRUE(arbiter.empty());
}

TEST(CommandArbiterTest, EmergencyTierCommandsPopInFifoOrder)
{
  CommandArbiter arbiter;
  arbiter.enqueue(make_cmd(kMavCmdComponentArmDisarm));
  arbiter.enqueue(make_cmd(kMavCmdNavReturnToLaunch));

  ASSERT_EQ(arbiter.pop_next()->command, kMavCmdComponentArmDisarm);
  ASSERT_EQ(arbiter.pop_next()->command, kMavCmdNavReturnToLaunch);
  EXPECT_TRUE(arbiter.empty());
}

// --- Emergency bypasses: enqueued-later Emergency command still pops
// first, ahead of an already-queued Phase-tier command — this is the
// "GCS emergency command arrives during an in-progress lower-priority
// command" scenario the ticket's own acceptance criteria singles out. ---

TEST(CommandArbiterTest, EmergencyCommandBypassesAlreadyQueuedPhaseCommand)
{
  CommandArbiter arbiter;
  arbiter.enqueue(make_cmd(kMcCmdPhaseTransition));  // queued first
  arbiter.enqueue(make_cmd(kMavCmdNavReturnToLaunch));  // arrives second, but higher priority

  const auto first = arbiter.pop_next();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->command, kMavCmdNavReturnToLaunch);

  const auto second = arbiter.pop_next();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->command, kMcCmdPhaseTransition);

  EXPECT_TRUE(arbiter.empty());
}

TEST(CommandArbiterTest, MultipleEmergencyCommandsAllDrainBeforeAnyPhaseCommand)
{
  CommandArbiter arbiter;
  arbiter.enqueue(make_cmd(kMcCmdPhaseTransition));
  arbiter.enqueue(make_cmd(kMavCmdComponentArmDisarm));
  arbiter.enqueue(make_cmd(kMcCmdPhaseTransition));
  arbiter.enqueue(make_cmd(kMavCmdNavReturnToLaunch));

  EXPECT_EQ(arbiter.pop_next()->command, kMavCmdComponentArmDisarm);
  EXPECT_EQ(arbiter.pop_next()->command, kMavCmdNavReturnToLaunch);
  EXPECT_EQ(arbiter.pop_next()->command, kMcCmdPhaseTransition);
  EXPECT_EQ(arbiter.pop_next()->command, kMcCmdPhaseTransition);
  EXPECT_TRUE(arbiter.empty());
}

}  // namespace mission_exec
