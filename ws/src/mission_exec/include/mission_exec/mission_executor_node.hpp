#pragma once

#include <rclcpp/rclcpp.hpp>

namespace mission_exec
{

// Mission phase state machine (SDD-MC-001 §4). Scaffold only — the
// transition table and FCC/swarm integration land with the FCC-0xx
// implementation work.
class MissionExecutorNode : public rclcpp::Node
{
public:
  MissionExecutorNode();
};

}  // namespace mission_exec
