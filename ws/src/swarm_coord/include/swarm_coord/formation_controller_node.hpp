// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §5, ICD-MC-001 §5).

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace swarm_coord
{

// Formation-keeping PD control loop (SDD-MC-001 §5). Scaffold only — the
// leader-tracking control math and mission-item target computation land
// with the swarm coordination implementation work.
class FormationControllerNode : public rclcpp::Node
{
public:
  FormationControllerNode();
};

}  // namespace swarm_coord
