// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §6, ICD-MC-001 §5).

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace swarm_coord
{

// Peer discovery/lifecycle, ISR election, and Voronoi partitioning
// (SDD-MC-001 §6). Scaffold only — the election scoring, consensus
// handshake, and Fortune's-sweep partitioning land with the swarm
// coordination implementation work.
class SwarmCoordinatorNode : public rclcpp::Node
{
public:
  SwarmCoordinatorNode();
};

}  // namespace swarm_coord
