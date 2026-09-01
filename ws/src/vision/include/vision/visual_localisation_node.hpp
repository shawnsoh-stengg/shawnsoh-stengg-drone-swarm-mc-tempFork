// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §8, ICD-MC-001 §3).
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace vision
{

// GNSS-denied visual odometry pipeline (SDD-MC-001 §8). Scaffold only —
// the FAST+ORB/KLT/5-point-RANSAC pipeline and GNSS handover logic land
// with the vision implementation work.
class VisualLocalisationNode : public rclcpp::Node
{
public:
  VisualLocalisationNode();
};

}  // namespace vision
