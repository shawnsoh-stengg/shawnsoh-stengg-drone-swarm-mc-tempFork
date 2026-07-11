#pragma once

#include <rclcpp/rclcpp.hpp>

namespace mission_exec
{

// Boustrophedon coverage path planning (SDD-MC-001 §7). Scaffold only —
// the min-area bounding rectangle, sweep-line generation, and WGS-84
// back-projection land with the FCC-0xx implementation work.
class SearchPlannerNode : public rclcpp::Node
{
public:
  SearchPlannerNode();
};

}  // namespace mission_exec
