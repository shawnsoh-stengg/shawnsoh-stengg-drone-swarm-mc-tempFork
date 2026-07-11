#pragma once

#include <rclcpp/rclcpp.hpp>

namespace gcs_comms
{

// GCS communication bridge (SDD-MC-001 §11). Scaffold only — the MAVROS
// integration, mission upload state machine, and telemetry multiplexer
// land with the gcs_comms implementation work.
class GcsMavlinkNode : public rclcpp::Node
{
public:
  GcsMavlinkNode();
};

}  // namespace gcs_comms
