#pragma once

#include <rclcpp/rclcpp.hpp>

namespace vision
{

// YOLOv8n/TensorRT AI detection pipeline with geolocation (SDD-MC-001 §9).
// Scaffold only — the TensorRT inference and camera-intrinsics/pose-based
// geolocation logic land with the vision implementation work.
class DetectionNode : public rclcpp::Node
{
public:
  DetectionNode();
};

}  // namespace vision
