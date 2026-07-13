#pragma once

#include <rclcpp/rclcpp.hpp>

namespace vision
{

// YOLOv8n/TensorRT AI detection pipeline with geolocation (SDD-MC-001 §9).
// Renamed from DetectionNode/detection_node alongside the front gimbal
// camera work (SDD-MC-001 §9a) which added a second, independent
// perception pipeline (perception_front_node).
// Scaffold only — the TensorRT inference and camera-intrinsics/pose-based
// geolocation logic land with the vision implementation work.
class PerceptionDownNode : public rclcpp::Node
{
public:
  PerceptionDownNode();
};

}  // namespace vision
