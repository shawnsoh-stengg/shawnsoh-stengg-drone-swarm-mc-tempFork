// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §9a, ICD-MC-001 §7).

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace video_downlink
{

// Front gimbal camera RTSP/H.265 ingest, TensorRT inference, and
// multi-frame tracking (SDD-MC-001 §9a.1). Scaffold only — the rtspsrc
// pipeline, inference reuse of perception_down_node's engine, and the
// IoU tracker land with the video_downlink implementation work.
class PerceptionFrontNode : public rclcpp::Node
{
public:
  PerceptionFrontNode();
};

}  // namespace video_downlink
