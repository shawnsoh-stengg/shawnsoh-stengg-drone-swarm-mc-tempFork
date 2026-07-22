#pragma once

#include <rclcpp/rclcpp.hpp>

namespace video_downlink
{

// GCS video downlink: re-encodes perception_front_node's frame stream to
// H.265, muxes a per-frame KLV-style metadata track, and serves it to the
// GCS over a dedicated RTSP session (SDD-MC-001 §9a.2). Scaffold only —
// the GStreamer encode/mux pipeline and RTSP server land with the
// video_downlink implementation work.
class VideoMuxNode : public rclcpp::Node
{
public:
  VideoMuxNode();
};

}  // namespace video_downlink
