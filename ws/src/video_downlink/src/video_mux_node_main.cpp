// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §9a, ICD-MC-001 §8).
#include <rclcpp/rclcpp.hpp>

#include "video_downlink/video_mux_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<video_downlink::VideoMuxNode>());
  rclcpp::shutdown();
  return 0;
}
