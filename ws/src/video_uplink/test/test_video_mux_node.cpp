#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "video_uplink/video_mux_node.hpp"

TEST(VideoMuxNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<video_uplink::VideoMuxNode>();
  EXPECT_STREQ(node->get_name(), "video_mux_node");
  rclcpp::shutdown();
}
