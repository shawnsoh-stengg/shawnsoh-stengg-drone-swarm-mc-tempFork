#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "video_uplink/perception_front_node.hpp"

TEST(PerceptionFrontNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<video_uplink::PerceptionFrontNode>();
  EXPECT_STREQ(node->get_name(), "perception_front_node");
  rclcpp::shutdown();
}
