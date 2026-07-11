#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vision/detection_node.hpp"

TEST(DetectionNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<vision::DetectionNode>();
  EXPECT_STREQ(node->get_name(), "detection_node");
  rclcpp::shutdown();
}
