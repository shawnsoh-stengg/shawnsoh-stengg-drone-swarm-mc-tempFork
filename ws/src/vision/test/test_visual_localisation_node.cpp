#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vision/visual_localisation_node.hpp"

TEST(VisualLocalisationNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<vision::VisualLocalisationNode>();
  EXPECT_STREQ(node->get_name(), "visual_localisation_node");
  rclcpp::shutdown();
}
