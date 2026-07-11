#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "gcs_comms/gcs_mavlink_node.hpp"

TEST(GcsMavlinkNode, ConstructsWithExpectedName)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<gcs_comms::GcsMavlinkNode>();
  EXPECT_STREQ(node->get_name(), "gcs_mavlink_node");
  rclcpp::shutdown();
}
