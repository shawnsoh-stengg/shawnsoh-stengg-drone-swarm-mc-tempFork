#include <rclcpp/rclcpp.hpp>

#include "gcs_comms/gcs_mavlink_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gcs_comms::GcsMavlinkNode>());
  rclcpp::shutdown();
  return 0;
}
