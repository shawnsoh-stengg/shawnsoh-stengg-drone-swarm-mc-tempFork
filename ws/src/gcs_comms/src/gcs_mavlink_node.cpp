#include "gcs_comms/gcs_mavlink_node.hpp"

namespace gcs_comms
{

GcsMavlinkNode::GcsMavlinkNode()
: Node("gcs_mavlink_node")
{
  RCLCPP_INFO(get_logger(), "gcs_mavlink_node started");
}

}  // namespace gcs_comms
