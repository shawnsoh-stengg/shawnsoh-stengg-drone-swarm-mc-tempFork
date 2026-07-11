#include "swarm_coord/formation_controller_node.hpp"

namespace swarm_coord
{

FormationControllerNode::FormationControllerNode()
: Node("formation_controller_node")
{
  RCLCPP_INFO(get_logger(), "formation_controller_node started");
}

}  // namespace swarm_coord
