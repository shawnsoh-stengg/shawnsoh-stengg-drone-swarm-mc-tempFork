#!/usr/bin/env python3
"""
mock_swarm_coord_node -- TEST-002 SITL system-test double for swarm_coord/search_planner.

`swarm_coordinator_node`/`formation_controller_node` (SC-002/003/004) and
`search_planner_node` (EXEC-002) are still pure scaffolds with no
`mc_msgs` dependency (CLAUDE.md), so nothing in a real system today
publishes `/mc/swarm/isr_role`, `/mc/swarm/voronoi_cell`, or
`/mc/search/coverage_path` -- the three topics `mission_manager_node`'s
own real transition guards (`guard_isr_selection_to_voronoi_partition()`,
`guard_voronoi_partition_to_search()`) block on. EXEC-001's own unit tests
already mock these same topics directly; this node applies the identical
precedent at SITL system-test scope so `sitl_mission_test.launch.py`'s
single-drone scenario can exercise `mission_manager_node`'s real state
machine end-to-end.

This does NOT validate ISR election or Voronoi partitioning -- it publishes
the minimum valid, single-drone-consistent messages needed to satisfy the
two guards above, a fixed short delay after observing the phase that leads
into each guard. Real multi-drone coordination remains untested until
SC-002/003/004 + EXEC-002 land (PROJECT-PLAN-5-MONTH.md TEST-002 entry).
"""
from geometry_msgs.msg import PoseStamped
from mc_msgs.msg import IsrRole, MissionPhase, MissionUpload, VoronoiCell
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node


class MockSwarmCoordNode(Node):

    def __init__(self):
        super().__init__('mock_swarm_coord_node')
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('isr_delay_s', 0.5)
        self.declare_parameter('voronoi_delay_s', 0.5)

        self._drone_id = int(self.get_parameter('drone_id').value)
        self._isr_delay_s = float(self.get_parameter('isr_delay_s').value)
        self._voronoi_delay_s = float(self.get_parameter('voronoi_delay_s').value)

        self._latest_upload_id = 0
        self._isr_timer = None
        self._voronoi_timer = None

        self._pub_isr_role = self.create_publisher(IsrRole, '/mc/swarm/isr_role', 10)
        self._pub_voronoi_cell = self.create_publisher(
            VoronoiCell, '/mc/swarm/voronoi_cell', 10)
        self._pub_coverage_path = self.create_publisher(Path, '/mc/search/coverage_path', 10)

        self.create_subscription(
            MissionUpload, '/mc/gcs/mission_upload', self._on_mission_upload, 10)
        self.create_subscription(
            MissionPhase, '/mc/mission_phase', self._on_mission_phase, 10)

        self.get_logger().info(
            'mock_swarm_coord_node standing in for swarm_coord/search_planner '
            '(drone_id={})'.format(self._drone_id))

    def _on_mission_upload(self, msg: MissionUpload):
        self._latest_upload_id = msg.upload_id

    def _on_mission_phase(self, msg: MissionPhase):
        # create_timer(..., one_shot doesn't exist in rclpy's timer API) --
        # cancel() inside each callback below is what makes these one-shot.
        if msg.phase == MissionPhase.ISR_SELECTION and self._isr_timer is None:
            self._isr_timer = self.create_timer(self._isr_delay_s, self._publish_isr_role)
        elif msg.phase == MissionPhase.VORONOI_PARTITION and self._voronoi_timer is None:
            self._voronoi_timer = self.create_timer(
                self._voronoi_delay_s, self._publish_voronoi_and_coverage_path)

    def _publish_isr_role(self):
        self._isr_timer.cancel()
        msg = IsrRole()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.role = IsrRole.ISR
        msg.score = 1.0
        msg.coordinator_id = self._drone_id
        msg.result_acked = True
        msg.all_peers_acked = True
        msg.election_round = 1
        self._pub_isr_role.publish(msg)

    def _publish_voronoi_and_coverage_path(self):
        self._voronoi_timer.cancel()
        cell = VoronoiCell()
        cell.header.stamp = self.get_clock().now().to_msg()
        cell.drone_id = self._drone_id
        cell.round = 0
        cell.upload_id = self._latest_upload_id
        self._pub_voronoi_cell.publish(cell)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for x, y in ((0.0, 0.0), (5.0, 0.0), (5.0, 5.0)):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._pub_coverage_path.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = MockSwarmCoordNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
