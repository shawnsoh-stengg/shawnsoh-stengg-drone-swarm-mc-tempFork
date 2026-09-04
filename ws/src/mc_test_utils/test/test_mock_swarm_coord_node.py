"""
TEST-002 pattern example: launch_testing against mock_swarm_coord_node.

Confirms the mock publishes valid IsrRole/VoronoiCell/coverage-path
messages that would actually satisfy mission_manager_node's real
guard_isr_selection_to_voronoi_partition()/guard_voronoi_partition_to_
search() guards, given the phase/mission-upload inputs those guards react
to -- without needing a real mission_manager_node instance running.
"""
import time
import unittest

from geographic_msgs.msg import GeoPoint
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from mc_msgs.msg import IsrRole, MissionPhase, MissionUpload, VoronoiCell
from nav_msgs.msg import Path
import pytest
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_swarm_coord = launch_ros.actions.Node(
        package='mc_test_utils',
        executable='mock_swarm_coord_node',
        parameters=[{'drone_id': 1, 'isr_delay_s': 0.05, 'voronoi_delay_s': 0.05}],
    )
    return (
        launch.LaunchDescription([mock_swarm_coord, launch_testing.actions.ReadyToTest()]),
        {'mock_swarm_coord': mock_swarm_coord},
    )


class TestMockSwarmCoordNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mock_swarm_coord_node_client')

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_isr_role_after_isr_selection_phase(self):
        received = []
        sub = self.node.create_subscription(
            IsrRole, '/mc/swarm/isr_role', lambda m: received.append(m), 10)
        pub = self.node.create_publisher(MissionPhase, '/mc/mission_phase', 10)

        phase_msg = MissionPhase()
        phase_msg.phase = MissionPhase.ISR_SELECTION

        deadline = time.time() + 5.0
        while time.time() < deadline and not received:
            pub.publish(phase_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(received, 'mock_swarm_coord_node did not publish IsrRole')
        self.assertEqual(received[-1].role, IsrRole.ISR)
        self.assertTrue(received[-1].all_peers_acked)

        self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub)

    def test_publishes_matching_voronoi_cell_and_coverage_path(self):
        received_cells = []
        received_paths = []
        subs = [
            self.node.create_subscription(
                VoronoiCell, '/mc/swarm/voronoi_cell', lambda m: received_cells.append(m), 10),
            self.node.create_subscription(
                Path, '/mc/search/coverage_path', lambda m: received_paths.append(m), 10),
        ]
        pub_upload = self.node.create_publisher(MissionUpload, '/mc/gcs/mission_upload', 10)
        pub_phase = self.node.create_publisher(MissionPhase, '/mc/mission_phase', 10)

        upload_msg = MissionUpload()
        upload_msg.upload_id = 42
        upload_msg.search_polygon = [GeoPoint()]

        phase_msg = MissionPhase()
        phase_msg.phase = MissionPhase.VORONOI_PARTITION

        deadline = time.time() + 5.0
        while time.time() < deadline and not (received_cells and received_paths):
            pub_upload.publish(upload_msg)
            pub_phase.publish(phase_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(received_cells, 'mock_swarm_coord_node did not publish VoronoiCell')
        self.assertTrue(received_paths, 'mock_swarm_coord_node did not publish coverage_path')
        self.assertEqual(received_cells[-1].upload_id, 42)
        self.assertGreater(len(received_paths[-1].poses), 0)

        for sub in subs:
            self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub_upload)
        self.node.destroy_publisher(pub_phase)
