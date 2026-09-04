"""
TEST-002 pattern example: mavlink_gcs_client driving a REAL gcs_mavlink_node.

Unlike test_mock_gcs_node.py (which deliberately tests mock_gcs_node in
isolation, playing "the MC side" by hand -- see that file's own docstring),
this test launches the actual gcs_comms gcs_mavlink_node executable and
drives it over real MAVLink, confirming mavlink_gcs_client's mission-upload
protocol handshake and GcsCommand sends work against production code before
sitl_mission_test.launch.py's own scenarios depend on it end-to-end.
"""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from mc_msgs.msg import GcsCommand, MissionUpload
from mc_test_utils import mavlink_gcs_client as gcs_client
import pytest
import rclpy

TEST_LOOPBACK_PORT = 25560
TEST_DRONE_ID = 1


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    gcs_mavlink = launch_ros.actions.Node(
        package='gcs_comms',
        executable='gcs_mavlink_node',
        additional_env={
            'GCS_MAVLINK_LOOPBACK_PORT': str(TEST_LOOPBACK_PORT),
            'DRONE_ID': str(TEST_DRONE_ID),
        },
    )
    return (
        launch.LaunchDescription([gcs_mavlink, launch_testing.actions.ReadyToTest()]),
        {'gcs_mavlink': gcs_mavlink},
    )


class TestMavlinkGcsClient(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mavlink_gcs_client_observer')
        self.client = gcs_client.MavlinkGcsClient(
            mc_port=TEST_LOOPBACK_PORT, target_system=TEST_DRONE_ID)

    def tearDown(self):
        self.client.close()
        self.node.destroy_node()

    def _wait_for_ready(self, sub):
        # Establish gcs_mavlink_node's reply-learning (handle_message()'s
        # last_gcs_endpoint_) before a mission upload can succeed -- a
        # MISSION_REQUEST_INT reply has somewhere to be sent.
        deadline = time.time() + 5.0
        while time.time() < deadline and sub.get_publisher_count() == 0:
            self.client.send_heartbeat()
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_arm_authorise_command_reaches_mission_manager_topic(self):
        received = []
        sub = self.node.create_subscription(
            GcsCommand, '/mc/gcs/command', lambda m: received.append(m), 10)
        self._wait_for_ready(sub)

        deadline = time.time() + 5.0
        while time.time() < deadline and not received:
            self.client.arm_authorise(True)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(received, 'gcs_mavlink_node did not forward ARM_AUTHORISE')
        self.assertEqual(received[-1].command, gcs_client.MC_CMD_ARM_AUTHORISE)
        self.assertAlmostEqual(received[-1].params[0], 1.0, places=3)

        self.node.destroy_subscription(sub)

    def test_upload_mission_completes_and_publishes_mission_upload(self):
        received = []
        sub = self.node.create_subscription(
            MissionUpload, '/mc/gcs/mission_upload', lambda m: received.append(m), 10)
        self._wait_for_ready(sub)

        items = [
            gcs_client.ingress_or_muster_item(
                gcs_client.CMD_INGRESS_WAYPOINT, lat_deg=0.0001, lon_deg=0.0001, alt_m=30.0),
            gcs_client.ingress_or_muster_item(
                gcs_client.CMD_MUSTER_POINT, lat_deg=0.0001, lon_deg=0.0001, alt_m=30.0),
            gcs_client.search_area_vertex_item(0, 3, 0.0001, 0.0001),
            gcs_client.search_area_vertex_item(1, 3, 0.0002, 0.0001),
            gcs_client.search_area_vertex_item(2, 3, 0.0002, 0.0002),
        ]

        accepted = self.client.upload_mission(items, timeout_s=5.0)
        self.assertTrue(accepted, 'mission upload handshake did not complete with ACCEPTED')

        deadline = time.time() + 3.0
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(received, 'gcs_mavlink_node did not publish /mc/gcs/mission_upload')
        upload = received[-1]
        self.assertEqual(len(upload.ingress_waypoints), 2)
        self.assertEqual(len(upload.search_polygon), 3)
        self.assertAlmostEqual(upload.ingress_waypoints[0].pose.position.z, 30.0, places=3)

        self.node.destroy_subscription(sub)
