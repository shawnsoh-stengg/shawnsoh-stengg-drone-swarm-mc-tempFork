"""
TP-NI pattern example: launch_testing against mock_gcs_node.

Channel B (ICD-MC-001 §4) is raw MAVLink 2/UDP, not a ROS topic, so like
test_mock_peer_node.py the test process talks to the launched mock with a
plain socket + mavlink_lite instead of rclpy subscriptions -- the pattern
a future TP-NI-010/014/015-style test of gcs_mavlink_node itself should
follow, substituting the real node for the raw-socket assertions below.
"""
import socket
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from mc_test_utils import mavlink_lite as mav
import pytest
import rclpy
from std_msgs.msg import UInt32

MC_LISTEN_PORT = 14552  # ICD-MC-001 §4.2 MC receive port; this test plays the MC side


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_gcs = launch_ros.actions.Node(
        package='mc_test_utils',
        executable='mock_gcs_node',
        parameters=[
            {
                'mc_port': MC_LISTEN_PORT,
                'heartbeat_period_s': 0.2,
                'command_period_s': 0.3,
                'command_id': 50002,
            }
        ],
    )
    return (
        launch.LaunchDescription([mock_gcs, launch_testing.actions.ReadyToTest()]),
        {'mock_gcs': mock_gcs},
    )


class TestMockGcsNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', MC_LISTEN_PORT))
        self.sock.settimeout(3.0)
        self.node = rclpy.create_node('test_mock_gcs_node_client')

    def tearDown(self):
        self.sock.close()
        self.node.destroy_node()

    def _recv_until(self, msgid, deadline_s=5.0):
        deadline = time.time() + deadline_s
        while time.time() < deadline:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                break
            frame = mav.decode(data)
            if frame is not None and frame.msgid == msgid:
                return frame, addr
        return None, None

    def test_sends_periodic_heartbeat(self):
        frame, _addr = self._recv_until(mav.MSG_HEARTBEAT)
        self.assertIsNotNone(frame, 'mock_gcs_node did not send a HEARTBEAT within the deadline')
        self.assertEqual(frame.fields['type'], 6)

    def test_sends_command_long_and_processes_ack(self):
        frame, addr = self._recv_until(mav.MSG_COMMAND_LONG)
        self.assertIsNotNone(frame, 'mock_gcs_node did not send a COMMAND_LONG in time')
        self.assertEqual(frame.fields['command'], 50002)

        last_rx = []
        sub = self.node.create_subscription(
            UInt32, '/mock/gcs/last_rx_msgid', lambda m: last_rx.append(m.data), 10
        )

        ack = mav.encode(
            mav.MSG_COMMAND_ACK, 0, 11, 191,
            command=50002, result=0, progress=100, result_param2=0,
            target_system=255, target_component=190,
        )
        self.sock.sendto(ack, addr)

        deadline = time.time() + 3.0
        while time.time() < deadline and mav.MSG_COMMAND_ACK not in last_rx:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIn(mav.MSG_COMMAND_ACK, last_rx, 'mock_gcs_node did not decode the ACK')
        self.node.destroy_subscription(sub)
