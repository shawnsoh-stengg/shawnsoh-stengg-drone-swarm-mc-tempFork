"""
TP-NI pattern example: launch_testing against mock_peer_node.

Swarm UDP (ICD-MC-001 §5) isn't a ROS topic, so unlike the mock_fcc_node/
mock_camera_node examples, the test process talks to the launched mock
node with a plain socket instead of rclpy subscriptions -- demonstrating
the pattern for any future TP-NI-004/005-style test of
swarm_coordinator_node once its own UDP layer lands (CLAUDE.md: currently
a §2.4/§5/§6 scaffold).
"""
import socket
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from mc_test_utils import swarm_protocol as proto
import pytest

TEST_SENDER_ID = 5


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_peer = launch_ros.actions.Node(
        package='mc_test_utils',
        executable='mock_peer_node',
        parameters=[{'sender_id': 1, 'heartbeat_period_s': 0.2}],
    )
    return (
        launch.LaunchDescription([mock_peer, launch_testing.actions.ReadyToTest()]),
        {'mock_peer': mock_peer},
    )


class TestMockPeerNode(unittest.TestCase):

    def setUp(self):
        self.hb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hb_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.hb_sock.bind(('127.0.0.1', proto.HEARTBEAT_PORT))
        self.hb_sock.settimeout(3.0)

        self.coord_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.coord_sock.settimeout(3.0)

    def tearDown(self):
        self.hb_sock.close()
        self.coord_sock.close()

    def test_broadcasts_swarm_heartbeat(self):
        data, _addr = self.hb_sock.recvfrom(2048)
        header, payload = proto.unpack_header(data)
        self.assertEqual(header.message_type, proto.MSG_SWARM_HEARTBEAT)
        fields = proto.unpack_heartbeat(payload)
        self.assertEqual(fields['mission_phase'], 7)
        self.assertEqual(fields['battery_pct'], 88)

    def test_answers_peer_state_request(self):
        # Retry the request rather than sending once: right after launch there's no
        # guarantee mock_peer_node's coordination socket is bound yet, so a single
        # early request can be dropped with nothing listening on the far end.
        request_payload = proto.pack_peer_state_request()
        request = proto.pack_header(TEST_SENDER_ID, proto.MSG_PEER_STATE_REQUEST, request_payload)
        self.coord_sock.settimeout(0.3)

        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.coord_sock.sendto(request, ('127.0.0.1', proto.COORDINATION_PORT))
            try:
                data, _addr = self.coord_sock.recvfrom(2048)
            except socket.timeout:
                continue
            header, payload = proto.unpack_header(data)
            if header.message_type == proto.MSG_PEER_STATE:
                fields = proto.unpack_peer_state(payload)
                self.assertAlmostEqual(fields['x'], 10.0, places=3)
                self.assertAlmostEqual(fields['z'], -50.0, places=3)
                self.assertFalse(fields['is_leader'])
                return
        self.fail('mock_peer_node did not answer PEER_STATE_REQUEST with a PEER_STATE reply')
