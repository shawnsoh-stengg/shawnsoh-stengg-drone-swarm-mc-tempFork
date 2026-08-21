"""
TP-NI pattern example: launch_testing against mock_fcc_node.

Demonstrates the pattern future node-integration tests should follow:
launch the mock (playing the hardware-facing role, TP-MC-001 §2.4) via
generate_test_description(), then drive/observe it as a plain rclpy client
node from the test process. Covers (loosely) TP-NI-001/002 against a mock
in place of the uORB/DDS side fcc_bridge_node itself would otherwise need.
"""
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from mc_msgs.msg import FccOffboardSetpoint, VehicleState
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import Imu

BAG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'test_data', 'bags', 'fcc_nominal'
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_fcc = launch_ros.actions.Node(
        package='mc_test_utils',
        executable='mock_fcc_node',
        parameters=[{'bag_path': BAG_PATH, 'tick_period_s': 0.01}],
    )
    return (
        launch.LaunchDescription([mock_fcc, launch_testing.actions.ReadyToTest()]),
        {'mock_fcc': mock_fcc},
    )


class TestMockFccNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mock_fcc_node_client')

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_vehicle_state_imu_odometry(self):
        received = {'vehicle_state': [], 'imu': [], 'odometry': []}

        def collect(key):
            return lambda msg: received[key].append(msg)

        subs = [
            self.node.create_subscription(
                VehicleState, '/mc/fcc/vehicle_state', collect('vehicle_state'), 10
            ),
            self.node.create_subscription(Imu, '/mc/fcc/imu', collect('imu'), 10),
            self.node.create_subscription(Odometry, '/mc/fcc/odometry', collect('odometry'), 10),
        ]
        deadline = time.time() + 10.0
        while time.time() < deadline and not all(received.values()):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        for key in received:
            self.assertGreater(len(received[key]), 0, f'mock_fcc_node did not publish {key}')

        vs = received['vehicle_state'][0]
        self.assertTrue(vs.fcc_link_ok)
        self.assertTrue(vs.gnss_ok)

        for sub in subs:
            self.node.destroy_subscription(sub)

    def test_accepts_and_echoes_trajectory_setpoint(self):
        echoed = []
        sub = self.node.create_subscription(
            FccOffboardSetpoint, '/mock/fcc/setpoint_echo', lambda m: echoed.append(m), 10
        )
        pub = self.node.create_publisher(FccOffboardSetpoint, '/mc/fcc/offboard_setpoint', 10)

        setpoint = FccOffboardSetpoint()
        setpoint.x, setpoint.y, setpoint.z, setpoint.yaw = 1.0, 2.0, -3.0, 0.5

        deadline = time.time() + 5.0
        while time.time() < deadline and not echoed:
            pub.publish(setpoint)
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.assertTrue(echoed, 'mock_fcc_node did not echo the offboard setpoint it received')
        self.assertAlmostEqual(echoed[-1].x, 1.0, places=3)
        self.assertAlmostEqual(echoed[-1].z, -3.0, places=3)

        self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub)
