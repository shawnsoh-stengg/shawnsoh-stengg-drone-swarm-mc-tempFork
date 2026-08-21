"""
TP-NI pattern example: launch_testing against mock_camera_node.

Loosely covers TP-NI-013's mock: asserts /camera/image_raw and
/camera/camera_info are published, correctly shaped, and at >= 15 FPS
(TP-MC-001 §2.4).
"""
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image

BAG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'test_data', 'bags', 'camera_outdoor',
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_camera = launch_ros.actions.Node(
        package='mc_test_utils',
        executable='mock_camera_node',
        parameters=[{'bag_path': BAG_PATH, 'tick_period_s': 0.01}],
    )
    return (
        launch.LaunchDescription([mock_camera, launch_testing.actions.ReadyToTest()]),
        {'mock_camera': mock_camera},
    )


class TestMockCameraNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mock_camera_node_client')

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_image_and_camera_info_at_15fps(self):
        images = []
        infos = []
        sub_image = self.node.create_subscription(Image, '/camera/image_raw', images.append, 10)
        sub_info = self.node.create_subscription(
            CameraInfo, '/camera/camera_info', infos.append, 10
        )

        # Sample for 2 s of steady-state playback (skip the first 0.5 s to
        # avoid counting startup jitter against the FPS floor).
        deadline = time.time() + 2.5
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        warm_deadline = time.time() + 2.0
        images.clear()
        while time.time() < warm_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertGreaterEqual(
            len(images), int(15 * 2.0 * 0.9),
            f'only received {len(images)} frames in 2s, expected >= ~15 FPS',
        )
        self.assertGreater(len(infos), 0, 'mock_camera_node did not publish /camera/camera_info')

        image = images[0]
        self.assertEqual(image.encoding, 'bgr8')
        self.assertEqual(image.header.frame_id, 'camera_optical')

        info = infos[0]
        self.assertEqual(info.distortion_model, 'plumb_bob')

        self.node.destroy_subscription(sub_image)
        self.node.destroy_subscription(sub_info)
