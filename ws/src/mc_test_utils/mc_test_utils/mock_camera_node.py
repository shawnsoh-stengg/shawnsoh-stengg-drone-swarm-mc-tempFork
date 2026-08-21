#!/usr/bin/env python3
"""
mock_camera_node -- TP-MC-001 §2.4 hardware-facing-node mock.

Stands in for the (not yet implemented) camera_node, publishing
sensor_msgs/Image + sensor_msgs/CameraInfo on the same topics ICD-MC-001
§6 (IFD-004) specifies, from a recorded (or synthetic) rosbag2 bag, looped,
at >= 15 FPS:

  /camera/image_raw    sensor_msgs/Image       bgr8, >= 15 FPS
  /camera/camera_info  sensor_msgs/CameraInfo
"""
from mc_test_utils.bag_playback import BagPlayback
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

_TOPIC_MSG_TYPES = {
    '/camera/image_raw': Image,
    '/camera/camera_info': CameraInfo,
}


class MockCameraNode(Node):

    def __init__(self):
        super().__init__('mock_camera_node')
        self.declare_parameter('bag_path', '')
        self.declare_parameter('tick_period_s', 0.01)

        bag_path = self.get_parameter('bag_path').value
        if not bag_path:
            raise RuntimeError(
                "mock_camera_node requires the 'bag_path' parameter "
                '(TP-MC-001 §3.4 camera_outdoor bag)'
            )
        self._playback = BagPlayback(bag_path)

        # Named _topic_pubs, not _publishers -- rclpy.node.Node already owns a
        # private list attribute called _publishers; shadowing it here breaks
        # every later create_publisher() call (AttributeError: 'dict' object
        # has no attribute 'append').
        self._topic_pubs = {
            topic: self.create_publisher(msg_type, topic, 10)
            for topic, msg_type in _TOPIC_MSG_TYPES.items()
        }

        self._start_time = self.get_clock().now()
        self.create_timer(float(self.get_parameter('tick_period_s').value), self._tick)
        self.get_logger().info(
            'mock_camera_node playing back {} ({} messages, {:.2f}s, looped)'.format(
                bag_path, self._playback.message_count, self._playback.duration_ns / 1e9
            )
        )

    def _tick(self):
        elapsed_ns = (self.get_clock().now() - self._start_time).nanoseconds
        for topic, msg in self._playback.due(elapsed_ns):
            self._topic_pubs[topic].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
