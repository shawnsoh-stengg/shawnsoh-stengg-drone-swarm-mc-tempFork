#!/usr/bin/env python3
"""
mock_fcc_node -- TP-MC-001 §2.4 hardware-facing-node mock.

Stands in for fcc_bridge_node itself in TP-NI tests (TP-MC-001 §2.4:
"Hardware-facing nodes ... are replaced by mock nodes"), not for the PX4
FCC underneath it -- it publishes directly at the /mc/fcc/* level so any
downstream node under test (mission_manager, formation_controller,
visual_localisation, health_monitor, gcs_mavlink -- ICD-MC-001 §9.1) sees
the same topics/types/rates fcc_bridge_node would provide, from a recorded
(or synthetic) rosbag2 bag, looped:

  /mc/fcc/vehicle_state  mc_msgs/VehicleState  >= 10 Hz
  /mc/fcc/odometry       nav_msgs/Odometry     >= 20 Hz
  /mc/fcc/imu            sensor_msgs/Imu       >= 100 Hz

Also subscribes /mc/fcc/offboard_setpoint (mc_msgs/FccOffboardSetpoint) like
the real node does, and republishes the latest one on
/mock/fcc/setpoint_echo -- a test-harness-only convenience topic (not part
of the ICD) so a TP-NI test can assert the mock actually received it
without needing a second introspection mechanism.
"""
from mc_msgs.msg import FccOffboardSetpoint, VehicleState
from mc_test_utils.bag_playback import BagPlayback
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

_TOPIC_MSG_TYPES = {
    '/mc/fcc/vehicle_state': VehicleState,
    '/mc/fcc/odometry': Odometry,
    '/mc/fcc/imu': Imu,
}


class MockFccNode(Node):

    def __init__(self):
        super().__init__('mock_fcc_node')
        self.declare_parameter('bag_path', '')
        self.declare_parameter('tick_period_s', 0.01)

        bag_path = self.get_parameter('bag_path').value
        if not bag_path:
            raise RuntimeError(
                "mock_fcc_node requires the 'bag_path' parameter (TP-MC-001 §3.4 fcc_nominal bag)"
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
        self._setpoint_echo_pub = self.create_publisher(
            FccOffboardSetpoint, '/mock/fcc/setpoint_echo', 10
        )
        self.create_subscription(
            FccOffboardSetpoint, '/mc/fcc/offboard_setpoint', self._on_setpoint, 10
        )

        self._start_time = self.get_clock().now()
        self.create_timer(float(self.get_parameter('tick_period_s').value), self._tick)
        self.get_logger().info(
            'mock_fcc_node playing back {} ({} messages, {:.2f}s, looped)'.format(
                bag_path, self._playback.message_count, self._playback.duration_ns / 1e9
            )
        )

    def _on_setpoint(self, msg: FccOffboardSetpoint):
        self._setpoint_echo_pub.publish(msg)

    def _tick(self):
        elapsed_ns = (self.get_clock().now() - self._start_time).nanoseconds
        for topic, msg in self._playback.due(elapsed_ns):
            self._topic_pubs[topic].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockFccNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
