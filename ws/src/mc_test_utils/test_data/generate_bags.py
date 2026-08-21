#!/usr/bin/env python3
"""
Generate the synthetic sample bags TP-MC-001 §3.4 lists.

Covers mock_fcc_node and mock_camera_node's fixtures (`fcc_nominal`,
`camera_outdoor`). No hardware capture exists yet (per TEST-001's
acceptance criteria, a synthetic equivalent is committed instead), so this
produces short, deterministic, entirely synthetic recordings -- not real
flight/camera data. Run manually to (re)generate the committed bags under
bags/; this is a one-off fixture-authoring script, not something colcon
build/test invokes.

Usage (inside a sourced ROS 2 Humble workspace with mc_msgs built):
    python3 generate_bags.py
"""
import math
import os
import shutil

from mc_msgs.msg import VehicleState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
BAGS_DIR = os.path.join(THIS_DIR, 'bags')

# Short fixtures by design: a TP-NI test only needs a handful of looped
# publish cycles to assert on rate/content, and short bags keep this
# package's git footprint small. Scale up if a future test needs longer
# continuous coverage (e.g. a fault injected partway through a bag).
FCC_DURATION_S = 6.0
FCC_VEHICLE_STATE_RATE_HZ = 10.0
FCC_ODOMETRY_RATE_HZ = 20.0
FCC_IMU_RATE_HZ = 100.0

CAMERA_DURATION_S = 3.0
CAMERA_FPS = 15.0
# Real camera_node publishes 1280x720 (ICD-MC-001 §6); this fixture uses a
# much smaller synthetic frame purely to keep the committed bag small --
# TEST-001's pass criteria are about frame rate/topic wiring, not
# resolution.
CAMERA_WIDTH = 64
CAMERA_HEIGHT = 48


def _write_bag(bag_dir: str, topic_specs, message_iter):
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'
        ),
    )
    for topic_name, type_name in topic_specs:
        writer.create_topic(
            rosbag2_py.TopicMetadata(name=topic_name, type=type_name, serialization_format='cdr')
        )
    for topic_name, msg, t_ns in message_iter:
        writer.write(topic_name, serialize_message(msg), t_ns)
    del writer


def generate_fcc_nominal():
    """Synthetic ~6 s circling flight profile: VehicleState/Odometry/Imu."""
    from sensor_msgs.msg import Imu

    entries = []
    n_vs = int(FCC_DURATION_S * FCC_VEHICLE_STATE_RATE_HZ)
    n_odom = int(FCC_DURATION_S * FCC_ODOMETRY_RATE_HZ)
    n_imu = int(FCC_DURATION_S * FCC_IMU_RATE_HZ)

    for i in range(n_vs):
        t = i / FCC_VEHICLE_STATE_RATE_HZ
        angle = 2 * math.pi * t / FCC_DURATION_S
        msg = VehicleState()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.lat = 1.3521 + 0.0001 * math.cos(angle)
        msg.lon = 103.8198 + 0.0001 * math.sin(angle)
        msg.alt_msl = 50.0
        msg.x = 10.0 * math.cos(angle)
        msg.y = 10.0 * math.sin(angle)
        msg.z = -50.0
        msg.vx = -1.0 * math.sin(angle)
        msg.vy = 1.0 * math.cos(angle)
        msg.vz = 0.0
        msg.groundspeed = 1.0
        msg.heading = angle
        msg.q = [1.0, 0.0, 0.0, 0.0]
        msg.nav_state = 4  # PX4 AUTO_MISSION-ish placeholder
        msg.arming_state = 2  # ARMED placeholder
        msg.battery_v = 22.2
        msg.battery_pct = max(0.2, 1.0 - t / (4 * FCC_DURATION_S))
        msg.battery_warning = 0
        msg.fix_type = 6
        msg.hdop = 0.8
        msg.vdop = 1.2
        msg.gnss_ok = True
        msg.fcc_link_ok = True
        entries.append(('/mc/fcc/vehicle_state', msg, int(t * 1e9)))

    for i in range(n_odom):
        t = i / FCC_ODOMETRY_RATE_HZ
        angle = 2 * math.pi * t / FCC_DURATION_S
        msg = Odometry()
        msg.header = Header()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = 10.0 * math.cos(angle)
        msg.pose.pose.position.y = 10.0 * math.sin(angle)
        msg.pose.pose.position.z = 50.0
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = -1.0 * math.sin(angle)
        msg.twist.twist.linear.y = 1.0 * math.cos(angle)
        entries.append(('/mc/fcc/odometry', msg, int(t * 1e9)))

    for i in range(n_imu):
        t = i / FCC_IMU_RATE_HZ
        msg = Imu()
        msg.header = Header()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.z = -9.81
        msg.angular_velocity.z = 0.05
        msg.orientation.w = 1.0
        entries.append(('/mc/fcc/imu', msg, int(t * 1e9)))

    entries.sort(key=lambda e: e[2])

    _write_bag(
        os.path.join(BAGS_DIR, 'fcc_nominal'),
        [
            ('/mc/fcc/vehicle_state', 'mc_msgs/msg/VehicleState'),
            ('/mc/fcc/odometry', 'nav_msgs/msg/Odometry'),
            ('/mc/fcc/imu', 'sensor_msgs/msg/Imu'),
        ],
        entries,
    )
    print(f'wrote {len(entries)} messages to bags/fcc_nominal')


def generate_camera_outdoor():
    """Synthetic ~3 s, 15 FPS Image + CameraInfo sequence."""
    entries = []
    n_frames = int(CAMERA_DURATION_S * CAMERA_FPS)
    frame_bytes = bytes([128]) * (CAMERA_WIDTH * CAMERA_HEIGHT * 3)  # flat grey bgr8 frame

    for i in range(n_frames):
        t_ns = int(i / CAMERA_FPS * 1e9)
        image = Image()
        image.header = Header()
        image.header.frame_id = 'camera_optical'
        image.height = CAMERA_HEIGHT
        image.width = CAMERA_WIDTH
        image.encoding = 'bgr8'
        image.is_bigendian = 0
        image.step = CAMERA_WIDTH * 3
        image.data = frame_bytes
        entries.append(('/camera/image_raw', image, t_ns))

        info = CameraInfo()
        info.header = image.header
        info.height = CAMERA_HEIGHT
        info.width = CAMERA_WIDTH
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [500.0, 0.0, CAMERA_WIDTH / 2, 0.0, 500.0, CAMERA_HEIGHT / 2, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            500.0, 0.0, CAMERA_WIDTH / 2, 0.0,
            0.0, 500.0, CAMERA_HEIGHT / 2, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        entries.append(('/camera/camera_info', info, t_ns))

    _write_bag(
        os.path.join(BAGS_DIR, 'camera_outdoor'),
        [
            ('/camera/image_raw', 'sensor_msgs/msg/Image'),
            ('/camera/camera_info', 'sensor_msgs/msg/CameraInfo'),
        ],
        entries,
    )
    print(f'wrote {len(entries)} messages to bags/camera_outdoor')


def main():
    rclpy.init()
    try:
        os.makedirs(BAGS_DIR, exist_ok=True)
        generate_fcc_nominal()
        generate_camera_outdoor()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
