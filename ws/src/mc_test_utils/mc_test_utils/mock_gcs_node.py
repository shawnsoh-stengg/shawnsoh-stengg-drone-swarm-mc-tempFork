#!/usr/bin/env python3
"""
mock_gcs_node -- TP-MC-001 §2.4 GCS MAVLink test double (ICD-MC-001 §4, IFD-002, Channel B).

Plays the GCS side of the Channel B loopback link a real gcs_mavlink_node
would otherwise need mavlink-router plus an actual GCS to exercise in a
TP-NI test: emits a periodic HEARTBEAT (ICD §4.8: IR-011, 1 Hz) and,
optionally, a periodic COMMAND_LONG (default MC_PHASE_TRANSITION, ICD
§4.5) toward the MC-side UDP port. Not a ROS-topic mock on the wire side --
Channel B is raw MAVLink 2/UDP, so a TP-NI test drives/observes this node
with mavlink_lite directly, not rclpy subscriptions. The only ROS-facing
surface is /mock/gcs/last_rx_msgid, a test-harness-only convenience (not
part of the ICD) that republishes the MAVLink msgid of whatever frame this
node last decoded, so a test can assert a COMMAND_ACK/HEARTBEAT/MC_STATUS
round-trip without needing its own second socket listener.
"""
import socket

from mc_test_utils import mavlink_lite as mav
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32

MC_PHASE_TRANSITION = 50001  # ICD-MC-001 §4.5


class MockGcsNode(Node):

    def __init__(self):
        super().__init__('mock_gcs_node')
        self.declare_parameter('bind_address', '127.0.0.1')
        self.declare_parameter('bind_port', 14551)  # ICD §4.2 GCS receive port
        self.declare_parameter('mc_address', '127.0.0.1')
        self.declare_parameter('mc_port', 14552)  # ICD §4.2 MC receive port
        self.declare_parameter('gcs_system_id', 255)  # ICD §4.2
        self.declare_parameter('heartbeat_period_s', 1.0)  # ICD §4.8 IR-011
        self.declare_parameter('command_period_s', 0.0)  # 0 = disabled
        self.declare_parameter('command_id', MC_PHASE_TRANSITION)
        self.declare_parameter('poll_period_s', 0.02)

        self._sysid = int(self.get_parameter('gcs_system_id').value)
        self._compid = 190  # MAV_COMP_ID_MISSIONPLANNER-ish placeholder;
        # ICD §4 doesn't pin down the GCS's own component id.
        self._mc_addr = (
            self.get_parameter('mc_address').value,
            int(self.get_parameter('mc_port').value),
        )
        self._seq = 0

        bind_address = self.get_parameter('bind_address').value
        bind_port = int(self.get_parameter('bind_port').value)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_address, bind_port))
        self._sock.setblocking(False)

        self._last_rx_pub = self.create_publisher(UInt32, '/mock/gcs/last_rx_msgid', 10)

        hb_period = float(self.get_parameter('heartbeat_period_s').value)
        self.create_timer(hb_period, self._send_heartbeat)
        command_period = float(self.get_parameter('command_period_s').value)
        if command_period > 0.0:
            self.create_timer(command_period, self._send_command)
        self.create_timer(float(self.get_parameter('poll_period_s').value), self._poll_socket)

        self.get_logger().info(
            'mock_gcs_node sysid={} -> mc={}, listening on {}:{}'.format(
                self._sysid, self._mc_addr, bind_address, bind_port
            )
        )

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (seq + 1) & 0xFF
        return seq

    def _send_heartbeat(self):
        frame = mav.encode(
            mav.MSG_HEARTBEAT, self._next_seq(), self._sysid, self._compid,
            custom_mode=0, type=6, autopilot=8, base_mode=0, system_status=4, mavlink_version=3,
        )
        self._sock.sendto(frame, self._mc_addr)

    def _send_command(self):
        command_id = int(self.get_parameter('command_id').value)
        frame = mav.encode(
            mav.MSG_COMMAND_LONG, self._next_seq(), self._sysid, self._compid,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0,
            command=command_id, target_system=11, target_component=191, confirmation=0,
        )
        self._sock.sendto(frame, self._mc_addr)

    def _poll_socket(self):
        while True:
            try:
                data, _addr = self._sock.recvfrom(2048)
            except BlockingIOError:
                return
            frame = mav.decode(data)
            if frame is not None:
                self._last_rx_pub.publish(UInt32(data=frame.msgid))

    def destroy_node(self):
        self._sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MockGcsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
