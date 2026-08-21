#!/usr/bin/env python3
"""
mock_peer_node -- TP-MC-001 §2.4 swarm-UDP test double (ICD-MC-001 §5, IFD-003).

Sends a periodic SWARM_HEARTBEAT (0x01) on the heartbeat port (14900) and
answers any PEER_STATE_REQUEST (0x31) it receives on the coordination port
(14901) with a PEER_STATE (0x30) reply, matching the wire format
swarm_coordinator_node's own UDP layer will need to interoperate with once
built -- CLAUDE.md notes both swarm_coord nodes are still §2.4/§5/§6
scaffolds with no mc_msgs dependency yet. Not a ROS-topic mock: the swarm
protocol is raw UDP, so a TP-NI test drives/observes this node with a plain
socket, not rclpy subscriptions.
"""
import socket
import time

from mc_test_utils import swarm_protocol as proto
import rclpy
from rclpy.node import Node


class MockPeerNode(Node):

    def __init__(self):
        super().__init__('mock_peer_node')
        self.declare_parameter('sender_id', 99)
        self.declare_parameter('bind_address', '127.0.0.1')
        self.declare_parameter('heartbeat_target', '127.0.0.1')
        self.declare_parameter('heartbeat_period_s', 1.0)  # ICD §5.3: SWARM_HEARTBEAT rate = 1 Hz
        self.declare_parameter('poll_period_s', 0.02)

        self._sender_id = int(self.get_parameter('sender_id').value)
        bind_addr = self.get_parameter('bind_address').value
        self._hb_target = self.get_parameter('heartbeat_target').value
        self._seq = {'heartbeat': 0, 'peer_state': 0}

        # Send-only: this mock never needs to *receive* other peers' heartbeats,
        # so it deliberately doesn't bind port 14900 -- letting the OS pick an
        # ephemeral source port leaves the well-known heartbeat port free for
        # a TP-NI test (or a real peer) to bind as the sole listener.
        self._hb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._coord_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._coord_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._coord_sock.bind((bind_addr, proto.COORDINATION_PORT))
        self._coord_sock.setblocking(False)

        hb_period = float(self.get_parameter('heartbeat_period_s').value)
        poll_period = float(self.get_parameter('poll_period_s').value)
        self.create_timer(hb_period, self._send_heartbeat)
        self.create_timer(poll_period, self._poll_coordination)

        self.get_logger().info(
            'mock_peer_node sender_id={} heartbeat={}:{} -> {} coordination={}:{}'.format(
                self._sender_id, bind_addr, proto.HEARTBEAT_PORT, self._hb_target,
                bind_addr, proto.COORDINATION_PORT,
            )
        )

    def _next_seq(self, key: str) -> int:
        seq = self._seq[key]
        self._seq[key] = (seq + 1) & 0xFFFF
        return seq

    def _send_heartbeat(self):
        payload = proto.pack_heartbeat(
            timestamp_us=time.time_ns() // 1000,
            lat_e7=133600000, lon_e7=1038400000, alt_mm=50000,
            vx=1.0, vy=0.0, vz=0.0,
            mission_phase=7, battery_pct=88, sensor_health=0, isr_role=0, fault_mask=0,
        )
        seq = self._next_seq('heartbeat')
        frame = proto.pack_header(
            self._sender_id, proto.MSG_SWARM_HEARTBEAT, payload, sequence=seq
        )
        self._hb_sock.sendto(frame, (self._hb_target, proto.HEARTBEAT_PORT))

    def _poll_coordination(self):
        while True:
            try:
                data, addr = self._coord_sock.recvfrom(2048)
            except BlockingIOError:
                return
            try:
                header, _payload = proto.unpack_header(data)
            except ValueError as exc:
                self.get_logger().warning(
                    'mock_peer_node: dropped malformed swarm frame from {}: {}'.format(addr, exc)
                )
                continue
            if header.message_type == proto.MSG_PEER_STATE_REQUEST:
                self._reply_peer_state(addr)

    def _reply_peer_state(self, addr):
        payload = proto.pack_peer_state(
            timestamp_us=time.time_ns() // 1000,
            x=10.0, y=5.0, z=-50.0, vx=1.0, vy=0.0, vz=0.0,
            heading=0.3, q=(1.0, 0.0, 0.0, 0.0), is_leader=False,
        )
        frame = proto.pack_header(
            self._sender_id, proto.MSG_PEER_STATE, payload, sequence=self._next_seq('peer_state')
        )
        self._coord_sock.sendto(frame, addr)

    def destroy_node(self):
        self._hb_sock.close()
        self._coord_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MockPeerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
