 #!/usr/bin/env python3
"""FT-001 AC2: send a real COMMAND_LONG on Channel B, confirm gcs_mavlink_node
decodes it and ACKs within the 500 ms bound (FR-061/FR-003).

Run this from the real GCS laptop (or a host that owns GCS_IP on the
192.168.10.x subnet) — mavlink-router's GCS_CHANNEL_B endpoint sends MC->GCS
traffic to a FIXED address (GCS_IP:14551), not back to whatever ephemeral
port this script's send-socket used, so a single udpout connection will
never see the reply. Two sockets are required: one to send (GCS_MC_RECV,
:14552) and one to receive (GCS_CHANNEL_B, :14551).
"""
import sys
import time

from pymavlink import mavutil

# --- EDIT THESE to match the *specific unit under test*'s real
# /etc/drone-mc/environment — do not use the ticket text's example values
# (192.168.10.11 / DRONE_ID=1) unless that's actually this unit's config.
MC_IP = "192.168.10.14"   # this unit's MC_ETH_IP
DRONE_ID = 4              # this unit's DRONE_ID (== target_system)

MC_CMD_END_MISSION = 50003
MC_COMPONENT_ID = 191  # MAV_COMP_ID_ONBOARD_COMPUTER (docs/GCS-MAVLINK-API-REFERENCE.md)

tx = mavutil.mavlink_connection(f"udpout:{MC_IP}:14552", source_system=255, source_component=0)
rx = mavutil.mavlink_connection("udpin:0.0.0.0:14551")

def send_heartbeat():
    tx.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

# Prime the link: GCS_MC_RECV needs an inbound frame before mavlink-router
# will treat this as a live peer, and gcs_mavlink_node's own 10s
# GCS_LINK_TIMEOUT_S logic needs a heartbeat to consider the link up.
send_heartbeat()
time.sleep(0.5)

t0 = time.time()
tx.mav.command_long_send(DRONE_ID, MC_COMPONENT_ID, MC_CMD_END_MISSION, 0, 0, 0, 0, 0, 0, 0, 0)

ack = tx.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
elapsed_ms = (time.time() - t0) * 1000

if ack is None:
  print(f"FAIL: no COMMAND_ACK within 1s (elapsed {elapsed_ms:.0f} ms)")
  sys.exit(1)

print(f"ACK after {elapsed_ms:.0f} ms: command={ack.command} result={ack.result}")
if ack.command != MC_CMD_END_MISSION:
  print(f"FAIL: ACK is for command {ack.command}, expected {MC_CMD_END_MISSION}")
  sys.exit(1)
if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
  print(f"FAIL: result={ack.result}, expected MAV_RESULT_ACCEPTED")
  sys.exit(1)
if elapsed_ms > 500:
  print(f"FAIL: {elapsed_ms:.0f} ms exceeds FR-061/FR-003's 500 ms bound")
  sys.exit(1)

print("PASS: COMMAND_LONG decoded and ACKed within bound")

# Keep the link alive at 1 Hz for the rest of the AC2 observation window —
# leave this running while you watch `ros2 topic hz /mc/gcs/link_status`
# on the Jetson in another terminal.
try:
  while True:
      send_heartbeat()
      time.sleep(1.0)
except KeyboardInterrupt:
  pass
