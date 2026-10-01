#!/usr/bin/env python3
"""FT-006 bench helper (INTEGRATION-AND-FLIGHT-TEST-PLAN.md): drive and
observe gnss_sim_node's HIL_GPS path against a real Pixhawk.

Run inside any drone-mc container with the workspace sourced (it needs
mc_msgs + px4_msgs Python bindings, --network=host, CycloneDDS config):

  flow                  mock /mc/sensors/optical_flow at 20 Hz (zero motion)
  anchor LAT LON ALT    send MC_CMD_INDOOR_GNSS_SIM_ANCHOR (50005) once
  watch                 1 Hz side-by-side: gnss_sim_status vs PX4's
                        /fmu/out/vehicle_gps_position, with device_id decoded

`flow` is the ticket's allowed substitution for missing H-Flow hardware
(FT-006's last AC) — record it in the test log when used. gnss_sim_node's
IGS-005 staleness watchdog latches after input_staleness_timeout_s (1 s)
without a flow sample, so `flow` must be running before `anchor`.

`anchor` publishes straight onto /mc/gcs/command, the topic
gcs_mavlink_node emits for a decoded Channel B COMMAND_LONG — Channel B's
own decode/ACK path is FT-001 AC2's scope, not this ticket's. The header
stamp matters: gnss_sim_node measures time_since_anchor (and so eph/epv
growth) from it, which is why this isn't a plain `ros2 topic pub`.

gnss_sim_node's vertical input is /mc/sensors/baro_altitude, which
fcc_bridge relays from the FCC's MAVLink ALTITUDE stream on the flow
instance (:14556) -- not mocked here. If `watch` shows `baro: none`, add
ALTITUDE to that instance's streams on the FCC, or IGS-005's watchdog
latches INDOOR_GNSS_SIM_LOST ("baro_altitude input stale") right after
the anchor.
"""
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mc_msgs.msg import BaroAltitude, FaultEvent, GcsCommand, IndoorGnssSimStatus, OpticalFlow
from px4_msgs.msg import SensorGps

MC_CMD_INDOOR_GNSS_SIM_ANCHOR = 50005
MC_COMPONENT_ID = 191  # MAV_COMP_ID_ONBOARD_COMPUTER

# PX4 v1.17 handle_message_hil_gps(): device_id = {bus_type=MAVLINK(6),
# bus=mavlink instance, address=msg sysid, devtype=DRV_GPS_DEVTYPE_SIM}.
BUS_TYPE_MAVLINK = 6
DRV_GPS_DEVTYPE_SIM = 0xAF

# Same QoS fcc_bridge's XrceDdsBackend uses for /fmu/* (PX4's own).
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def decode_device_id(devid):
    bus_type = devid & 0x07
    bus = (devid >> 3) & 0x1F
    address = (devid >> 8) & 0xFF
    devtype = (devid >> 16) & 0xFF
    return bus_type, bus, address, devtype


def run_flow(node):
    pub = node.create_publisher(OpticalFlow, "/mc/sensors/optical_flow", 10)

    def tick():
        msg = OpticalFlow()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.vx = 0.0
        msg.vy = 0.0
        msg.ground_distance = 0.5
        msg.quality = 200
        pub.publish(msg)

    node.create_timer(0.05, tick)
    print("flow: publishing mock /mc/sensors/optical_flow at 20 Hz (Ctrl-C to stop)")
    rclpy.spin(node)


def run_anchor(node, lat, lon, alt):
    pub = node.create_publisher(GcsCommand, "/mc/gcs/command", 10)
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print("anchor: FAIL -- no /mc/gcs/command subscriber (is gnss_sim_node running?)")
        sys.exit(1)

    msg = GcsCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.command = MC_CMD_INDOOR_GNSS_SIM_ANCHOR
    msg.params = [float(lat), float(lon), float(alt), 0.0, 0.0, 0.0, 0.0]
    msg.target_component = MC_COMPONENT_ID
    pub.publish(msg)
    # Let the sample actually leave before the process exits.
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"anchor: sent {MC_CMD_INDOOR_GNSS_SIM_ANCHOR} lat={lat} lon={lon} alt={alt}")


def run_watch(node):
    latest = {"sim": None, "gps": None, "gps_rx": 0.0, "gps_n": 0, "lost": None,
              "baro": None, "baro_rx": 0.0}

    def on_sim(msg):
        prev = latest["sim"]
        if prev is not None and msg.time_since_anchor_s < prev.time_since_anchor_s:
            latest["lost"] = None  # re-anchored -- gnss_sim_node cleared its latch too
        latest["sim"] = msg

    def on_gps(msg):
        latest["gps"] = msg
        latest["gps_rx"] = time.time()
        latest["gps_n"] += 1

    def on_baro(msg):
        latest["baro"] = msg
        latest["baro_rx"] = time.time()

    def on_fault(msg):
        # IndoorGnssSimStatus.active stays true after IGS-005's watchdog
        # latches, so the latch is only visible here -- and only if this
        # watch was already running when it fired (edge-triggered, not
        # latched on the topic). Cleared again by the next re-anchor.
        if msg.fault_id == FaultEvent.INDOOR_GNSS_SIM_LOST:
            latest["lost"] = msg.description

    node.create_subscription(IndoorGnssSimStatus, "/mc/indoor/gnss_sim_status", on_sim, 10)
    node.create_subscription(FaultEvent, "/mc/fault", on_fault, 10)
    node.create_subscription(BaroAltitude, "/mc/sensors/baro_altitude", on_baro, 10)
    node.create_subscription(SensorGps, "/fmu/out/vehicle_gps_position", on_gps, PX4_QOS)

    def report():
        sim = latest["sim"]
        gps = latest["gps"]
        sim_s = ("sim: no status yet" if sim is None else
                 f"sim: active={int(sim.active)} eph={sim.eph:.2f} epv={sim.epv:.2f} "
                 f"t={sim.time_since_anchor_s:.0f}s")
        if gps is None:
            gps_s = "px4: no vehicle_gps_position yet"
        else:
            bus_type, bus, address, devtype = decode_device_id(gps.device_id)
            src = ("HIL_GPS" if bus_type == BUS_TYPE_MAVLINK and devtype == DRV_GPS_DEVTYPE_SIM
                   else "other")
            age = time.time() - latest["gps_rx"]
            gps_s = (f"px4: fix={gps.fix_type} eph={gps.eph:.2f} epv={gps.epv:.2f} "
                     f"sats={gps.satellites_used} lat={gps.latitude_deg:.7f} "
                     f"lon={gps.longitude_deg:.7f} src={src}(bus={bus_type}/{bus} "
                     f"sysid={address} devtype=0x{devtype:02X}) age={age:.1f}s n={latest['gps_n']}")
        baro = latest["baro"]
        baro_s = ("baro: none" if baro is None else
                  f"baro={baro.alt_m:.2f}m age={time.time() - latest['baro_rx']:.1f}s")
        print(f"{time.strftime('%H:%M:%S')}  {sim_s}  |  {baro_s}  |  {gps_s}", flush=True)
        if latest["lost"] is not None:
            print(f"          LATCHED INDOOR_GNSS_SIM_LOST: {latest['lost']}", flush=True)

    node.create_timer(1.0, report)
    rclpy.spin(node)


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("flow", "anchor", "watch"):
        print(__doc__)
        sys.exit(2)
    rclpy.init()
    node = Node("ft006_bench_" + sys.argv[1])
    try:
        if sys.argv[1] == "flow":
            run_flow(node)
        elif sys.argv[1] == "anchor":
            if len(sys.argv) != 5:
                print("usage: ft006_bench.py anchor LAT LON ALT_MSL")
                sys.exit(2)
            run_anchor(node, *sys.argv[2:5])
        else:
            run_watch(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or SIGTERM from `timeout N` -- both are normal exits
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
