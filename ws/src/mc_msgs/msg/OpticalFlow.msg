# Downward-facing optical-flow + rangefinder sample (HIF-008, HID-MC-001.md
# §10). IGS-001 (indoor_nav/gnss_sim_node) defined this message ahead of
# IGS-002 landing, to develop its dead-reckoning fusion core against a
# concrete topic rather than assuming one.
#
# Real hardware: the selected part, Holybro H-Flow, connects via CAN1
# (DroneCAN) to the Pixhawk FCC -- PX4's own uavcan_flow driver ingests it
# into the `sensor_optical_flow`/`vehicle_optical_flow` uORB topics on the
# FCC side, neither of which stock PX4 v1.14 bridges to the companion
# computer over uXRCE-DDS (confirmed against the real v1.14.4 tag's
# dds_topics.yaml). Populated by `fcc_bridge`'s `OpticalFlowRelay`
# (ws/src/fcc_bridge/include/fcc_bridge/optical_flow_relay.hpp) decoding a
# second, dedicated MC<->FCC MAVLink channel (`OPTICAL_FLOW_RAD` +
# `DISTANCE_SENSOR`, ICD-MC-001.md §3.3a) -- chosen over a custom PX4
# firmware/dds_topics.yaml change because it needs no firmware rebuild
# pipeline. Published on /mc/sensors/optical_flow by fcc_bridge_node.
#
# vx/vy are computed on the MC side from OPTICAL_FLOW_RAD's raw
# integrated_x/y + integrated_xgyro/ygyro, replicating PX4 v1.14.4's own
# VehicleOpticalFlow.cpp formula exactly (gyro-compensated, scaled by the
# paired rangefinder distance) -- confirmed against the real v1.14.4 tag,
# see ICD-MC-001.md §3.3a for the formula. gnss_sim_node integrates the
# result directly; no further pixel-flow-to-velocity scaling is needed on
# its side.
std_msgs/Header header
float32 vx               # m/s, body FRD X (forward)
float32 vy               # m/s, body FRD Y (right)
float32 ground_distance  # m, paired rangefinder height AGL
uint8 quality             # 0-255, sensor-reported confidence; 0 = invalid/no return
