# Raw barometric altitude from the FCC (FT-006 finding). Published on
# /mc/sensors/baro_altitude by fcc_bridge_node, decoded from MAVLink
# ALTITUDE.altitude_monotonic on the dedicated flow-relay channel
# (ICD-MC-001.md §3.3a) -- PX4 fills that field straight from
# vehicle_air_data.baro_alt_meter, the barometer alone, not EKF2's fused
# estimate. Stock PX4 v1.17 bridges no baro topic over uXRCE-DDS.
#
# gnss_sim_node's vertical input. It used VehicleState.alt_msl before, which
# is EKF2 output -- once PX4 accepts gnss_sim_node's HIL_GPS it fuses that
# altitude back into alt_msl, closing a loop that ran PX4's altitude estimate
# ~42 km off on the orin-nx bench (FT-006, 2026-09-26).
#
# Standard-atmosphere pressure altitude: only differences are meaningful,
# not the absolute value (no QNH correction).
std_msgs/Header header
float32 alt_m  # m, pressure altitude
