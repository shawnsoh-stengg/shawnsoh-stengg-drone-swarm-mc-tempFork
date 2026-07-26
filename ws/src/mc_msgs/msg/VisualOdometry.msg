# GAP NOTE: ICD-MC-001 §9's topic table has cited mc_msgs/VisualOdometry
# since it was added, but no message body existed anywhere in the ICD
# (v0.1-DRAFT, 2026-06-20) until this file. Authored locally 2026-07-26 to
# close that hole — not sourced from an upstream ICD revision; needs
# ICD-owner review/ratification.
#
# GNSS-denied pose estimate from visual_localisation_node, forwarded to the
# FCC via FccBackend::publish_visual_odometry() on /mc/fcc/visual_odometry
# while /mc/localisation_mode == VISUAL_ODOMETRY (SDD §8.1/§8.3), >= 10 Hz.
# NED frame, matches px4_msgs::msg::VehicleOdometry field layout
# (position/q/velocity/variance) on the XrceDdsBackend side.
# position_variance is float32[3] (not a scalar) to match the shape of the
# fcc_bridge::VisualOdometry struct (ws/src/fcc_bridge/include/fcc_bridge/
# visual_odometry.hpp) this message must eventually convert into, avoiding
# lossy flattening of anisotropic NED variance at that conversion boundary.
std_msgs/Header header
float32 x                    # N, m — VO origin reset to last GNSS fix on handover (§8.3)
float32 y                    # E, m
float32 z                    # D, m
float32[4] q                 # [w,x,y,z] NED-to-body, same convention as VehicleState.q
float32 vx                   # m/s NED
float32 vy                   # m/s NED
float32 vz                   # m/s NED
float32[3] position_variance # m^2 per NED axis, grows with time since last GNSS fix (§8.4)
float32 orientation_variance # rad^2
uint32  reset_counter        # increments on each VO re-initialisation (GNSS handover, §8.3)
int8    quality               # tracked-feature-count-derived confidence, 0-100; -1 = unknown
