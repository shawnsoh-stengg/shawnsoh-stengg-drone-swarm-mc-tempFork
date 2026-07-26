# GNSS-denied pose estimate from visual_localisation_node, forwarded to the
# FCC via FccBackend::publish_visual_odometry() on /mc/fcc/visual_odometry
# while /mc/localisation_mode == VISUAL_ODOMETRY (SDD §8.1/§8.3), >= 10 Hz.
# NED frame, matches px4_msgs::msg::VehicleOdometry field layout
# (position/q/velocity/variance) on the XrceDdsBackend side.
std_msgs/Header header
float32 x                    # N, m — VO origin reset to last GNSS fix on handover (§8.3)
float32 y                    # E, m
float32 z                    # D, m
float32[4] q                 # [w,x,y,z] NED-to-body, same convention as VehicleState.q
float32 vx                   # m/s NED
float32 vy                   # m/s NED
float32 vz                   # m/s NED
float32 position_variance    # m^2, grows with time since last GNSS fix (§8.4 drift bound)
float32 orientation_variance # rad^2
uint32  reset_counter        # increments on each VO re-initialisation (GNSS handover, §8.3)
int8    quality               # tracked-feature-count-derived confidence, 0-100; -1 = unknown
