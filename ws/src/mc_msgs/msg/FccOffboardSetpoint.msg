# Continuous OFFBOARD position setpoint, published on /mc/fcc/offboard_setpoint
# while an OFFBOARD-flagged leg is active (FR-027/028). NED frame, matches
# px4_msgs::msg::TrajectorySetpoint / OffboardControlMode field layout on the
# XrceDdsBackend side. Velocity/acceleration fields are deliberately omitted
# for this initial cut — formation-tracking only needs position+yaw, the same
# as the FORMATION_SLOT mission-item-replace path this supersedes on
# OFFBOARD-flagged legs (SDD §5.2). Extend later if a use case needs velocity
# control.
std_msgs/Header header
float32 x            # N, m
float32 y            # E, m
float32 z            # D, m
float32 yaw          # rad, NED
