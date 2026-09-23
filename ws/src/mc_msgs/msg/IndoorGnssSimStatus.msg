# gnss_sim_node false-origin/uncertainty status (INDOOR-GNSS-SIMULATION-
# BACKLOG.md, IGS-003) -- published on /mc/indoor/gnss_sim_status so
# health_monitor_node/GCS can honestly assess how much to trust this
# module's synthetic GPS_INPUT fix (IGS-004) before PX4 ever sees it.
#
# Deliberately carries no lat/lon/alt itself -- that WGS-84 fix is
# IGS-004's own GPS_INPUT payload to send, not a second ROS-visible copy
# of it. GnssSimNode::wgs84_fix() is the in-process seam IGS-004 (and this
# ticket's own node-integration tests) reads that from directly.
std_msgs/Header header

bool    active                # false until the operator has issued at least
                               # one anchor command (IGS-003 Open Item 4) --
                               # no fix should be trusted before this is true

float32 eph                   # m, 1-sigma horizontal position uncertainty.
                               # Monotonically non-decreasing since the last
                               # anchor/reset -- never silently reset mid-
                               # session (Safety Considerations #1: reporting
                               # a permanently tight eph/epv would defeat
                               # PX4's own GPS-quality failsafes).
float32 epv                   # m, 1-sigma vertical position uncertainty.
                               # Same monotonicity guarantee as eph.

float32 time_since_anchor_s   # s, time since the last (re-)anchor command.
                               # Resets to 0 only on an explicit, logged
                               # operator re-anchor -- see header.stamp for
                               # the wall-clock time of that reset.

float32 waypoint_deviation_m  # m, sanity-bound-only signal (IGS-005's own
                               # concern: fault trigger, never a position
                               # correction) -- distance from the current
                               # local estimate to the nearest uploaded
                               # ingress waypoint (GnssSimNode::
                               # compute_waypoint_deviation()). 0 until a
                               # mission has been uploaded (nothing to
                               # compare against yet), not a fabricated
                               # "zero deviation".
