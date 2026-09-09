# Downward-facing optical-flow sensor sample (HIF-008, IGS-002 -- hardware
# and driver not yet built, INDOOR-GNSS-SIMULATION-BACKLOG.md). This
# definition is provisional: IGS-001 (indoor_nav/gnss_sim_node) needs a
# concrete topic to develop its dead-reckoning fusion core against ahead of
# IGS-002 landing (see that ticket's own "can develop against a mocked flow
# topic" relationship note) -- IGS-002 owns the real driver and may revise
# this shape once the physical sensor is selected.
#
# vx/vy are already scaled to metric body-frame velocity (flow rate x
# ground_distance), matching common flow+rangefinder modules that fuse this
# on-sensor -- gnss_sim_node integrates them directly rather than doing its
# own pixel-flow-to-velocity scaling.
std_msgs/Header header
float32 vx               # m/s, body FRD X (forward)
float32 vy               # m/s, body FRD Y (right)
float32 ground_distance  # m, paired rangefinder height AGL
uint8 quality            # 0-255 sensor-reported confidence; 0 = invalid/no return
