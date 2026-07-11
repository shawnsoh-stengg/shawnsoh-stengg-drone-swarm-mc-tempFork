# A single PX4 mission item — mirrors fcc_bridge::MissionItem (ICD-MC-001
# §3.3 "Mission Protocol"). Published standalone on /mc/fcc/mission_item_update
# (FORMATION_SLOT tracking, >=5 Hz during INGRESS) and reused as the element
# type of mc_msgs/FccMission.items (FCC-005).
std_msgs/Header header
uint16  seq
uint8   frame               # MAV_FRAME (5 = FRAME_GLOBAL_INT)
uint16  command              # MAV_CMD (16 = NAV_WAYPOINT, 17 = NAV_LOITER_UNLIM)
int32   lat                  # 1e7 deg
int32   lon                  # 1e7 deg
float32 alt                  # m MSL
float32 acceptance_radius    # m
