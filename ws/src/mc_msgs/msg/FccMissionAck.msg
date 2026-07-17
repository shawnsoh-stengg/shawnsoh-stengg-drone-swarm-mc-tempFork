# Ack for a mission upload or mission-item-update request forwarded by
# fcc_bridge_node (the /mc/fcc/mission_upload and /mc/fcc/mission_item_update
# subscriptions, FCC-005) — reports whether the FCC's
# MISSION_ACK/MAV_MISSION_ACCEPTED response arrived before timeout (FCC-006).
# Event-driven: published once per forwarded request, no periodic republish.
std_msgs/Header header

uint8 UPLOAD      = 0
uint8 ITEM_UPDATE = 1
uint8 request_type       # UPLOAD or ITEM_UPDATE, see constants above

uint32 upload_id         # valid iff request_type == UPLOAD; echoes FccMission.upload_id
uint16 seq                # valid iff request_type == ITEM_UPDATE; echoes FccMissionItem.seq

bool success              # true = MAV_MISSION_ACCEPTED before timeout; false = timeout/reject
