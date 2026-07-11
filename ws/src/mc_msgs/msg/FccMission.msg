# Full-replace mission upload on /mc/fcc/mission_upload — on-change only, not
# periodic (SDD-MC-001 §3.4 item 2, FCC-005).
std_msgs/Header header
uint32 upload_id             # correlates with the eventual /mc/fcc/mission_ack (FCC-006)
mc_msgs/FccMissionItem[] items
