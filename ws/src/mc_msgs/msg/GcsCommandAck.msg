# EXEC-003: outcome-level acknowledgement of a Phase-tier GCS command (SDD
# §4.3 priority 2 — "rejected if guard fails; ACKed <= 500 ms"), published
# by mission_manager_node. Distinct from gcs_mavlink_node's own immediate
# COMMAND_ACK (ICD §4.4/MAVLINK_MSG_ID_COMMAND_ACK), which only confirms the
# frame was received/decoded, not whether the guard actually accepted it.
# Relaying this as a second, outcome-reflecting MAVLink COMMAND_ACK via
# gcs_mavlink_node is follow-up wiring, out of this ticket's scope.
std_msgs/Header header
uint32 command

uint8 RESULT_ACCEPTED = 0
uint8 RESULT_REJECTED = 1
uint8 result

string reason   # populated when result == RESULT_REJECTED
