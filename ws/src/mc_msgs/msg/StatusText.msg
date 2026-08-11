# Human-readable status/warning/error text destined for the GCS operator
# (ICD-MC-001 §4.4/§4.7 STATUSTEXT severity table) — e.g. EXEC-002's
# pre-arm-check failure reasons. gcs_mavlink_node does not yet subscribe to
# this topic and relay it as a real MAVLink STATUSTEXT frame (the way
# mission_orchestrator_node's own send_statustext() does for Channel C) —
# that relay is follow-up wiring, out of scope for the ticket that
# introduced this message.
std_msgs/Header header

uint8 SEVERITY_EMERGENCY = 0
uint8 SEVERITY_ALERT     = 1
uint8 SEVERITY_CRITICAL  = 2
uint8 SEVERITY_ERROR     = 3
uint8 SEVERITY_WARNING   = 4
uint8 SEVERITY_NOTICE    = 5
uint8 SEVERITY_INFO      = 6
uint8 SEVERITY_DEBUG     = 7
uint8 severity           # see SEVERITY_* constants above (MAV_SEVERITY)

string text               # truncated to 50 chars by the eventual MAVLink relay (ICD §4.7)
