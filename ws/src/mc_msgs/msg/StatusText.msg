# Human-readable status/warning/error text destined for the GCS operator
# (ICD-MC-001 §4.4/§4.7 STATUSTEXT severity table) — e.g. EXEC-002's
# pre-arm-check failure reasons. gcs_mavlink_node relays every message as a
# MAVLink STATUSTEXT on Channel B, split into v2 chunks (id/chunk_seq) when
# longer than 50 chars.
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

string text               # up to 200 chars reach the GCS (4 STATUSTEXT chunks, ICD §4.7)
