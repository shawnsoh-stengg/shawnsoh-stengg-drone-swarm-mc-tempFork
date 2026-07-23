# Current mission phase — published by mission_manager_node on
# /mc/mission_phase, on change + 1 Hz keepalive (ICD-MC-001 §9.2, §10.6).
std_msgs/Header header

uint8 PREFLIGHT        = 0
uint8 ARMING            = 1
uint8 TAKEOFF           = 2
uint8 INGRESS           = 3
uint8 MUSTER            = 4
uint8 ISR_SELECTION     = 5
uint8 VORONOI_PARTITION = 6
uint8 SEARCH            = 7
uint8 RETURN            = 8
uint8 LAND              = 9
uint8 FAULT             = 255
uint8 phase             # see phase-ID constants above

string phase_name       # Human-readable phase name
uint64 phase_entered_us # UTC us when this phase was entered

uint8 SOURCE_GCS_COMMAND = 0
uint8 SOURCE_AUTONOMOUS  = 1
uint8 SOURCE_FAILSAFE    = 2
uint8 source             # see SOURCE_* constants above
