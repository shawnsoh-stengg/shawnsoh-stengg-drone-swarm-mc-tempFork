# GNSS<->visual-odometry handover state — published by
# visual_localisation_node on /mc/localisation_mode, on change
# (ICD-MC-001 §9.2).
std_msgs/Header header

uint8 MODE_GNSS_PRIMARY    = 0
uint8 MODE_VISUAL_ODOMETRY = 1
uint8   mode               # see MODE_* constants above

uint8 REASON_NOMINAL         = 0
uint8 REASON_GNSS_FIX_LOST   = 1
uint8 REASON_HDOP_HIGH       = 2
uint8 REASON_GNSS_REACQUIRED = 3
uint8   reason              # see REASON_* constants above

float32 hdop                # Current HDOP
uint8   fix_type            # Current GNSS fix type (mc_msgs/VehicleState.fix_type values)
