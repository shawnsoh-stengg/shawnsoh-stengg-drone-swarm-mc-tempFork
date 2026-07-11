# A single fault event — published on /mc/fault (ICD-MC-001 §7.2, §8.8).

# Fault mask bits (ICD §8.8). fault_id carries exactly one bit set — the
# same bit values are OR-ed together into mc_msgs/SystemHealth.fault_mask.
uint16 FCC_LINK_LOST    = 1
uint16 GCS_LINK_LOST    = 2
uint16 EXECUTOR_DEAD    = 4
uint16 LOW_BATTERY      = 8
uint16 PEER_LOST        = 16
uint16 GPU_OVERTEMP     = 32
uint16 DISK_FULL        = 64
uint16 CAMERA_FAIL      = 128
uint16 GNSS_DEGRADED    = 256
uint16 VISUAL_ODOM_FAIL = 512

std_msgs/Header header
uint16 fault_id          # see constants above
uint8 severity           # MAV_SEVERITY level
string description

uint8 ACTION_NONE = 0
uint8 ACTION_HOLD = 1
uint8 ACTION_RTL  = 2
uint8 ACTION_LAND = 3
uint8 failsafe_action    # see ACTION_* constants above
