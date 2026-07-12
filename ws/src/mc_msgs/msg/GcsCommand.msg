# Decoded GCS MAVLink command (ICD-MC-001 §7.2, §4.5)
std_msgs/Header header
uint32 command          # MAV_CMD ID
float32[7] params       # param1-param7
uint8  target_system
uint8  target_component
uint8  confirmation
