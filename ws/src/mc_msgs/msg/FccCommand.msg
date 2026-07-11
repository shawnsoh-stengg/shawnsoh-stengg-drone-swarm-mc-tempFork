# Vehicle command forwarded to the FCC on /mc/fcc/send_command (arm/disarm,
# mode change, RTL, HOLD, ...) — event-driven, mirrors
# FccBackend::send_command() (SDD-MC-001 §3.1, FCC-005).
std_msgs/Header header
uint32 cmd                   # MAV_CMD
float32[7] params            # param1-param7
