# video_mux_node health — published on /mc/video/downlink_status (Stream A,
# gimbal) and /mc/video/down_downlink_status (Stream B, downward), >=1 Hz
# (ICD-MC-001 §9.2, IFD-007 §8.3).
std_msgs/Header header
bool    streaming
uint32  bitrate_kbps
bool    client_connected
