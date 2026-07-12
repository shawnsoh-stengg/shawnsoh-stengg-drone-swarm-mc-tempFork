# GCS<->MC link health (ICD-MC-001 §7.2, §4.8)
std_msgs/Header header
bool    gcs_link_ok
uint64  last_heartbeat_us  # UTC us of last GCS heartbeat received
float32 link_quality       # 0.0-1.0 (RSSI-derived if available)
uint32  rx_packet_count
uint32  tx_packet_count
uint32  rx_drop_count
