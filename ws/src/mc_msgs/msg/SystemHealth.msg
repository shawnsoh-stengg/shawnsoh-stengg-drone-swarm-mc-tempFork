# Aggregate system health — published by health_monitor_node on /mc/health,
# >=1 Hz (ICD-MC-001 §9.2).
std_msgs/Header header
uint8   cpu_pct           # 0-100
uint8   mem_pct           # 0-100
uint8   disk_pct          # 0-100
uint8   gpu_temp_c        # degC
uint8   cpu_temp_c        # degC
uint16  fault_mask        # OR of mc_msgs/FaultEvent bit constants, ICD §10.8
uint16  node_alive_mask   # see ICD §10.9 node bit assignments
uint32  uptime_s
