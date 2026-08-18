# Aggregate system health — published by health_monitor_node on /mc/health,
# >=1 Hz (ICD-MC-001 §9.2).

# node_alive_mask bits (ICD §10.9). Bit set = health_monitor_node received a
# message on that node's designated liveness topic within its configured
# timeout (SDD §10.2). HEALTH_MONITOR_ALIVE is always set by health_monitor_
# node itself (it is self-evidently running if it is publishing this
# message). FORMATION_CONTROLLER_ALIVE and SEARCH_PLANNER_ALIVE occupy bit
# positions in the ICD's 12-bit layout but have no heartbeat topic assigned
# in SDD §10.2's tracked-node table, so health_monitor_node leaves them
# unset — a known gap between the two documents, not modeled here.
uint16 FCC_BRIDGE_ALIVE           = 1
uint16 GCS_MAVLINK_ALIVE          = 2
uint16 SWARM_COORDINATOR_ALIVE    = 4
uint16 FORMATION_CONTROLLER_ALIVE = 8
uint16 MISSION_MANAGER_ALIVE      = 16
uint16 SEARCH_PLANNER_ALIVE       = 32
uint16 VISUAL_LOCALISATION_ALIVE  = 64
uint16 CAMERA_ALIVE               = 128
uint16 PERCEPTION_DOWN_ALIVE      = 256
uint16 HEALTH_MONITOR_ALIVE       = 512
uint16 PERCEPTION_FRONT_ALIVE     = 1024
uint16 VIDEO_MUX_ALIVE            = 2048

std_msgs/Header header
uint8   cpu_pct           # 0-100
uint8   mem_pct           # 0-100
uint8   disk_pct          # 0-100
uint8   gpu_temp_c        # degC
uint8   cpu_temp_c        # degC
uint16  fault_mask        # OR of mc_msgs/FaultEvent bit constants, ICD §10.8
uint16  node_alive_mask   # OR of the node-alive bit constants above, ICD §10.9
uint32  uptime_s
