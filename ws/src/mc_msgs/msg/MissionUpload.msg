# Full mission data uploaded by GCS — published by gcs_mavlink_node on
# /mc/gcs/mission_upload, event-driven (ICD-MC-001 §9.2).
std_msgs/Header header
uint32 upload_id                             # Unique ID for this upload

# Ingress waypoints
geometry_msgs/PoseStamped[] ingress_waypoints

# Per-drone formation offsets
mc_msgs/FormationOffset[] formation_offsets

# Search area polygon (geodetic)
geographic_msgs/GeoPoint[] search_polygon

# Swarm manifest
mc_msgs/SwarmPeer[] swarm_manifest

bool    arm_authorised
float32 low_battery_threshold  # 0.0 - 1.0 (default 0.20)

# bit N = mc_msgs/MissionPhase phase-ID N requires OFFBOARD for its
# FCC-facing legs; 0 = all MISSION mode (pre-FR-026 behaviour). See
# FR-020/026-029.
uint16 offboard_phase_mask
