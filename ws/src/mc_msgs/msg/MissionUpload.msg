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

# bit N = skip mc_msgs/MissionPhase phase-ID N, so a plan needn't fly every
# stage to complete. Only these values are accepted (anything else fails
# the pre-arm check, and both GCS decoders reject it at upload):
#   0                                   full swarm mission
#   SKIP_SWARM_PHASES                   solo: TAKEOFF->INGRESS->SEARCH->RETURN->LAND,
#                                       searching the whole search_polygon
#   SKIP_SWARM_PHASES | SKIP_SEARCH     transit: TAKEOFF->INGRESS->RETURN->LAND
uint16 SKIP_SWARM_PHASES = 112  # MUSTER | ISR_SELECTION | VORONOI_PARTITION (bits 4-6)
uint16 SKIP_SEARCH       = 128  # SEARCH (bit 7)
uint16 skip_phase_mask
