# Assigned Voronoi cell boundary — published by swarm_coordinator_node on
# /mc/swarm/voronoi_cell, on change (ICD-MC-001 §9.2, SDD-MC-001 §3.2).
std_msgs/Header header
uint8   drone_id
uint8   round                        # partition round (0 = initial, 1+ = re-partition)
geographic_msgs/GeoPoint[] vertices  # CCW order
uint32  upload_id                    # corresponding mission upload ID
