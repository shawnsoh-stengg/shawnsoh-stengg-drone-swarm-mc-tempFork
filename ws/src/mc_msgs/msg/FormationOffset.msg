# Per-drone formation offset relative to the swarm leader, NED meters —
# element type of mc_msgs/MissionUpload.formation_offsets (ICD-MC-001 §9.2).
uint8   drone_id
float32 n              # m NED North offset
float32 e              # m NED East offset
float32 d              # m NED Down offset
