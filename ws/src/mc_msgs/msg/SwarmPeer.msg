# A single peer's last-known state — element type of
# mc_msgs/SwarmPeerArray.peers and mc_msgs/MissionUpload.swarm_manifest
# (ICD-MC-001 §9.2).
uint8   drone_id
string  ip_address
uint8   mission_phase      # see mc_msgs/MissionPhase phase-ID constants
float64 lat                # deg
float64 lon                # deg
float32 alt_msl            # m
float32 vx                 # m/s NED
float32 vy                 # m/s NED
float32 vz                 # m/s NED
float32 battery_pct        # 0.0 - 1.0

# ICD-MC-001 §9.2 cites "§10.10" for this bitmask's layout, but no §10.10
# exists in the current ICD revision (v0.1-DRAFT) — meaning TBD upstream.
uint8   sensor_flags

uint16  fault_mask         # OR of mc_msgs/FaultEvent bit constants, ICD §10.8

uint8 STATUS_UNKNOWN = 0
uint8 STATUS_ALIVE   = 1
uint8 STATUS_LOST    = 2
uint8 STATUS_DEAD    = 3
uint8   peer_status        # see STATUS_* constants above

bool    is_leader
bool    isr_role
uint64  last_heartbeat_us  # UTC us of last received heartbeat
