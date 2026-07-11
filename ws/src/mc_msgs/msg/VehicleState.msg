# Aggregated FCC vehicle state — published by fcc_bridge_node at >=10 Hz
# (SDD-MC-001 §3.4, ICD-MC-001 §7.2, FCC-004).
std_msgs/Header header

# Position
float64 lat             # deg  WGS-84
float64 lon             # deg  WGS-84
float32 alt_msl         # m    MSL
float32 x               # m    NED North
float32 y               # m    NED East
float32 z               # m    NED Down

# Velocity
float32 vx              # m/s  NED
float32 vy              # m/s  NED
float32 vz              # m/s  NED
float32 groundspeed     # m/s

# Attitude
float32 heading         # rad  from North
float32[4] q            # [w,x,y,z] NED-to-body

# Status
uint8 nav_state         # PX4 nav state (ICD §8.1)
uint8 arming_state      # PX4 arming state (ICD §8.2)

# Battery
float32 battery_v       # V
float32 battery_pct     # 0.0 - 1.0
uint8 battery_warning   # ICD §8.3

# GNSS
uint8 fix_type          # 0=no fix, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
float32 hdop            # horizontal DOP
float32 vdop            # vertical DOP
bool gnss_ok            # fix_type >= 3 AND hdop < threshold

# Link
bool fcc_link_ok        # uXRCE-DDS bridge alive
