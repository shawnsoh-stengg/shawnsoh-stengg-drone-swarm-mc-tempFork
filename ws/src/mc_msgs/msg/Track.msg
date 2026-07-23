# A single tracked target from perception_front_node (gimbal camera),
# persists across frames — element type of mc_msgs/TrackArray.tracks
# (ICD-MC-001 §9.2).
uint32  track_id           # Persists across frames for the same target
uint16  class_id
string  class_name
float32 confidence         # 0.0 - 1.0
float64 lat                # deg — estimated target geodetic position
float64 lon                # deg
float32 alt_msl            # m
uint32  frames_tracked     # Consecutive frames this track has been held
