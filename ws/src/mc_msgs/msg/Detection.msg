# A single AI detection result, geolocated via camera intrinsics + drone pose
# — element type of mc_msgs/DetectionArray.detections (ICD-MC-001 §9.2,
# SDD-MC-001 §3.5).
uint32  detection_id       # Monotonic counter
uint64  frame_timestamp_us # UTC us of source frame
uint16  class_id
string  class_name
float32 confidence         # 0.0 - 1.0

# Estimated target geodetic position
float64 lat                # deg
float64 lon                # deg
float32 alt_msl            # m
float32 pos_uncertainty_m  # 1-sigma horizontal position uncertainty

# Bounding box in source image (px)
uint32  bbox_x
uint32  bbox_y
uint32  bbox_w
uint32  bbox_h
