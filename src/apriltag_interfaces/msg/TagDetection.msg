# TagDetection.msg
# Single AprilTag detection result (for /tag_detections topic)
int32 tag_id
float32[4] corners_x
float32[4] corners_y
float64 decision_margin
geometry_msgs/Pose pose
