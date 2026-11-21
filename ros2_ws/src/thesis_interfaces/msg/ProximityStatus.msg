# Timestamp and reference frame used for the geometric evaluation.
builtin_interfaces/Time stamp
string reference_frame

# Identity of the exact robot/tool geometry that produced this measurement.
# A consumer must fail closed when geometry_valid is false.
string robot_id
string geometry_profile_id
string geometry_version
string geometry_asset_hash
string geometry_state
bool geometry_valid

# ALLOW, WARNING, REDUCTION or STOP.
string state

# Surface-to-surface clearance in metres. Negative means overlap.
float64 minimum_clearance
string limiting_segment

# Geometric witnesses used for diagnostics and RViz rendering.
geometry_msgs/Point closest_robot_point
geometry_msgs/Point obstacle_center
geometry_msgs/Vector3 obstacle_velocity
float64 obstacle_radius
