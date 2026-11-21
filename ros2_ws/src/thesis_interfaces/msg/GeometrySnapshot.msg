# Immutable, time-bounded geometric evidence used by a safety decision.
builtin_interfaces/Time stamp
builtin_interfaces/Duration valid_for

# Identifies one simulation/replay epoch. A coordinator rejects mixed epochs.
string session_id

# synthetic_state, gazebo or replay. Physical sources are not admitted by the
# offline three-simulation launch.
string source_kind

string robot_id
string profile_id
string geometry_version
string asset_hash
string state
string reference_frame
string mount_frame
string tcp_frame

# False means the complete descriptor must be rejected before TF projection.
bool valid

# Parallel primitive descriptors. A CAPSULE uses start/end frames and radius;
# a SPHERE uses start_frame as its centre frame and radius. Offsets are
# expressed in their associated local frames; SPHERE ignores end_frames and
# uses start_offsets as its local centre.
string[] primitive_names
string[] primitive_types
string[] start_frames
string[] end_frames
geometry_msgs/Vector3[] start_offsets
geometry_msgs/Vector3[] end_offsets
float64[] radii
