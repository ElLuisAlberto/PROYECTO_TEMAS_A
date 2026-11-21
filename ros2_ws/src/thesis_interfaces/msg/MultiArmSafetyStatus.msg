# Global safety result for exactly arm_01 through arm_04.
builtin_interfaces/Time stamp
builtin_interfaces/Duration valid_for

string session_id
uint64 decision_sequence
string reference_frame

# ALLOW or STOP. The runtime publishes STOP whenever valid is false.
string state
bool valid
string reason_code
string reason

float64 clearance_threshold
float64 minimum_clearance
float64 maximum_snapshot_age
float64 snapshot_skew

string first_robot_id
string first_primitive
string second_robot_id
string second_primitive

uint32 evaluated_robot_pairs
uint64 evaluated_primitive_pairs
