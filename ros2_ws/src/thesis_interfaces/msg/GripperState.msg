# State that determines which conservative tool envelope is active.
builtin_interfaces/Time stamp

# Stable identity of the robot instance: arm_01 through arm_04.
string robot_id
string profile_id
string geometry_version
string asset_hash

# cerrado, nominal, abierto, con_carga or desconocido.
string state
float64 aperture
bool has_attached_object
string attached_object_profile_id

# False means the safety layer must not treat the tool as an empty volume.
bool valid
