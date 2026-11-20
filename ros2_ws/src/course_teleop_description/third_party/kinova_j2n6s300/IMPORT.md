# Kinova JACO2 J2N6S300 import

This directory records the third-party portion of the visual and kinematic
description used by the `jaco2_sim` Gazebo Sim launch.

## Source

- Upstream: <https://github.com/Kinovarobotics/kinova-ros>
- Local authorized source: `/home/luis/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros/kinova_description`
- Upstream revision: `924781d3dfe241b2b94b3c72a804b80d3658cf02`
- Upstream branch: `noetic-devel`
- Model: Kinova JACO2 `j2n6s300` (six joints, non-spherical wrist, three-finger hand).

## Imported material

The mesh files in `../../meshes/kinova_j2n6s300/` and the following URDF
macro files are derived from `kinova_description`:

- `urdf/j2n6s300.xacro`
- `urdf/kinova_common.xacro`
- `urdf/kinova_finger_set.xacro`
- `urdf/kinova_inertial.xacro`

The import retains the upstream geometry, link/joint names, limits and
inertial values. Package references were changed to
`course_teleop_description`; Gazebo Classic plugins and ROS 1 transmission
blocks were removed. The ROS 2 simulation wrapper is
`course_teleop_simulation/urdf/jaco2_gazebo.urdf.xacro` and uses
`gz_ros2_control`.

## License

The imported and derived Kinova material is BSD-3-Clause, copyright 2017
Kinova Robotics inc. Its unmodified license text is in
`LICENSE-KINOVA-BSD-3-CLAUSE`. Project-authored files remain Apache-2.0 unless
they state otherwise.
