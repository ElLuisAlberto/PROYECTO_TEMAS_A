# Kinova JACO2 J2N6S300 provenance

This package contains the J2N6S300 robot-description subset used by the
cooperative-teleoperation simulator.

## Upstream attribution

- Upstream repository: `https://github.com/Kinovarobotics/kinova-ros`
- Upstream package: `kinova_description`
- Upstream branch documented by the source package: `noetic-devel`
- Selected model: `j2n6s300` (six arm joints, three fingers)
- License: BSD-3-Clause; the complete notice is in `KINOVA_LICENSE`.

## Reproducible extraction record

The immediate source snapshot was
`ElLuisAlberto/IMPLEMENTACION_ROS2_CORE` at commit
`7cb7522fb5bd7c214b4605e9c3cd99d2cda011dd`, path
`src/thesis_description`. The then-observed head of upstream branch
`noetic-devel` was `924781d3dfe241b2b94b3c72a804b80d3658cf02`.

Only the meshes required by J2N6S300 and its Xacro dependency chain were
imported. Other Kinova variants, launch files and RViz configurations were
excluded.

## Local transformations

- The ROS package was renamed from `thesis_description` to
  `jaco2_description`; all internal Xacro and `package://` references were
  updated accordingly.
- The legacy Gazebo Classic include was removed. A future ROS 2 Fortress
  integration must provide its own `gz_ros2_control` configuration.
- No kinematic dimensions, inertias, joint names, limits or mesh geometry were
  altered.

This package is a description asset only. It does not authorize, provide or
enable physical-robot control.
