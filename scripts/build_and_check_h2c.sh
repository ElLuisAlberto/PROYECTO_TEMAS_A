#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${repository_root}/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${workspace}"
colcon build \
  --symlink-install \
  --packages-select course_teleop_description course_teleop_simulation
set +u
source install/setup.bash
set -u

temporary_urdf="$(mktemp)"
trap 'rm -f "${temporary_urdf}"' EXIT

xacro \
  src/course_teleop_description/urdf/course_six_dof_arm.urdf.xacro \
  > "${temporary_urdf}"
check_urdf "${temporary_urdf}"
ign sdf -k src/course_teleop_simulation/worlds/teleoperation_lab.sdf

python3 "${repository_root}/scripts/validate_description.py" \
  src/course_teleop_description/urdf/course_six_dof_arm.urdf.xacro \
  src/course_teleop_simulation/config/controllers.yaml
