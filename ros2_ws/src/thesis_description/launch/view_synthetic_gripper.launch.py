from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('thesis_description')
    model = PathJoinSubstitution([share, 'urdf', 'jaco_synthetic_gripper_view.xacro'])
    config = PathJoinSubstitution([share, 'rviz', 'jaco.rviz'])
    description = {'robot_description': ParameterValue(
        Command(['xacro ', model]), value_type=str)}
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[description], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config], output='screen'),
    ])
