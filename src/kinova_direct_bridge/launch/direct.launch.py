import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sdk_root = os.environ.get('KINOVA_ROOT', os.path.expanduser(
        '~/Escritorio/TESIS_2_DEPENDENCIES/kinova-ros/kinova_driver'))
    library_dir = os.path.join(sdk_root, 'lib', 'x86_64-linux-gnu')
    return LaunchDescription([
        DeclareLaunchArgument('mock_hardware', default_value='true'),
        Node(package='kinova_direct_bridge', executable='kinova_direct_node',
             output='screen', additional_env={'LD_LIBRARY_PATH': [
                 library_dir, ':', EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]}, parameters=[{'mock_hardware': ParameterValue(
                 LaunchConfiguration('mock_hardware'), value_type=bool)}]),
        Node(package='kinova_direct_bridge', executable='kinova_cartesian_adapter',
             output='screen'),
    ])
