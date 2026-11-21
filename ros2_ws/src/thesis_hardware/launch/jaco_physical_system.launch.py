"""Start the complete preventive stack for the physical Kinova JACO2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the physical stack with output disabled by default."""
    bridge_share = get_package_share_directory(
        'thesis_hardware_bridge'
    )
    description_share = get_package_share_directory(
        'thesis_description'
    )

    default_hardware_config = os.path.join(
        bridge_share, 'config', 'jaco_hardware.yaml'
    )
    xacro_file = os.path.join(
        description_share, 'urdf', 'j2n6s300_standalone.xacro'
    )
    rviz_file = os.path.join(
        description_share, 'rviz', 'jaco.rviz'
    )

    mock_hardware = LaunchConfiguration('mock_hardware')
    output_enabled = LaunchConfiguration('hardware_output_enabled')
    expected_serial = LaunchConfiguration('expected_serial_number')
    hardware_config = LaunchConfiguration('hardware_config_file')
    start_rviz = LaunchConfiguration('start_rviz')
    kinova_library_dir = LaunchConfiguration('kinova_library_dir')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        )
    }

    declarations = [
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Use an offline JACO state and motion backend.',
        ),
        DeclareLaunchArgument(
            'hardware_output_enabled',
            default_value='false',
            description=(
                'Permit runtime arming of physical or mock motion.'
            ),
        ),
        DeclareLaunchArgument(
            'expected_serial_number',
            default_value='',
            description='Exact serial required when more than one arm exists.',
        ),
        DeclareLaunchArgument(
            'hardware_config_file',
            default_value=default_hardware_config,
            description='JACO adapter ROS 2 parameter file.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2.',
        ),
        DeclareLaunchArgument(
            'kinova_library_dir',
            default_value=os.path.join(
                os.environ.get(
                    'KINOVA_ROOT',
                    os.path.abspath(os.path.join(
                        bridge_share, '..', '..', '..', '..', '..',
                        'third_party', 'kinova_usb',
                    )),
                ),
                'lib',
                'x86_64-linux-gnu',
            ),
            description=(
                'Directory containing the Kinova command and communication '
                'layer shared libraries.'
            ),
        ),
    ]

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {
                'use_sim_time': False,
                'publish_frequency': 40.0,
            }],
        ),
        Node(
            package='thesis_hardware_bridge',
            executable='jaco_hardware_node',
            name='jaco_hardware_adapter',
            output='screen',
            additional_env={
                'LD_LIBRARY_PATH': [
                    kinova_library_dir,
                    ':',
                    EnvironmentVariable(
                        'LD_LIBRARY_PATH', default_value=''
                    ),
                ],
            },
            parameters=[
                hardware_config,
                {
                    'mock_hardware': ParameterValue(
                        mock_hardware, value_type=bool
                    ),
                    'hardware_output_enabled': ParameterValue(
                        output_enabled, value_type=bool
                    ),
                    'expected_serial_number': expected_serial,
                },
            ],
        ),
        Node(
            package='thesis_simulation',
            executable='cartesian_jog_adapter',
            name='cartesian_jog_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'require_hardware_arm_epoch': True,
                'output_topic': '/thesis/supervised_jog_command',
            }],
        ),
        Node(
            package='thesis_hardware',
            executable='hardware_readiness',
            name='hardware_readiness',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'require_armed': False,
                'timeout_sec': 0.50,
                'rate_hz': 10.0,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(start_rviz),
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_file],
        ),
    ]
    return LaunchDescription(declarations + nodes)
