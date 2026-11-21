"""Supervised JACO joint stack; no synthetic obstacles or physical output by default."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge = get_package_share_directory('thesis_hardware_bridge')
    core = get_package_share_directory('thesis_core')
    description = get_package_share_directory('thesis_description')
    mock = LaunchConfiguration('mock_hardware')
    output = LaunchConfiguration('hardware_output_enabled')
    gui = LaunchConfiguration('start_gui')
    require_proximity = LaunchConfiguration('require_proximity_status')
    kinova_library_dir = LaunchConfiguration('kinova_library_dir')
    xacro = os.path.join(description, 'urdf', 'j2n6s300_standalone.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('mock_hardware', default_value='true'),
        DeclareLaunchArgument('hardware_output_enabled', default_value='false'),
        DeclareLaunchArgument('start_gui', default_value='false'),
        DeclareLaunchArgument('require_proximity_status', default_value='false',
                              description='Require a real obstacle source and proximity monitor.'),
        DeclareLaunchArgument('kinova_library_dir', default_value=os.path.join(
            os.environ.get('KINOVA_ROOT', os.path.abspath(os.path.join(
                bridge, '..', '..', '..', '..', '..', 'third_party', 'kinova_usb'))),
            'lib', 'x86_64-linux-gnu')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[{
                 'use_sim_time': False, 'publish_frequency': 40.0,
                 'robot_description': ParameterValue(Command(['xacro ', xacro]), value_type=str)}]),
        Node(package='thesis_hardware_bridge', executable='jaco_hardware_node',
             name='jaco_hardware_adapter', output='screen',
             additional_env={'LD_LIBRARY_PATH': [kinova_library_dir, ':',
                 EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]},
             parameters=[os.path.join(bridge, 'config', 'jaco_hardware.yaml'), {
                 'mock_hardware': ParameterValue(mock, value_type=bool),
                 'hardware_output_enabled': ParameterValue(output, value_type=bool),
                 'require_proximity_status': ParameterValue(require_proximity, value_type=bool)}]),
        Node(package='thesis_core', executable='obstacle_input_bridge',
             name='obstacle_input_bridge', output='screen', condition=IfCondition(require_proximity), parameters=[{
                 'use_sim_time': False, 'input_topic': '/thesis/obstacle_input',
                 'output_topic': '/thesis/obstacle_in_model_frame', 'target_frame': 'world'}]),
        Node(package='thesis_core', executable='proximity_monitor',
             name='proximity_monitor', output='screen', condition=IfCondition(require_proximity), parameters=[
                 os.path.join(core, 'config', 'jaco_capsules.yaml'), {
                     'use_sim_time': False, 'obstacle_source_mode': 'topic',
                     'obstacle_input_topic': '/thesis/obstacle_in_model_frame'}]),
        Node(package='thesis_core', executable='safety_supervisor',
             name='safety_supervisor_node', output='screen', parameters=[{
                 'use_sim_time': False,
                 'require_proximity_status': ParameterValue(require_proximity, value_type=bool)}]),
        Node(package='thesis_hardware', executable='hardware_readiness',
             name='hardware_readiness', output='screen', parameters=[{
                 'use_sim_time': False, 'require_armed': False,
                 'require_proximity_status': ParameterValue(require_proximity, value_type=bool),
                 'rate_hz': 10.0, 'timeout_sec': 0.5}]),
        Node(package='thesis_ui', executable='joint_gui', name='joint_control_gui_node',
             output='screen', condition=IfCondition(gui), parameters=[{
                 'use_sim_time': False, 'require_system_readiness': True,
                 'operation_mode': 'hardware', 'allow_hardware_arm_control': True,
                 'require_proximity_status': ParameterValue(require_proximity, value_type=bool)}]),
    ])
