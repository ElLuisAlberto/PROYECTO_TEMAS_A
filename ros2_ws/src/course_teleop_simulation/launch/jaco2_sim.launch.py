"""Launch the Kinova JACO2 J2N6S300 visual simulation in Gazebo Sim."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start Jaco2 with GUI by default; use headless:=true for CI or servers."""
    headless = LaunchConfiguration("headless")
    description_share = FindPackageShare("course_teleop_description")
    simulation_share = FindPackageShare("course_teleop_simulation")
    xacro_file = PathJoinSubstitution([simulation_share, "urdf", "jaco2_gazebo.urdf.xacro"])
    world_file = PathJoinSubstitution([simulation_share, "worlds", "jaco2_lab.sdf"])
    controllers_file = PathJoinSubstitution([simulation_share, "config", "jaco2_controllers.yaml"])
    # The URDF's package:// resources are converted by Fortress to model://
    # resources.  Gazebo must search the parent of the installed package share.
    description_parent = PathJoinSubstitution([description_share, ".."])
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r ", IfElseSubstitution(headless, "-s ", ""), "-v 2 ", world_file],
            "gz_version": "6",
            "on_exit_shutdown": "true",
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_jaco2",
        output="screen",
        arguments=["-world", "teleoperation_lab", "-name", "jaco2_j2n6s300", "-topic", "robot_description", "-allow_renaming", "false"],
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "arm_controller", "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60", "--switch-timeout", "30", "--activate-as-group",
            "--param-file", controllers_file,
        ],
        output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="Run without the Gazebo GUI."),
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[description_parent, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
        ),
        # Also support Gazebo distributions that use the renamed variable.
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[description_parent, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),
        gazebo, robot_state_publisher, clock_bridge, spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[controller_spawner])),
    ])
