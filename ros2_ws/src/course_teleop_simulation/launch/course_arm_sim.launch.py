"""Start the independent six-DOF substitute in Gazebo Fortress."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the complete headless H2c simulation."""
    description_share = FindPackageShare("course_teleop_description")
    simulation_share = FindPackageShare("course_teleop_simulation")
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "course_six_dof_arm.urdf.xacro"]
    )
    world_file = PathJoinSubstitution(
        [simulation_share, "worlds", "teleoperation_lab.sdf"]
    )
    controllers_file = PathJoinSubstitution(
        [simulation_share, "config", "controllers.yaml"]
    )
    robot_description = ParameterValue(
        Command(
            [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["-r -s -v 2 ", world_file],
            "gz_version": "6",
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_course_arm",
        output="screen",
        arguments=[
            "-world",
            "teleoperation_lab",
            "-name",
            "course_six_dof_arm",
            "-topic",
            "robot_description",
            "-allow_renaming",
            "false",
        ],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "30",
            "--activate-as-group",
            "--param-file",
            controllers_file,
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            clock_bridge,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[controller_spawner],
                )
            ),
        ]
    )
