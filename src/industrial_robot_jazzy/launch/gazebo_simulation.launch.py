import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create a launch description for simulating the robot with Gazebo (gz)."""

    pkg_share = FindPackageShare("industrial_robot_jazzy").find("industrial_robot_jazzy")

    # Paths
    controllers_file = os.path.join(pkg_share, "config", "controllers.yaml")

    # Generate robot description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("industrial_robot_jazzy"), "urdf", "robot.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Gazebo (gz) simulator
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4"],
        output="screen",
    )

    # Robot description publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control controller manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    # Spawn robot into gz
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "industrial_robot", "-topic", "robot_description"],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Position Controller
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    # Delayed start for controllers
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            gazebo,
            ros2_control_node,
            robot_state_publisher,
            spawn_entity,
            delay_joint_state_broadcaster,
            delay_position_controller,
        ]
    )

