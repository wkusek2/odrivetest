import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = get_package_share_directory("industrial_robot_jazzy")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    odrive = Node(
        package="odrive_can",
        executable="odrive_can_node",
        name="odrive_can_node",
        parameters=[
            {"can_interface": "can0"},
            {"axis_id": 0},
            {"use_sim_time": False},
        ],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
        output="both",
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": False}],
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("industrial_robot_jazzy"), "rviz", "view.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    delayed_nodes = TimerAction(
        period=2.0,
        actions=[rsp, jsp, rviz],
    )

    return LaunchDescription([odrive, delayed_nodes])
