import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = get_package_share_directory("industrial_robot_jazzy")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("industrial_robot_jazzy"), "rviz", "view.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([rsp, jsp, rviz])
