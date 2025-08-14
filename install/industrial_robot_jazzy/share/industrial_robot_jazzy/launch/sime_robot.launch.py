import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("industrial_robot_jazzy"), "urdf", "robot.urdf.xacro"])
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Joint State Publisher with GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[robot_description],
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("industrial_robot_jazzy"),
        "config",
        "robot_view.rviz"
    ])
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=LaunchConfigurationIfCondition("use_rviz")
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        # rviz,  # odkomentuj jeśli chcesz automatycznie uruchamiać RViz
    ])