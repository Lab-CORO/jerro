import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory("jerro_description")
    
    # Define the path to the URDF and RViz files
    urdf_file = os.path.join(package_share_directory, "urdf", "jerro.urdf")
    print(urdf_file)
    # Read the URDF content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description}],
    )
    
    # Joint state publisher with robot_description parameter
    joint_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {"source_list": ['/set_joint_states']},
        ],
    )
    
    return LaunchDescription([
        joint_pub,
        robot_state_publisher_node,
    ])