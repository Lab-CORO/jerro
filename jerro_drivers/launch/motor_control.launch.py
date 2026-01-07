import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    jerro_drivers_dir = get_package_share_directory('jerro_drivers')

    # Path to config files
    motor_config_file = os.path.join(jerro_drivers_dir, 'config', 'motor_pid_config.yaml')

    # Check for tuned gains file (created by auto-tune action in Phase 5)
    tuned_gains_file = os.path.join(jerro_drivers_dir, 'config', 'pid_gains.yaml')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock if true'
    )

    # Encoder publisher node
    encoder_node = Node(
        package='jerro_drivers',
        executable='encoder_publisher',
        name='encoder_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Motor PID controller node
    # Load base config, then tuned gains if they exist
    motor_pid_params = [motor_config_file, {'use_sim_time': use_sim_time}]
    if os.path.exists(tuned_gains_file):
        motor_pid_params.insert(1, tuned_gains_file)
        print(f"Loading tuned gains from: {tuned_gains_file}")
    else:
        print(f"Tuned gains not found, using defaults from: {motor_config_file}")

    motor_pid_node = Node(
        package='jerro_drivers',
        executable='motor_pid_controller',
        name='motor_pid_controller',
        output='screen',
        parameters=motor_pid_params
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_argument)

    # Add all nodes
    ld.add_action(encoder_node)
    ld.add_action(motor_pid_node)

    return ld
