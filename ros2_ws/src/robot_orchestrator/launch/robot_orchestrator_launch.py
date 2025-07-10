import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('robot_orchestrator'),
        'config',
        'assays.yaml'
    )

    # Define the nodes
    experiment_manager_node = Node(
        package='robot_orchestrator',
        executable='experiment_manager_node',
        parameters=[{'config_file': config_file}],
        output='screen'
    )

    control_node = Node(
        package='robot_orchestrator',
        executable='control_node',
        output='screen',
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    logger_node = Node(
        package='robot_orchestrator',
        executable='logger_node',
        output='screen'
    )

    # Create and return the launch description
    return LaunchDescription([
        experiment_manager_node,
        control_node,
        logger_node
    ]) 