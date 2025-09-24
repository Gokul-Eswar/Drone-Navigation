import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Top-level launch file for the Indoor Drone Navigation System.
    This launch file starts all the necessary nodes for a basic PX4 setup,
    based on the refactored microservice architecture.
    """

    # Drone Interface Microservices
    state_aggregator_node = Node(
        package='indoor_drone_nav_v2',
        executable='state_aggregator_node',
        name='state_aggregator_node',
        output='screen'
    )

    action_server_node = Node(
        package='indoor_drone_nav_v2',
        executable='drone_action_server_node',
        name='drone_action_server_node',
        output='screen'
    )

    # Mission Execution
    mission_executor_node = Node(
        package='indoor_drone_nav_v2',
        executable='mission_executor_node',
        name='mission_executor_node',
        output='screen'
    )

    # GUI Server
    gui_server_node = Node(
        package='indoor_drone_nav_v2',
        executable='gui_server_node',
        name='gui_server_node',
        output='screen'
    )

    # Cartographer SLAM
    cartographer_config_dir = os.path.join(get_package_share_directory('indoor_drone_nav_v2'), 'config', 'slam')
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua'
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu')
        ]
    )

    # Extended Kalman Filter for Sensor Fusion
    ekf_config_file = os.path.join(get_package_share_directory('indoor_drone_nav_v2'), 'config', 'ekf', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        state_aggregator_node,
        action_server_node,
        mission_executor_node,
        gui_server_node,
        cartographer_node,
        ekf_node,
        # In a complete system, other nodes for SLAM, safety, etc., would be added here.
    ])
