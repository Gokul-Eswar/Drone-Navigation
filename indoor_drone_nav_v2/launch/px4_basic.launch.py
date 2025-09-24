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

    velocity_bridge_node = Node(
        package='indoor_drone_nav_v2',
        executable='velocity_bridge_node',
        name='velocity_bridge_node',
        output='screen'
    )

    # ML Modules
    object_detection_node = Node(
        package='indoor_drone_nav_v2',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen'
    )

    visual_servoing_node = Node(
        package='indoor_drone_nav_v2',
        executable='visual_servoing_node',
        name='visual_servoing_node',
        output='screen'
    )

    # RTAB-Map SLAM
    rtabmap_launch_file = os.path.join(
        get_package_share_directory('indoor_drone_nav_v2'),
        'launch',
        'rtabmap.launch.py'
    )

    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file)
    )

    # Nav2 Stack
    nav2_launch_file = os.path.join(
        get_package_share_directory('indoor_drone_nav_v2'),
        'launch',
        'nav2.launch.py'
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file)
    )

    # Control System
    control_launch_file = os.path.join(
        get_package_share_directory('indoor_drone_nav_v2'),
        'launch',
        'control.launch.py'
    )

    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_file)
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
        velocity_bridge_node,
        mission_executor_node,
        gui_server_node,
        object_detection_node,
        visual_servoing_node,
        rtabmap_node,
        nav2_node,
        control_node,
        ekf_node,
    ])
