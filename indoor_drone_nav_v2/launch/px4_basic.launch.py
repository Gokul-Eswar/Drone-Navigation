from launch import LaunchDescription
from launch_ros.actions import Node

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

    # Safety System
    safety_monitor_node = Node(
        package='indoor_drone_nav_v2',
        executable='advanced_safety_monitor_node',
        name='advanced_safety_monitor_node',
        output='screen'
    )

    return LaunchDescription([
        state_aggregator_node,
        action_server_node,
        mission_executor_node,
        gui_server_node,
        safety_monitor_node,
        # In a complete system, other nodes for SLAM, safety, etc., would be added here.
    ])
