import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for starting the control system, including the twist_mux.
    """
    pkg_share = get_package_share_directory('indoor_drone_nav_v2')

    twist_mux_params = os.path.join(pkg_share, 'config', 'control', 'twist_mux.yaml')

    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out', '/cmd_vel_out')] # Explicitly remap the output
        )
    ])
