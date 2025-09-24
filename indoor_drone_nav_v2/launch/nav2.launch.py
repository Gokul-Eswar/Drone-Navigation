import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Launch file for starting the Nav2 stack.
    """
    # Get the share directory for this package
    pkg_share = get_package_share_directory('indoor_drone_nav_v2')

    # Get the Nav2 bringup directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Get the path to the custom Nav2 parameters file
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2', 'nav2.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': '', # We are using a SLAM-generated map
                'use_sim_time': 'false',
                'params_file': nav2_params_file,
                'autostart': 'true',
                'use_composition': 'true'
            }.items(),
        )
    ])
