from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file for running the system in simulation.
    """

    pkg_share = FindPackageShare('indoor_drone_nav_v2')

    master_launch_file = PathJoinSubstitution([
        pkg_share, 'launch', 'master_launch.py'
    ])

    # Arguments for simulation
    launch_arguments = {
        'drone_config': 'simulation.yaml',
        'sensor_config': 'simulation_sensors.yaml',
        'safety_config': 'standard_safety.yaml',
        'navigation_config': 'basic_navigation.yaml',
        'simulation': 'true',
        'gui': 'true'
    }

    master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(master_launch_file),
        launch_arguments=launch_arguments.items()
    )

    return LaunchDescription([
        master_launch
    ])
