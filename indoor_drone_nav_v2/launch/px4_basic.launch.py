from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file for a basic PX4 configuration.
    This launch file includes the master_launch.py file with specific arguments
    for the PX4 setup.
    """

    # Find the package share directory
    pkg_share = FindPackageShare('indoor_drone_nav_v2')

    # Path to the master launch file
    master_launch_file = PathJoinSubstitution([
        pkg_share, 'launch', 'master_launch.py'
    ])

    # Arguments to pass to the master launch file
    launch_arguments = {
        'drone_config': 'px4_quadcopter_basic.yaml',
        'sensor_config': 'camera_imu_basic.yaml',
        'safety_config': 'standard_safety.yaml',
        'navigation_config': 'basic_navigation.yaml', # Assuming there's a nav config
        'simulation': 'false',
        'gui': 'true'
    }

    # Include the master launch file with the specified arguments
    master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(master_launch_file),
        launch_arguments=launch_arguments.items()
    )

    return LaunchDescription([
        master_launch
    ])
