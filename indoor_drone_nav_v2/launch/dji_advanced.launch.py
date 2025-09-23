from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file for an advanced DJI Matrice configuration.
    """

    pkg_share = FindPackageShare('indoor_drone_nav_v2')

    master_launch_file = PathJoinSubstitution([
        pkg_share, 'launch', 'master_launch.py'
    ])

    # Arguments for a professional DJI setup
    launch_arguments = {
        'drone_config': 'dji_matrice.yaml',
        'sensor_config': 'lidar_camera_professional.yaml',
        'safety_config': 'commercial_safety.yaml',
        'navigation_config': 'professional_navigation.yaml', # Assuming this exists
        'simulation': 'false',
        'gui': 'true'
    }

    master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(master_launch_file),
        launch_arguments=launch_arguments.items()
    )

    return LaunchDescription([
        master_launch
    ])
