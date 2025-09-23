from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os
from typing import List

def launch_drone_interface(drone_params: dict) -> List[Node]:
    print(f"INFO: Preparing to launch drone interface for type: {drone_params.get('drone', {}).get('type')}")
    # In a real scenario, this would return a list of Nodes to launch
    return []

def launch_sensor_processing(sensor_params: dict) -> List[Node]:
    print("INFO: Preparing to launch sensor processing nodes...")
    return []

def launch_slam_system(sensor_params: dict) -> List[Node]:
    print("INFO: Preparing to launch SLAM system...")
    return []

def launch_navigation_stack() -> List[Node]:
    print("INFO: Preparing to launch navigation stack (Nav2)...")
    return []

def launch_safety_systems() -> List[Node]:
    print("INFO: Preparing to launch safety systems...")
    return []

def launch_mission_planner() -> List[Node]:
    print("INFO: Preparing to launch mission planner...")
    return []

def launch_data_management() -> List[Node]:
    print("INFO: Preparing to launch data management...")
    return []

def launch_gui_interface() -> List[Node]:
    print("INFO: Preparing to launch GUI interface...")
    # This is a placeholder for launching the FastAPI server.
    # A real implementation might use a lifecycle node or a simple script executor.
    return []


def launch_setup(context, *args, **kwargs):
    """
    Dynamic launch setup that reads configuration files and prepares nodes.
    This function is executed at launch time.
    """

    # Resolve launch arguments to their actual values
    config_dir_arg = LaunchConfiguration('config_dir').perform(context)
    drone_config_arg = LaunchConfiguration('drone_config').perform(context)
    sensor_config_arg = LaunchConfiguration('sensor_config').perform(context)

    # Find the package share directory
    pkg_share_path = FindPackageShare('indoor_drone_nav_v2').find('indoor_drone_nav_v2')

    # Construct full paths to the configuration files
    # Assumes a structure like: <install_share>/<pkg_name>/config/drones/active.yaml
    drone_config_path = os.path.join(pkg_share_path, config_dir_arg, 'drones', drone_config_arg)
    sensor_config_path = os.path.join(pkg_share_path, config_dir_arg, 'sensors', sensor_config_arg)

    print(f"--- ROS2 Launch ---")
    print(f"Loading drone config from: {drone_config_path}")
    print(f"Loading sensor config from: {sensor_config_path}")

    # Load YAML files
    try:
        with open(drone_config_path, 'r') as f:
            drone_params = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"❌ ERROR: Drone config file not found: {drone_config_path}")
        return []

    try:
        with open(sensor_config_path, 'r') as f:
            sensor_params = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"❌ ERROR: Sensor config file not found: {sensor_config_path}")
        return []

    # This list will hold all the nodes and actions to be launched
    launch_actions = []

    # Call placeholder functions to generate nodes for each module
    launch_actions.extend(launch_drone_interface(drone_params))
    launch_actions.extend(launch_sensor_processing(sensor_params))
    launch_actions.extend(launch_slam_system(sensor_params))
    launch_actions.extend(launch_navigation_stack())
    launch_actions.extend(launch_safety_systems())
    launch_actions.extend(launch_mission_planner())
    launch_actions.extend(launch_data_management())

    if LaunchConfiguration('gui').perform(context) == 'true':
        launch_actions.extend(launch_gui_interface())

    print("--- Launch setup complete. Returning actions. ---")
    return launch_actions

def generate_launch_description():
    """Master launch file that reads config and launches the appropriate nodes."""

    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value='config',
                            description='Path to config directory relative to package share.'),
        DeclareLaunchArgument('drone_config', default_value='active.yaml',
                            description='Drone configuration file in <config_dir>/drones.'),
        DeclareLaunchArgument('sensor_config', default_value='active.yaml',
                            description='Sensor configuration file in <config_dir>/sensors.'),
        DeclareLaunchArgument('safety_config', default_value='active.yaml',
                            description='Safety configuration file in <config_dir>/safety.'),
        DeclareLaunchArgument('simulation', default_value='false',
                            description='Run in simulation mode.'),
        DeclareLaunchArgument('gui', default_value='true',
                            description='Launch GUI interface.'),
        DeclareLaunchArgument('recording', default_value='false',
                            description='Enable data recording.'),
        DeclareLaunchArgument('log_level', default_value='info',
                            description='Logging level for nodes.'),

        # OpaqueFunction allows us to resolve launch arguments and then
        # use their values to conditionally or dynamically create more actions.
        OpaqueFunction(function=launch_setup)
    ])
