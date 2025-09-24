from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'indoor_drone_nav_v2'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config/presets'), glob('config/presets/*.yaml')),
        (os.path.join('share', package_name, 'config/drones'), glob('config/drones/*')),
        (os.path.join('share', package_name, 'config/sensors'), glob('config/sensors/*')),
        (os.path.join('share', package_name, 'config/safety'), glob('config/safety/*')),
        (os.path.join('share', package_name, 'config/environments'), glob('config/environments/*')),
        (os.path.join('share', package_name, 'config/missions'), glob('config/missions/*')),
        (os.path.join('share', package_name, 'config/platform_optimizations'), glob('config/platform_optimizations/*')),
        # Install GUI files
        (os.path.join('share', package_name, 'gui_interface/web_gui/templates'), glob('gui_interface/web_gui/templates/*.html')),
        (os.path.join('share', package_name, 'gui_interface/web_gui/static/css'), glob('gui_interface/web_gui/static/css/*.css')),
        (os.path.join('share', package_name, 'gui_interface/web_gui/static/js'), glob('gui_interface/web_gui/static/js/*.js')),
        # Install scripts
        (os.path.join('lib', package_name), glob('scripts/*')),
        # Install test files
        (os.path.join('share', package_name, 'testing_framework/integration_tests'), glob('testing_framework/integration_tests/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jules',
    maintainer_email='jules@example.com',
    description='Indoor Drone Navigation System V2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_aggregator_node = indoor_drone_nav_v2.drone_interfaces.state_aggregator_node:main',
            'drone_action_server_node = indoor_drone_nav_v2.drone_interfaces.drone_action_server_node:main',
            'gui_server_node = indoor_drone_nav_v2.gui_interface.mission_control_server:main',
            'mission_executor_node = indoor_drone_nav_v2.mission_planning.mission_executor_node:main',
            'path_planner_node = indoor_drone_nav_v2.mission_planning.path_planner_node:main',
            'object_detection_node = indoor_drone_nav_v2.ml_modules.object_detection_node:main',
            'visual_servoing_node = indoor_drone_nav_v2.ml_modules.visual_servoing_node:main',
            'velocity_bridge_node = indoor_drone_nav_v2.drone_interfaces.velocity_bridge_node:main',
        ],
    },
)
