from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for starting the RTAB-Map SLAM node for 3D mapping.
    """
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                # Basic RTAB-Map parameters
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_scan': True,
                'approx_sync': True,
                'queue_size': 10,

                # SLAM mode
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithImage': 'true',

                # Feature detection
                'SURF/HessianThreshold': '100',

                # Loop closure detection
                'RGBD/LoopClosureReextractFeatures': 'true',
                'RGBD/AngularUpdate': '0.01',
                'RGBD/LinearUpdate': '0.01',
                'LccIcp/Type': '2', # 2=ICP
                'LccReextract/LoopClosureReextractFeatures': 'true'
            }],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu'),
                ('odom', '/odometry/filtered'), # Use the EKF output as odometry
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
            ]
        )
    ])
