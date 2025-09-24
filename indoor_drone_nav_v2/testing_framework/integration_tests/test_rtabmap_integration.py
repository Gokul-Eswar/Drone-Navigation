import os
import unittest
import pytest
import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory

import time
import threading
import numpy as np
from sensor_msgs.msg import LaserScan, Imu, Image, CameraInfo
from indoor_drone_nav_v2.msg import DroneState

PKG_NAME = 'indoor_drone_nav_v2'

@pytest.mark.launch_test
def generate_test_description():
    """Generates the launch description for the RTAB-Map integration test."""
    pkg_share = get_package_share_directory(PKG_NAME)
    main_launch_file = os.path.join(pkg_share, 'launch', 'px4_basic.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(main_launch_file)),
        ReadyToTest(),
    ])

class RtabmapIntegrationTest(unittest.TestCase):

    def setUp(self):
        """Set up the test node and subscribers/publishers."""
        rclpy.init()
        self.node = rclpy.create_node('rtabmap_integration_tester')

        # Publishers for mock sensor data
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.node.create_publisher(Imu, '/imu', 10)
        self.depth_pub = self.node.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.node.create_publisher(CameraInfo, '/camera/color/camera_info', 10)

        # Subscriber to check the result
        self.state_sub = self.node.create_subscription(
            DroneState, '/indoor_drone/state', self.drone_state_callback, 10)
        self.received_states = []

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

    def tearDown(self):
        """Clean up resources."""
        self.node.destroy_node()
        rclpy.shutdown()
        self.executor_thread.join()

    def drone_state_callback(self, msg):
        """Callback for the /indoor_drone/state topic."""
        self.received_states.append(msg)

    def test_rtabmap_produces_pose(self):
        """
        The core test logic.
        1. Publish mock sensor data for a few seconds.
        2. Wait for the system to process it.
        3. Check if we have received DroneState messages with a valid pose.
        """
        time.sleep(15) # Give the system time to start up

        # Create mock messages
        scan_msg = LaserScan() # Populate later
        imu_msg = Imu() # Populate later
        depth_msg = Image() # Populate later
        info_msg = CameraInfo() # Populate later

        # --- Populate messages with plausible data ---
        H, W = 480, 640

        # Camera Info
        info_msg.header.frame_id = 'camera_link'
        info_msg.height = H
        info_msg.width = W
        info_msg.k = [554.38, 0.0, 320.5, 0.0, 554.38, 240.5, 0.0, 0.0, 1.0] # Typical Kinect intrinsics
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Depth Image
        depth_msg.header.frame_id = 'camera_link'
        depth_msg.height = H
        depth_msg.width = W
        depth_msg.encoding = '16UC1'
        depth_image = np.full((H, W), 2000, dtype=np.uint16) # Flat plane 2m away
        depth_msg.data = depth_image.tobytes()
        depth_msg.step = W * 2 # 2 bytes per pixel

        # Laser Scan
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = -1.57
        scan_msg.angle_max = 1.57
        scan_msg.angle_increment = 0.01745 # 1 degree
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [5.0] * 180

        # IMU
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation.w = 1.0

        end_time = time.time() + 10 # Publish for 10 seconds
        while time.time() < end_time:
            now = self.node.get_clock().now().to_msg()
            scan_msg.header.stamp = now
            imu_msg.header.stamp = now
            depth_msg.header.stamp = now
            info_msg.header.stamp = now

            self.scan_pub.publish(scan_msg)
            self.imu_pub.publish(imu_msg)
            self.depth_pub.publish(depth_msg)
            self.info_pub.publish(info_msg)

            time.sleep(0.1)

        self.assertGreater(len(self.received_states), 0, "Did not receive any DroneState messages.")
        last_state = self.received_states[-1]
        self.assertNotEqual(last_state.pose.position.x, 0.0)
        self.assertEqual(last_state.header.frame_id, 'map')
