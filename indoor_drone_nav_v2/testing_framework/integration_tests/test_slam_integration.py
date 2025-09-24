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

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
from indoor_drone_nav_v2.msg import DroneState
import time
import threading

# The package name
PKG_NAME = 'indoor_drone_nav_v2'

@pytest.mark.launch_test
def generate_test_description():
    """
    Generates the launch description for the integration test.
    This will launch the main system launch file.
    """
    pkg_share = get_package_share_directory(PKG_NAME)

    main_launch_file = os.path.join(
        pkg_share, 'launch', 'px4_basic.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch_file)
        ),
        ReadyToTest(),
    ])


class SLAMIntegrationTest(unittest.TestCase):

    def setUp(self):
        """Set up the test node and subscribers/publishers."""
        rclpy.init()
        self.node = rclpy.create_node('slam_integration_tester')

        # Publisher to mock sensor data
        self.scan_publisher = self.node.create_publisher(LaserScan, '/scan', 10)
        self.imu_publisher = self.node.create_publisher(Imu, '/imu', 10)

        # Subscriber to check the result
        self.drone_state_subscriber = self.node.create_subscription(
            DroneState, '/indoor_drone/state', self.drone_state_callback, 10
        )
        self.received_states = []

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Start the executor in a separate thread
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

    def test_slam_produces_pose(self):
        """
        The core test logic.
        1. Publish mock sensor data for a few seconds.
        2. Wait for the system to process it.
        3. Check if we have received DroneState messages with a valid pose.
        """
        # Give the system time to start up
        time.sleep(15) # Needs some time for all nodes to be ready

        # Publish some mock data
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = -3.14
        scan_msg.angle_max = 3.14
        scan_msg.angle_increment = 0.0174533 # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [5.0] * 360 # A simple circle of points

        imu_msg = Imu()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation.w = 1.0 # No rotation

        end_time = time.time() + 10 # Publish for 10 seconds
        while time.time() < end_time:
            now = self.node.get_clock().now().to_msg()
            scan_msg.header.stamp = now
            imu_msg.header.stamp = now
            self.scan_publisher.publish(scan_msg)
            self.imu_publisher.publish(imu_msg)
            time.sleep(0.1)

        # Check the results
        self.assertGreater(len(self.received_states), 0, "Did not receive any DroneState messages.")

        # Check if the pose is not zero (i.e., it has been updated from the default)
        last_state = self.received_states[-1]
        self.assertNotEqual(last_state.pose.position.x, 0.0)
        self.assertNotEqual(last_state.pose.orientation.w, 0.0) # Default is 0
        self.assertEqual(last_state.header.frame_id, 'map')
