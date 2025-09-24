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
from geometry_msgs.msg import Twist

PKG_NAME = 'indoor_drone_nav_v2'

@pytest.mark.launch_test
def generate_test_description():
    """Generates the launch description for the twist_mux integration test."""
    pkg_share = get_package_share_directory(PKG_NAME)
    control_launch_file = os.path.join(pkg_share, 'launch', 'control.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch_file)),
        ReadyToTest(),
    ])

class TwistMuxIntegrationTest(unittest.TestCase):

    def setUp(self):
        """Set up the test node and subscribers/publishers."""
        rclpy.init()
        self.node = rclpy.create_node('twist_mux_integration_tester')

        # Publishers for the input topics
        self.nav_pub = self.node.create_publisher(Twist, '/cmd_vel/nav', 10)
        self.servo_pub = self.node.create_publisher(Twist, '/cmd_vel/servo', 10)

        # Subscriber to the output topic
        self.out_sub = self.node.create_subscription(
            Twist, '/cmd_vel_out', self.cmd_vel_out_callback, 10)
        self.received_twists = []

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

    def tearDown(self):
        """Clean up resources."""
        self.node.destroy_node()
        rclpy.shutdown()
        self.executor_thread.join()

    def cmd_vel_out_callback(self, msg):
        """Callback for the /cmd_vel_out topic."""
        self.received_twists.append(msg)

    def test_priority(self):
        """
        Test that the higher-priority topic is chosen.
        1. Publish to the low-priority topic (/cmd_vel/nav).
        2. Check that the output matches.
        3. Publish to the high-priority topic (/cmd_vel/servo).
        4. Check that the output now matches the high-priority command.
        """
        time.sleep(1) # Give the system time to start up

        # Publish to low-priority topic
        nav_msg = Twist()
        nav_msg.linear.x = 1.0
        self.nav_pub.publish(nav_msg)
        time.sleep(0.5)

        self.assertGreater(len(self.received_twists), 0)
        self.assertAlmostEqual(self.received_twists[-1].linear.x, 1.0)

        # Publish to high-priority topic
        servo_msg = Twist()
        servo_msg.linear.x = 2.0
        self.servo_pub.publish(servo_msg)
        time.sleep(0.5)

        self.assertGreater(len(self.received_twists), 1)
        self.assertAlmostEqual(self.received_twists[-1].linear.x, 2.0)
