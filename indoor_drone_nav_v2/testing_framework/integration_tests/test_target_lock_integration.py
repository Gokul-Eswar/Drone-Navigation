import os
import unittest
import pytest
import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory

import time
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

PKG_NAME = 'indoor_drone_nav_v2'

@pytest.mark.launch_test
def generate_test_description():
    """Generates the launch description for the target lock integration test."""
    return LaunchDescription([
        LaunchNode(
            package=PKG_NAME,
            executable='object_detection_node',
            name='object_detection_node'
        ),
        LaunchNode(
            package=PKG_NAME,
            executable='visual_servoing_node',
            name='visual_servoing_node'
        ),
        ReadyToTest(),
    ])

class TargetLockIntegrationTest(unittest.TestCase):

    def setUp(self):
        """Set up the test node and subscribers/publishers."""
        rclpy.init()
        self.node = rclpy.create_node('target_lock_integration_tester')

        self.image_pub = self.node.create_publisher(Image, '/camera/color/image_raw', 10)
        self.cmd_vel_sub = self.node.create_subscription(
            Twist, '/cmd_vel/servo', self.cmd_vel_callback, 10)
        self.received_twists = []

        self.bridge = CvBridge()

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()

    def tearDown(self):
        """Clean up resources."""
        self.node.destroy_node()
        rclpy.shutdown()
        self.executor_thread.join()

    def cmd_vel_callback(self, msg):
        """Callback for the /cmd_vel/servo topic."""
        self.received_twists.append(msg)

    def test_target_locking(self):
        """
        Test that the system generates velocity commands when a target is present.
        """
        time.sleep(2) # Give the system time to start up

        # 1. Test with a green square in the image
        green_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(green_image, (300, 220), (340, 260), (0, 255, 0), -1)
        ros_image = self.bridge.cv2_to_imgmsg(green_image, "bgr8")

        self.image_pub.publish(ros_image)
        time.sleep(1)

        self.assertGreater(len(self.received_twists), 0, "Did not receive any cmd_vel.")
        last_twist = self.received_twists[-1]
        # We expect a non-zero yaw command because the box is off-center
        self.assertNotAlmostEqual(last_twist.angular.z, 0.0)

        # 2. Test with a blank image
        self.received_twists.clear()
        blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
        ros_image_blank = self.bridge.cv2_to_imgmsg(blank_image, "bgr8")

        self.image_pub.publish(ros_image_blank)
        time.sleep(1.5) # Wait for the stop timer in the servoing node to fire

        self.assertGreater(len(self.received_twists), 0, "Did not receive stop command.")
        last_twist_stopped = self.received_twists[-1]
        self.assertAlmostEqual(last_twist_stopped.angular.z, 0.0)
        self.assertAlmostEqual(last_twist_stopped.linear.z, 0.0)
