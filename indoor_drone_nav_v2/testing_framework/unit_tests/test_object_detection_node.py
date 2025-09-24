import unittest
from unittest.mock import patch
import rclpy
from sensor_msgs.msg import Image
from indoor_drone_nav_v2.ml_modules.object_detection_node import ObjectDetectionNode

class TestObjectDetectionNode(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        with patch('cv_bridge.CvBridge'):
            self.node = ObjectDetectionNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        """Test that the node can be created."""
        self.assertIsNotNone(self.node)

    def test_image_callback(self):
        """Test that the image callback can be called."""
        # Create a mock image message
        mock_image = Image()

        # This will fail if the callback crashes
        self.node.image_callback(mock_image)

if __name__ == '__main__':
    unittest.main()
