import unittest
import rclpy
from indoor_drone_nav_v2.msg import BoundingBox
from indoor_drone_nav_v2.ml_modules.visual_servoing_node import VisualServoingNode

class TestVisualServoingNode(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = VisualServoingNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        """Test that the node can be created."""
        self.assertIsNotNone(self.node)

    def test_bbox_callback(self):
        """Test that the bounding box callback can be called."""
        # Create a mock bounding box message
        mock_bbox = BoundingBox()
        mock_bbox.xmin = 100
        mock_bbox.xmax = 200
        mock_bbox.ymin = 100
        mock_bbox.ymax = 200

        # This will fail if the callback crashes
        self.node.bbox_callback(mock_bbox)

if __name__ == '__main__':
    unittest.main()
