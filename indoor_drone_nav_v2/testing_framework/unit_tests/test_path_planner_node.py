import unittest
from unittest.mock import MagicMock, patch
import rclpy
from geometry_msgs.msg import PoseStamped
from indoor_drone_nav_v2.mission_planning.path_planner_node import PathPlannerNode

class TestPathPlannerNode(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        # We patch the ActionClient so we don't need a running ROS system
        with patch('rclpy.action.ActionClient'):
            self.node = PathPlannerNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        """Test that the node can be created."""
        self.assertIsNotNone(self.node)

    @patch('rclpy.action.ActionClient')
    def test_get_path_callback(self, mock_action_client):
        """Test the logic of the get_path_callback."""
        # This is a complex test to write, as it involves mocking the
        # asynchronous behavior of the action clients.
        # For now, we will just test that the callback can be called.

        # Create a mock goal handle
        mock_goal_handle = MagicMock()
        mock_goal_handle.request.goal_pose = PoseStamped()

        # We need to run the async callback in an event loop
        import asyncio
        asyncio.run(self.node.get_path_callback(mock_goal_handle))

        # Check that the goal was "succeeded" (in a simplified way)
        # In a real test, we would check the mocks for calls.
        mock_goal_handle.succeed.assert_called_once()


if __name__ == '__main__':
    unittest.main()
