import unittest
from unittest.mock import MagicMock, patch, AsyncMock
import rclpy
import asyncio
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from indoor_drone_nav_v2.action import ExecuteMission, GetPath
from indoor_drone_nav_v2.mission_planning.path_planner_node import PathPlannerNode

class TestPathPlannerNode(unittest.TestCase):

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    @patch('rclpy.action.ActionClient')
    def test_get_path_callback_success(self, mock_action_client_constructor):
        """Test the successful execution of the get_path_callback."""

        node = PathPlannerNode()

        # Mock the Nav2 path client
        mock_nav2_client = MagicMock()
        mock_nav2_client.send_goal_async = AsyncMock()

        # Mock the mission client
        mock_mission_client = MagicMock()
        mock_mission_client.send_goal_async = AsyncMock()

        # The constructor patch will return our mocks
        mock_action_client_constructor.side_effect = [mock_nav2_client, mock_mission_client]

        # Configure the mock return values for Nav2
        mock_nav2_goal_handle = MagicMock()
        mock_nav2_goal_handle.accepted = True
        mock_nav2_goal_handle.get_result_async = AsyncMock()
        mock_nav2_client.send_goal_async.return_value = mock_nav2_goal_handle

        nav2_result = ComputePathToPose.Result()
        nav2_result.path.poses.append(PoseStamped()) # A path with one waypoint
        mock_nav2_goal_handle.get_result_async.return_value.result = nav2_result

        # Configure the mock return values for the mission client
        mock_mission_goal_handle = MagicMock()
        mock_mission_goal_handle.accepted = True
        mock_mission_goal_handle.get_result_async = AsyncMock()
        mock_mission_client.send_goal_async.return_value = mock_mission_goal_handle

        mission_result = ExecuteMission.Result()
        mission_result.success = True
        mock_mission_goal_handle.get_result_async.return_value.result = mission_result

        # Create a mock goal handle for the GetPath action
        mock_get_path_goal_handle = MagicMock()
        mock_get_path_goal_handle.request.goal_pose = PoseStamped()

        # Run the async callback
        result = asyncio.run(node.get_path_callback(mock_get_path_goal_handle))

        # Assertions
        mock_nav2_client.send_goal_async.assert_called_once()
        mock_mission_client.send_goal_async.assert_called_once()
        mock_get_path_goal_handle.succeed.assert_called_once()
        self.assertTrue(result.success)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
