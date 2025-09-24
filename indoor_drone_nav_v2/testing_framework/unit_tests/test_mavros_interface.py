import pytest
import asyncio
from unittest.mock import MagicMock, AsyncMock, patch

# This is a workaround to allow importing from the src directory
# In a real colcon test, the paths would be set up correctly.
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../src')))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from indoor_drone_nav_v2.drone_interfaces.mavros_interface import MAVROSInterface
from indoor_drone_nav_v2.action import Arm, Takeoff, Land, GotoPosition
from indoor_drone_nav_v2.msg import DroneState
from geometry_msgs.msg import PoseStamped

@pytest.fixture
def mock_node():
    """Fixture to create a mock rclpy Node."""
    if not rclpy.ok():
        rclpy.init()
    node = Node('mock_testing_node')
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

@pytest.fixture
def mavros_client(mock_node):
    """
    Fixture to create an instance of our MAVROSInterface client library.
    It patches the ActionClient to avoid real ROS2 communication.
    """
    with patch('rclpy.action.ActionClient') as MockActionClient:
        # Set up the mock to return a specific AsyncMock instance for each client
        mock_arm_client = AsyncMock(spec=ActionClient)
        mock_takeoff_client = AsyncMock(spec=ActionClient)
        mock_land_client = AsyncMock(spec=ActionClient)
        mock_goto_client = AsyncMock(spec=ActionClient)

        # When ActionClient is called, return the correct mock based on the action name
        def side_effect(node, action_type, action_name):
            if action_name == 'drone/arm': return mock_arm_client
            if action_name == 'drone/takeoff': return mock_takeoff_client
            if action_name == 'drone/land': return mock_land_client
            if action_name == 'drone/goto_position': return mock_goto_client
            return AsyncMock()

        MockActionClient.side_effect = side_effect

        client = MAVROSInterface(node=mock_node)
        yield client

async def configure_mock_action_client(mock_client, success=True):
    """Helper function to configure a mock action client for a successful run."""
    mock_client.wait_for_server.return_value = True

    mock_goal_handle = AsyncMock()
    mock_goal_handle.accepted = True

    # The result of get_result_async is a "result" object that has its own "result" attribute
    action_type = mock_client.action_type
    result_message = action_type.Result()
    result_message.success = success

    result_future = asyncio.Future()
    result_future.set_result(MagicMock(result=result_message))
    mock_goal_handle.get_result_async.return_value = result_future

    send_goal_future = asyncio.Future()
    send_goal_future.set_result(mock_goal_handle)
    mock_client.send_goal_async.return_value = send_goal_future


@pytest.mark.asyncio
async def test_arm_calls_action_client(mavros_client: MAVROSInterface):
    """Test that the arm() method correctly calls the arm action client."""
    mock_client = mavros_client.arm_client
    await configure_mock_action_client(mock_client, success=True)

    # Call the method we are testing
    result = await mavros_client.arm()

    assert result is True
    mock_client.send_goal_async.assert_called_once()
    sent_goal = mock_client.send_goal_async.call_args[0][0]
    assert isinstance(sent_goal, Arm.Goal)
    assert sent_goal.arm is True

@pytest.mark.asyncio
async def test_takeoff_calls_action_client(mavros_client: MAVROSInterface):
    """Test that the takeoff() method correctly calls the takeoff action client."""
    mock_client = mavros_client.takeoff_client
    await configure_mock_action_client(mock_client, success=True)

    result = await mavros_client.takeoff(altitude=5.0)

    assert result is True
    mock_client.send_goal_async.assert_called_once()
    sent_goal = mock_client.send_goal_async.call_args[0][0]
    assert isinstance(sent_goal, Takeoff.Goal)
    assert sent_goal.altitude == 5.0

@pytest.mark.asyncio
async def test_goto_position_calls_action_client(mavros_client: MAVROSInterface):
    """Test that goto_position() correctly calls the goto_position action client."""
    mock_client = mavros_client.goto_position_client
    await configure_mock_action_client(mock_client, success=True)

    result = await mavros_client.goto_position(x=1.0, y=2.0, z=3.0)

    assert result is True
    mock_client.send_goal_async.assert_called_once()
    sent_goal = mock_client.send_goal_async.call_args[0][0]
    assert isinstance(sent_goal, GotoPosition.Goal)
    assert sent_goal.target_pose.pose.position.x == 1.0
    assert sent_goal.target_pose.pose.position.y == 2.0
    assert sent_goal.target_pose.pose.position.z == 3.0

@pytest.mark.asyncio
async def test_state_callback_updates_state(mavros_client: MAVROSInterface):
    """Test that the state callback correctly updates the internal state."""

    # Create a mock DroneState message
    msg = DroneState()
    msg.armed = True
    msg.flight_mode = "OFFBOARD"
    msg.pose.position.x = 10.0

    # Call the callback directly
    mavros_client._state_callback(msg)

    # Check that the client's state object was updated
    assert mavros_client.state.armed is True
    assert mavros_client.state.flight_mode == "OFFBOARD"
    assert mavros_client.state.position[0] == 10.0
