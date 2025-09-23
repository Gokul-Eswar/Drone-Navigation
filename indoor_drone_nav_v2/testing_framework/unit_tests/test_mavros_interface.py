import pytest
import asyncio
from unittest.mock import MagicMock, AsyncMock, patch

# This is a workaround to allow importing from the src directory
# In a real colcon test, the paths would be set up correctly.
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../src')))


# We need to initialize rclpy for the node to be created
import rclpy
rclpy.init()

from indoor_drone_nav_v2.drone_interfaces.mavros_interface import MAVROSInterface
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


@pytest.fixture
def mavros_interface_node():
    """
    Fixture to create a MAVROSInterface node instance for testing.
    It uses patching to prevent the node from actually creating ROS2 entities,
    allowing us to test the logic in isolation.
    """
    with patch('rclpy.node.Node.create_subscription'), \
         patch('rclpy.node.Node.create_publisher', return_value=MagicMock(spec=['publish'])), \
         patch('rclpy.node.Node.create_client') as mock_create_client, \
         patch('rclpy.node.Node.create_timer'):

        node = MAVROSInterface(config={})

        # Manually attach mock clients to the instance for manipulation in tests
        node.arming_client = AsyncMock(spec=node.create_client(CommandBool, '').__class__)
        node.set_mode_client = AsyncMock(spec=node.create_client(SetMode, '').__class__)
        node.takeoff_client = AsyncMock()
        node.land_client = AsyncMock()

        # Mock the publisher
        node.setpoint_publisher = MagicMock(spec=['publish'])

        yield node

    # We need to destroy the node manually after the test
    if rclpy.ok():
        node.destroy_node()


@pytest.mark.asyncio
async def test_connect_success(mavros_interface_node: MAVROSInterface):
    """Test the connect method for a successful connection."""
    node = mavros_interface_node
    # Simulate a successful connection by setting the state before calling
    node.state.connected = True
    assert await node.connect() is True

@pytest.mark.asyncio
async def test_connect_failure(mavros_interface_node: MAVROSInterface):
    """Test the connect method for a failed connection (times out)."""
    node = mavros_interface_node
    node.state.connected = False # Ensure it starts as not connected
    assert await node.connect() is False

@pytest.mark.asyncio
async def test_arm_success(mavros_interface_node: MAVROSInterface):
    """Test the arm method for a successful arming call."""
    node = mavros_interface_node

    # Configure the mock service client
    node.arming_client.wait_for_service.return_value = True
    future = asyncio.Future()
    future.set_result(CommandBool.Response(success=True))
    node.arming_client.call_async.return_value = future

    # In a real scenario, the state would update via callback. We simulate it.
    async def state_updater():
        await asyncio.sleep(0.05)
        node.state.armed = True

    # Run both the method and the state updater concurrently
    results = await asyncio.gather(node.arm(), state_updater())

    assert results[0] is True # The arm() method should return True
    node.arming_client.call_async.assert_called_once()
    assert node.arming_client.call_async.call_args[0][0].value is True

@pytest.mark.asyncio
async def test_set_mode_success(mavros_interface_node: MAVROSInterface):
    """Test setting the flight mode successfully."""
    node = mavros_interface_node
    node.set_mode_client.wait_for_service.return_value = True

    future = asyncio.Future()
    future.set_result(SetMode.Response(mode_sent=True))
    node.set_mode_client.call_async.return_value = future

    # Simulate state update via callback
    async def state_updater():
        await asyncio.sleep(0.05)
        node.state.flight_mode = "OFFBOARD"

    # Run concurrently and check the result of the set_mode call
    results = await asyncio.gather(node.set_mode("OFFBOARD"), state_updater())
    assert results[0] is True
    node.set_mode_client.call_async.assert_called_once_with(SetMode.Request(custom_mode="OFFBOARD"))

@pytest.mark.asyncio
async def test_goto_position_updates_target_and_starts_timer(mavros_interface_node: MAVROSInterface):
    """Test that goto_position updates the target pose and starts the setpoint timer."""
    node = mavros_interface_node

    # Mock the timer to check if it's managed correctly
    node.setpoint_timer = MagicMock(spec=['is_canceled', 'reset', 'cancel'])
    node.setpoint_timer.is_canceled.return_value = True

    # Mock dependencies for a successful run (already armed and in offboard)
    node.state.armed = True
    node.state.flight_mode = "OFFBOARD"

    # Call the method
    result = await node.goto_position(5.0, 6.0, 7.0)

    # Assertions
    assert result is True
    assert node._target_pose.pose.position.x == 5.0
    assert node._target_pose.pose.position.y == 6.0
    assert node._target_pose.pose.position.z == 7.0
    node.setpoint_timer.reset.assert_called_once()

@pytest.mark.asyncio
async def test_setpoint_callback_publishes(mavros_interface_node: MAVROSInterface):
    """Test that the setpoint callback publishes a message."""
    node = mavros_interface_node

    # Set a target pose
    node._target_pose.pose.position.x = 1.0

    # Call the callback
    node._setpoint_callback()

    # Check that publish was called
    node.setpoint_publisher.publish.assert_called_once()

    # Check that the message has the correct data
    sent_pose = node.setpoint_publisher.publish.call_args[0][0]
    assert isinstance(sent_pose, PoseStamped)
    assert sent_pose.pose.position.x == 1.0
