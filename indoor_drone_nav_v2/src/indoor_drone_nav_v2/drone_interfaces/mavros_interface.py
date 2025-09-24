import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio

from .universal_interface import UniversalDroneInterface
from indoor_drone_nav_v2.msg import DroneState
from indoor_drone_nav_v2.action import Arm, Takeoff, Land, GotoPosition
from geometry_msgs.msg import PoseStamped

class MAVROSInterface(UniversalDroneInterface):
    """
    A client library that provides a simple, high-level API for controlling a drone.
    It interacts with the `StateAggregatorNode` and `DroneActionServerNode` by
    subscribing to the aggregated state topic and calling the high-level actions.
    This class is NOT a ROS2 node itself, but it uses a node provided to it.
    """

    def __init__(self, node: Node):
        self.node = node
        self.logger = self.node.get_logger()

        # Initialize the superclass (which sets up self.state, etc.)
        super().__init__(config={})

        # --- Subscriber to our aggregated state topic ---
        self.node.create_subscription(DroneState, '/indoor_drone/state', self._state_callback, 10)

        # --- Action Clients ---
        self.arm_client = ActionClient(self.node, Arm, 'drone/arm')
        self.takeoff_client = ActionClient(self.node, Takeoff, 'drone/takeoff')
        self.land_client = ActionClient(self.node, Land, 'drone/land')
        self.goto_position_client = ActionClient(self.node, GotoPosition, 'drone/goto_position')

        self.logger.info("MAVROSInterface client library initialized.")

    def _state_callback(self, msg: DroneState):
        """Update the internal state from the aggregated topic."""
        self.state.armed = msg.armed
        self.state.connected = msg.connected
        self.state.flight_mode = msg.flight_mode
        self.state.position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.state.orientation = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.state.velocity = (msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z)
        self.state.battery_level = msg.battery_percentage

    # --- ABC Method Implementations ---

    async def connect(self) -> bool:
        # In this architecture, connection is just waiting for the first state message.
        self.logger.info("Waiting for connection...")
        for _ in range(20): # Wait up to 10 seconds
            if self.state.connected:
                self.logger.info("Successfully connected (received state from aggregator).")
                return True
            await asyncio.sleep(0.5)
        self.logger.error("Failed to connect (did not receive state from aggregator).")
        return False

    async def _send_goal_and_wait(self, client: ActionClient, goal_msg) -> bool:
        """Helper function to send a goal and wait for the result."""
        if not client.wait_for_server(timeout_sec=2.0):
            self.logger.error(f"Action server for {client.action_name} not available.")
            return False

        goal_handle = await client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.logger.error("Goal was rejected by the action server.")
            return False

        result = await goal_handle.get_result_async()
        return result.result.success

    async def arm(self) -> bool:
        self.logger.info("Sending arm goal...")
        return await self._send_goal_and_wait(self.arm_client, Arm.Goal(arm=True))

    async def takeoff(self, altitude: float = 1.5) -> bool:
        self.logger.info(f"Sending takeoff goal to altitude {altitude}...")
        return await self._send_goal_and_wait(self.takeoff_client, Takeoff.Goal(altitude=altitude))

    async def land(self) -> bool:
        self.logger.info("Sending land goal...")
        return await self._send_goal_and_wait(self.land_client, Land.Goal())

    async def goto_position(self, x: float, y: float, z: float, yaw: float = 0) -> bool:
        self.logger.info(f"Sending goto_position goal to ({x},{y},{z})...")
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        # TODO: Yaw to quaternion conversion
        goal_msg = GotoPosition.Goal(target_pose=pose)
        return await self._send_goal_and_wait(self.goto_position_client, goal_msg)

    async def emergency_stop(self) -> bool:
        self.logger.warn("Sending EMERGENCY STOP (disarm) goal...")
        return await self._send_goal_and_wait(self.arm_client, Arm.Goal(arm=False))
