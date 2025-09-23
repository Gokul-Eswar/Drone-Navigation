import rclpy
from rclpy.node import Node
import asyncio
from typing import Dict

from .universal_interface import UniversalDroneInterface, DroneState

# Import ROS2 message and service types that will be used.
# This makes the dependencies of this module explicit.
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState

class MAVROSInterface(Node, UniversalDroneInterface):
    """
    ROS2 implementation of the UniversalDroneInterface for MAVROS.
    This class is a ROS2 node that subscribes to MAVROS topics and
    calls MAVROS services to control a drone.
    """

    def __init__(self, config: Dict):
        # Initialize the ROS2 Node part of this class first
        super().__init__('mavros_interface')

        # Then, initialize the UniversalDroneInterface part of this class
        UniversalDroneInterface.__init__(self, config)

        self.get_logger().info("MAVROS Interface Node starting up...")

        # Placeholder for the internal MAVROS state
        self._mavros_state = State()

        # In the following steps, we will initialize publishers, subscribers,
        # and service clients here.

        # --- Subscribers ---
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            10)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            10)

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            10)

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_callback,
            10)

        self.get_logger().info("MAVROS subscribers have been created.")

        # --- Service Clients ---
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.get_logger().info("MAVROS service clients have been created.")

        # --- Publishers ---
        self.setpoint_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # --- Timers ---
        # This timer will stream setpoints to the flight controller.
        # It's created in a canceled state and will be activated by goto_position.
        self.setpoint_timer_period = 1.0 / 20.0  # 20Hz
        self.setpoint_timer = self.create_timer(self.setpoint_timer_period, self._setpoint_callback)
        self.setpoint_timer.cancel()

        self._target_pose = PoseStamped()

    # --- Service Calls ---

    async def set_mode(self, mode: str) -> bool:
        """Set the flight mode of the drone."""
        if self._mavros_state.mode == mode:
            self.get_logger().info(f"Drone is already in {mode} mode.")
            return True

        self.get_logger().info(f"Attempting to set mode to {mode}...")
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Set mode service not available.')
            return False

        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)

        try:
            response = await asyncio.wait_for(future, timeout=5.0)
            if response.mode_sent:
                # Wait for the mode to actually change
                for _ in range(10):
                    if self.state.flight_mode == mode:
                        self.get_logger().info(f"Mode changed to {mode} successfully.")
                        return True
                    await asyncio.sleep(0.2)
                self.get_logger().error(f"Set mode call succeeded, but mode did not change to {mode}.")
                return False
            else:
                self.get_logger().error(f"Failed to send set mode request.")
                return False
        except asyncio.TimeoutError:
            self.get_logger().error("Service call to set mode timed out.")
            return False


    # --- Timer Callbacks ---

    def _setpoint_callback(self):
        """
        Callback for the setpoint timer. This function publishes the
        target pose to the MAVROS setpoint topic.
        """
        self._target_pose.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_publisher.publish(self._target_pose)

    # --- Subscriber Callbacks ---

    def _state_callback(self, msg: State):
        """Callback for the MAVROS state topic."""
        self._mavros_state = msg
        self.state.armed = msg.armed
        self.state.connected = msg.connected
        self.state.flight_mode = msg.mode

    def _pose_callback(self, msg: PoseStamped):
        """Callback for the local position topic."""
        self.state.position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        self.state.orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

    def _velocity_callback(self, msg: TwistStamped):
        """Callback for the local velocity topic."""
        self.state.velocity = (
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        )

    def _battery_callback(self, msg: BatteryState):
        """Callback for the battery state topic."""
        # MAVROS BatteryState percentage is a float from 0.0 to 1.0
        self.state.battery_level = msg.percentage * 100.0

    # --- ABC Method Implementations ---
    # The async methods from the ABC are implemented here. In the following steps,
    # they will be filled with actual ROS2 logic.

    async def connect(self) -> bool:
        """
        Connects to the drone by waiting for a heartbeat from MAVROS.
        """
        self.get_logger().info("Attempting to connect to MAVROS by checking for heartbeat...")

        # In a ROS2 node, we don't need to manually loop to check for topics.
        # The callbacks will be processed by the executor.
        # We can check the internal state variable that is updated by the callback.
        # We'll wait for a short period to see if the connection flag becomes true.
        for i in range(20): # Wait for up to 10 seconds
            if self.state.connected:
                self.get_logger().info("Connection to MAVROS successful.")
                return True
            if not rclpy.ok():
                self.get_logger().error("RCLPY shutdown while trying to connect.")
                return False
            await asyncio.sleep(0.5)

        self.get_logger().error("Could not connect to MAVROS after 10 seconds. Is it running and connected to a flight controller?")
        return False

    async def arm(self) -> bool:
        """Arms the drone using the MAVROS arming service."""
        self.get_logger().info("Attempting to arm drone...")
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Arming service not available.')
            return False

        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)

        try:
            response = await asyncio.wait_for(future, timeout=5.0)
            if response.success:
                self.get_logger().info("Drone armed successfully.")
                # We should wait for the state to reflect this
                for _ in range(10):
                    if self.state.armed: return True
                    await asyncio.sleep(0.1)
                self.get_logger().warn("Arm command sent, but state did not update to armed.")
                return False # Or True, depending on desired strictness
            else:
                self.get_logger().error(f"Failed to arm drone. Service call returned success={response.success}, result={response.result}.")
                return False
        except asyncio.TimeoutError:
            self.get_logger().error("Service call to arm timed out.")
            return False

    async def takeoff(self, altitude: float = 1.5) -> bool:
        """Takes off to a specific altitude using the MAVROS takeoff service."""
        self.get_logger().info(f"Attempting takeoff to {altitude}m...")
        if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Takeoff service not available.')
            return False

        if not self.state.armed:
            self.get_logger().warn("Cannot takeoff, drone is not armed.")
            return False

        req = CommandTOL.Request()
        req.altitude = altitude

        future = self.takeoff_client.call_async(req)

        try:
            response = await asyncio.wait_for(future, timeout=10.0)
            if response.success:
                self.get_logger().info("Takeoff command sent successfully.")
                return True
            else:
                self.get_logger().error(f"Takeoff command failed with result: {response.result}")
                return False
        except asyncio.TimeoutError:
            self.get_logger().error("Service call to takeoff timed out.")
            return False

    async def goto_position(self, x: float, y: float, z: float, yaw: float = 0) -> bool:
        """
        Commands the drone to a specific local position.
        This method handles starting the setpoint stream and switching to OFFBOARD mode.
        """
        self.get_logger().info(f"Goto position command received: ({x}, {y}, {z})")

        # Set the target pose that will be streamed by the timer
        self._target_pose.pose.position.x = x
        self._target_pose.pose.position.y = y
        self._target_pose.pose.position.z = z
        # TODO: Implement yaw to quaternion conversion

        # To switch to OFFBOARD mode, we must be streaming setpoints.
        # If the timer is not active, we start it.
        if self.setpoint_timer.is_canceled():
            self.get_logger().info("Starting setpoint timer for OFFBOARD mode.")
            # We first stream the current position to avoid a sudden jump.
            current_pose = self._target_pose
            current_pose.pose.position.x = self.state.position[0]
            current_pose.pose.position.y = self.state.position[1]
            current_pose.pose.position.z = self.state.position[2]
            self.setpoint_publisher.publish(current_pose)
            await asyncio.sleep(0.1) # Give it a moment to publish
            self.setpoint_timer.reset()

        # Ensure we are in OFFBOARD mode
        if self.state.flight_mode != "OFFBOARD":
            if not await self.set_mode("OFFBOARD"):
                self.get_logger().error("Failed to set OFFBOARD mode.")
                # self.setpoint_timer.cancel() # Optional: stop streaming on failure
                return False

        # Ensure we are armed
        if not self.state.armed:
            self.get_logger().warn("Drone is not armed. Attempting to arm...")
            if not await self.arm():
                self.get_logger().error("Failed to arm drone for goto_position.")
                # self.setpoint_timer.cancel()
                return False

        self.get_logger().info(f"Setpoint updated to: ({x}, {y}, {z}). Streaming is active.")
        return True

    async def land(self) -> bool:
        """Lands the drone using the MAVROS land service."""
        self.get_logger().info("Attempting to land...")
        if not self.land_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Land service not available.')
            return False

        req = CommandTOL.Request()
        # Using default values for the request is sufficient to land at the current position.

        future = self.land_client.call_async(req)

        try:
            response = await asyncio.wait_for(future, timeout=10.0)
            if response.success:
                self.get_logger().info("Land command sent successfully.")
                return True
            else:
                self.get_logger().error(f"Land command failed with result: {response.result}")
                return False
        except asyncio.TimeoutError:
            self.get_logger().error("Service call to land timed out.")
            return False

    async def emergency_stop(self) -> bool:
        """Disarms the drone as an emergency measure."""
        self.get_logger().warn("EMERGENCY STOP: Attempting to disarm drone...")
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Arming service not available for emergency stop.')
            return False

        req = CommandBool.Request()
        req.value = False  # False to disarm
        future = self.arming_client.call_async(req)

        try:
            response = await asyncio.wait_for(future, timeout=5.0)
            if response.success:
                self.get_logger().info("Drone disarmed successfully via emergency stop.")
                return True
            else:
                self.get_logger().error("Failed to disarm drone during emergency stop.")
                return False
        except asyncio.TimeoutError:
            self.get_logger().error("Service call to disarm timed out during emergency stop.")
            return False

def main(args=None):
    """A main function to allow running this node standalone for testing."""
    rclpy.init(args=args)

    # In a real system, a config dictionary would be passed.
    config = {}
    mavros_interface = MAVROSInterface(config)

    # Spin the node to process callbacks
    rclpy.spin(mavros_interface)

    # Shutdown
    mavros_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
