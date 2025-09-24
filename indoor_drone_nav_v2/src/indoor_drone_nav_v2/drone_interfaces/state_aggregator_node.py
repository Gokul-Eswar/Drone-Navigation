import rclpy
from rclpy.node import Node

# Import standard ROS2 message types
from mavros_msgs.msg import State as MavrosState
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

# Import our custom message type
# This will be available after the workspace is built with `colcon build`
from indoor_drone_nav_v2.msg import DroneState

class StateAggregatorNode(Node):
    """
    A ROS2 node that subscribes to various MAVROS topics and the EKF output,
    aggregates the information into a single custom DroneState message,
    and publishes it for other nodes to use.
    """
    def __init__(self):
        super().__init__('state_aggregator_node')
        self.get_logger().info("State Aggregator Node starting up...")

        # Initialize an empty DroneState message to hold the latest state
        self._drone_state = DroneState()

        # --- Subscribers ---
        # Subscribe to the fused odometry from the EKF
        self.create_subscription(Odometry, '/odometry/filtered', self._odometry_callback, 10)
        # Subscribe to other MAVROS topics
        self.create_subscription(MavrosState, '/mavros/state', self._mavros_state_callback, 10)
        self.create_subscription(BatteryState, '/mavros/battery', self._battery_callback, 10)
        self.get_logger().info("Subscribers created.")

        # --- Publisher for the aggregated state ---
        self._state_publisher = self.create_publisher(DroneState, '/indoor_drone/state', 10)
        self.get_logger().info("Publisher for /indoor_drone/state created.")

        # --- Timer to publish the aggregated state at a regular interval ---
        self.publish_timer = self.create_timer(1.0 / 20.0, self._publish_state) # 20Hz

    def _odometry_callback(self, msg: Odometry):
        """Update pose, velocity, and header information from the EKF."""
        self._drone_state.header = msg.header
        self._drone_state.pose = msg.pose.pose
        self._drone_state.velocity = msg.twist.twist

    def _mavros_state_callback(self, msg: MavrosState):
        """Update armed status, connection status, and flight mode."""
        self._drone_state.armed = msg.armed
        self._drone_state.connected = msg.connected
        self._drone_state.flight_mode = msg.mode

    def _battery_callback(self, msg: BatteryState):
        """Update battery percentage."""
        self._drone_state.battery_percentage = msg.percentage * 100.0

    def _publish_state(self):
        """Periodically publish the aggregated DroneState message."""
        # Update the timestamp to the current time before publishing
        self._drone_state.header.stamp = self.get_clock().now().to_msg()
        self._state_publisher.publish(self._drone_state)

def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    node = StateAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
