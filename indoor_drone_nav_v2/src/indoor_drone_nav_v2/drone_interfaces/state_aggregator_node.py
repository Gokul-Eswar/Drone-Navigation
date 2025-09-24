import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, TransformStamped

# Import standard ROS2 message types
from mavros_msgs.msg import State as MavrosState
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

# Import our custom message type
from indoor_drone_nav_v2.msg import DroneState

class StateAggregatorNode(Node):
    """
    A ROS2 node that subscribes to various MAVROS topics and the EKF output,
    looks up the drone's pose in the map frame, aggregates the information
    into a single custom DroneState message, and publishes it.
    """
    def __init__(self):
        super().__init__('state_aggregator_node')
        self.get_logger().info("State Aggregator Node starting up...")

        # Initialize TF2 buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Initialize an empty DroneState message to hold the latest state
        self._drone_state = DroneState()

        # --- Subscribers ---
        # Subscribe to the fused odometry from the EKF (for velocity)
        self.create_subscription(Odometry, '/odometry/filtered', self._odometry_callback, 10)
        # Subscribe to other MAVROS topics
        self.create_subscription(MavrosState, '/mavros/state', self._mavros_state_callback, 10)
        self.create_subscription(BatteryState, '/mavros/battery', self._battery_callback, 10)
        # The velocity from the EKF is in the odom frame, which is fine
        # self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self._velocity_callback, 10)
        self.get_logger().info("Subscribers created.")

        # --- Publisher for the aggregated state ---
        self._state_publisher = self.create_publisher(DroneState, '/indoor_drone/state', 10)
        self.get_logger().info("Publisher for /indoor_drone/state created.")

        # --- Timer to publish the aggregated state at a regular interval ---
        self.publish_timer = self.create_timer(1.0 / 20.0, self._publish_state) # 20Hz

    def _odometry_callback(self, msg: Odometry):
        """Update velocity from the EKF's odometry message."""
        # The pose is now handled by the TF lookup in _publish_state
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
        """
        Periodically look up the drone's pose in the map frame and publish
        the aggregated DroneState message.
        """
        try:
            # Look up the transform from the map frame to the drone's base_link frame
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Update the pose in our custom message
            self._drone_state.pose.position.x = trans.transform.translation.x
            self._drone_state.pose.position.y = trans.transform.translation.y
            self._drone_state.pose.position.z = trans.transform.translation.z
            self._drone_state.pose.orientation = trans.transform.rotation

            # Update the header
            self._drone_state.header.stamp = trans.header.stamp
            self._drone_state.header.frame_id = 'map'

            # Publish the aggregated state
            self._state_publisher.publish(self._drone_state)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform from map to base_link: {e}')

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
