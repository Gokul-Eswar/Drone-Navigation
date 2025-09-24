import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityBridgeNode(Node):
    """
    A simple node that subscribes to the output of the twist_mux and
    publishes it to the MAVROS setpoint_velocity topic.
    """
    def __init__(self):
        super().__init__('velocity_bridge_node')
        self.get_logger().info("Velocity Bridge Node starting...")

        # Publisher to MAVROS
        self.mavros_publisher = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        # Subscriber to twist_mux output
        self.twist_mux_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.twist_mux_callback,
            10
        )

        self.get_logger().info("Velocity Bridge Node started.")

    def twist_mux_callback(self, msg: Twist):
        """Callback for the twist_mux subscriber."""
        # Simply republish the message to the MAVROS topic
        self.mavros_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
