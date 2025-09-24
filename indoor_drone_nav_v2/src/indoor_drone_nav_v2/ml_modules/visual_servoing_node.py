import rclpy
from rclpy.node import Node
from indoor_drone_nav_v2.msg import BoundingBox
from geometry_msgs.msg import Twist

class VisualServoingNode(Node):
    """
    A node for performing visual servoing to keep a target in the center of the frame.
    """
    def __init__(self):
        super().__init__('visual_servoing_node')
        self.get_logger().info("Visual Servoing Node starting...")

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel/servo', 10)

        # Subscriber for the bounding box
        self.bbox_subscriber = self.create_subscription(
            BoundingBox,
            '/detection/bounding_box',
            self.bbox_callback,
            10
        )

        # Controller gains (proportional)
        self.yaw_gain = -0.005 # Negative gain to turn correctly
        self.z_gain = 0.005

        # Assume a standard camera resolution
        self.image_width = 640
        self.image_height = 480

        # Timer to stop the drone if no new bounding box is received
        self.stop_timer = self.create_timer(1.0, self._publish_stop_command)

        self.get_logger().info("Visual Servoing Node started.")

    def _publish_stop_command(self):
        """Publishes a zero-velocity Twist message."""
        self.get_logger().warn("No bounding box received recently, stopping.")
        self.cmd_vel_publisher.publish(Twist())
        # We can cancel the timer after it has fired once
        self.stop_timer.cancel()

    def bbox_callback(self, msg: BoundingBox):
        """Callback for the bounding box subscriber."""
        # Reset the stop timer, since we have received a new bounding box
        self.stop_timer.reset()

        # Calculate the center of the bounding box
        box_center_x = (msg.xmin + msg.xmax) / 2.0
        box_center_y = (msg.ymin + msg.ymax) / 2.0

        # Calculate the center of the image
        image_center_x = self.image_width / 2.0
        image_center_y = self.image_height / 2.0

        # Calculate the error
        error_x = box_center_x - image_center_x
        error_y = box_center_y - image_center_y

        # Create a Twist message
        twist_msg = Twist()

        # Proportional control
        twist_msg.angular.z = self.yaw_gain * error_x # Yaw control
        twist_msg.linear.z = self.z_gain * error_y    # Altitude control

        # Publish the command
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
