import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from indoor_drone_nav_v2.msg import BoundingBox
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ObjectDetectionNode(Node):
    """
    A node for detecting objects in an image stream.
    This is a placeholder implementation.
    """
    def __init__(self):
        super().__init__('object_detection_node')
        self.get_logger().info("Object Detection Node starting...")

        # Publisher for the detected bounding boxes
        self.bbox_publisher = self.create_publisher(BoundingBox, '/detection/bounding_box', 10)

        # Subscriber for the image stream
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # CV Bridge for converting ROS images
        self.bridge = CvBridge()

        self.get_logger().info("Object Detection Node started.")

    def image_callback(self, msg: Image):
        """Callback for the image subscriber."""
        try:
            # Convert the ROS Image message to a CV2 image.
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # --- Placeholder Detection Logic ---
        # In a real implementation, you would run a model here.
        # For now, we'll just publish a fake bounding box in the center of the image.

        H, W, _ = cv_image.shape

        bbox_msg = BoundingBox()
        bbox_msg.object_class = "person"
        bbox_msg.probability = 0.95
        bbox_msg.xmin = W // 2 - 50
        bbox_msg.ymin = H // 2 - 100
        bbox_msg.xmax = W // 2 + 50
        bbox_msg.ymax = H // 2 + 100

        self.bbox_publisher.publish(bbox_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
