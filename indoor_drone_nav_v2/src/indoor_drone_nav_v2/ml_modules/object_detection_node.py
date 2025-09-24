import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from indoor_drone_nav_v2.msg import BoundingBox
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    """
    A node for detecting objects in an image stream based on color.
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

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for bright green color
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # Only publish if the contour is a reasonable size
            if area > 100:
                x, y, w, h = cv2.boundingRect(largest_contour)

                bbox_msg = BoundingBox()
                bbox_msg.object_class = "green_target"
                bbox_msg.probability = 0.99
                bbox_msg.xmin = x
                bbox_msg.ymin = y
                bbox_msg.xmax = x + w
                bbox_msg.ymax = y + h

                self.bbox_publisher.publish(bbox_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
