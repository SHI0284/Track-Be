import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")

        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )

        self.detection_publisher = self.create_publisher(
            String,
            "/color_detection",
            10,
        )

        self.min_area = 300

        self.get_logger().info("Color detector started.")
        self.get_logger().info("Subscribing to /camera/image_raw")
        self.get_logger().info("Publishing detection result to /color_detection")

    def image_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )
        except Exception as error:
            self.get_logger().error(f"Failed to convert image: {error}")
            return

        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        red_detection = self.detect_red(hsv_image)
        green_detection = self.detect_green(hsv_image)

        detected_messages = []

        if red_detection is not None:
            center_x, center_y, area = red_detection
            detection_text = f"RED,{center_x},{center_y},{area:.0f}"
            detected_messages.append(detection_text)

            self.get_logger().info(
                f"RED detected: center=({center_x}, {center_y}), area={area:.0f}"
            )

        if green_detection is not None:
            center_x, center_y, area = green_detection
            detection_text = f"GREEN,{center_x},{center_y},{area:.0f}"
            detected_messages.append(detection_text)

            self.get_logger().info(
                f"GREEN detected: center=({center_x}, {center_y}), area={area:.0f}"
            )

        if detected_messages:
            for detection_text in detected_messages:
                self.publish_detection(detection_text)
        else:
            self.publish_detection("NONE")

    def publish_detection(self, text):
        msg = String()
        msg.data = text
        self.detection_publisher.publish(msg)

    def detect_red(self, hsv_image):
        lower_red_1 = np.array([0, 100, 80])
        upper_red_1 = np.array([10, 255, 255])

        lower_red_2 = np.array([170, 100, 80])
        upper_red_2 = np.array([180, 255, 255])

        mask_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)

        red_mask = cv2.bitwise_or(mask_1, mask_2)

        return self.find_largest_blob(red_mask)

    def detect_green(self, hsv_image):
        lower_green = np.array([40, 80, 80])
        upper_green = np.array([85, 255, 255])

        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        return self.find_largest_blob(green_mask)

    def find_largest_blob(self, mask):
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area < self.min_area:
            return None

        moments = cv2.moments(largest_contour)

        if moments["m00"] == 0:
            return None

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])

        return center_x, center_y, area


def main():
    rclpy.init()

    node = ColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()