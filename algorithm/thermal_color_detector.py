#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ThermalColorDetector(Node):
    def __init__(self):
        super().__init__("thermal_color_detector")

        self.bridge = CvBridge()

        self.latest_rgb_image = None
        self.latest_thermal_image = None

        self.rgb_subscriber = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.rgb_callback,
            10,
        )

        self.thermal_subscriber = self.create_subscription(
            Image,
            "/thermal_camera/image_raw",
            self.thermal_callback,
            10,
        )

        self.detection_publisher = self.create_publisher(
            String,
            "/thermal_color_detection",
            10,
        )

        self.min_area = 300

        # thermal image에서 밝은 영역을 열원으로 판단하는 기준
        # 너무 안 잡히면 120 정도로 낮추고, 너무 많이 잡히면 180 정도로 올리면 됨
        self.hot_threshold = 150

        self.timer = self.create_timer(0.1, self.detect)

        self.get_logger().info("Thermal + color detector started.")
        self.get_logger().info("RGB topic: /camera/image_raw")
        self.get_logger().info("Thermal topic: /thermal_camera/image_raw")
        self.get_logger().info("Publish topic: /thermal_color_detection")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )
        except Exception as error:
            self.get_logger().error(f"Failed to convert RGB image: {error}")

    def thermal_callback(self, msg):
        try:
            # thermal image가 mono8이면 그대로 받고,
            # rgb8로 들어오면 아래에서 grayscale로 변환함
            if msg.encoding in ["mono8", "8UC1"]:
                self.latest_thermal_image = self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="mono8",
                )
            else:
                thermal_rgb = self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="rgb8",
                )
                self.latest_thermal_image = cv2.cvtColor(
                    thermal_rgb,
                    cv2.COLOR_RGB2GRAY,
                )
        except Exception as error:
            self.get_logger().error(f"Failed to convert thermal image: {error}")

    def detect(self):
        if self.latest_rgb_image is None:
            self.publish_detection("NONE")
            return

        if self.latest_thermal_image is None:
            # thermal topic이 아직 안 들어와도 RGB만으로 기본 감지는 가능하게 함
            self.detect_with_rgb_only()
            return

        rgb_image = self.latest_rgb_image
        thermal_image = self.latest_thermal_image

        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        red_blob = self.detect_red(hsv_image)
        green_blob = self.detect_green(hsv_image)
        hot_blob = self.detect_hot(thermal_image)

        detected_messages = []

        # FIRE 판단:
        # 빨간색 후보가 있고, 같은 위치 근처에 thermal hot 영역이 있으면 FIRE로 확정
        if red_blob is not None:
            red_x, red_y, red_area = red_blob

            if self.is_thermal_matched(red_x, red_y, thermal_image):
                detected_messages.append(f"FIRE,{red_x},{red_y},{red_area:.0f}")
                self.get_logger().warn(
                    f"FIRE detected: center=({red_x}, {red_y}), area={red_area:.0f}"
                )
            else:
                detected_messages.append(f"RED_ONLY,{red_x},{red_y},{red_area:.0f}")

        # SURVIVOR 판단:
        # 초록색 생존자 후보는 RGB 기준으로 잡고,
        # thermal 값도 같이 확인해서 신뢰도를 높임
        if green_blob is not None:
            green_x, green_y, green_area = green_blob

            if self.is_warm_region(green_x, green_y, thermal_image):
                detected_messages.append(
                    f"SURVIVOR,{green_x},{green_y},{green_area:.0f}"
                )
                self.get_logger().info(
                    f"SURVIVOR detected: center=({green_x}, {green_y}), area={green_area:.0f}"
                )
            else:
                detected_messages.append(
                    f"GREEN_ONLY,{green_x},{green_y},{green_area:.0f}"
                )

        # 색상은 안 보이는데 열만 보이는 경우
        if not detected_messages and hot_blob is not None:
            hot_x, hot_y, hot_area = hot_blob
            detected_messages.append(f"HEAT_UNKNOWN,{hot_x},{hot_y},{hot_area:.0f}")
            self.get_logger().warn(
                f"Unknown heat source detected: center=({hot_x}, {hot_y}), area={hot_area:.0f}"
            )

        if detected_messages:
            for text in detected_messages:
                self.publish_detection(text)
        else:
            self.publish_detection("NONE")

    def detect_with_rgb_only(self):
        rgb_image = self.latest_rgb_image
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        red_blob = self.detect_red(hsv_image)
        green_blob = self.detect_green(hsv_image)

        if red_blob is not None:
            x, y, area = red_blob
            self.publish_detection(f"RED_ONLY,{x},{y},{area:.0f}")
            return

        if green_blob is not None:
            x, y, area = green_blob
            self.publish_detection(f"GREEN_ONLY,{x},{y},{area:.0f}")
            return

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
        lower_green = np.array([40, 60, 30])
        upper_green = np.array([85, 255, 255])

        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        return self.find_largest_blob(green_mask)

    def detect_hot(self, thermal_image):
        _, hot_mask = cv2.threshold(
            thermal_image,
            self.hot_threshold,
            255,
            cv2.THRESH_BINARY,
        )

        return self.find_largest_blob(hot_mask)

    def is_thermal_matched(self, center_x, center_y, thermal_image):
        mean_value = self.get_mean_thermal_value(
            thermal_image,
            center_x,
            center_y,
            radius=12,
        )

        return mean_value >= self.hot_threshold

    def is_warm_region(self, center_x, center_y, thermal_image):
        mean_value = self.get_mean_thermal_value(
            thermal_image,
            center_x,
            center_y,
            radius=12,
        )

        # 사람은 불보다 낮은 온도일 수 있으니까 기준을 조금 낮춤
        return mean_value >= 80

    def get_mean_thermal_value(self, image, center_x, center_y, radius):
        height, width = image.shape[:2]

        x1 = max(0, center_x - radius)
        x2 = min(width, center_x + radius)
        y1 = max(0, center_y - radius)
        y2 = min(height, center_y + radius)

        roi = image[y1:y2, x1:x2]

        if roi.size == 0:
            return 0

        return float(np.mean(roi))

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

    node = ThermalColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
