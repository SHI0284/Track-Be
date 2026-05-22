import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DetectedTargetManager(Node):
    def __init__(self):
        super().__init__("detected_target_manager")

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.image_width = 320

        self.estimated_distance = 1.5
        self.merge_distance = 1.5

        self.detection_cooldown_sec = 2.0
        self.last_red_register_time = 0.0
        self.last_green_register_time = 0.0
        self.min_register_area = 250.0

        self.center_x_min = 0
        self.center_x_max = 320

        self.max_danger_points = 10
        self.max_survivor_points = 10

        self.detected_danger_points = []
        self.detected_survivor_points = []

        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.color_subscriber = self.create_subscription(
            String,
            "/color_detection",
            self.color_callback,
            10,
        )

        self.get_logger().info("Detected target manager started.")
        self.get_logger().info("Subscribing to /odom and /color_detection")
        self.get_logger().info(
            "This node estimates candidate positions from camera color detections."
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def color_callback(self, msg):
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            self.get_logger().info("Waiting for /odom...")
            return

        data = msg.data.strip()

        if data == "NONE":
            return

        parts = data.split(",")

        if len(parts) != 4:
            self.get_logger().warn(f"Invalid detection message: {data}")
            return

        color = parts[0]

        try:
            center_x = int(parts[1])
            center_y = int(parts[2])
            area = float(parts[3])
        except ValueError:
            self.get_logger().warn(f"Invalid detection values: {data}")
            return

        if not self.is_valid_detection(center_x, area):
            # self.get_logger().info(
            #     f"Ignored {color} detection. "
            #     f"center_x={center_x}, area={area:.0f}"
            # )
            return

        if color == "RED":
            self.handle_red_detection(center_x, center_y, area)

        elif color == "GREEN":
            self.handle_green_detection(center_x, center_y, area)

        else:
            self.get_logger().warn(f"Unknown color type: {color}")

    def is_valid_detection(self, center_x, area):
        if area < self.min_register_area:
            return False

        if center_x < self.center_x_min or center_x > self.center_x_max:
            return False

        return True

    def handle_red_detection(self, center_x, center_y, area):
        now = time.time()

        if now - self.last_red_register_time < self.detection_cooldown_sec:
            return

        point = self.estimate_and_round_point(center_x)

        if self.is_near_existing_point(point, self.detected_danger_points):
            self.get_logger().warn(
                f"Fire danger already registered near {point}"
            )
            self.last_red_register_time = now
            return

        if len(self.detected_danger_points) >= self.max_danger_points:
            self.get_logger().warn(
                "Danger point list is full. Ignoring new red detection."
            )
            self.last_red_register_time = now
            return

        self.detected_danger_points.append(point)
        self.last_red_register_time = now

        self.get_logger().warn(
            f"New fire danger detected. "
            f"image_center=({center_x}, {center_y}), "
            f"area={area:.0f}, "
            f"estimated_position={point}"
        )

        self.print_current_detected_points()

    def handle_green_detection(self, center_x, center_y, area):
        now = time.time()

        if now - self.last_green_register_time < self.detection_cooldown_sec:
            return

        point = self.estimate_and_round_point(center_x)

        if self.is_near_existing_point(point, self.detected_survivor_points):
            self.get_logger().info(
                f"Survivor already registered near {point}"
            )
            self.last_green_register_time = now
            return

        if len(self.detected_survivor_points) >= self.max_survivor_points:
            self.get_logger().warn(
                "Survivor point list is full. Ignoring new green detection."
            )
            self.last_green_register_time = now
            return

        self.detected_survivor_points.append(point)
        self.last_green_register_time = now

        self.get_logger().info(
            f"New survivor detected. "
            f"image_center=({center_x}, {center_y}), "
            f"area={area:.0f}, "
            f"estimated_position={point}"
        )

        self.print_current_detected_points()

    def estimate_and_round_point(self, center_x):
        target_x, target_y = self.estimate_target_position(center_x)
        return self.round_point(target_x, target_y)

    def estimate_target_position(self, center_x):
        normalized_offset = (center_x - self.image_width / 2.0) / (
            self.image_width / 2.0
        )

        max_angle_offset = math.radians(30.0)
        angle_offset = normalized_offset * max_angle_offset

        target_angle = self.robot_yaw + angle_offset

        target_x = self.robot_x + self.estimated_distance * math.cos(target_angle)
        target_y = self.robot_y + self.estimated_distance * math.sin(target_angle)

        return target_x, target_y

    def round_point(self, x, y):
        grid_size = 0.5

        rounded_x = round(x / grid_size) * grid_size
        rounded_y = round(y / grid_size) * grid_size

        return (round(rounded_x, 2), round(rounded_y, 2))

    def is_near_existing_point(self, new_point, point_list):
        for point in point_list:
            dx = new_point[0] - point[0]
            dy = new_point[1] - point[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < self.merge_distance:
                return True

        return False

    def print_current_detected_points(self):
        self.get_logger().info(
            f"Detected danger points: {self.detected_danger_points}"
        )
        self.get_logger().info(
            f"Detected survivor points: {self.detected_survivor_points}"
        )


def main():
    rclpy.init()

    node = DetectedTargetManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()