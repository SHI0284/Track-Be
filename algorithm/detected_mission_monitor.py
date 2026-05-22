#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DetectedMissionMonitor(Node):
    def __init__(self):
        super().__init__("detected_mission_monitor")

        self.detected_danger_points = []
        self.detected_survivor_points = []

        self.danger_subscriber = self.create_subscription(
            String,
            "/detected_danger_points",
            self.danger_points_callback,
            10,
        )

        self.survivor_subscriber = self.create_subscription(
            String,
            "/detected_survivor_points",
            self.survivor_points_callback,
            10,
        )

        self.get_logger().info("Detected mission monitor started.")
        self.get_logger().info("Subscribing to /detected_danger_points")
        self.get_logger().info("Subscribing to /detected_survivor_points")

    def danger_points_callback(self, msg):
        new_points = self.parse_points(msg.data)

        if new_points != self.detected_danger_points:
            self.detected_danger_points = new_points
            self.print_detected_points()

    def survivor_points_callback(self, msg):
        new_points = self.parse_points(msg.data)

        if new_points != self.detected_survivor_points:
            self.detected_survivor_points = new_points
            self.print_detected_points()

    def parse_points(self, text):
        text = text.strip()

        if not text:
            return []

        points = []

        point_texts = text.split(";")

        for point_text in point_texts:
            point_text = point_text.strip()

            if not point_text:
                continue

            parts = point_text.split(",")

            if len(parts) != 2:
                self.get_logger().warn(f"Invalid point format: {point_text}")
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
            except ValueError:
                self.get_logger().warn(f"Invalid point value: {point_text}")
                continue

            points.append((x, y))

        return points

    def print_detected_points(self):
        self.get_logger().info("")
        self.get_logger().info("========== Detected Mission Points ==========")

        if self.detected_danger_points:
            self.get_logger().info("Detected danger points:")
            for index, point in enumerate(self.detected_danger_points, start=1):
                self.get_logger().info(f"  Danger {index}: {point}")
        else:
            self.get_logger().info("Detected danger points: none")

        if self.detected_survivor_points:
            self.get_logger().info("Detected survivor points:")
            for index, point in enumerate(self.detected_survivor_points, start=1):
                self.get_logger().info(f"  Survivor {index}: {point}")
        else:
            self.get_logger().info("Detected survivor points: none")

        self.get_logger().info("============================================")
        self.get_logger().info("")

    def get_nearest_survivor(self, current_pos):
        if not self.detected_survivor_points:
            return None

        cx, cy = current_pos

        nearest_point = None
        nearest_distance = float("inf")

        for point in self.detected_survivor_points:
            px, py = point
            distance = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_point = point

        return nearest_point

    def get_nearest_danger(self, current_pos):
        if not self.detected_danger_points:
            return None

        cx, cy = current_pos

        nearest_point = None
        nearest_distance = float("inf")

        for point in self.detected_danger_points:
            px, py = point
            distance = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_point = point

        return nearest_point


def main():
    rclpy.init()

    node = DetectedMissionMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()