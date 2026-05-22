#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class DetectedSurvivorPatrol(Node):
    def __init__(self):
        super().__init__("detected_survivor_patrol")

        self.detected_survivor_points = []
        self.visited_survivor_points = []

        self.current_robot_position = None

        self.goal_tolerance = 0.8
        self.same_target_distance = 1.0

        self.survivor_subscriber = self.create_subscription(
            String,
            "/detected_survivor_points",
            self.survivor_points_callback,
            10,
        )

        self.get_logger().info("Detected survivor patrol node started.")
        self.get_logger().info("Subscribing to /detected_survivor_points")

    def survivor_points_callback(self, msg):
        points = self.parse_points(msg.data)

        if points != self.detected_survivor_points:
            self.detected_survivor_points = points
            self.get_logger().info(f"Updated detected survivor points: {points}")

    def parse_points(self, text):
        text = text.strip()

        if not text:
            return []

        points = []

        for point_text in text.split(";"):
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

    def get_unvisited_survivors(self):
        unvisited = []

        for point in self.detected_survivor_points:
            if not self.is_visited(point):
                unvisited.append(point)

        return unvisited

    def is_visited(self, point):
        for visited_point in self.visited_survivor_points:
            if self.distance(point, visited_point) < self.same_target_distance:
                return True

        return False

    def get_nearest_survivor(self, current_position):
        unvisited = self.get_unvisited_survivors()

        if not unvisited:
            return None

        nearest_point = None
        nearest_distance = float("inf")

        for point in unvisited:
            distance = self.distance(current_position, point)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_point = point

        return nearest_point

    def mark_survivor_visited(self, point):
        if not self.is_visited(point):
            self.visited_survivor_points.append(point)

        self.get_logger().info(f"Survivor rescued at {point}")
        self.get_logger().info(
            f"Total rescued survivors: {len(self.visited_survivor_points)}"
        )

    def distance(self, point_a, point_b):
        ax, ay = point_a
        bx, by = point_b

        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def create_goal_pose(navigator, x, y):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0

    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    return goal_pose


def get_robot_position_from_amcl(node):
    """
    간단한 테스트용 함수입니다.
    /amcl_pose를 직접 한 번 받아오는 대신, 여기서는 Nav2 goal 이동 결과 중심으로만 판단합니다.
    시작 위치는 (0, 0)으로 두고, goal 도착 후 current_position을 goal로 갱신합니다.
    """
    return node.current_robot_position


def main():
    rclpy.init()

    patrol_node = DetectedSurvivorPatrol()

    navigator = BasicNavigator()

    patrol_node.get_logger().info("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    patrol_node.get_logger().info("Nav2 is active.")

    # 테스트 시작 위치. 실제 로봇 위치는 Nav2 내부에서 관리합니다.
    # nearest survivor 선택용으로만 사용합니다.
    patrol_node.current_robot_position = (0.0, 0.0)

    patrol_node.get_logger().info("Waiting for detected survivor points...")

    try:
        while rclpy.ok():
            rclpy.spin_once(patrol_node, timeout_sec=0.2)

            unvisited = patrol_node.get_unvisited_survivors()

            if not unvisited:
                time.sleep(0.5)
                continue

            current_position = get_robot_position_from_amcl(patrol_node)

            if current_position is None:
                current_position = (0.0, 0.0)

            target = patrol_node.get_nearest_survivor(current_position)

            if target is None:
                time.sleep(0.5)
                continue

            target_x, target_y = target

            patrol_node.get_logger().info(
                f"Next detected survivor target: ({target_x}, {target_y})"
            )

            goal_pose = create_goal_pose(navigator, target_x, target_y)

            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                rclpy.spin_once(patrol_node, timeout_sec=0.1)

                feedback = navigator.getFeedback()

                if feedback is not None:
                    remaining = feedback.distance_remaining
                    patrol_node.get_logger().info(
                        f"Moving to survivor {target}. "
                        f"Distance remaining: {remaining:.2f} m"
                    )

                time.sleep(0.5)

            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                patrol_node.get_logger().info(f"Reached survivor target: {target}")
                patrol_node.mark_survivor_visited(target)
                patrol_node.current_robot_position = target

            elif result == TaskResult.CANCELED:
                patrol_node.get_logger().warn(f"Navigation canceled: {target}")

            elif result == TaskResult.FAILED:
                patrol_node.get_logger().warn(f"Navigation failed: {target}")

                # 테스트 단계에서는 실패해도 너무 가까우면 방문 처리
                patrol_node.current_robot_position = target

            time.sleep(1.0)

    except KeyboardInterrupt:
        patrol_node.get_logger().info("Detected survivor patrol stopped by user.")

    finally:
        navigator.cancelTask()
        patrol_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()