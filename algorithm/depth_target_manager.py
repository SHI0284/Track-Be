#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


DEPTH_TOPIC = (
    "/world/complex_maze_large/model/turtlebot3_waffle/link/"
    "camera_link/sensor/intel_realsense_r200_depth/depth_image"
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DepthTargetManager(Node):
    def __init__(self):
        super().__init__("depth_target_manager")

        # Robot pose in map frame from /amcl_pose
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.latest_depth_msg = None

        # Camera image settings
        self.image_width = 320
        self.image_height = 240
        self.horizontal_fov = 1.047

        # Camera intrinsic values calculated from horizontal FOV
        self.fx = self.image_width / (2.0 * math.tan(self.horizontal_fov / 2.0))
        self.fy = self.fx
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0

        # Camera offset from base_link
        # xacro에서 카메라 pose를 0.064 -0.047 0.107 근처에 두었기 때문에
        # 2D map 좌표 계산에는 x, y offset만 간단히 반영함
        self.camera_forward_offset = 0.064
        self.camera_left_offset = -0.047

        # Detection filtering
        self.min_register_area = 1200.0
        self.center_x_min = 60
        self.center_x_max = 260

        # Depth filtering
        self.min_depth = 0.2
        self.max_depth = 5.0
        self.depth_window_radius = 3  # 7x7 median depth

        # Duplicate prevention
        self.merge_distance = 2.0
        self.detection_cooldown_sec = 3.0
        self.last_red_register_time = 0.0
        self.last_green_register_time = 0.0

        self.max_danger_points = 10
        self.max_survivor_points = 10

        self.detected_danger_points = []
        self.detected_survivor_points = []

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            10,
        )

        self.create_subscription(
            String,
            "/color_detection",
            self.color_callback,
            10,
        )

        self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            10,
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/detected_targets_marker",
            10,
        )

        self.danger_points_publisher = self.create_publisher(
            PointStamped,
            "/detected_danger_points",
            10,
        )

        self.survivor_points_publisher = self.create_publisher(
            PointStamped,
            "/detected_survivor_points",
            10,
        )

        self.danger_points_text_publisher = self.create_publisher(
            String,
            "/detected_danger_points_text",
            10,
        )

        self.survivor_points_text_publisher = self.create_publisher(
            String,
            "/detected_survivor_points_text",
            10,
        )

        self.get_logger().info("Depth target manager started.")
        self.get_logger().info("3D projection mode enabled.")
        self.get_logger().info(
            "Subscribing to /amcl_pose, /color_detection, and depth image."
        )
        self.get_logger().info("Publishing RViz markers to /detected_targets_marker.")
        self.get_logger().info("Publishing danger points to /detected_danger_points.")
        self.get_logger().info("Publishing survivor points to /detected_survivor_points.")
        self.get_logger().info("Marker frame: map")
        self.get_logger().info(f"Depth topic: {DEPTH_TOPIC}")
        self.get_logger().info(
            f"Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.1f}, cy={self.cy:.1f}"
        )

    def amcl_pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def depth_callback(self, msg):
        self.latest_depth_msg = msg

    def color_callback(self, msg):
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            self.get_logger().info("Waiting for /amcl_pose...")
            return

        if self.latest_depth_msg is None:
            self.get_logger().info("Waiting for depth image...")
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
            return

        depth = self.read_median_depth(center_x, center_y)

        if depth is None:
            return

        map_point = self.pixel_depth_to_map_point(center_x, center_y, depth)

        if map_point is None:
            return

        if color == "RED":
            self.handle_detection(
                color_name="RED",
                point_list=self.detected_danger_points,
                center_x=center_x,
                center_y=center_y,
                area=area,
                depth=depth,
                map_point=map_point,
            )

        elif color == "GREEN":
            self.handle_detection(
                color_name="GREEN",
                point_list=self.detected_survivor_points,
                center_x=center_x,
                center_y=center_y,
                area=area,
                depth=depth,
                map_point=map_point,
            )

    def is_valid_detection(self, center_x, area):
        if area < self.min_register_area:
            return False

        if center_x < self.center_x_min or center_x > self.center_x_max:
            return False

        return True

    def read_median_depth(self, center_x, center_y):
        msg = self.latest_depth_msg

        if msg.encoding != "32FC1":
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
            return None

        depth_values = []

        for dy in range(-self.depth_window_radius, self.depth_window_radius + 1):
            for dx in range(-self.depth_window_radius, self.depth_window_radius + 1):
                x = center_x + dx
                y = center_y + dy

                depth = self.read_depth_at_pixel(msg, x, y)

                if depth is None:
                    continue

                depth_values.append(depth)

        if not depth_values:
            return None

        depth_values.sort()
        median_index = len(depth_values) // 2

        return depth_values[median_index]

    def read_depth_at_pixel(self, msg, x, y):
        if x < 0 or x >= msg.width or y < 0 or y >= msg.height:
            return None

        index = y * msg.step + x * 4

        if index + 4 > len(msg.data):
            return None

        depth = struct.unpack_from("f", bytes(msg.data), index)[0]

        if not math.isfinite(depth):
            return None

        if depth < self.min_depth or depth > self.max_depth:
            return None

        return depth

    def pixel_depth_to_map_point(self, center_x, center_y, depth):
        # Camera frame approximation:
        # x_forward: camera forward direction
        # y_left: camera left/right direction
        #
        # center_x > cx means object appears on the right side of the image.
        # In robot coordinates, left is positive y, so right becomes negative y.
        x_forward = depth
        y_left = -((center_x - self.cx) * depth / self.fx)

        # z_up is calculated for completeness, but not used in 2D map position.
        z_up = -((center_y - self.cy) * depth / self.fy)

        # Apply camera offset from base_link.
        x_base = x_forward + self.camera_forward_offset
        y_base = y_left + self.camera_left_offset

        # Transform base-relative 2D point into map using AMCL pose.
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        map_x = self.robot_x + x_base * cos_yaw - y_base * sin_yaw
        map_y = self.robot_y + x_base * sin_yaw + y_base * cos_yaw

        rounded_point = self.round_point(map_x, map_y)

        self.get_logger().debug(
            f"3D projection: pixel=({center_x},{center_y}), "
            f"depth={depth:.2f}, "
            f"camera_xyz=({x_forward:.2f},{y_left:.2f},{z_up:.2f}), "
            f"map={rounded_point}"
        )

        return rounded_point

    def handle_detection(
        self,
        color_name,
        point_list,
        center_x,
        center_y,
        area,
        depth,
        map_point,
    ):
        now = time.time()

        if color_name == "RED":
            if now - self.last_red_register_time < self.detection_cooldown_sec:
                return

        elif color_name == "GREEN":
            if now - self.last_green_register_time < self.detection_cooldown_sec:
                return

        if self.is_near_existing_point(map_point, point_list):
            if color_name == "RED":
                self.last_red_register_time = now
            else:
                self.last_green_register_time = now
            return

        if color_name == "RED" and len(point_list) >= self.max_danger_points:
            return

        if color_name == "GREEN" and len(point_list) >= self.max_survivor_points:
            return

        point_list.append(map_point)

        if color_name == "RED":
            self.last_red_register_time = now
            self.get_logger().warn(
                f"New fire danger detected. "
                f"image_center=({center_x}, {center_y}), "
                f"area={area:.0f}, depth={depth:.2f}m, "
                f"map_position={map_point}"
            )

        else:
            self.last_green_register_time = now
            self.get_logger().info(
                f"New survivor detected. "
                f"image_center=({center_x}, {center_y}), "
                f"area={area:.0f}, depth={depth:.2f}m, "
                f"map_position={map_point}"
            )

        self.print_current_detected_points()
        self.publish_markers()
        self.publish_detected_points()

    def round_point(self, x, y):
        grid_size = 0.5

        rounded_x = round(x / grid_size) * grid_size
        rounded_y = round(y / grid_size) * grid_size

        return (round(rounded_x, 2), round(rounded_y, 2))

    def is_near_existing_point(self, new_point, point_list):
        for point in point_list:
            dx = new_point[0] - point[0]
            dy = new_point[1] - point[1]
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < self.merge_distance:
                return True

        return False

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        # Delete existing markers before drawing current list.
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for x, y in self.detected_danger_points:
            marker = self.create_sphere_marker(
                marker_id=marker_id,
                x=x,
                y=y,
                z=0.2,
                namespace="detected_danger",
                r=1.0,
                g=0.0,
                b=0.0,
                scale=0.8,
            )
            marker_array.markers.append(marker)
            marker_id += 1

        for x, y in self.detected_survivor_points:
            marker = self.create_sphere_marker(
                marker_id=marker_id,
                x=x,
                y=y,
                z=0.2,
                namespace="detected_survivor",
                r=0.0,
                g=1.0,
                b=0.0,
                scale=0.8,
            )
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_publisher.publish(marker_array)

    def create_sphere_marker(self, marker_id, x, y, z, namespace, r, g, b, scale):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.9

        marker.lifetime.sec = 0

        return marker

    def publish_detected_points(self):
        for x, y in self.detected_danger_points:
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = float(x)
            msg.point.y = float(y)
            msg.point.z = 0.0

            self.danger_points_publisher.publish(msg)

        for x, y in self.detected_survivor_points:
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = float(x)
            msg.point.y = float(y)
            msg.point.z = 0.0

            self.survivor_points_publisher.publish(msg)

        danger_text_msg = String()
        survivor_text_msg = String()

        danger_text_msg.data = self.points_to_string(self.detected_danger_points)
        survivor_text_msg.data = self.points_to_string(self.detected_survivor_points)

        self.danger_points_text_publisher.publish(danger_text_msg)
        self.survivor_points_text_publisher.publish(survivor_text_msg)

    def points_to_string(self, points):
        if not points:
            return ""

        return ";".join([f"{x},{y}" for x, y in points])

    def print_current_detected_points(self):
        self.get_logger().info(
            f"Detected danger map points: {self.detected_danger_points}"
        )
        self.get_logger().info(
            f"Detected survivor map points: {self.detected_survivor_points}"
        )


def main():
    rclpy.init()

    node = DepthTargetManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()