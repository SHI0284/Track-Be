import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo


KEEP_OUT_FILTER_TYPE = 0

DANGER_RADIUS_M = 0.45
DANGER_DUPLICATE_DISTANCE = 0.5

MASK_FREE = 0
MASK_KEEP_OUT = 100


def distance_2d(a, b):
    ax, ay = a
    bx, by = b
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


class DynamicKeepoutUpdater(Node):
    def __init__(self):
        super().__init__("dynamic_keepout_updater")

        self.map_msg = None
        self.mask_msg = None
        self.danger_points = []
        self.pending_danger_points = []

        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10,
        )

        self.danger_sub = self.create_subscription(
            PointStamped,
            "/detected_danger_points",
            self.danger_callback,
            10,
        )

        self.mask_pub = self.create_publisher(
            OccupancyGrid,
            "/keepout_filter_mask",
            transient_qos,
        )

        self.filter_info_pub = self.create_publisher(
            CostmapFilterInfo,
            "/costmap_filter_info",
            transient_qos,
        )

        self.timer = self.create_timer(1.0, self.publish_keepout_data)

        self.get_logger().info("Dynamic keepout updater started.")

    def map_callback(self, msg):
        if self.map_msg is not None:
            return

        self.map_msg = msg
        self.create_empty_mask_from_map(msg)

        self.get_logger().info(
            f"Map received. width={msg.info.width}, "
            f"height={msg.info.height}, "
            f"resolution={msg.info.resolution}"
        )

        for point in self.pending_danger_points:
            self.add_danger_point_to_mask(point)

        self.pending_danger_points.clear()
        self.publish_keepout_data()

    def create_empty_mask_from_map(self, map_msg):
        self.mask_msg = OccupancyGrid()

        self.mask_msg.header.frame_id = "map"
        self.mask_msg.header.stamp = self.get_clock().now().to_msg()

        self.mask_msg.info = map_msg.info
        size = map_msg.info.width * map_msg.info.height

        self.mask_msg.data = [MASK_FREE] * size

    def danger_callback(self, msg):
        point = (msg.point.x, msg.point.y)

        if self.is_duplicate_danger(point):
            return

        self.danger_points.append(point)

        self.get_logger().info(
            f"Danger detected at map=({point[0]:.2f}, {point[1]:.2f})"
        )

        if self.mask_msg is None:
            self.pending_danger_points.append(point)
            self.get_logger().warn("Map is not ready yet. Danger point saved.")
            return

        self.add_danger_point_to_mask(point)
        self.publish_keepout_data()

    def is_duplicate_danger(self, point):
        for saved_point in self.danger_points:
            if distance_2d(saved_point, point) < DANGER_DUPLICATE_DISTANCE:
                return True
        return False

    def world_to_grid(self, x, y):
        origin_x = self.mask_msg.info.origin.position.x
        origin_y = self.mask_msg.info.origin.position.y
        resolution = self.mask_msg.info.resolution

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        return grid_x, grid_y

    def is_inside_grid(self, gx, gy):
        width = self.mask_msg.info.width
        height = self.mask_msg.info.height

        return 0 <= gx < width and 0 <= gy < height

    def grid_index(self, gx, gy):
        width = self.mask_msg.info.width
        return gy * width + gx

    def add_danger_point_to_mask(self, point):
        center_x, center_y = point
        center_gx, center_gy = self.world_to_grid(center_x, center_y)

        resolution = self.mask_msg.info.resolution
        radius_cells = int(DANGER_RADIUS_M / resolution)

        changed_count = 0

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                gx = center_gx + dx
                gy = center_gy + dy

                if not self.is_inside_grid(gx, gy):
                    continue

                distance_m = math.sqrt(dx * dx + dy * dy) * resolution

                if distance_m <= DANGER_RADIUS_M:
                    index = self.grid_index(gx, gy)

                    if self.mask_msg.data[index] != MASK_KEEP_OUT:
                        self.mask_msg.data[index] = MASK_KEEP_OUT
                        changed_count += 1

        self.get_logger().info(
            f"Keepout mask updated around "
            f"({center_x:.2f}, {center_y:.2f}), "
            f"changed_cells={changed_count}"
        )

    def publish_filter_info(self):
        info = CostmapFilterInfo()

        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = "map"

        info.type = KEEP_OUT_FILTER_TYPE
        info.filter_mask_topic = "/keepout_filter_mask"
        info.base = 0.0
        info.multiplier = 1.0

        self.filter_info_pub.publish(info)

    def publish_mask(self):
        if self.mask_msg is None:
            return

        self.mask_msg.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(self.mask_msg)

    def publish_keepout_data(self):
        self.publish_filter_info()
        self.publish_mask()


def main():
    rclpy.init()

    node = DynamicKeepoutUpdater()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()