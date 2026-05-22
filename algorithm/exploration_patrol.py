import math
import rclpy

from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# =========================
# 수정해서 쓰는 부분
# =========================

REQUIRED_SURVIVOR_COUNT = 3

# 지도 위를 탐색할 waypoint들
# 현재 map 좌표계 기준으로 직접 수정해서 쓰면 됩니다.
PATROL_WAYPOINTS = [
    (0.0, 0.0),
    (1.5, 0.0),
    (1.5, 1.5),
    (0.0, 1.5),
    (-1.5, 1.5),
    (-1.5, 0.0),
    (-1.5, -1.5),
    (0.0, -1.5),
    (1.5, -1.5),
]

# 출구 좌표
# 실제 map 기준 출구 좌표로 수정하세요.
EXIT_POINT = (3.0, 0.0)

# 같은 구조자를 중복 인식하지 않기 위한 거리 기준
SURVIVOR_DUPLICATE_DISTANCE = 0.6

# waypoint 도착 허용 거리
NORMAL_CLOSE_ENOUGH_DISTANCE = 0.6
EXIT_CLOSE_ENOUGH_DISTANCE = 0.3


def create_pose(navigator, x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def distance_2d(a, b):
    ax, ay = a
    bx, by = b
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


class ExplorationPatrol(BasicNavigator):
    def __init__(self):
        super().__init__()

        self.discovered_survivors = []
        self.found_all_survivors = False

        self.create_subscription(
            PointStamped,
            "/detected_survivor_points",
            self.survivor_callback,
            10,
        )

    def survivor_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        new_point = (x, y)

        if self.is_duplicate_survivor(new_point):
            return

        self.discovered_survivors.append(new_point)

        print(
            f"\n[Survivor detected] "
            f"{len(self.discovered_survivors)}/{REQUIRED_SURVIVOR_COUNT} "
            f"at map=({x:.2f}, {y:.2f})"
        )

        if len(self.discovered_survivors) >= REQUIRED_SURVIVOR_COUNT:
            self.found_all_survivors = True
            print("\nAll survivors detected. Canceling current patrol goal...")
            self.cancelTask()

    def is_duplicate_survivor(self, point):
        for saved_point in self.discovered_survivors:
            if distance_2d(saved_point, point) < SURVIVOR_DUPLICATE_DISTANCE:
                return True
        return False

    def wait_until_goal_finished_with_callbacks(self, allowed_distance):
        last_distance_remaining = None
        last_print_time = self.get_clock().now()

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.found_all_survivors:
                return TaskResult.CANCELED, last_distance_remaining

            feedback = self.getFeedback()

            if feedback:
                last_distance_remaining = feedback.distance_remaining

                now = self.get_clock().now()
                elapsed = (now - last_print_time).nanoseconds / 1e9

                if elapsed >= 1.0:
                    print(f"Distance remaining: {last_distance_remaining:.2f} m")
                    last_print_time = now

        result = self.getResult()
        return result, last_distance_remaining

    def is_goal_accepted(self, result, last_distance_remaining, allowed_distance):
        if result == TaskResult.SUCCEEDED:
            return True

        if result == TaskResult.FAILED:
            return (
                last_distance_remaining is not None
                and last_distance_remaining < allowed_distance
            )

        return False

    def go_to_single_goal(self, point, label, allowed_distance):
        x, y = point

        print(f"\nMoving to {label}: {point}")

        goal_pose = create_pose(self, x, y)
        self.goToPose(goal_pose)

        result, last_distance_remaining = self.wait_until_goal_finished_with_callbacks(
            allowed_distance
        )

        if self.found_all_survivors and label != "exit":
            return False

        reached = self.is_goal_accepted(
            result,
            last_distance_remaining,
            allowed_distance,
        )

        if reached:
            print(f"Reached {label}: {point}")
            return True

        if result == TaskResult.CANCELED:
            print(f"Navigation canceled while moving to {label}: {point}")
        elif result == TaskResult.FAILED:
            print(f"Failed to reach {label}: {point}")
        else:
            print(f"Unknown result while moving to {label}: {point}")

        return False

    def run_patrol_until_three_survivors(self):
        print("Waiting for Nav2 to become active...")
        # self.waitUntilNav2Active()

        print("\nExploration patrol started.")
        print(f"Required survivors: {REQUIRED_SURVIVOR_COUNT}")

        patrol_index = 0

        while rclpy.ok() and not self.found_all_survivors:
            point = PATROL_WAYPOINTS[patrol_index]
            label = f"patrol waypoint {patrol_index + 1}/{len(PATROL_WAYPOINTS)}"

            self.go_to_single_goal(
                point,
                label,
                NORMAL_CLOSE_ENOUGH_DISTANCE,
            )

            patrol_index = (patrol_index + 1) % len(PATROL_WAYPOINTS)

        print("\nPatrol phase finished.")
        print(f"Detected survivors: {len(self.discovered_survivors)}")

        if len(self.discovered_survivors) < REQUIRED_SURVIVOR_COUNT:
            print("Mission failed: not enough survivors detected.")
            return

        print("\nMoving to final exit...")

        self.found_all_survivors = False

        exit_reached = self.go_to_single_goal(
            EXIT_POINT,
            "exit",
            EXIT_CLOSE_ENOUGH_DISTANCE,
        )

        print("\nNavigation finished.")

        if exit_reached:
            print("Mission succeeded: 3 survivors detected and exit reached.")
        else:
            print("Mission failed: exit was not reached.")


def main():
    rclpy.init()

    navigator = ExplorationPatrol()
    navigator.run_patrol_until_three_survivors()

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()