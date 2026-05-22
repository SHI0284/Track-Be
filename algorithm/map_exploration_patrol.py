import math
import time
import rclpy

from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


# Mission settings
REQUIRED_SURVIVOR_COUNT = 3

EXIT_CANDIDATES = [
    (8.0, -8.0),
    (7.5, -8.0),
    (8.0, -7.5),
    (7.5, -7.5),
]

PATROL_MIN_X = -9.0
PATROL_MAX_X = 9.0
PATROL_MIN_Y = -9.0
PATROL_MAX_Y = 9.0

WAYPOINT_SPACING_M = 0.6
COVERAGE_GRID_COLS = 10
COVERAGE_GRID_ROWS = 6
MAX_PATROL_WAYPOINTS = 200
MIN_DISTANCE_BETWEEN_PATROL_WAYPOINTS = 1.3

BACKTRACK_PENALTY_DISTANCE = 3.0
BACKTRACK_PENALTY_WEIGHT = 4.0
DIRECTION_WEIGHT = 1.2

SAFE_MARGIN_M = 0.40
SURVIVOR_DUPLICATE_DISTANCE = 4
DANGER_DUPLICATE_DISTANCE = 1.2
DANGER_WAYPOINT_SKIP_DISTANCE = 1.5
DANGER_TOO_CLOSE_DISTANCE = 1.8

CANCEL_GOAL_ON_DANGER = True
CANCEL_GOAL_WHEN_TOO_CLOSE_TO_DANGER = True

NORMAL_CLOSE_ENOUGH_DISTANCE = 0.65
EXIT_CLOSE_ENOUGH_DISTANCE = 0.35

FREE_THRESHOLD = 20
UNKNOWN_AS_BLOCKED = True

DANGER_CANCEL_COOLDOWN_SEC = 8.0
DANGER_PROXIMITY_CANCEL_COOLDOWN_SEC = 8.0

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


class MapExplorationPatrol(BasicNavigator):
    def __init__(self):
        super().__init__()

        self.map_msg = None
        self.map_received = False
        self.patrol_waypoints = []

        self.current_robot_position = None

        self.discovered_survivors = []
        self.found_all_survivors = False

        self.discovered_dangers = []
        self.danger_detected_during_current_goal = False
        self.too_close_to_danger = False
        self.cancel_requested = False

        self.last_danger_cancel_time = 0.0
        self.last_proximity_cancel_time = 0.0

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            10,
        )

        self.create_subscription(
            PointStamped,
            "/detected_survivor_points",
            self.survivor_callback,
            10,
        )

        self.create_subscription(
            PointStamped,
            "/detected_danger_points",
            self.danger_callback,
            10,
        )

    # Map handling
    def map_callback(self, msg):
        if self.map_received:
            return

        self.map_msg = msg
        self.map_received = True

        print(
            f"Map received: width={msg.info.width}, "
            f"height={msg.info.height}, "
            f"resolution={msg.info.resolution}, "
            f"origin=({msg.info.origin.position.x:.2f}, "
            f"{msg.info.origin.position.y:.2f})"
        )

    def wait_for_map(self):
        print("Waiting for /map...")

        while rclpy.ok() and not self.map_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("Map is ready.")

    def is_inside_grid(self, gx, gy):
        width = self.map_msg.info.width
        height = self.map_msg.info.height

        return 0 <= gx < width and 0 <= gy < height

    def grid_to_world(self, gx, gy):
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        resolution = self.map_msg.info.resolution

        wx = origin_x + (gx + 0.5) * resolution
        wy = origin_y + (gy + 0.5) * resolution

        return wx, wy

    def world_to_grid(self, wx, wy):
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        resolution = self.map_msg.info.resolution

        gx = int((wx - origin_x) / resolution)
        gy = int((wy - origin_y) / resolution)

        return gx, gy

    def is_inside_patrol_bounds(self, wx, wy):
        return (
            PATROL_MIN_X <= wx <= PATROL_MAX_X
            and PATROL_MIN_Y <= wy <= PATROL_MAX_Y
        )

    def is_free_cell(self, gx, gy):
        if not self.is_inside_grid(gx, gy):
            return False

        width = self.map_msg.info.width
        index = gy * width + gx
        value = self.map_msg.data[index]

        if value == -1:
            return not UNKNOWN_AS_BLOCKED

        return value < FREE_THRESHOLD

    def is_safe_area_around_cell(self, gx, gy):
        resolution = self.map_msg.info.resolution
        radius_cells = max(1, int(SAFE_MARGIN_M / resolution))

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx = gx + dx
                ny = gy + dy

                if not self.is_free_cell(nx, ny):
                    return False

        return True

    def build_patrol_waypoints_from_map(self):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution

        step_cells = max(1, int(WAYPOINT_SPACING_M / resolution))

        raw_waypoints = []

        for gy in range(0, height, step_cells):
            for gx in range(0, width, step_cells):
                wx, wy = self.grid_to_world(gx, gy)

                if not self.is_inside_patrol_bounds(wx, wy):
                    continue

                if not self.is_free_cell(gx, gy):
                    continue

                if not self.is_safe_area_around_cell(gx, gy):
                    continue

                raw_waypoints.append((wx, wy))

        print(f"Raw candidate waypoints: {len(raw_waypoints)}")

        selected_waypoints = self.select_coverage_waypoints(raw_waypoints)

        spaced_waypoints = self.remove_too_close_waypoints(
            selected_waypoints,
            MIN_DISTANCE_BETWEEN_PATROL_WAYPOINTS,
        )

        ordered_waypoints = self.order_waypoints_sweep_with_less_backtracking(
            spaced_waypoints
        )

        if len(ordered_waypoints) > MAX_PATROL_WAYPOINTS:
            ordered_waypoints = ordered_waypoints[:MAX_PATROL_WAYPOINTS]

        print(f"Coverage-selected waypoints: {len(selected_waypoints)}")
        print(f"After distance filtering: {len(spaced_waypoints)}")
        print(f"Generated patrol waypoints: {len(ordered_waypoints)}")

        print("\nFinal patrol waypoint order:")
        for i, point in enumerate(ordered_waypoints, start=1):
            print(f"{i}: ({point[0]:.2f}, {point[1]:.2f})")

        return ordered_waypoints

    def select_coverage_waypoints(self, raw_waypoints):
        if not raw_waypoints:
            return []

        cell_width = (PATROL_MAX_X - PATROL_MIN_X) / COVERAGE_GRID_COLS
        cell_height = (PATROL_MAX_Y - PATROL_MIN_Y) / COVERAGE_GRID_ROWS

        buckets = {}

        for wx, wy in raw_waypoints:
            col = int((wx - PATROL_MIN_X) / cell_width)
            row = int((wy - PATROL_MIN_Y) / cell_height)

            col = min(max(col, 0), COVERAGE_GRID_COLS - 1)
            row = min(max(row, 0), COVERAGE_GRID_ROWS - 1)

            key = (row, col)

            if key not in buckets:
                buckets[key] = []

            buckets[key].append((wx, wy))

        selected = []

        for row in range(COVERAGE_GRID_ROWS):
            row_selected = []

            for col in range(COVERAGE_GRID_COLS):
                key = (row, col)

                if key not in buckets:
                    continue

                representative = self.pick_representative_waypoint(
                    buckets[key],
                    row,
                    col,
                    cell_width,
                    cell_height,
                )

                row_selected.append(representative)

            if row % 2 == 1:
                row_selected.reverse()

            selected.extend(row_selected)


        return selected

    def pick_representative_waypoint(self, points, row, col, cell_width, cell_height):
        center_x = PATROL_MIN_X + (col + 0.5) * cell_width
        center_y = PATROL_MIN_Y + (row + 0.5) * cell_height

        best_point = None
        best_distance = float("inf")

        for point in points:
            distance = distance_2d(point, (center_x, center_y))

            if distance < best_distance:
                best_distance = distance
                best_point = point

        return best_point

    def remove_too_close_waypoints(self, waypoints, min_distance):
        filtered = []

        for point in waypoints:
            too_close = False

            for saved_point in filtered:
                if distance_2d(point, saved_point) < min_distance:
                    too_close = True
                    break

            if not too_close:
                filtered.append(point)

        return filtered

    def order_waypoints_sweep_with_less_backtracking(self, waypoints):
        if not waypoints:
            return []

        rows = self.group_waypoints_by_coverage_row(waypoints)

        sweep_order = []

        for row_index in sorted(rows.keys()):
            row_points = rows[row_index]
            row_points.sort(key=lambda p: p[0])

            if row_index % 2 == 1:
                row_points.reverse()

            sweep_order.extend(row_points)

        if not sweep_order:
            return []

        start_index = self.find_nearest_waypoint_index(sweep_order)
        ordered = sweep_order[start_index:] + sweep_order[:start_index]

        ordered = self.reduce_local_backtracking(ordered)

        return ordered

    def group_waypoints_by_coverage_row(self, waypoints):
        cell_height = (PATROL_MAX_Y - PATROL_MIN_Y) / COVERAGE_GRID_ROWS

        rows = {}

        for wx, wy in waypoints:
            row = int((wy - PATROL_MIN_Y) / cell_height)
            row = min(max(row, 0), COVERAGE_GRID_ROWS - 1)

            if row not in rows:
                rows[row] = []

            rows[row].append((wx, wy))

        return rows

    def find_nearest_waypoint_index(self, waypoints):
        if self.current_robot_position is None:
            current = (0.0, 0.0)
        else:
            current = self.current_robot_position

        nearest_index = 0
        nearest_distance = float("inf")

        for i, point in enumerate(waypoints):
            d = distance_2d(current, point)

            if d < nearest_distance:
                nearest_distance = d
                nearest_index = i

        return nearest_index

    def reduce_local_backtracking(self, waypoints):
        if len(waypoints) <= 3:
            return waypoints

        remaining = waypoints.copy()
        ordered = []

        if self.current_robot_position is None:
            current = remaining.pop(0)
        else:
            current = self.current_robot_position

        previous = None

        while remaining:
            best_point = None
            best_score = float("inf")

            for point in remaining:
                score = self.calculate_waypoint_score(
                    previous,
                    current,
                    point,
                    ordered,
                )

                if score < best_score:
                    best_score = score
                    best_point = point

            ordered.append(best_point)
            remaining.remove(best_point)

            previous = current
            current = best_point

        return ordered

    def calculate_waypoint_score(self, previous, current, candidate, visited):
        score = distance_2d(current, candidate)

        for visited_point in visited:
            d = distance_2d(candidate, visited_point)

            if d < BACKTRACK_PENALTY_DISTANCE:
                score += BACKTRACK_PENALTY_WEIGHT * (
                    BACKTRACK_PENALTY_DISTANCE - d
                )

        if previous is not None:
            prev_vec = (
                current[0] - previous[0],
                current[1] - previous[1],
            )
            next_vec = (
                candidate[0] - current[0],
                candidate[1] - current[1],
            )

            prev_norm = math.sqrt(prev_vec[0] ** 2 + prev_vec[1] ** 2)
            next_norm = math.sqrt(next_vec[0] ** 2 + next_vec[1] ** 2)

            if prev_norm > 0.001 and next_norm > 0.001:
                dot = (
                    prev_vec[0] * next_vec[0]
                    + prev_vec[1] * next_vec[1]
                ) / (prev_norm * next_norm)

                if dot < -0.3:
                    score += DIRECTION_WEIGHT * abs(dot)

        return score

    # Current robot pose / AMCL
    def amcl_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.current_robot_position = (x, y)

        if not self.is_robot_too_close_to_danger():
            return

        now = time.time()

        if now - self.last_proximity_cancel_time < DANGER_PROXIMITY_CANCEL_COOLDOWN_SEC:
            return

        self.last_proximity_cancel_time = now
        self.too_close_to_danger = True

        print(
            f"\n[Danger proximity warning] "
            f"Robot is too close to wildfire zone. "
            f"robot=({x:.2f}, {y:.2f})"
        )

        if CANCEL_GOAL_WHEN_TOO_CLOSE_TO_DANGER:
            print("Requesting current goal cancel for safety...")
            self.cancel_requested = True

    def is_robot_too_close_to_danger(self):
        if self.current_robot_position is None:
            return False

        for danger_point in self.discovered_dangers:
            distance = distance_2d(self.current_robot_position, danger_point)

            if distance < DANGER_TOO_CLOSE_DISTANCE:
                return True

        return False

    # Survivor detection
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
            self.cancel_requested = True

    def is_duplicate_survivor(self, point):
        for saved_point in self.discovered_survivors:
            if distance_2d(saved_point, point) < SURVIVOR_DUPLICATE_DISTANCE:
                return True

        return False

    # Danger / wildfire detection
    def danger_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        new_point = (x, y)

        if self.is_duplicate_danger(new_point):
            return

        self.discovered_dangers.append(new_point)

        print(
            f"\n[Danger detected / 산불 감지] "
            f"{len(self.discovered_dangers)} danger point(s), "
            f"at map=({x:.2f}, {y:.2f})"
        )

        if not CANCEL_GOAL_ON_DANGER:
            return

        now = time.time()

        if now - self.last_danger_cancel_time < DANGER_CANCEL_COOLDOWN_SEC:
            print("Danger cancel skipped because cooldown is active.")
            return

        self.last_danger_cancel_time = now
        self.danger_detected_during_current_goal = True
        self.cancel_requested = True

        print("Requesting current goal cancel to replan around new danger zone...")

    def is_duplicate_danger(self, point):
        for saved_point in self.discovered_dangers:
            if distance_2d(saved_point, point) < DANGER_DUPLICATE_DISTANCE:
                return True

        return False

    def is_waypoint_near_danger(self, point):
        for danger_point in self.discovered_dangers:
            if distance_2d(point, danger_point) < DANGER_WAYPOINT_SKIP_DISTANCE:
                return True

        return False

    # Navigation
    def wait_until_goal_finished_with_callbacks(self, ignore_danger_cancel=False):
        last_distance_remaining = None
        last_print_time = self.get_clock().now()

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.cancel_requested:
                if ignore_danger_cancel and (
                    self.danger_detected_during_current_goal or self.too_close_to_danger
                ):
                    print("Danger cancel ignored while moving to exit.")
                    self.cancel_requested = False
                    self.danger_detected_during_current_goal = False
                    self.too_close_to_danger = False
                else:
                    self.cancel_requested = False
                    self.cancelTask()
                    return TaskResult.CANCELED, last_distance_remaining

            if self.found_all_survivors:
                return TaskResult.CANCELED, last_distance_remaining

            if not ignore_danger_cancel:
                if self.danger_detected_during_current_goal:
                    return TaskResult.CANCELED, last_distance_remaining

                if self.too_close_to_danger:
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
        self.danger_detected_during_current_goal = False
        self.too_close_to_danger = False
        self.cancel_requested = False

        x, y = point

        if label != "exit" and self.is_waypoint_near_danger(point):
            print(
                f"\nSkipping {label}: ({x:.2f}, {y:.2f}) "
                f"because it is near detected danger."
            )
            return False

        print(f"\nMoving to {label}: ({x:.2f}, {y:.2f})")

        goal_pose = create_pose(self, x, y)
        self.goToPose(goal_pose)

        ignore_danger_cancel = label == "exit"

        result, last_distance_remaining = self.wait_until_goal_finished_with_callbacks(
            ignore_danger_cancel=ignore_danger_cancel
        )

        if self.found_all_survivors and label != "exit":
            return False

        if self.danger_detected_during_current_goal and label != "exit":
            print("Current goal was canceled because danger was detected.")
            return False

        if self.too_close_to_danger and label != "exit":
            print("Current goal was canceled because robot got too close to danger.")
            return False

        reached = self.is_goal_accepted(
            result,
            last_distance_remaining,
            allowed_distance,
        )

        if reached:
            print(f"Reached {label}: ({x:.2f}, {y:.2f})")
            return True

        if result == TaskResult.CANCELED:
            print(f"Navigation canceled while moving to {label}: ({x:.2f}, {y:.2f})")
        elif result == TaskResult.FAILED:
            print(f"Failed to reach {label}: ({x:.2f}, {y:.2f})")
        else:
            print(f"Unknown result while moving to {label}: ({x:.2f}, {y:.2f})")

        return False

    def run(self):
        # waitUntilNav2Active()는 AMCL 대기 문제 때문에 사용하지 않음
        # Nav2 lifecycle이 active인 상태에서 이 파일을 실행해야 함
        print("Nav2 is already active. Starting map-based exploration patrol...")

        self.wait_for_map()

        # /amcl_pose가 아직 안 들어온 경우 시작 위치 기반 정렬이 약해질 수 있으므로 잠깐 대기
        print("Waiting briefly for /amcl_pose...")
        wait_count = 0
        while rclpy.ok() and self.current_robot_position is None and wait_count < 20:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_count += 1

        if self.current_robot_position is None:
            print("Warning: /amcl_pose not received yet. Ordering from (0.0, 0.0).")
        else:
            print(
                f"Current robot position: "
                f"({self.current_robot_position[0]:.2f}, "
                f"{self.current_robot_position[1]:.2f})"
            )

        self.patrol_waypoints = self.build_patrol_waypoints_from_map()

        if not self.patrol_waypoints:
            print("No patrol waypoints generated from map.")
            return

        print("\nExploration patrol started.")
        print(f"Required survivors: {REQUIRED_SURVIVOR_COUNT}")
        print(f"Total patrol waypoints: {len(self.patrol_waypoints)}")
        print(f"Exit candidates: {EXIT_CANDIDATES}")

        unvisited_waypoints = self.patrol_waypoints.copy()
        visited_count = 0

        while rclpy.ok() and unvisited_waypoints:
            if self.found_all_survivors:
                print("All survivors found. Leaving patrol loop.")
                break

            if self.current_robot_position is not None:
                next_point = min(
                    unvisited_waypoints,
                    key=lambda p: distance_2d(self.current_robot_position, p),
                )
            else:
                next_point = unvisited_waypoints[0]

            unvisited_waypoints.remove(next_point)
            visited_count += 1

            label = (
                f"patrol waypoint {visited_count}/"
                f"{len(self.patrol_waypoints)}"
            )

            if self.is_waypoint_near_danger(next_point):
                print(
                    f"\nSkipping {label}: "
                    f"({next_point[0]:.2f}, {next_point[1]:.2f}) "
                    f"because it is near detected danger."
                )
                continue

            reached = self.go_to_single_goal(
                next_point,
                label,
                NORMAL_CLOSE_ENOUGH_DISTANCE,
            )

            if self.found_all_survivors:
                print("All survivors found during current goal. Going to exit now.")
                break

            if not reached:
                print("Skipping or replanning from unreachable/danger waypoint.")

            print(
                f"Remaining unvisited waypoints: {len(unvisited_waypoints)}"
            )

        print("\nPatrol phase finished.")
        print(f"Detected survivors: {len(self.discovered_survivors)}")
        print(f"Detected danger zones: {len(self.discovered_dangers)}")

        if len(self.discovered_survivors) < REQUIRED_SURVIVOR_COUNT:
            print("Mission failed: not enough survivors detected.")
            print("All generated patrol waypoints were already visited or skipped.")
            return
        print("\nMoving to final exit...")

        exit_reached = False

        for i, exit_point in enumerate(EXIT_CANDIDATES, start=1):
            print(
                f"\nTrying exit candidate {i}/{len(EXIT_CANDIDATES)}: "
                f"({exit_point[0]:.2f}, {exit_point[1]:.2f})"
            )

            self.found_all_survivors = False
            self.danger_detected_during_current_goal = False
            self.too_close_to_danger = False
            self.cancel_requested = False

            print("Clearing costmaps before exit attempt...")
            self.clearAllCostmaps()

            wait_start = time.time()
            while rclpy.ok() and time.time() - wait_start < 1.0:
                rclpy.spin_once(self, timeout_sec=0.1)

            exit_reached = self.go_to_single_goal(
                exit_point,
                "exit",
                EXIT_CLOSE_ENOUGH_DISTANCE,
            )

            if exit_reached:
                print(f"Exit reached using candidate {i}.")
                break

            print(f"Exit candidate {i} failed. Trying next candidate...")

        print("\nNavigation finished.")

        if exit_reached:
            print("Mission succeeded: 3 survivors detected and exit reached.")
        else:
            print("Mission failed: exit was not reached.")

def main():
    rclpy.init()

    navigator = MapExplorationPatrol()
    navigator.run()

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()