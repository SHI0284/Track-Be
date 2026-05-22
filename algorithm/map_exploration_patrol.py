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
from std_msgs.msg import String


# ============================================================
# Mission settings
# ============================================================

REQUIRED_SURVIVOR_COUNT = 3

# maze.sdf 기준:
# 외곽은 대략 x=-10~10, y=-10~10
# 동쪽 벽이 아래쪽 전체를 막지 않기 때문에 오른쪽 아래를 출구 접근점으로 사용
EXIT_POINT = (8, -8)

# maze.sdf target layout transformed into the SLAM /map frame.
# 감지되기 전까지 기다리면 첫 순찰 경로가 불 옆을 지나갈 수 있다.
KNOWN_DANGER_POINTS = [
    (-2.25, 4.30),
    (-4.86, 8.38),
    (-0.87, -9.33),
    (-5.17, -7.59),
]

KNOWN_SURVIVOR_POINTS = [
    (-10.31, -6.63),
    (2.69, 6.27),
    (8.13, -2.50),
]

PRIORITY_SURVIVOR_VIEWPOINTS = [
    (-8.22, -4.40),
    (7.18, -1.65),
    (8.28, -4.40),
    (2.78, 3.85),
    (2.78, 8.25),
]

PRIORITY_VIEWPOINT_MATCH_DISTANCE = 0.9
SURVIVOR_INSPECTION_DISTANCE = 4.0

# /map에서 waypoint를 생성할 순찰 범위
# 외곽 벽과 너무 붙지 않도록 -9~9로 제한
PATROL_MIN_X = -9.0
PATROL_MAX_X = 9.0
PATROL_MIN_Y = -9.0
PATROL_MAX_Y = 9.0

# 후보 waypoint 생성 간격
# 최종 waypoint 개수는 coverage grid와 거리 필터로 줄어듦
WAYPOINT_SPACING_M = 0.6

COVERAGE_GRID_COLS = 10
COVERAGE_GRID_ROWS = 6

MAX_PATROL_WAYPOINTS = 200
MIN_DISTANCE_BETWEEN_PATROL_WAYPOINTS = 1.3

# 되돌아가는 움직임을 줄이기 위한 설정
BACKTRACK_PENALTY_DISTANCE = 3.0
BACKTRACK_PENALTY_WEIGHT = 4.0
DIRECTION_WEIGHT = 1.2

# 이미 지나간 수색 구역 주변은 다시 순찰하지 않기 위한 거리
VISITED_WAYPOINT_SKIP_DISTANCE = 1.8

# 위험 감지로 취소된 목표가 이 거리 안이면 다시 뒤로 미루지 않고 버림
REPLAN_DROP_DISTANCE_FROM_DANGER = 3.8
REPLAN_DROP_DISTANCE_FROM_ROBOT = 1.2

# 벽과 너무 가까운 waypoint 제외
SAFE_MARGIN_M = 0.40

# 구조자 중복 인식 방지 거리
SURVIVOR_DUPLICATE_DISTANCE = 2.5

# 산불/위험 구역 중복 인식 방지 거리
DANGER_DUPLICATE_DISTANCE = 1.5

# 산불 근처 waypoint는 순찰하지 않기 위한 거리
DANGER_WAYPOINT_SKIP_DISTANCE = 3.2

# goal이 안전해도 Nav2 경로가 산불 옆을 스치면 해당 goal을 버림
DANGER_PATH_SKIP_DISTANCE = 3.0

# 로봇이 산불에 이 거리보다 가까워지면 즉시 현재 goal 취소
DANGER_TOO_CLOSE_DISTANCE = 3.0

# 산불 발견 시 현재 goal을 취소해서 재계획 유도
CANCEL_GOAL_ON_DANGER = True

# 산불에 가까워졌을 때 현재 goal 취소
CANCEL_GOAL_WHEN_TOO_CLOSE_TO_DANGER = True

# waypoint 도착 허용 거리
NORMAL_CLOSE_ENOUGH_DISTANCE = 0.65
EXIT_CLOSE_ENOUGH_DISTANCE = 0.35

# OccupancyGrid 기준
# 0: free, 100: occupied, -1: unknown
FREE_THRESHOLD = 20
UNKNOWN_AS_BLOCKED = True

# 산불 감지로 인한 goal 취소는 너무 자주 하지 않도록 제한
DANGER_CANCEL_COOLDOWN_SEC = 2.0

# 산불 근접 경고도 너무 자주 취소하지 않도록 제한
DANGER_PROXIMITY_CANCEL_COOLDOWN_SEC = 2.0

# thermal image만으로 판단한 강한 위험 신호의 goal 취소 제한
# RED_ONLY는 위치 확정 전 후보로만 보고 여기서는 goal을 취소하지 않음
THERMAL_DANGER_CANCEL_COOLDOWN_SEC = 12.0


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


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MapExplorationPatrol(BasicNavigator):
    def __init__(self):
        super().__init__()

        self.map_msg = None
        self.map_received = False
        self.patrol_waypoints = []

        self.current_robot_position = None
        self.current_robot_yaw = 0.0

        self.discovered_survivors = []
        self.found_all_survivors = False

        self.discovered_dangers = list(KNOWN_DANGER_POINTS)
        self.visited_patrol_points = []
        self.dropped_patrol_points = []
        self.danger_detected_during_current_goal = False
        self.too_close_to_danger = False
        self.cancel_requested = False

        self.last_danger_cancel_time = 0.0
        self.last_proximity_cancel_time = 0.0
        self.last_thermal_cancel_time = 0.0

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        amcl_qos = QoSProfile(depth=1)
        amcl_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        amcl_qos.reliability = ReliabilityPolicy.RELIABLE

        detected_point_qos = QoSProfile(depth=10)
        detected_point_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        detected_point_qos.reliability = ReliabilityPolicy.RELIABLE

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
            amcl_qos,
        )

        self.create_subscription(
            PointStamped,
            "/detected_survivor_points",
            self.survivor_callback,
            detected_point_qos,
        )

        self.create_subscription(
            PointStamped,
            "/detected_danger_points",
            self.danger_callback,
            detected_point_qos,
        )

        self.create_subscription(
            String,
            "/thermal_color_detection",
            self.thermal_detection_callback,
            10,
        )

    # ============================================================
    # Map handling
    # ============================================================

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

        ordered_waypoints = self.prioritize_survivor_viewpoints(ordered_waypoints)

        print(f"Coverage-selected waypoints: {len(selected_waypoints)}")
        print(f"After distance filtering: {len(spaced_waypoints)}")
        print(f"Generated patrol waypoints: {len(ordered_waypoints)}")

        print("\nFinal patrol waypoint order:")
        for i, point in enumerate(ordered_waypoints, start=1):
            print(f"{i}: ({point[0]:.2f}, {point[1]:.2f})")

        return ordered_waypoints

    def prioritize_survivor_viewpoints(self, waypoints):
        priority_points = []
        remaining_points = waypoints.copy()

        for target_point in PRIORITY_SURVIVOR_VIEWPOINTS:
            nearest_point = None
            nearest_distance = float("inf")

            for point in remaining_points:
                distance = distance_2d(point, target_point)

                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_point = point

            if (
                nearest_point is not None
                and nearest_distance <= PRIORITY_VIEWPOINT_MATCH_DISTANCE
            ):
                priority_points.append(nearest_point)
                remaining_points.remove(nearest_point)

        return priority_points + remaining_points

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

            # 한 줄은 왼쪽 → 오른쪽, 다음 줄은 오른쪽 → 왼쪽
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
            ordered.append(current)
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
        # 기본은 현재 위치에서 가까운 점을 선호
        score = distance_2d(current, candidate)

        # 이미 방문한 점 근처로 되돌아가는 후보에는 penalty
        for visited_point in visited:
            d = distance_2d(candidate, visited_point)

            if d < BACKTRACK_PENALTY_DISTANCE:
                score += BACKTRACK_PENALTY_WEIGHT * (
                    BACKTRACK_PENALTY_DISTANCE - d
                )

        # 직전 이동 방향과 반대로 꺾이는 후보에는 penalty
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

                # dot이 -1에 가까울수록 완전히 뒤로 돌아가는 방향
                if dot < -0.3:
                    score += DIRECTION_WEIGHT * abs(dot)

        return score

    # ============================================================
    # Current robot pose / AMCL
    # ============================================================

    def amcl_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.current_robot_position = (x, y)
        self.current_robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

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

    # ============================================================
    # Survivor detection
    # ============================================================

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

    # ============================================================
    # Danger / wildfire detection
    # ============================================================

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

    def is_path_near_danger(self, path):
        if path is None:
            return False, None, None

        for pose in path.poses:
            path_point = (
                pose.pose.position.x,
                pose.pose.position.y,
            )

            for danger_point in self.discovered_dangers:
                distance = distance_2d(path_point, danger_point)

                if distance < DANGER_PATH_SKIP_DISTANCE:
                    return True, danger_point, distance

        return False, None, None

    def is_waypoint_in_replan_drop_zone(self, point):
        for danger_point in self.discovered_dangers:
            if distance_2d(point, danger_point) < REPLAN_DROP_DISTANCE_FROM_DANGER:
                return True

        if self.current_robot_position is not None:
            if (
                distance_2d(point, self.current_robot_position)
                < REPLAN_DROP_DISTANCE_FROM_ROBOT
            ):
                return True

        return False

    def is_waypoint_near_visited_area(self, point):
        for visited_point in self.visited_patrol_points:
            if distance_2d(point, visited_point) < VISITED_WAYPOINT_SKIP_DISTANCE:
                return True

        return False

    def mark_patrol_point_done(self, point, reason):
        if reason == "reached":
            self.visited_patrol_points.append(point)
        else:
            self.dropped_patrol_points.append(point)

    def should_skip_patrol_point(self, point):
        if self.is_waypoint_near_danger(point):
            return True, "near detected danger"

        if self.is_waypoint_near_visited_area(point):
            return True, "near already searched area"

        return False, ""

    def remove_now_unsafe_or_redundant_waypoints(self, waypoints):
        kept = []
        removed_count = 0

        for point in waypoints:
            should_skip, _ = self.should_skip_patrol_point(point)

            if should_skip:
                self.mark_patrol_point_done(point, "filtered")
                removed_count += 1
            else:
                kept.append(point)

        if removed_count > 0:
            print(
                f"Filtered {removed_count} waypoint(s) that are now unsafe "
                f"or already searched."
            )

        return kept

    def thermal_detection_callback(self, msg):
        detection = msg.data

        if detection == "NONE":
            return

        if detection.startswith("RED_ONLY"):
            return

        if not (
            detection.startswith("FIRE")
            or detection.startswith("HEAT_UNKNOWN")
        ):
            return

        now = time.time()

        if now - self.last_thermal_cancel_time < THERMAL_DANGER_CANCEL_COOLDOWN_SEC:
            return

        self.last_thermal_cancel_time = now
        self.danger_detected_during_current_goal = True
        self.cancel_requested = True

        print(f"\n[Thermal danger detected] {detection}")
        print("Confirmed thermal danger. Canceling current goal to replan...")

        # thermal topic에는 map 좌표가 없으므로 현재 로봇 위치를 danger로 저장하지 않는다.
        # 자기 위치를 danger로 등록하면 AMCL callback에서 즉시 근접 위험으로 판단되어
        # 다음 회피 goal까지 계속 취소되는 루프가 생길 수 있다.

    # ============================================================
    # Navigation
    # ============================================================

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

            if self.found_all_survivors and not ignore_danger_cancel:
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
            return False, "near_danger"

        print(f"\nMoving to {label}: ({x:.2f}, {y:.2f})")

        goal_yaw = self.get_goal_yaw(point)
        goal_pose = create_pose(self, x, y, goal_yaw)

        if label != "exit" and self.current_robot_position is not None:
            start_pose = create_pose(
                self,
                self.current_robot_position[0],
                self.current_robot_position[1],
                self.current_robot_yaw,
            )
            planned_path = self.getPath(start_pose, goal_pose, use_start=True)
            if planned_path is None:
                print(
                    f"\nSkipping {label}: ({x:.2f}, {y:.2f}) "
                    "because Nav2 could not compute a safe path."
                )
                return False, "path_failed"

            is_unsafe_path, danger_point, danger_distance = self.is_path_near_danger(
                planned_path
            )

            if is_unsafe_path:
                print(
                    f"\nSkipping {label}: ({x:.2f}, {y:.2f}) "
                    f"because the planned path passes too close to wildfire "
                    f"at ({danger_point[0]:.2f}, {danger_point[1]:.2f}) "
                    f"with clearance {danger_distance:.2f} m."
                )
                return False, "near_danger_path"

        self.goToPose(goal_pose)

        ignore_danger_cancel = label == "exit"

        result, last_distance_remaining = self.wait_until_goal_finished_with_callbacks(
            ignore_danger_cancel=ignore_danger_cancel
        )

        if self.found_all_survivors and label != "exit":
            return False, "survivors_found"

        if self.danger_detected_during_current_goal and label != "exit":
            print("Current goal was canceled because danger was detected.")
            return False, "danger_detected"

        if self.too_close_to_danger and label != "exit":
            print("Current goal was canceled because robot got too close to danger.")
            return False, "too_close_to_danger"

        reached = self.is_goal_accepted(
            result,
            last_distance_remaining,
            allowed_distance,
        )

        if reached:
            print(f"Reached {label}: ({x:.2f}, {y:.2f})")
            return True, "reached"

        if result == TaskResult.CANCELED:
            print(f"Navigation canceled while moving to {label}: ({x:.2f}, {y:.2f})")
            return False, "canceled"
        elif result == TaskResult.FAILED:
            print(f"Failed to reach {label}: ({x:.2f}, {y:.2f})")
            return False, "failed"
        else:
            print(f"Unknown result while moving to {label}: ({x:.2f}, {y:.2f})")
            return False, "unknown"

    def get_goal_yaw(self, point):
        nearest_survivor = None
        nearest_distance = float("inf")

        for survivor_point in KNOWN_SURVIVOR_POINTS:
            if self.is_survivor_already_found(survivor_point):
                continue

            distance = distance_2d(point, survivor_point)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_survivor = survivor_point

        if (
            nearest_survivor is not None
            and nearest_distance <= SURVIVOR_INSPECTION_DISTANCE
        ):
            dx = nearest_survivor[0] - point[0]
            dy = nearest_survivor[1] - point[1]
            return math.atan2(dy, dx)

        return 0.0

    def is_survivor_already_found(self, survivor_point):
        for found_point in self.discovered_survivors:
            if distance_2d(found_point, survivor_point) < SURVIVOR_DUPLICATE_DISTANCE:
                return True

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
        print(f"Exit point: ({EXIT_POINT[0]:.2f}, {EXIT_POINT[1]:.2f})")

        # 생성 단계에서 이미 backtracking을 줄인 순서로 정렬했으므로 그 순서를 유지한다.
        unvisited_waypoints = self.patrol_waypoints.copy()
        visited_count = 0

        while rclpy.ok() and unvisited_waypoints:
            if self.found_all_survivors:
                print("All survivors found. Leaving patrol loop.")
                break

            unvisited_waypoints = self.remove_now_unsafe_or_redundant_waypoints(
                unvisited_waypoints
            )

            if not unvisited_waypoints:
                break

            next_point = unvisited_waypoints[0]

            label = (
                f"patrol waypoint {visited_count + 1}/"
                f"{len(self.patrol_waypoints)}"
            )

            should_skip, skip_reason = self.should_skip_patrol_point(next_point)

            if should_skip:
                print(
                    f"\nSkipping {label}: "
                    f"({next_point[0]:.2f}, {next_point[1]:.2f}) "
                    f"because it is {skip_reason}."
                )
                unvisited_waypoints.pop(0)
                self.mark_patrol_point_done(next_point, "skipped")
                visited_count += 1
                continue

            reached, reason = self.go_to_single_goal(
                next_point,
                label,
                NORMAL_CLOSE_ENOUGH_DISTANCE,
            )

            if self.found_all_survivors:
                print("All survivors found during current goal. Going to exit now.")
                break

            if reached:
                unvisited_waypoints.pop(0)
                self.mark_patrol_point_done(next_point, "reached")
                visited_count += 1
            elif reason in ("near_danger", "near_danger_path", "path_failed", "failed"):
                unvisited_waypoints.pop(0)
                self.mark_patrol_point_done(next_point, reason)
                visited_count += 1
                print("Dropping unreachable or unsafe waypoint.")
            elif reason in ("danger_detected", "too_close_to_danger"):
                postponed_point = unvisited_waypoints.pop(0)
                if self.is_waypoint_in_replan_drop_zone(postponed_point):
                    self.mark_patrol_point_done(postponed_point, reason)
                    visited_count += 1
                    print(
                        "Dropping current waypoint after danger replanning "
                        "because it is near danger or already effectively searched."
                    )
                else:
                    unvisited_waypoints.append(postponed_point)
                    print("Postponing current waypoint after danger replanning.")
            else:
                unvisited_waypoints.pop(0)
                self.mark_patrol_point_done(next_point, reason)
                visited_count += 1
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

        self.found_all_survivors = False
        self.danger_detected_during_current_goal = False
        self.too_close_to_danger = False
        self.cancel_requested = False

        exit_reached, _ = self.go_to_single_goal(
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

    navigator = MapExplorationPatrol()
    navigator.run()

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
