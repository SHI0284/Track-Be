import heapq
import xml.etree.ElementTree as ET
from pathlib import Path

GRID_SIZE = 20

BASE_DIR = Path(__file__).resolve().parent.parent
SDF_PATH = BASE_DIR / "maps" / "maze.sdf"

ROBOT_START = (0, 0)
EXIT_POS = (8, -8)

BLOCKED_COST = 999
DANGER_BLOCK_RADIUS = 1
DANGER_HIGH_COST_RADIUS = 4
SURVIVOR_APPROACH_RADIUS = 1


def world_to_grid(x, y):
    return round(x), round(y)


def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def is_inside_grid(point):
    x, y = point
    return -GRID_SIZE <= x <= GRID_SIZE and -GRID_SIZE <= y <= GRID_SIZE


def read_color_from_model(model):
    for tag in model.iter():
        if tag.tag not in ["ambient", "diffuse"]:
            continue

        if not tag.text:
            continue

        values = tag.text.strip().split()

        if len(values) < 3:
            continue

        r = float(values[0])
        g = float(values[1])
        b = float(values[2])

        if r > 0.7 and g < 0.3 and b < 0.3:
            return "red"

        if g > 0.7 and r < 0.3 and b < 0.3:
            return "green"

    return "unknown"


def parse_pose(model):
    pose = model.find("pose")

    if pose is None or pose.text is None:
        return None

    values = pose.text.strip().split()

    if len(values) < 2:
        return None

    return float(values[0]), float(values[1])


def load_walls_from_sdf(sdf_path):
    wall_points = set()

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for model in root.iter("model"):
        model_name = model.attrib.get("name", "").lower()

        is_wall_model = any(
            keyword in model_name
            for keyword in ["wall", "box", "obstacle", "border", "narrow_path"]
        )

        if not is_wall_model:
            continue

        pose = parse_pose(model)

        if pose is None:
            continue

        center_x, center_y = pose

        size_tag = model.find(".//box/size")

        if size_tag is None or size_tag.text is None:
            wall_points.add(world_to_grid(center_x, center_y))
            continue

        size_values = size_tag.text.strip().split()

        if len(size_values) < 2:
            wall_points.add(world_to_grid(center_x, center_y))
            continue

        size_x = float(size_values[0])
        size_y = float(size_values[1])

        min_x = round(center_x - size_x / 2)
        max_x = round(center_x + size_x / 2)
        min_y = round(center_y - size_y / 2)
        max_y = round(center_y + size_y / 2)

        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                point = (x, y)

                if is_inside_grid(point):
                    wall_points.add(point)

    return wall_points


def load_points_from_sdf(sdf_path):
    danger_points = set()
    survivor_points = []

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for model in root.iter("model"):
        model_name = model.attrib.get("name", "").lower()

        pose = parse_pose(model)

        if pose is None:
            continue

        grid_pos = world_to_grid(*pose)
        color = read_color_from_model(model)

        is_danger = (
            color == "red"
            or "red" in model_name
            or "danger" in model_name
            or "fire" in model_name
        )

        is_survivor = (
            color == "green"
            or "green" in model_name
            or "person" in model_name
            or "people" in model_name
            or "survivor" in model_name
        )

        if is_danger:
            danger_points.add(grid_pos)

        elif is_survivor:
            survivor_points.append(grid_pos)

    return danger_points, survivor_points


OBSTACLES = load_walls_from_sdf(SDF_PATH)
DANGER_POINTS, SURVIVOR_POINTS = load_points_from_sdf(SDF_PATH)


def get_min_danger_distance(point):
    if not DANGER_POINTS:
        return None

    return min(
        manhattan_distance(point, danger)
        for danger in DANGER_POINTS
    )


def calculate_risk(point):
    if point in OBSTACLES:
        return BLOCKED_COST

    min_danger_dist = get_min_danger_distance(point)

    if min_danger_dist is None:
        return 1

    if min_danger_dist <= DANGER_BLOCK_RADIUS:
        return BLOCKED_COST

    if min_danger_dist <= DANGER_HIGH_COST_RADIUS:
        return 100 + (DANGER_HIGH_COST_RADIUS - min_danger_dist) * 50

    return 1


def is_movable(point):
    return is_inside_grid(point) and calculate_risk(point) < BLOCKED_COST


def get_neighbors(point):
    x, y = point

    candidates = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1),
    ]

    return [candidate for candidate in candidates if is_movable(candidate)]


def find_path(start, goal):
    if not is_movable(start):
        print(f"Start point is not movable: {start}")
        return None

    if not is_movable(goal):
        print(f"Goal point is not movable: {goal}")
        return None

    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {start: None}
    cost_so_far = {start: 0}

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current == goal:
            break

        for next_point in get_neighbors(current):
            new_cost = cost_so_far[current] + calculate_risk(next_point)

            if next_point not in cost_so_far or new_cost < cost_so_far[next_point]:
                cost_so_far[next_point] = new_cost
                priority = new_cost + manhattan_distance(goal, next_point)
                heapq.heappush(open_list, (priority, next_point))
                came_from[next_point] = current

    if goal not in came_from:
        return None

    path = []
    current = goal

    while current is not None:
        path.append(current)
        current = came_from[current]

    path.reverse()
    return path


def get_approach_candidates(survivor_pos, radius=SURVIVOR_APPROACH_RADIUS):
    sx, sy = survivor_pos
    candidates = []

    for r in range(1, radius + 1):
        candidates.extend(
            [
                (sx + r, sy),
                (sx - r, sy),
                (sx, sy + r),
                (sx, sy - r),
                (sx + r, sy + r),
                (sx + r, sy - r),
                (sx - r, sy + r),
                (sx - r, sy - r),
            ]
        )

    candidates = list(dict.fromkeys(candidates))

    safe_candidates = []

    for candidate in candidates:
        if not is_movable(candidate):
            continue

        if manhattan_distance(candidate, survivor_pos) > radius:
            continue

        safe_candidates.append(candidate)

    return safe_candidates


def find_best_approach_path(current_pos, survivor_pos):
    candidates = get_approach_candidates(survivor_pos)

    best_point = None
    best_path = None
    best_cost = float("inf")

    for candidate in candidates:
        path = find_path(current_pos, candidate)

        if path is None:
            continue

        distance_to_survivor = manhattan_distance(candidate, survivor_pos)
        danger_distance = get_min_danger_distance(candidate)

        danger_penalty = 0

        if danger_distance is not None and danger_distance <= DANGER_HIGH_COST_RADIUS:
            danger_penalty = 200

        cost = (
            len(path)
            + distance_to_survivor * 30
            + danger_penalty
        )

        if cost < best_cost:
            best_cost = cost
            best_point = candidate
            best_path = path

    return best_point, best_path


def find_next_survivor_route(current_pos, remaining_survivors):
    best_survivor = None
    best_approach_point = None
    best_path = None
    best_cost = float("inf")

    for survivor in remaining_survivors:
        approach_point, path = find_best_approach_path(current_pos, survivor)

        if path is None:
            continue

        cost = len(path)

        if cost < best_cost:
            best_cost = cost
            best_survivor = survivor
            best_approach_point = approach_point
            best_path = path

    return best_survivor, best_approach_point, best_path


def get_rescue_mission():
    current_pos = ROBOT_START
    remaining_survivors = SURVIVOR_POINTS.copy()

    full_path = [ROBOT_START]
    rescue_targets = []

    print("\nRescue mission calculation started.")
    print("All rescue targets:", remaining_survivors)

    while remaining_survivors:
        survivor, approach_point, path_to_survivor = find_next_survivor_route(
            current_pos,
            remaining_survivors,
        )

        if path_to_survivor is None:
            print("No safe path to remaining survivors.")
            print("Remaining survivors:", remaining_survivors)
            return None, None, None

        print(f"\nNext survivor target: {survivor}")
        print(f"Approach point for survivor {survivor}: {approach_point}")

        full_path += path_to_survivor[1:]

        rescue_targets.append(
            {
                "survivor": survivor,
                "approach": approach_point,
            }
        )

        current_pos = approach_point
        remaining_survivors.remove(survivor)

        print("Remaining survivors:", remaining_survivors)

    print(f"\nAll survivors visited. Moving to exit: {EXIT_POS}")

    path_to_exit = find_path(current_pos, EXIT_POS)

    if path_to_exit is None:
        print(f"No safe path to exit: {EXIT_POS}")
        return None, None, None

    full_path += path_to_exit[1:]

    return full_path, rescue_targets, EXIT_POS


def simplify_path_keep_points(path, keep_points):
    if path is None or len(path) <= 2:
        return path

    keep_points = set(keep_points)
    simplified = [path[0]]

    prev_dx = path[1][0] - path[0][0]
    prev_dy = path[1][1] - path[0][1]

    for i in range(1, len(path) - 1):
        current = path[i]
        next_point = path[i + 1]

        dx = next_point[0] - current[0]
        dy = next_point[1] - current[1]

        direction_changed = dx != prev_dx or dy != prev_dy
        must_keep = current in keep_points

        if direction_changed or must_keep:
            if simplified[-1] != current:
                simplified.append(current)

        prev_dx = dx
        prev_dy = dy

    if simplified[-1] != path[-1]:
        simplified.append(path[-1])

    return simplified


def build_nav_waypoints(path, mandatory_points, max_gap=3):
    if path is None:
        return None

    mandatory_points = set(mandatory_points)
    waypoints = [path[0]]
    last_kept = path[0]

    for point in path[1:]:
        must_keep = point in mandatory_points
        far_enough = manhattan_distance(last_kept, point) >= max_gap

        if must_keep or far_enough:
            if waypoints[-1] != point:
                waypoints.append(point)
                last_kept = point

    if waypoints[-1] != path[-1]:
        waypoints.append(path[-1])

    return waypoints


def get_safe_escape_path():
    full_path, _, _ = get_rescue_mission()
    return full_path


def main():
    print("Loaded danger points:")
    print(DANGER_POINTS)

    print("\nLoaded survivor points:")
    print(SURVIVOR_POINTS)

    print("\nLoaded obstacle count:")
    print(len(OBSTACLES))

    full_path, rescue_targets, exit_point = get_rescue_mission()

    if full_path is None:
        print("\nNo safe rescue mission path found.")
        return

    mandatory_points = [target["approach"] for target in rescue_targets]
    mandatory_points.append(exit_point)

    waypoints = build_nav_waypoints(
        full_path,
        mandatory_points,
        max_gap=3,
    )

    print("\nRescue targets:")
    for i, target in enumerate(rescue_targets, start=1):
        print(
            f"Survivor {i}: survivor={target['survivor']}, "
            f"approach={target['approach']}"
        )

    print("\nFull rescue path:")
    print(full_path)

    print("\nNav2 waypoints:")
    print(waypoints)


if __name__ == "__main__":
    main()