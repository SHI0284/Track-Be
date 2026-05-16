import heapq
import xml.etree.ElementTree as ET
from pathlib import Path

GRID_SIZE = 20

BASE_DIR = Path(__file__).resolve().parent.parent
SDF_PATH = BASE_DIR / "maze.sdf"

robot_start = (0, 0)
exit_pos = (9, 9)

obstacles = set()


def world_to_grid(x, y):
    return (round(x), round(y))


def distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def read_color_from_model(model):
    color_texts = []

    for tag in model.iter():
        if tag.tag in ["ambient", "diffuse"] and tag.text:
            color_texts.append(tag.text.strip())

    for color in color_texts:
        values = color.split()

        if len(values) >= 3:
            r = float(values[0])
            g = float(values[1])
            b = float(values[2])

            if r > 0.7 and g < 0.3 and b < 0.3:
                return "red"

            if g > 0.7 and r < 0.3 and b < 0.3:
                return "green"

    return "unknown"


def load_points_from_sdf(sdf_path):
    danger_points = set()
    people_points = []

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for model in root.iter("model"):
        model_name = model.attrib.get("name", "").lower()

        pose = model.find("pose")
        if pose is None or pose.text is None:
            continue

        pose_values = pose.text.strip().split()

        if len(pose_values) < 2:
            continue

        x = float(pose_values[0])
        y = float(pose_values[1])

        grid_pos = world_to_grid(x, y)

        color = read_color_from_model(model)

        if color == "red" or "red" in model_name or "danger" in model_name or "fire" in model_name:
            danger_points.add(grid_pos)

        elif color == "green" or "green" in model_name or "person" in model_name or "people" in model_name:
            people_points.append(grid_pos)

    return danger_points, people_points


danger_points, people_points = load_points_from_sdf(SDF_PATH)

if len(people_points) > 0:
    person_pos = people_points[0]
else:
    person_pos = (1, 7)


def calculate_risk(point):
    if point in obstacles:
        return 999

    if point in danger_points:
        return 999

    if len(danger_points) == 0:
        return 1

    min_danger_dist = min(distance(point, danger) for danger in danger_points)

    if min_danger_dist == 1:
        return 80
    elif min_danger_dist == 2:
        return 40
    elif min_danger_dist == 3:
        return 15
    else:
        return 1


def scan_area(robot_pos, scan_range=4):
    x, y = robot_pos
    scanned_points = []

    for i in range(x - scan_range, x + scan_range + 1):
        for j in range(y - scan_range, y + scan_range + 1):
            if -GRID_SIZE <= i <= GRID_SIZE and -GRID_SIZE <= j <= GRID_SIZE:
                scanned_points.append((i, j))

    return scanned_points


def mark_safe_points(scanned_points):
    safe_points = []

    for point in scanned_points:
        risk = calculate_risk(point)

        if risk <= 15:
            safe_points.append(point)

    return safe_points


def get_neighbors(point):
    x, y = point

    candidates = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1)
    ]

    neighbors = []

    for candidate in candidates:
        cx, cy = candidate

        if -GRID_SIZE <= cx <= GRID_SIZE and -GRID_SIZE <= cy <= GRID_SIZE:
            if calculate_risk(candidate) < 999:
                neighbors.append(candidate)

    return neighbors


def find_escape_path(start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    cost_so_far = {}

    came_from[start] = None
    cost_so_far[start] = 0

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current == goal:
            break

        for next_point in get_neighbors(current):
            new_cost = cost_so_far[current] + calculate_risk(next_point)

            if next_point not in cost_so_far or new_cost < cost_so_far[next_point]:
                cost_so_far[next_point] = new_cost
                priority = new_cost + distance(goal, next_point)
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


def main():
    print("Loaded danger points from red cylinders:")
    print(danger_points)

    print("\nLoaded people points from green cylinders:")
    print(people_points)

    print("\nSelected person position:")
    print(person_pos)

    print("\nRobot is scanning around...")
    scanned_points = scan_area(robot_start)
    safe_points = mark_safe_points(scanned_points)

    print("\nMarked safe points:")
    print(safe_points)

    print("\nFinding path: Robot -> Person")
    path_to_person = find_escape_path(robot_start, person_pos)
    print(path_to_person)

    print("\nFinding path: Person -> Exit")
    path_to_exit = find_escape_path(person_pos, exit_pos)
    print(path_to_exit)

    if path_to_person is None or path_to_exit is None:
        print("\nNo safe escape path found.")
        return

    final_path = path_to_person + path_to_exit[1:]

    print("\nFinal safe escape path:")
    print(final_path)


if __name__ == "__main__":
    main()