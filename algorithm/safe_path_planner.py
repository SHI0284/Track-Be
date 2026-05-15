import heapq

GRID_SIZE = 10

robot_start = (0, 0)
person_pos = (1, 7)
exit_pos = (9, 9)

danger_points = {
    (5, 3), (5, 4), (6, 3)
}

obstacles = {
    (2, 2), (3, 2), (4, 5)
}


def distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def calculate_risk(point):
    if point in obstacles:
        return 999

    if point in danger_points:
        return 999

    min_danger_dist = min(distance(point, danger) for danger in danger_points)

    if min_danger_dist == 1:
        return 50
    elif min_danger_dist == 2:
        return 20
    else:
        return 1


def scan_area(robot_pos, scan_range=3):
    x, y = robot_pos
    scanned_points = []

    for i in range(x - scan_range, x + scan_range + 1):
        for j in range(y - scan_range, y + scan_range + 1):
            if 0 <= i < GRID_SIZE and 0 <= j < GRID_SIZE:
                scanned_points.append((i, j))

    return scanned_points


def mark_safe_points(scanned_points):
    safe_points = []

    for point in scanned_points:
        risk = calculate_risk(point)

        if risk <= 20:
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

        if 0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE:
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
    print("Robot is scanning around...")

    scanned_points = scan_area(robot_start)
    safe_points = mark_safe_points(scanned_points)

    print("Scanned points:")
    print(scanned_points)

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