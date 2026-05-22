import math
import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from safe_path_planner import get_rescue_mission, build_nav_waypoints


NORMAL_CLOSE_ENOUGH_DISTANCE = 0.60
RESCUE_CLOSE_ENOUGH_DISTANCE = 0.20
FINAL_CLOSE_ENOUGH_DISTANCE = 0.30


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


def wait_until_goal_finished(navigator):
    last_distance_remaining = None

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

        if feedback:
            last_distance_remaining = feedback.distance_remaining
            print(f"Distance remaining: {last_distance_remaining:.2f} m")

    return navigator.getResult(), last_distance_remaining


def get_allowed_distance(is_rescue_goal, is_final_goal):
    if is_rescue_goal:
        return RESCUE_CLOSE_ENOUGH_DISTANCE

    if is_final_goal:
        return FINAL_CLOSE_ENOUGH_DISTANCE

    return NORMAL_CLOSE_ENOUGH_DISTANCE


def is_goal_accepted(result, last_distance_remaining, allowed_distance, is_rescue_goal):
    if result == TaskResult.SUCCEEDED:
        return True

    if is_rescue_goal:
        return False

    if result == TaskResult.FAILED:
        return (
            last_distance_remaining is not None
            and last_distance_remaining < allowed_distance
        )

    return False


def main():
    rclpy.init()

    navigator = BasicNavigator()

    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    full_path, rescue_targets, exit_point = get_rescue_mission()

    if full_path is None:
        print("No rescue mission path found.")
        rclpy.shutdown()
        return

    rescue_approach_points = [target["approach"] for target in rescue_targets]
    mandatory_points = rescue_approach_points + [exit_point]

    waypoints = build_nav_waypoints(
        full_path,
        mandatory_points,
        max_gap=3,
    )

    if waypoints:
        waypoints = waypoints[1:]

    print("\nRescue targets:")
    for i, target in enumerate(rescue_targets, start=1):
        print(
            f"Survivor {i}: survivor={target['survivor']}, "
            f"approach={target['approach']}"
        )

    print("\nWaypoints to follow:")
    print(waypoints)

    rescued_count = 0
    visited_rescue_points = set()
    final_exit_reached = False

    for index, point in enumerate(waypoints, start=1):
        x, y = point

        is_final_goal = index == len(waypoints)
        is_rescue_goal = point in rescue_approach_points

        if is_rescue_goal:
            print(
                f"\n[{index}/{len(waypoints)}] "
                f"Moving to survivor approach point: {point}"
            )

        elif is_final_goal:
            print(
                f"\n[{index}/{len(waypoints)}] "
                f"Moving to final exit: {point}"
            )

        else:
            print(f"\n[{index}/{len(waypoints)}] Moving to waypoint: {point}")

        goal_pose = create_pose(navigator, x, y)
        navigator.goToPose(goal_pose)

        result, last_distance_remaining = wait_until_goal_finished(navigator)

        allowed_distance = get_allowed_distance(
            is_rescue_goal,
            is_final_goal,
        )

        reached = is_goal_accepted(
            result,
            last_distance_remaining,
            allowed_distance,
            is_rescue_goal,
        )

        if not reached:
            if result == TaskResult.CANCELED:
                print(f"Navigation canceled at waypoint: {point}")
            elif result == TaskResult.FAILED:
                print(f"Failed to reach waypoint: {point}")
            else:
                print(f"Unknown result at waypoint: {point}")

            break

        if result == TaskResult.FAILED:
            print(f"Goal reached within acceptable distance: {point}")

        if is_rescue_goal and point not in visited_rescue_points:
            visited_rescue_points.add(point)
            rescued_count += 1

            print(
                f"Survivor rescued! "
                f"({rescued_count}/{len(rescue_targets)})"
            )

        elif is_final_goal:
            final_exit_reached = True
            print(f"\nFinal exit reached: {point}")

        else:
            print(f"Reached waypoint: {point}")

    mission_success = (
        rescued_count == len(rescue_targets)
        and final_exit_reached
    )

    print("\nNavigation finished.")
    print(f"Total rescued survivors: {rescued_count}/{len(rescue_targets)}")
    print(f"Final exit reached: {final_exit_reached}")

    if mission_success:
        print("Mission succeeded: all survivors were rescued and final exit was reached.")
    else:
        print("Mission failed.")

        if rescued_count != len(rescue_targets):
            print("Reason: not all survivors were rescued.")

        if not final_exit_reached:
            print("Reason: final exit was not reached.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()