#!/usr/bin/env python3
"""Fly the drone to a goal and report how the flight went.

    ros2 run drone_bringup mission_metrics.py 19.5 0
    ros2 run drone_bringup mission_metrics.py 19.5 0 --csv runs.csv --label "unknown_cost 1"

Sends the goal like RViz's 2D Goal Pose, waits for the planner's "Goal reached" (or a
timeout), and reports the flight: time, path length, peak speed and height, number of
replans, and the closest the drone came to an obstacle. Positions are base_link in the world
frame, from TF. Obstacles are the static boxes of the world's SDF (no rotations, as in our
worlds), and clearance is measured from the drone's center to their surfaces; below the
drone's radius it touched.

--csv appends one row per run (with a header for a new file), for the benchmark table.
Needs a sim running with use_sim_time; run it with use_sim_time too (the default here).
"""

import argparse
import csv
import math
import os
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener

DRONE_RADIUS = 0.39  # [m] x500 rotors 0.25 m from the center, plus 0.14 m props


def world_boxes(world):
    """(name, center xyz, half size xyz) of every box collision in the world's SDF."""
    path = os.path.join(get_package_share_directory("drone_sim"), "worlds", world + ".sdf")
    boxes = []
    for model in ET.parse(path).getroot().iter("model"):
        origin = _pose(model.find("pose"))
        for col in model.iter("collision"):
            size = col.find("geometry/box/size")
            if size is None:
                continue
            offset = _pose(col.find("pose"))
            center = [o + d for o, d in zip(origin, offset)]
            half = [float(v) / 2.0 for v in size.text.split()]
            boxes.append((model.get("name"), center, half))
    return boxes


def _pose(element):
    if element is None:
        return [0.0, 0.0, 0.0]
    return [float(v) for v in element.text.split()[:3]]


def clearance(p, boxes):
    """Distance from p to the nearest box surface, and that box's name."""
    best = (math.inf, None)
    for name, c, h in boxes:
        d = math.sqrt(sum(max(abs(pi - ci) - hi, 0.0) ** 2 for pi, ci, hi in zip(p, c, h)))
        best = min(best, (d, name))
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("--world", default="factory", help="world in drone_sim/worlds")
    parser.add_argument("--timeout", type=float, default=180.0, help="[s] of wall time")
    parser.add_argument("--csv", help="append the result to this CSV file")
    parser.add_argument("--label", default="", help="free text for the CSV row")
    parser.add_argument("--frame", default="odom")
    parser.add_argument("--body-frame", default="base_link")
    args = parser.parse_args()

    boxes = world_boxes(args.world)
    rclpy.init()
    node = rclpy.create_node(
        "mission_metrics", parameter_overrides=[Parameter("use_sim_time", value=True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    logs = []
    node.create_subscription(
        Log, "/rosout",
        lambda m: logs.append(m.msg) if m.name in ("path_planner", "trajectory_server") else None,
        100)
    goal_pub = node.create_publisher(PoseStamped, "/goal_pose", 10)

    def position():
        try:
            t = tf_buffer.lookup_transform(args.frame, args.body_frame, rclpy.time.Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)

    # Sim time reads 0 until the first /clock message: wait for it too, or the flight time
    # would count from the start of the simulation.
    def ready():
        return goal_pub.get_subscription_count() > 0 and position() is not None and \
            node.get_clock().now().nanoseconds > 0

    wall_start = time.time()
    while not ready() and time.time() - wall_start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not ready():
        node.get_logger().error("No drone pose on TF or no /clock; is the sim running?")
        return 1

    goal = PoseStamped()
    goal.header.frame_id = args.frame
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = args.x, args.y
    goal_pub.publish(goal)
    start_time = node.get_clock().now()
    wall_start = time.time()
    node.get_logger().info(f"Goal ({args.x:.2f}, {args.y:.2f}) sent")

    path_length, max_speed, max_z = 0.0, 0.0, -math.inf
    min_clear = (math.inf, None)
    last = position()
    last_time = start_time
    reached = False
    while time.time() - wall_start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        if any("Goal reached" in m for m in logs):
            reached = True
            break
        p, now = position(), node.get_clock().now()
        dt = (now - last_time).nanoseconds * 1e-9
        if p is None or dt < 0.05:
            continue
        step = math.dist(p, last)
        path_length += step
        max_speed = max(max_speed, step / dt)
        max_z = max(max_z, p[2])
        min_clear = min(min_clear, clearance(p, boxes))
        last, last_time = p, now

    flight_time = (node.get_clock().now() - start_time).nanoseconds * 1e-9
    end = position()
    error = math.dist(end, (args.x, args.y, end[2]))
    replans = sum("blocked" in m and "replanning" in m for m in logs)
    shorter = sum("Shorter way found" in m for m in logs)
    stops = sum("braking to a stop" in m or "braking now" in m for m in logs)
    touched = min_clear[0] < DRONE_RADIUS

    print(f"{'reached' if reached else 'NOT reached (timeout)'} in {flight_time:.1f} s, "
          f"{path_length:.1f} m flown, end {error:.2f} m from the goal (horizontally)")
    print(f"peak speed {max_speed:.2f} m/s, peak height {max_z:.2f} m; "
          f"{replans} blocked replans, {shorter} shorter ways, {stops} stops (brakes)")
    print(f"min clearance {min_clear[0]:.2f} m to {min_clear[1]}"
          + (f" -- TOUCHED (drone radius {DRONE_RADIUS} m)" if touched else ""))

    if args.csv:
        row = {
            "label": args.label, "goal_x": args.x, "goal_y": args.y, "reached": reached,
            "flight_time_s": round(flight_time, 2), "path_length_m": round(path_length, 2),
            "end_error_m": round(error, 3), "max_speed_mps": round(max_speed, 2),
            "max_height_m": round(max_z, 2), "replans_blocked": replans,
            "replans_shorter": shorter, "stops": stops,
            "min_clearance_m": round(min_clear[0], 3), "closest_obstacle": min_clear[1],
        }
        new_file = not os.path.exists(args.csv)
        with open(args.csv, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        print(f"appended to {args.csv}")
    rclpy.shutdown()
    return 0 if reached and not touched else 1


if __name__ == "__main__":
    raise SystemExit(main())
