#!/usr/bin/env python3
"""Send fixed goals, from a straight line to a curve, and report if each is reached."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
except ImportError:  # pragma: no cover
    EntityState = None
    SetEntityState = None


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def line_points(x0, y0, yaw, distance, count):
    points = []
    for index in range(1, count + 1):
        travelled = distance * index / count
        points.append((
            x0 + travelled * math.cos(yaw),
            y0 + travelled * math.sin(yaw),
            yaw,
        ))
    return points


def corner_points(x0, y0, yaw):
    right = yaw - math.pi / 2.0
    corner = (
        x0 + 1.5 * math.cos(yaw),
        y0 + 1.5 * math.sin(yaw),
        right,
    )
    finish = (
        corner[0] + 1.5 * math.cos(right),
        corner[1] + 1.5 * math.sin(right),
        right,
    )
    return [corner, finish]


def avoid_points(x0, y0, yaw):
    # Points measured on the track texture. They stay on the dark road and
    # pass the cylinder on the left only after the lane opens up.
    # A one-metre offset from the start crosses the white shoulder.
    del x0, y0
    return [
        (-1.00, 0.50, yaw),
        (-1.15, 0.95, yaw),
        (-1.32, 1.22, yaw),
        (-1.22, 1.70, yaw),
        (-1.10, 2.00, yaw),
    ]


def far_points(x0, y0, yaw):
    # On-road route from the upper junction to the right-hand straight.
    # Straight to that goal crosses the white islands, so the car follows the loop.
    del x0, y0, yaw
    waypoints = (
        (-1.20, 1.55),
        (-1.70, 1.20),
        (-1.85, 0.55),
        (-1.85, -0.15),
        (-1.40, -0.60),
        (-0.50, -0.60),
        (0.40, -0.60),
        (1.20, -0.72),
        (1.55, -0.62),
        (1.82, -0.25),
        (1.88, 0.15),
        (1.88, 0.45),
    )
    points = []
    for index, (x, y) in enumerate(waypoints):
        if index + 1 < len(waypoints):
            nxt = waypoints[index + 1]
            heading = math.atan2(nxt[1] - y, nxt[0] - x)
        else:
            heading = math.pi / 2.0
        points.append((x, y, heading))
    return points


def curve_points(x0, y0, yaw):
    radius = 1.0
    center_x = x0 + radius * math.sin(yaw)
    center_y = y0 - radius * math.cos(yaw)
    points = []
    for step in range(1, 5):
        turn = math.pi / 2.0 * step / 4.0
        angle = yaw + math.pi / 2.0 - turn
        points.append((
            center_x + radius * math.cos(angle),
            center_y + radius * math.sin(angle),
            yaw - turn,
        ))
    return points


class TrajectoryTester(Node):
    def __init__(self):
        super().__init__("trajectory_tester")
        self.declare_parameter("shape", "line")
        self.declare_parameter("reach_distance", 0.10)
        self.declare_parameter("timeout", 40.0)
        self.pose = None
        self.index = 0
        self.hold = 0
        self.started = False
        self.points = []
        self.deadline = None
        self.goal_pub = self.create_publisher(PoseStamped, "/mission/goal", 10)
        self.path_pub = self.create_publisher(Path, "/planning/trajectory", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(0.1, self.tick)
        self.marker = None
        if SetEntityState is not None:
            self.marker = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.get_logger().info("trajectory tester waiting for the robot pose")

    def on_odom(self, msg: Odometry) -> None:
        self.pose = msg
        if self.started:
            return
        self.started = True
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        shape = str(self.get_parameter("shape").value)
        if shape == "corner":
            self.points = corner_points(x, y, yaw)
        elif shape == "curve":
            self.points = curve_points(x, y, yaw)
        elif shape == "avoid":
            self.points = avoid_points(x, y, yaw)
        elif shape == "far":
            self.points = far_points(x, y, yaw)
        elif shape == "demo":
            self.points = avoid_points(x, y, yaw) + far_points(x, y, yaw)
        else:
            self.points = line_points(x, y, yaw, 2.5, 1)
        now = self.get_clock().now()
        self.deadline = now if now.nanoseconds > 0 else None
        goal = self.points[0]
        listed = ", ".join(f"({px:.2f}, {py:.2f})" for px, py, _ in self.points)
        self.get_logger().info(
            f"test '{shape}' start ({x:.2f}, {y:.2f}) goals {listed}"
        )

    def tick(self) -> None:
        if not self.started or self.index >= len(self.points):
            return
        x, y, yaw = self.points[self.index]
        self.publish_goal(x, y, yaw)
        robot = self.pose.pose.pose.position
        distance = math.hypot(x - robot.x, y - robot.y)
        if distance < float(self.get_parameter("reach_distance").value):
            self.hold += 1
        else:
            self.hold = 0
        if self.hold >= 10:
            limit = float(self.get_parameter("reach_distance").value)
            self.get_logger().info(
                f"POINT {self.index + 1} REACHED, error {distance:.3f} m, limit {limit:.2f} m"
            )
            self.index += 1
            self.hold = 0
            self.deadline = self.get_clock().now()
            if self.index >= len(self.points):
                self.get_logger().info("TEST FINISHED: all points reached")
            return
        now = self.get_clock().now()
        if self.deadline is None or self.deadline.nanoseconds == 0:
            if now.nanoseconds > 0:
                self.deadline = now
            return
        elapsed = (now - self.deadline).nanoseconds / 1e9
        if elapsed < 0.0:
            self.deadline = now
            return
        if elapsed > float(self.get_parameter("timeout").value):
            self.get_logger().error(
                f"POINT {self.index + 1} FAILED, still {distance:.3f} m away"
            )
            self.index = len(self.points)

    def publish_goal(self, x, y, yaw) -> None:
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = quaternion_from_yaw(yaw)
        self.goal_pub.publish(goal)

        path = Path()
        path.header = goal.header
        start = self.pose.pose.pose.position
        for step in range(6):
            pose = PoseStamped()
            pose.header = goal.header
            ratio = step / 5.0
            pose.pose.position.x = start.x + ratio * (x - start.x)
            pose.pose.position.y = start.y + ratio * (y - start.y)
            pose.pose.orientation = goal.pose.orientation
            path.poses.append(pose)
        self.path_pub.publish(path)
        if self.marker is not None and self.marker.service_is_ready():
            state = EntityState()
            state.name = "goal"
            state.pose = goal.pose
            state.pose.position.z = 0.02
            request = SetEntityState.Request()
            request.state = state
            self.marker.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTester()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
