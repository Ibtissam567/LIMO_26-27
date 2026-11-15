#!/usr/bin/env python3
"""Plan a forward path on the dark road and send it to the polar controller."""

import heapq
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool
from PIL import Image

MAP_PATH = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
PLANE_X = 4.88
PLANE_Y = 6.2
CELL = 0.15


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


class RoadMap:
    def __init__(self):
        image = Image.open(MAP_PATH).convert("RGB")
        self.width, self.height = image.size
        self.pixels = image.load()
        self.grid = set()
        x = -2.3
        while x < 2.4:
            y = -1.4
            while y < 2.9:
                cell = (round(x, 3), round(y, 3))
                if self.free(*cell):
                    self.grid.add(cell)
                y += CELL
            x += CELL

    def white_fraction(self, x, y):
        hits = 0
        for ix in range(-2, 3):
            for iy in range(-2, 3):
                px = x + ix * 0.04
                py = y + iy * 0.04
                u = (px + PLANE_X / 2.0) / PLANE_X
                v = 1.0 - (py + PLANE_Y / 2.0) / PLANE_Y
                if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
                    hits += 1
                    continue
                red, green, blue = self.pixels[
                    min(self.width - 1, int(u * self.width)),
                    min(self.height - 1, int(v * self.height)),
                ]
                if red > 170 and green > 170 and blue > 170:
                    hits += 1
        return hits / 25.0

    def free(self, x, y):
        if self.white_fraction(x, y) > 0.15:
            return False
        for step in range(8):
            angle = 2.0 * math.pi * step / 8.0
            if self.white_fraction(x + 0.09 * math.cos(angle), y + 0.09 * math.sin(angle)) > 0.45:
                return False
        return True

    def nearest(self, point):
        best = None
        best_distance = 99.0
        for cell in self.grid:
            distance = math.hypot(cell[0] - point[0], cell[1] - point[1])
            if distance < best_distance:
                best = cell
                best_distance = distance
        return best

    def plan(self, start_xy, yaw, goal_xy):
        start_cell = self.nearest(start_xy)
        goal_cell = self.nearest(goal_xy)
        if start_cell is None or goal_cell is None:
            return []
        start_bin = int(round(wrap(yaw) / (math.pi / 4.0))) % 8
        start = (start_cell[0], start_cell[1], start_bin)
        goal_key = goal_cell
        openq = [(0.0, start)]
        came = {}
        cost = {start: 0.0}
        found = None
        while openq:
            _, current = heapq.heappop(openq)
            if (current[0], current[1]) == goal_key:
                found = current
                break
            x, y, heading = current
            for dx, dy in ((CELL, 0.0), (-CELL, 0.0), (0.0, CELL), (0.0, -CELL)):
                nxt_xy = (round(x + dx, 3), round(y + dy, 3))
                if nxt_xy not in self.grid:
                    continue
                direction = math.atan2(dy, dx)
                turn = abs(wrap(direction - heading * math.pi / 4.0))
                if turn > math.pi / 2.0 + 0.2:
                    continue
                nxt = (nxt_xy[0], nxt_xy[1], int(round(wrap(direction) / (math.pi / 4.0))) % 8)
                new_cost = cost[current] + CELL * (1.0 + 0.8 * turn)
                if new_cost < cost.get(nxt, 1e9):
                    cost[nxt] = new_cost
                    came[nxt] = current
                    estimate = new_cost + math.hypot(nxt[0] - goal_cell[0], nxt[1] - goal_cell[1])
                    heapq.heappush(openq, (estimate, nxt))
        if found is None:
            return []
        path = [found]
        while path[-1] != start:
            path.append(came[path[-1]])
        path.reverse()
        return self._simplify([(item[0], item[1]) for item in path])

    def _chord_ok(self, start, goal):
        steps = max(1, int(math.hypot(goal[0] - start[0], goal[1] - start[1]) / 0.05))
        for index in range(steps + 1):
            ratio = index / steps
            if not self.free(
                start[0] + ratio * (goal[0] - start[0]),
                start[1] + ratio * (goal[1] - start[1]),
            ):
                return False
        return True

    def _simplify(self, path):
        kept = [path[0]]
        index = 0
        while index < len(path) - 1:
            farthest = index + 1
            for probe in range(index + 1, len(path)):
                if math.hypot(path[probe][0] - path[index][0], path[probe][1] - path[index][1]) > 0.45:
                    break
                if self._chord_ok(path[index], path[probe]):
                    farthest = probe
            kept.append(path[farthest])
            index = farthest
        if kept[-1] != path[-1]:
            kept.append(path[-1])
        return kept


class RoadPlanner(Node):
    def __init__(self):
        super().__init__("road_planner")
        self.declare_parameter("goal_x", 1.88)
        self.declare_parameter("goal_y", 0.45)
        self.declare_parameter("reach_distance", 0.15)
        self.declare_parameter("timeout", 90.0)
        self.pose = None
        self.points = []
        self.planned = False
        self.index = 0
        self.hold = 0
        self.deadline = None
        self.block_count = 0
        self.road = None
        self.publisher = self.create_publisher(Path, "/planning/route", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Bool, "/perception/obstacle_stop", self.on_obstacle, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info("road planner waiting for the robot pose")

    def on_odom(self, msg: Odometry) -> None:
        self.pose = msg
        if self.planned:
            return
        self.planned = True
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        road = RoadMap()
        self.road = road
        self.points = road.plan((position.x, position.y), yaw, goal)
        self.deadline = self.get_clock().now()
        if not self.points:
            self.get_logger().error("no on-road path to the goal")
            return
        listed = ", ".join(f"({x:.2f}, {y:.2f})" for x, y in self.points)
        self.get_logger().info(
            f"path from ({position.x:.2f}, {position.y:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f}): {listed}"
        )

    def on_obstacle(self, msg: Bool) -> None:
        if not msg.data or self.pose is None or self.road is None:
            self.block_count = 0
            return
        self.block_count += 1
        if self.block_count < 12:
            return
        self.block_count = 0
        position = self.pose.pose.pose.position
        yaw = yaw_from_quaternion(self.pose.pose.pose.orientation)
        blocked = (
            round(position.x + 0.45 * math.cos(yaw), 3),
            round(position.y + 0.45 * math.sin(yaw), 3),
        )
        self.road.grid = {
            cell
            for cell in self.road.grid
            if math.hypot(cell[0] - blocked[0], cell[1] - blocked[1]) > 0.4
        }
        goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self.points = self.road.plan((position.x, position.y), yaw, goal)
        self.index = 0
        self.hold = 0
        self.get_logger().info("replanned around the obstacle")

    def tick(self) -> None:
        if self.pose is None or not self.points or self.index >= len(self.points):
            return
        self._publish_route()
        x, y = self.points[self.index]
        robot = self.pose.pose.pose.position
        distance = math.hypot(x - robot.x, y - robot.y)
        if distance < float(self.get_parameter("reach_distance").value):
            self.hold += 1
        else:
            self.hold = 0
        if self.hold >= 8:
            self.get_logger().info(f"POINT {self.index + 1} REACHED, error {distance:.3f} m")
            self.index += 1
            self.hold = 0
            self.deadline = self.get_clock().now()
            if self.index >= len(self.points):
                self.get_logger().info("ROUTE FINISHED")
            return
        if self.deadline is None or self.deadline.nanoseconds == 0:
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                self.deadline = now
            return
        elapsed = (self.get_clock().now() - self.deadline).nanoseconds / 1e9
        if elapsed < 0.0:
            self.deadline = self.get_clock().now()
            return
        if elapsed > float(self.get_parameter("timeout").value):
            self.get_logger().info(f"POINT {self.index + 1} is in a turn, replanning")
            self._replan()
            return

    def _replan(self) -> None:
        if self.pose is None or self.road is None:
            return
        position = self.pose.pose.pose.position
        yaw = yaw_from_quaternion(self.pose.pose.pose.orientation)
        goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self.points = self.road.plan((position.x, position.y), yaw, goal)
        self.index = 0
        self.hold = 0
        self.deadline = self.get_clock().now()
        self.get_logger().info("replanned from the current pose")

    def _publish_route(self) -> None:
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for index, (px, py) in enumerate(self.points[self.index :]):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            nxt = self.points[min(self.index + index + 1, len(self.points) - 1)]
            pose.pose.orientation = quaternion_from_yaw(math.atan2(nxt[1] - py, nxt[0] - px))
            path.poses.append(pose)
        self.publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = RoadPlanner()
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
