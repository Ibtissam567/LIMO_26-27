#!/usr/bin/env python3
"""Cluster front lidar returns and publish obstacles plus a stop flag."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from limo_interfaces.msg import Obstacle, ObstacleArray


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("stop_distance", 0.40)
        self.declare_parameter("front_angle", 0.55)
        self.declare_parameter("max_range", 2.5)
        self.declare_parameter("cluster_gap", 0.25)  # legacy alias for base
        self.declare_parameter("cluster_gap_base", 0.12)
        self.declare_parameter("cluster_gap_k", 2.5)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("block_half_width", 0.28)
        self.declare_parameter("max_obstacle_radius", 0.45)
        self.declare_parameter("wall_stop_distance", 0.28)
        self.declare_parameter("stop_confirm_scans", 3)
        topic = self.get_parameter("scan_topic").value
        self._last_scan_time = None
        self._block_streak = 0
        self.create_subscription(LaserScan, topic, self.on_scan, qos_profile_sensor_data)
        self.stop_pub = self.create_publisher(Bool, "/perception/obstacle_stop", 10)
        self.array_pub = self.create_publisher(ObstacleArray, "/perception/obstacles", 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(f"obstacle_detector listening on {topic}")

    def on_scan(self, msg: LaserScan) -> None:
        self._last_scan_time = self.get_clock().now()
        points = front_points(
            msg,
            float(self.get_parameter("front_angle").value),
            float(self.get_parameter("max_range").value),
        )
        base = float(self.get_parameter("cluster_gap_base").value)
        if base <= 0.0:
            base = float(self.get_parameter("cluster_gap").value)
        clusters = cluster_points(
            points,
            base,
            float(self.get_parameter("cluster_gap_k").value),
            float(msg.angle_increment),
            int(self.get_parameter("min_cluster_points").value),
        )
        stop_distance = float(self.get_parameter("stop_distance").value)
        block_half_width = float(self.get_parameter("block_half_width").value)
        max_radius = float(self.get_parameter("max_obstacle_radius").value)
        wall_stop = float(self.get_parameter("wall_stop_distance").value)
        array = ObstacleArray()
        array.header = msg.header
        blocked_now = False
        for cluster in clusters:
            obstacle = obstacle_from_cluster(
                cluster,
                stop_distance,
                block_half_width,
                max_radius,
                wall_stop,
            )
            array.obstacles.append(obstacle)
            blocked_now = blocked_now or obstacle.blocks_current_segment

        needed = max(1, int(self.get_parameter("stop_confirm_scans").value))
        if blocked_now:
            self._block_streak += 1
        else:
            self._block_streak = 0
        stop = Bool()
        stop.data = self._block_streak >= needed
        self.stop_pub.publish(stop)
        self.array_pub.publish(array)

    def _watchdog(self) -> None:
        timeout = float(self.get_parameter("scan_timeout").value)
        if timeout <= 0.0:
            return
        if self._last_scan_time is None:
            stop = Bool()
            stop.data = True
            self.stop_pub.publish(stop)
            return
        age = (self.get_clock().now() - self._last_scan_time).nanoseconds * 1e-9
        if age > timeout:
            stop = Bool()
            stop.data = True
            self.stop_pub.publish(stop)
            empty = ObstacleArray()
            self.array_pub.publish(empty)


def front_points(msg, front_angle, max_range):
    points = []
    angle = msg.angle_min
    for distance in msg.ranges:
        valid = (
            math.isfinite(distance)
            and msg.range_min < distance < min(msg.range_max, max_range)
            and abs(angle) <= front_angle
        )
        if valid:
            points.append((distance * math.cos(angle), distance * math.sin(angle), distance))
        angle += msg.angle_increment
    return points


def adaptive_gap(range_m, gap_base, gap_k, angle_increment):
    """gap(r) = d0 + k * r * Δθ — points spread with range."""
    return float(gap_base + gap_k * max(range_m, 0.0) * abs(angle_increment))


def cluster_points(points, gap_base, gap_k=2.5, angle_increment=0.01, min_points=3):
    if not points:
        return []
    ordered = sorted(points, key=lambda item: math.atan2(item[1], item[0]))
    clusters = [[ordered[0]]]
    for point in ordered[1:]:
        previous = clusters[-1][-1]
        mid_range = 0.5 * (previous[2] + point[2])
        gap = adaptive_gap(mid_range, gap_base, gap_k, angle_increment)
        if math.hypot(point[0] - previous[0], point[1] - previous[1]) <= gap:
            clusters[-1].append(point)
        else:
            clusters.append([point])
    return [cluster for cluster in clusters if len(cluster) >= max(int(min_points), 2)]


def obstacle_from_cluster(
    cluster,
    stop_distance,
    block_half_width=0.28,
    max_obstacle_radius=0.45,
    wall_stop_distance=0.28,
):
    xs = [point[0] for point in cluster]
    ys = [point[1] for point in cluster]
    distances = [point[2] for point in cluster]
    nearest = min(distances)
    # Use the nearest return's lateral offset for the corridor check (not the centroid).
    nearest_point = min(cluster, key=lambda item: item[2])
    nearest_y = float(nearest_point[1])
    obstacle = Obstacle()
    obstacle.x = float(sum(xs) / len(xs))
    obstacle.y = float(sum(ys) / len(ys))
    obstacle.distance = float(nearest)
    span = max(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 0.05)
    obstacle.radius = float(span / 2.0)
    in_corridor = abs(nearest_y) < block_half_width
    if obstacle.radius > max_obstacle_radius:
        # Large cluster = wall / curb along a turn — only emergency-stop when very close.
        obstacle.blocks_current_segment = in_corridor and nearest < wall_stop_distance
    else:
        obstacle.blocks_current_segment = in_corridor and nearest < stop_distance
    return obstacle


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
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
