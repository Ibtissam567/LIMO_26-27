#!/usr/bin/env python3
"""Cluster front lidar returns and publish obstacles plus a stop flag."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from limo_interfaces.msg import Obstacle, ObstacleArray


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("stop_distance", 0.55)
        self.declare_parameter("front_angle", 0.70)
        self.declare_parameter("max_range", 2.5)
        self.declare_parameter("cluster_gap", 0.25)
        topic = self.get_parameter("scan_topic").value
        self.create_subscription(LaserScan, topic, self.on_scan, 10)
        self.stop_pub = self.create_publisher(Bool, "/perception/obstacle_stop", 10)
        self.array_pub = self.create_publisher(ObstacleArray, "/perception/obstacles", 10)
        self.get_logger().info(f"obstacle_detector listening on {topic}")

    def on_scan(self, msg: LaserScan) -> None:
        points = front_points(
            msg,
            float(self.get_parameter("front_angle").value),
            float(self.get_parameter("max_range").value),
        )
        clusters = cluster_points(points, float(self.get_parameter("cluster_gap").value))
        stop_distance = float(self.get_parameter("stop_distance").value)
        array = ObstacleArray()
        array.header = msg.header
        blocked = False
        for cluster in clusters:
            obstacle = obstacle_from_cluster(cluster, stop_distance)
            array.obstacles.append(obstacle)
            blocked = blocked or obstacle.blocks_current_segment
        stop = Bool()
        stop.data = blocked
        self.stop_pub.publish(stop)
        self.array_pub.publish(array)


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


def cluster_points(points, gap):
    if not points:
        return []
    ordered = sorted(points, key=lambda item: math.atan2(item[1], item[0]))
    clusters = [[ordered[0]]]
    for point in ordered[1:]:
        previous = clusters[-1][-1]
        if math.hypot(point[0] - previous[0], point[1] - previous[1]) <= gap:
            clusters[-1].append(point)
        else:
            clusters.append([point])
    return [cluster for cluster in clusters if len(cluster) >= 2]


def obstacle_from_cluster(cluster, stop_distance):
    xs = [point[0] for point in cluster]
    ys = [point[1] for point in cluster]
    distances = [point[2] for point in cluster]
    nearest = min(distances)
    obstacle = Obstacle()
    obstacle.x = float(sum(xs) / len(xs))
    obstacle.y = float(sum(ys) / len(ys))
    obstacle.distance = float(nearest)
    span = max(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 0.05)
    obstacle.radius = float(span / 2.0)
    obstacle.blocks_current_segment = nearest < stop_distance and abs(obstacle.y) < 0.45
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
