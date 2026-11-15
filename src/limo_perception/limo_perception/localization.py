#!/usr/bin/env python3
"""Publish the robot pose used by planning from odometry."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")
        self.declare_parameter("odom_topic", "/odom")
        topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, topic, self.on_odom, 10)
        self.publisher = self.create_publisher(PoseStamped, "/localization/pose", 10)
        self.get_logger().info(f"localization_node listening on {topic}")

    def on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose
        # Keep the yaw in [-pi, pi] so downstream nodes share one convention.
        yaw = yaw_from_quaternion(pose.pose.orientation)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(pose)


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
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
