#!/usr/bin/env python3
"""Keep the LIMO on the detected lane."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from limo_interfaces.msg import LaneInfo


class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")
        self.declare_parameter("speed", 0.25)
        self.declare_parameter("kp_lateral", 1.1)
        self.declare_parameter("kp_heading", 0.8)
        self.declare_parameter("max_turn", 0.8)
        self.declare_parameter("min_confidence", 0.4)
        self.create_subscription(LaneInfo, "/perception/lane", self.on_lane, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("lane_follower started")

    def on_lane(self, msg: LaneInfo) -> None:
        command = Twist()
        if msg.confidence < float(self.get_parameter("min_confidence").value):
            self.publisher.publish(command)
            return

        turn = (
            -float(self.get_parameter("kp_lateral").value) * msg.lateral_offset
            - float(self.get_parameter("kp_heading").value) * msg.heading_error
        )
        limit = float(self.get_parameter("max_turn").value)
        command.linear.x = float(self.get_parameter("speed").value)
        command.angular.z = max(-limit, min(limit, turn))
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
