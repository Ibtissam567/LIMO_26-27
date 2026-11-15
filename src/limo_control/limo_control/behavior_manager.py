#!/usr/bin/env python3
"""Decide the mission state from perception. Does not publish /cmd_vel."""

import rclpy
from limo_interfaces.msg import MissionState, TrafficSignArray
from rclpy.node import Node
from std_msgs.msg import Bool


class BehaviorManager(Node):
    def __init__(self):
        super().__init__("behavior_manager")
        self.obstacle = False
        self.sign_stop = False
        self.red_light = False
        self.state = ""
        self.publisher = self.create_publisher(MissionState, "/mission/state", 10)
        self.create_subscription(Bool, "/perception/obstacle_stop", self.on_obstacle, 10)
        self.create_subscription(Bool, "/perception/sign_stop", self.on_sign, 10)
        self.create_subscription(TrafficSignArray, "/perception/signs", self.on_signs, 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info("behavior_manager started")

    def on_obstacle(self, msg: Bool) -> None:
        self.obstacle = bool(msg.data)

    def on_sign(self, msg: Bool) -> None:
        self.sign_stop = bool(msg.data)

    def on_signs(self, msg: TrafficSignArray) -> None:
        self.red_light = any(
            sign.type == "FEU_ROUGE" and sign.confidence >= 0.4 for sign in msg.signs
        )

    def tick(self) -> None:
        if self.obstacle:
            state = "STOP_OBSTACLE"
        elif self.sign_stop:
            state = "STOP_SIGN"
        elif self.red_light:
            state = "STOP_RED_LIGHT"
        else:
            state = "FOLLOW_ROAD"
        if state != self.state:
            self.get_logger().info(f"state {state}")
            self.state = state
        message = MissionState()
        message.state = state
        message.current_segment = "road"
        message.goal_id = "lane"
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
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
