#!/usr/bin/env python3
"""ROS integration: drive behavior_manager into RECOVER_REVERSE with synthetic topics."""

import math
import time
import unittest

import rclpy
from geometry_msgs.msg import Quaternion
from limo_interfaces.msg import LaneInfo, MissionState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from limo_control.behavior_manager import BehaviorManager


def _quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RecoveryFsmRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_enters_recover_reverse(self):
        behavior = BehaviorManager()
        behavior.set_parameters(
            [
                rclpy.parameter.Parameter("blocked_time", value=0.4),
                rclpy.parameter.Parameter("recovery_timeout", value=5.0),
                rclpy.parameter.Parameter("unsafe_confirm_frames", value=1),
                rclpy.parameter.Parameter("scan_max_age", value=2.0),
                rclpy.parameter.Parameter("odom_max_age", value=2.0),
            ]
        )
        helper = Node("recovery_fsm_helper")
        lane_pub = helper.create_publisher(LaneInfo, "/perception/lane", 10)
        odom_pub = helper.create_publisher(Odometry, "/odom", 10)
        scan_pub = helper.create_publisher(LaserScan, "/scan", 10)
        obs_pub = helper.create_publisher(Bool, "/perception/obstacle_stop", 10)
        sign_pub = helper.create_publisher(Bool, "/perception/sign_stop", 10)
        states = []

        def on_state(msg: MissionState):
            states.append(msg.state)

        helper.create_subscription(MissionState, "/mission/state", on_state, 10)

        def spin_both(seconds: float):
            end = time.time() + seconds
            while time.time() < end:
                rclpy.spin_once(behavior, timeout_sec=0.05)
                rclpy.spin_once(helper, timeout_sec=0.05)

        def publish(x, y, yaw, v, safe, left_c, right_c):
            now = behavior.get_clock().now().to_msg()
            lane = LaneInfo()
            lane.header.stamp = now
            lane.footprint_safe = safe
            lane.left_clearance = left_c
            lane.right_clearance = right_c
            lane.confidence = 0.9
            lane_pub.publish(lane)
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation = _quat(yaw)
            odom.twist.twist.linear.x = v
            odom_pub.publish(odom)
            scan = LaserScan()
            scan.header.stamp = now
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = math.pi / 180.0
            n = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
            scan.range_min = 0.05
            scan.range_max = 8.0
            scan.ranges = [5.0] * n
            scan_pub.publish(scan)
            obs_pub.publish(Bool(data=False))
            sign_pub.publish(Bool(data=False))

        # Build safe history while moving.
        for i in range(15):
            publish(-0.7, 0.2 + 0.02 * i, 1.57, 0.05, True, 0.2, 0.2)
            spin_both(0.08)

        # Boundary stop: unsafe, no progress.
        for _ in range(25):
            publish(-0.55, 0.55, 1.4, 0.0, False, -0.05, 0.2)
            spin_both(0.08)

        uniq = set(states)
        self.assertIn("FOLLOW_ROAD", uniq)
        self.assertIn("STOP_BOUNDARY", uniq)
        self.assertIn(
            "RECOVER_REVERSE",
            uniq,
            msg=f"states seen={states[-20:]}",
        )
        behavior.destroy_node()
        helper.destroy_node()


if __name__ == "__main__":
    unittest.main(verbosity=2)
