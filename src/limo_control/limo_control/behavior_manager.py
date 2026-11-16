#!/usr/bin/env python3
"""Decide the mission state from perception. Does not publish /cmd_vel.

Adds a small recovery FSM: STOP_BOUNDARY stuck → RECOVER_REVERSE → REPLANNING → FOLLOW_ROAD.
Reverse is allowed only when rear /scan clearance and sensor freshness are verified.
"""

import math

import rclpy
from limo_interfaces.msg import LaneInfo, MissionEvent, MissionState, TrafficSignArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from limo_control.motion_history import MotionHistory, MotionSample


class BehaviorManager(Node):
    def __init__(self):
        super().__init__("behavior_manager")
        self.declare_parameter("unsafe_confirm_frames", 3)
        self.declare_parameter("history_size", 50)
        self.declare_parameter("blocked_time", 1.8)
        self.declare_parameter("blocked_speed", 0.02)
        self.declare_parameter("blocked_progress", 0.04)
        self.declare_parameter("rear_clearance", 0.35)
        self.declare_parameter("rear_half_angle", 0.70)
        self.declare_parameter("scan_max_age", 0.5)
        self.declare_parameter("odom_max_age", 0.5)
        self.declare_parameter("recovery_timeout", 6.0)
        self.declare_parameter("replan_hold", 1.0)
        self.declare_parameter("min_recover_clearance", 0.0)
        self.declare_parameter("recovery_enable", True)

        self.obstacle = False
        self.sign_stop = False
        self.red_light = False
        self.footprint_safe = True
        self._unsafe_streak = 0
        self.state = ""
        self._lane = None
        self._odom = None
        self._odom_stamp = None
        self._scan = None
        self._scan_stamp = None
        self._history = MotionHistory(int(self.get_parameter("history_size").value))
        self._boundary_since = None
        self._boundary_pose = None
        self._recovery_since = None
        self._replan_since = None

        self.publisher = self.create_publisher(MissionState, "/mission/state", 10)
        self.event_pub = self.create_publisher(MissionEvent, "/mission/event", 10)
        self.create_subscription(Bool, "/perception/obstacle_stop", self.on_obstacle, 10)
        self.create_subscription(Bool, "/perception/sign_stop", self.on_sign, 10)
        self.create_subscription(TrafficSignArray, "/perception/signs", self.on_signs, 10)
        self.create_subscription(LaneInfo, "/perception/lane", self.on_lane, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
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

    def on_lane(self, msg: LaneInfo) -> None:
        self._lane = msg
        safe = bool(getattr(msg, "footprint_safe", True))
        needed = max(1, int(self.get_parameter("unsafe_confirm_frames").value))
        if safe:
            self._unsafe_streak = 0
            self.footprint_safe = True
        else:
            self._unsafe_streak += 1
            self.footprint_safe = self._unsafe_streak < needed

    def on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_stamp = self.get_clock().now()
        if self._lane is None:
            return
        yaw = _yaw(msg.pose.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._history.push(
            MotionSample(
                x=float(msg.pose.pose.position.x),
                y=float(msg.pose.pose.position.y),
                yaw=yaw,
                v=float(msg.twist.twist.linear.x),
                w=float(msg.twist.twist.angular.z),
                t=stamp,
                left_clearance=float(getattr(self._lane, "left_clearance", 0.0)),
                right_clearance=float(getattr(self._lane, "right_clearance", 0.0)),
                footprint_safe=bool(getattr(self._lane, "footprint_safe", False)),
                mission=self.state or "FOLLOW_ROAD",
            )
        )

    def on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_stamp = self.get_clock().now()

    def tick(self) -> None:
        state = self._decide_state()
        if state != self.state:
            self.get_logger().info(f"state {state}")
            self._emit_event(state)
            self.state = state
        message = MissionState()
        message.state = state
        message.current_segment = "road"
        message.goal_id = "lane" if state != "RECOVER_REVERSE" else "recover"
        self.publisher.publish(message)

    def _decide_state(self) -> str:
        # Active recovery / replan take priority over re-entering STOP_BOUNDARY.
        if self.state == "RECOVER_REVERSE":
            return self._tick_recover()
        if self.state == "REPLANNING":
            return self._tick_replan()

        if self.obstacle:
            self._reset_boundary_timer()
            return "STOP_OBSTACLE"
        if not self.footprint_safe:
            return self._tick_boundary()
        if self.sign_stop:
            self._reset_boundary_timer()
            return "STOP_SIGN"
        if self.red_light:
            self._reset_boundary_timer()
            return "STOP_RED_LIGHT"
        self._reset_boundary_timer()
        return "FOLLOW_ROAD"

    def _tick_boundary(self) -> str:
        now = self.get_clock().now()
        if self._boundary_since is None:
            self._boundary_since = now
            if self._odom is not None:
                self._boundary_pose = (
                    float(self._odom.pose.pose.position.x),
                    float(self._odom.pose.pose.position.y),
                )
            return "STOP_BOUNDARY"

        elapsed = (now - self._boundary_since).nanoseconds * 1e-9
        if not bool(self.get_parameter("recovery_enable").value):
            return "STOP_BOUNDARY"
        if elapsed < float(self.get_parameter("blocked_time").value):
            return "STOP_BOUNDARY"
        if not self._is_blocked():
            return "STOP_BOUNDARY"
        if not self._recovery_precheck_ok():
            self._emit_event("RECOVERY_DENIED", "rear/scan/odom/history unsafe")
            # Restart timer so we do not spam checks every tick.
            self._boundary_since = now
            return "STOP_BOUNDARY"

        self._recovery_since = now
        self._emit_event("RECOVER_REVERSE", "verified reverse recovery")
        return "RECOVER_REVERSE"

    def _tick_recover(self) -> str:
        now = self.get_clock().now()
        if self._recovery_since is None:
            self._recovery_since = now
        elapsed = (now - self._recovery_since).nanoseconds * 1e-9
        if elapsed > float(self.get_parameter("recovery_timeout").value):
            self._emit_event("RECOVERY_TIMEOUT", "remain stopped")
            self._recovery_since = None
            self._boundary_since = now
            return "STOP_BOUNDARY"
        if not self._rear_clear() or not self._sensors_fresh():
            self._emit_event("RECOVERY_ABORT", "lost rear clearance or stale sensors")
            self._recovery_since = None
            self._boundary_since = now
            return "STOP_BOUNDARY"
        # Resume when footprint is safe again and we moved away from the block pose.
        if self.footprint_safe and self._moved_from_boundary():
            self._replan_since = now
            self._recovery_since = None
            self._boundary_since = None
            return "REPLANNING"
        return "RECOVER_REVERSE"

    def _tick_replan(self) -> str:
        now = self.get_clock().now()
        if self._replan_since is None:
            self._replan_since = now
        elapsed = (now - self._replan_since).nanoseconds * 1e-9
        if elapsed >= float(self.get_parameter("replan_hold").value):
            self._replan_since = None
            if self.footprint_safe and not self.obstacle:
                return "FOLLOW_ROAD"
            return "STOP_BOUNDARY"
        return "REPLANNING"

    def _is_blocked(self) -> bool:
        if self._odom is None or self._boundary_pose is None:
            return False
        speed = abs(float(self._odom.twist.twist.linear.x))
        if speed > float(self.get_parameter("blocked_speed").value):
            return False
        x = float(self._odom.pose.pose.position.x)
        y = float(self._odom.pose.pose.position.y)
        progress = math.hypot(x - self._boundary_pose[0], y - self._boundary_pose[1])
        if progress > float(self.get_parameter("blocked_progress").value):
            return False
        return self._history.last_safe(
            float(self.get_parameter("min_recover_clearance").value)
        ) is not None

    def _recovery_precheck_ok(self) -> bool:
        if not self._sensors_fresh():
            return False
        if not self._rear_clear():
            return False
        return self._history.last_safe(
            float(self.get_parameter("min_recover_clearance").value)
        ) is not None

    def _sensors_fresh(self) -> bool:
        now = self.get_clock().now()
        if self._scan_stamp is None or self._odom_stamp is None:
            return False
        scan_age = (now - self._scan_stamp).nanoseconds * 1e-9
        odom_age = (now - self._odom_stamp).nanoseconds * 1e-9
        return (
            scan_age <= float(self.get_parameter("scan_max_age").value)
            and odom_age <= float(self.get_parameter("odom_max_age").value)
        )

    def _rear_clear(self) -> bool:
        """True if no close returns in the rear sector of /scan."""
        msg = self._scan
        if msg is None:
            return False
        half = float(self.get_parameter("rear_half_angle").value)
        need = float(self.get_parameter("rear_clearance").value)
        angle = msg.angle_min
        nearest = None
        for distance in msg.ranges:
            # Rear around ±π.
            wrapped = math.atan2(math.sin(angle), math.cos(angle))
            if abs(abs(wrapped) - math.pi) <= half or abs(wrapped) >= (math.pi - half):
                if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                    nearest = distance if nearest is None else min(nearest, distance)
            angle += msg.angle_increment
        if nearest is None:
            # Empty rear sector: treat as clear in open sim.
            return True
        return nearest >= need

    def _moved_from_boundary(self) -> bool:
        if self._odom is None or self._boundary_pose is None:
            return self.footprint_safe
        x = float(self._odom.pose.pose.position.x)
        y = float(self._odom.pose.pose.position.y)
        return math.hypot(x - self._boundary_pose[0], y - self._boundary_pose[1]) >= 0.12

    def _reset_boundary_timer(self) -> None:
        self._boundary_since = None
        self._boundary_pose = None

    def _emit_event(self, event_type: str, description: str = "") -> None:
        event = MissionEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.type = event_type
        event.description = description
        if self._odom is not None:
            event.pose = self._odom.pose.pose
        self.event_pub.publish(event)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
