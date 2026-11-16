#!/usr/bin/env python3
"""Polar pose controller from last year's LIMO lab.

Law, with the gains of fichierlimo.m:
    v = sens * k_rho * rho
    omega = sens * (k_alpha * alpha + k_beta * beta)

sens is -1 when the goal is behind the robot (alpha outside +/- pi/2),
as in the Simulink block fcn(alpha). Gazebo turns (v, omega) into a
steering angle, so the arctan(L*omega/v) block is not repeated here.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from limo_interfaces.msg import MissionState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class PolarController(Node):
    def __init__(self):
        super().__init__("polar_controller")
        self.declare_parameter("k_rho", 4.0)
        self.declare_parameter("k_alpha", 40.0)
        self.declare_parameter("k_beta", -10.0)
        self.declare_parameter("x_star", -0.725)
        self.declare_parameter("y_star", 2.41)
        self.declare_parameter("theta_star", 1.5708)
        self.declare_parameter("v_max", 0.08)
        self.declare_parameter("w_max", 0.60)
        self.declare_parameter("slow_distance", 0.50)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("stop_distance", 0.10)
        self.declare_parameter("wheelbase", 0.24)
        self.declare_parameter("max_steer", 0.45)
        # Gazebo's Ackermann plugin reads angular.z as a steering angle.
        self.declare_parameter("steering_output", True)
        self.declare_parameter("avoid_distance", 0.45)
        self.declare_parameter("stop_obstacle", 0.25)
        self.declare_parameter("reverse_v_max", 0.03)
        self.declare_parameter("rear_clearance", 0.35)
        self.declare_parameter("rear_half_angle", 0.70)
        self.declare_parameter("scan_max_age", 0.5)
        self.goal = None
        self.obstacle = False
        self.sign = False
        self.mission = "FOLLOW_ROAD"
        self.scan_left = None
        self.scan_right = None
        self.scan_rear = None
        self._scan_stamp = None

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(PoseStamped, "/mission/goal", self.on_goal, 10)
        self.create_subscription(Bool, "/perception/obstacle_stop", self.on_obstacle, 10)
        self.create_subscription(Bool, "/perception/sign_stop", self.on_sign, 10)
        self.create_subscription(MissionState, "/mission/state", self.on_state, 10)
        self.get_logger().info("polar controller started")

    def on_state(self, msg: MissionState) -> None:
        self.mission = msg.state or "FOLLOW_ROAD"

    def on_scan(self, msg: LaserScan) -> None:
        self._scan_stamp = self.get_clock().now()
        left = None
        right = None
        rear = None
        half_rear = float(self.get_parameter("rear_half_angle").value)
        angle = msg.angle_min
        for distance in msg.ranges:
            valid = msg.range_min < distance < msg.range_max and math.isfinite(distance)
            if valid and abs(angle) < 0.7:
                if angle >= 0.0:
                    left = distance if left is None else min(left, distance)
                else:
                    right = distance if right is None else min(right, distance)
            if valid:
                wrapped = math.atan2(math.sin(angle), math.cos(angle))
                if abs(abs(wrapped) - math.pi) <= half_rear or abs(wrapped) >= (math.pi - half_rear):
                    rear = distance if rear is None else min(rear, distance)
            angle += msg.angle_increment
        self.scan_left = left
        self.scan_right = right
        self.scan_rear = rear

    def on_goal(self, msg: PoseStamped) -> None:
        self.goal = msg

    def on_obstacle(self, msg: Bool) -> None:
        self.obstacle = msg.data

    def on_sign(self, msg: Bool) -> None:
        self.sign = msg.data

    def _scan_fresh(self) -> bool:
        if self._scan_stamp is None:
            return False
        age = (self.get_clock().now() - self._scan_stamp).nanoseconds * 1e-9
        return age <= float(self.get_parameter("scan_max_age").value)

    def _rear_ok(self) -> bool:
        if not self._scan_fresh():
            return False
        need = float(self.get_parameter("rear_clearance").value)
        if self.scan_rear is None:
            return True
        return self.scan_rear >= need

    def on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = yaw_from_quaternion(msg.pose.pose.orientation)

        command = Twist()
        recovering = self.mission == "RECOVER_REVERSE"
        if self.mission == "REPLANNING":
            self.publisher.publish(command)
            return
        if self.obstacle or self.sign or self.mission.startswith("STOP"):
            self.publisher.publish(command)
            return
        if recovering and not self._rear_ok():
            self.publisher.publish(command)
            return

        if self.goal is not None:
            x_star = self.goal.pose.position.x
            y_star = self.goal.pose.position.y
            theta_star = yaw_from_quaternion(self.goal.pose.orientation)
        else:
            x_star = float(self.get_parameter("x_star").value)
            y_star = float(self.get_parameter("y_star").value)
            theta_star = float(self.get_parameter("theta_star").value)
        k_rho = float(self.get_parameter("k_rho").value)
        k_alpha = float(self.get_parameter("k_alpha").value)
        k_beta = float(self.get_parameter("k_beta").value)

        dx = x_star - x
        dy = y_star - y
        rho = math.hypot(dx, dy)
        if rho < float(self.get_parameter("stop_distance").value):
            self.publisher.publish(command)
            return

        direction = math.atan2(dy, dx)
        alpha = wrap(direction - theta)
        beta = wrap(theta_star - direction)
        # Same test as last year's Simulink function fcn(alpha).
        allow_reverse = bool(self.get_parameter("allow_reverse").value)
        if abs(alpha) <= math.pi / 2.0 or not allow_reverse:
            sens = 1.0
            if abs(alpha) > math.pi / 2.0:
                alpha = math.copysign(math.pi / 2.0, alpha)
        else:
            sens = -1.0
            alpha = wrap(alpha - math.pi if alpha > 0.0 else alpha + math.pi)

        v_raw = k_rho * rho
        slow_distance = float(self.get_parameter("slow_distance").value)
        if rho < slow_distance:
            v_raw = min(v_raw, 0.20 * rho)
        omega = sens * (k_alpha * alpha + k_beta * beta)
        v_max = float(self.get_parameter("v_max").value)
        if recovering:
            v_max = min(v_max, float(self.get_parameter("reverse_v_max").value))
        w_max = float(self.get_parameter("w_max").value)
        v = max(-v_max, min(v_max, sens * v_raw))
        omega = max(-w_max, min(w_max, omega))
        # During verified reverse, keep translation negative / slow and skip front dodge.
        if recovering:
            if v > 0.0:
                v = -min(abs(v), float(self.get_parameter("reverse_v_max").value))
            command.linear.x = v
            if bool(self.get_parameter("steering_output").value):
                wheelbase = float(self.get_parameter("wheelbase").value)
                max_steer = float(self.get_parameter("max_steer").value)
                if abs(v) <= 0.01:
                    command.angular.z = 0.0
                else:
                    gamma = math.atan(wheelbase * omega / abs(v))
                    command.angular.z = max(-max_steer, min(max_steer, gamma))
            else:
                command.angular.z = omega
            self.publisher.publish(command)
            return

        avoid_distance = float(self.get_parameter("avoid_distance").value)
        stop_obstacle = float(self.get_parameter("stop_obstacle").value)
        left = self.scan_left
        right = self.scan_right
        avoid = 0.0
        # Positive gamma turns left, so a close return on the left steers right.
        if left is not None and left < avoid_distance:
            avoid -= 0.50 * (avoid_distance - left) / avoid_distance
        if right is not None and right < avoid_distance:
            avoid += 0.50 * (avoid_distance - right) / avoid_distance
        # A cylinder on the centre line is seen on both sides and those two terms cancel.
        # Keep the clearer side. If they match, pass on the left.
        if (
            left is not None
            and right is not None
            and left < avoid_distance
            and right < avoid_distance
            and abs(left - right) < 0.20
        ):
            avoid += 0.18 if right <= left else -0.18
        nearest = None
        if left is not None:
            nearest = left
        if right is not None:
            nearest = right if nearest is None else min(nearest, right)
        if nearest is not None and nearest < avoid_distance:
            v *= max(0.45, nearest / avoid_distance)
        if nearest is not None and nearest < stop_obstacle:
            v = math.copysign(0.03, v if abs(v) > 1e-3 else sens)
            avoid += 0.20 if (right if right is not None else 99.0) <= (left if left is not None else 99.0) else -0.20

        command.linear.x = v
        if bool(self.get_parameter("steering_output").value):
            wheelbase = float(self.get_parameter("wheelbase").value)
            max_steer = float(self.get_parameter("max_steer").value)
            if abs(v) <= 0.02 and abs(avoid) < 0.05:
                gamma = 0.0
            elif abs(v) <= 0.02:
                gamma = avoid
            else:
                gamma = math.atan(wheelbase * omega / abs(v)) + avoid
            command.angular.z = max(-max_steer, min(max_steer, gamma))
        else:
            command.angular.z = omega + avoid
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = PolarController()
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
