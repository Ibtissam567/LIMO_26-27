#!/usr/bin/env python3
"""Place the polar-control goal on the lane seen by the camera."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from limo_interfaces.msg import LaneInfo, MissionState

try:
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
except ImportError:  # pragma: no cover
    SetEntityState = None
    EntityState = None


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class LanePlanner(Node):
    def __init__(self):
        super().__init__("lane_planner")
        self.declare_parameter("lookahead", 1.2)
        self.declare_parameter("lane_gain", 0.45)
        self.declare_parameter("goal_x", -0.725)
        self.declare_parameter("goal_y", 2.413)
        self.declare_parameter("goal_stop", 0.35)
        self.lane = None
        self.pose = None
        self.route = []
        self.mission = "FOLLOW_ROAD"
        self.create_subscription(LaneInfo, "/perception/lane", self.on_lane, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Path, "/planning/route", self.on_route, 10)
        self.create_subscription(MissionState, "/mission/state", self.on_state, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/mission/goal", 10)
        self.path_pub = self.create_publisher(Path, "/planning/trajectory", 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.goal_client = None
        if SetEntityState is not None:
            self.goal_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.get_logger().info("lane_planner started")

    def on_lane(self, msg: LaneInfo) -> None:
        self.lane = msg

    def on_route(self, msg: Path) -> None:
        self.route = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def on_state(self, msg: MissionState) -> None:
        self.mission = msg.state or "FOLLOW_ROAD"

    def on_odom(self, msg: Odometry) -> None:
        self.pose = msg

    def tick(self) -> None:
        if self.pose is None:
            return
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        theta = yaw_from_quaternion(self.pose.pose.pose.orientation)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        if math.hypot(x - goal_x, y - goal_y) < float(self.get_parameter("goal_stop").value):
            self._publish(goal_x, goal_y, theta, x, y)
            return
        if self.mission.startswith("STOP"):
            self._publish(x, y, theta, x, y)
            return
        # Only drop the route for real off-road recovery (low confidence).
        # Soft wheel nudges must keep following /planning/route or the car freezes on curbs.
        perception_first = (
            self.lane is not None
            and abs(float(self.lane.lateral_offset)) >= 0.55
            and float(self.lane.confidence) <= 0.55
        )
        waypoint = None if perception_first else self._waypoint(x, y, theta)
        if waypoint is not None:
            goal_x, goal_y, goal_yaw = waypoint
            if self.lane is not None and self.lane.confidence >= 0.5:
                nudge = -float(self.lane.lateral_offset) * 0.2
                goal_x -= nudge * math.sin(goal_yaw)
                goal_y += nudge * math.cos(goal_yaw)
        else:
            lookahead = 0.0
            lateral = 0.0
            goal_yaw = theta
            if self.lane is not None:
                offset = float(self.lane.lateral_offset)
                conf = float(self.lane.confidence)
                # Large offset = recovery (white edge / off-road). Apply even when
                # confidence is honest/low; strong lane follow still needs conf >= 0.5.
                recovery = abs(offset) >= 0.55
                if conf >= 0.5 or recovery:
                    heading = float(self.lane.heading_error) if conf >= 0.5 else 0.0
                    urgency = min(abs(heading), 1.0)
                    lookahead = float(self.get_parameter("lookahead").value) * (1.0 - 0.75 * urgency)
                    if conf < 0.5 and recovery:
                        lookahead = max(lookahead, 0.15)
                    lateral = -offset * float(self.get_parameter("lane_gain").value)
                    goal_yaw = theta - heading * 0.9
            goal_x = x + lookahead * math.cos(theta) - lateral * math.sin(theta)
            goal_y = y + lookahead * math.sin(theta) + lateral * math.cos(theta)
        self._publish(goal_x, goal_y, goal_yaw, x, y)

    def _waypoint(self, x, y, theta):
        """Next black-road point in front of the car. The polar law then drives to it."""
        best = None
        best_distance = 99.0
        for point in self.route:
            dx = point[0] - x
            dy = point[1] - y
            distance = math.hypot(dx, dy)
            if distance < 0.3 or distance > 0.8:
                continue
            if dx * math.cos(theta) + dy * math.sin(theta) < 0.2:
                continue
            if distance < best_distance:
                best = point
                best_distance = distance
        if best is None:
            return None
        yaw = math.atan2(best[1] - y, best[0] - x)
        return (best[0], best[1], yaw)

    def _publish(self, goal_x, goal_y, goal_yaw, x, y) -> None:
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation = quaternion_from_yaw(goal_yaw)
        self.goal_pub.publish(goal)

        path = Path()
        path.header = goal.header
        for step in range(6):
            pose = PoseStamped()
            pose.header = goal.header
            ratio = step / 5.0
            pose.pose.position.x = x + ratio * (goal_x - x)
            pose.pose.position.y = y + ratio * (goal_y - y)
            pose.pose.orientation = goal.pose.orientation
            path.poses.append(pose)
        self.path_pub.publish(path)
        self._move_marker(goal_x, goal_y, goal_yaw)

    def _move_marker(self, goal_x, goal_y, goal_yaw) -> None:
        if self.goal_client is None or not self.goal_client.service_is_ready():
            return
        state = EntityState()
        state.name = "goal"
        state.pose.position.x = goal_x
        state.pose.position.y = goal_y
        state.pose.position.z = 0.02
        state.pose.orientation = quaternion_from_yaw(goal_yaw)
        request = SetEntityState.Request()
        request.state = state
        self.goal_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = LanePlanner()
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
