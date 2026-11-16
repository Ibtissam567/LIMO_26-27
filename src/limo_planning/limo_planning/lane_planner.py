#!/usr/bin/env python3
"""Place the polar-control goal on the lane seen by the camera.

Adds short-horizon candidate scoring (footprint + margin) and recovery goals
from a bounded safe-pose history when mission state is RECOVER_REVERSE.
"""

import math
import os
from collections import deque

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from limo_interfaces.msg import LaneInfo, MissionState
from limo_perception.footprint_safety import (
    DEFAULT_LUMA_MAX_ROAD,
    DEFAULT_SAFETY_MARGIN,
    DEFAULT_TRACK,
    DEFAULT_WHEEL_LENGTH,
    DEFAULT_WHEELBASE,
    DEFAULT_WHEEL_RADIUS,
    evaluate_pose,
    robot_half_width,
    robot_length,
)
from limo_planning.candidate_trajectories import select_best_candidate

try:
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
except ImportError:  # pragma: no cover
    SetEntityState = None
    EntityState = None

DEFAULT_MAP_RELATIVE = (
    "models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
)


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


def resolve_map_path(map_path, map_package, map_relative_path):
    if map_path:
        return map_path
    try:
        share = get_package_share_directory(map_package)
    except Exception:  # noqa: BLE001
        return ""
    return os.path.join(share, map_relative_path or DEFAULT_MAP_RELATIVE)


class LanePlanner(Node):
    def __init__(self):
        super().__init__("lane_planner")
        self.declare_parameter("lookahead", 1.2)
        self.declare_parameter("lane_gain", 0.45)
        self.declare_parameter("goal_x", -0.725)
        self.declare_parameter("goal_y", 2.413)
        self.declare_parameter("goal_stop", 0.35)
        self.declare_parameter("clearance_nudge_gain", 0.55)
        self.declare_parameter("min_clearance_warn", 0.05)
        self.declare_parameter("predictive_enable", True)
        self.declare_parameter("predictive_lateral_nudge", 0.12)
        self.declare_parameter("history_size", 50)
        self.declare_parameter("min_reverse_distance", 0.18)
        self.declare_parameter("safety_margin", DEFAULT_SAFETY_MARGIN)
        self.declare_parameter("track", DEFAULT_TRACK)
        self.declare_parameter("wheel_length", DEFAULT_WHEEL_LENGTH)
        self.declare_parameter("wheelbase", DEFAULT_WHEELBASE)
        self.declare_parameter("wheel_radius", DEFAULT_WHEEL_RADIUS)
        self.declare_parameter("luma_max_road", DEFAULT_LUMA_MAX_ROAD)
        self.declare_parameter("map_path", "")
        self.declare_parameter("map_package", "qcar_large_track")
        self.declare_parameter("map_relative_path", DEFAULT_MAP_RELATIVE)

        self.lane = None
        self.pose = None
        self.route = []
        self.mission = "FOLLOW_ROAD"
        self._safe_history = deque(maxlen=int(self.get_parameter("history_size").value))
        self._last_pick = "base"
        map_file = resolve_map_path(
            str(self.get_parameter("map_path").value),
            str(self.get_parameter("map_package").value),
            str(self.get_parameter("map_relative_path").value),
        )
        self._map_gray = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE) if map_file else None
        if self._map_gray is None:
            self.get_logger().warn("lane_planner: road map unavailable; predictive uses lane clearances only")

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
        self._record_history(msg)

    def _record_history(self, msg: Odometry) -> None:
        if self.lane is None:
            return
        if not bool(getattr(self.lane, "footprint_safe", False)):
            return
        left_c = float(getattr(self.lane, "left_clearance", 0.0))
        right_c = float(getattr(self.lane, "right_clearance", 0.0))
        if left_c < 0.0 or right_c < 0.0:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        v = float(msg.twist.twist.linear.x)
        w = float(msg.twist.twist.angular.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._safe_history.append(
            {
                "x": x,
                "y": y,
                "yaw": yaw,
                "v": v,
                "w": w,
                "t": stamp,
                "left_clearance": left_c,
                "right_clearance": right_c,
                "footprint_safe": True,
            }
        )

    def tick(self) -> None:
        if self.pose is None:
            return
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        theta = yaw_from_quaternion(self.pose.pose.pose.orientation)
        mission_gx = float(self.get_parameter("goal_x").value)
        mission_gy = float(self.get_parameter("goal_y").value)
        if math.hypot(x - mission_gx, y - mission_gy) < float(self.get_parameter("goal_stop").value):
            self._publish(mission_gx, mission_gy, theta, x, y)
            return

        if self.mission == "RECOVER_REVERSE":
            recovered = self._recovery_goal(x, y, theta)
            if recovered is None:
                self._publish(x, y, theta, x, y)
            else:
                self._publish(recovered[0], recovered[1], recovered[2], x, y)
            return

        if self.mission == "REPLANNING":
            # Hold briefly while road_planner rebuilds the route.
            self._publish(x, y, theta, x, y)
            return

        if self.mission.startswith("STOP"):
            self._publish(x, y, theta, x, y)
            return

        # Footprint past safety margin: hold pose (behavior also latches STOP_BOUNDARY).
        if self.lane is not None and not bool(getattr(self.lane, "footprint_safe", True)):
            self._publish(x, y, theta, x, y)
            return

        goal_x, goal_y, goal_yaw = self._base_goal(x, y, theta)
        if bool(self.get_parameter("predictive_enable").value) and self._map_gray is not None:
            picked = select_best_candidate(
                self._map_gray,
                (x, y, theta),
                (goal_x, goal_y, goal_yaw),
                float(self.get_parameter("lookahead").value),
                self.route,
                (mission_gx, mission_gy),
                lateral_nudge=float(self.get_parameter("predictive_lateral_nudge").value),
                safety_margin=float(self.get_parameter("safety_margin").value),
                track=float(self.get_parameter("track").value),
                wheel_length=float(self.get_parameter("wheel_length").value),
                wheelbase=float(self.get_parameter("wheelbase").value),
                wheel_radius=float(self.get_parameter("wheel_radius").value),
                luma_max=float(self.get_parameter("luma_max_road").value),
                min_clearance=0.0,
            )
            if picked is None:
                # No safe candidate: shorten to a crawl goal or hold.
                crawl = max(0.08, 0.35 * float(self.get_parameter("lookahead").value))
                goal_x = x + crawl * math.cos(theta)
                goal_y = y + crawl * math.sin(theta)
                goal_yaw = theta
                self._last_pick = "hold_short"
            else:
                goal_x, goal_y, goal_yaw = picked["goal"]
                if picked["name"] != self._last_pick:
                    self.get_logger().info(
                        f"predictive pick {picked['name']} score={picked['score']:.2f} "
                        f"clear={picked['min_clearance']:.3f}"
                    )
                    self._last_pick = picked["name"]
        self._publish(goal_x, goal_y, goal_yaw, x, y)

    def _base_goal(self, x, y, theta):
        """Existing lane / route goal construction (unchanged logic)."""
        perception_first = (
            self.lane is not None
            and abs(float(self.lane.lateral_offset)) >= 0.55
            and float(self.lane.confidence) <= 0.55
        )
        waypoint = None if perception_first else self._waypoint(x, y, theta)
        if waypoint is not None:
            goal_x, goal_y, goal_yaw = waypoint
            if self.lane is not None and self.lane.confidence >= 0.5:
                nudge = -float(self.lane.lateral_offset) * 0.35
                left_c = float(getattr(self.lane, "left_clearance", 1.0))
                right_c = float(getattr(self.lane, "right_clearance", 1.0))
                warn = float(self.get_parameter("min_clearance_warn").value)
                if left_c < warn or right_c < warn:
                    clearance_bias = 0.0
                    if left_c < right_c:
                        clearance_bias = float(self.get_parameter("clearance_nudge_gain").value)
                    else:
                        clearance_bias = -float(self.get_parameter("clearance_nudge_gain").value)
                    nudge += clearance_bias * 0.15
                goal_x -= nudge * math.sin(goal_yaw)
                goal_y += nudge * math.cos(goal_yaw)
            return goal_x, goal_y, goal_yaw

        lookahead = 0.0
        lateral = 0.0
        goal_yaw = theta
        if self.lane is not None:
            offset = float(self.lane.lateral_offset)
            conf = float(self.lane.confidence)
            recovery = abs(offset) >= 0.55
            if conf >= 0.5 or recovery:
                heading = float(self.lane.heading_error) if conf >= 0.5 else 0.0
                urgency = min(abs(heading), 1.0)
                lookahead = float(self.get_parameter("lookahead").value) * (1.0 - 0.75 * urgency)
                if conf < 0.5 and recovery:
                    lookahead = max(lookahead, 0.15)
                lateral = -offset * float(self.get_parameter("lane_gain").value)
                left_c = float(getattr(self.lane, "left_clearance", 1.0))
                right_c = float(getattr(self.lane, "right_clearance", 1.0))
                warn = float(self.get_parameter("min_clearance_warn").value)
                if left_c < warn or right_c < warn:
                    gain = float(self.get_parameter("clearance_nudge_gain").value)
                    if left_c < right_c:
                        lateral -= gain * 0.12
                    else:
                        lateral += gain * 0.12
                goal_yaw = theta - heading * 0.9
        goal_x = x + lookahead * math.cos(theta) - lateral * math.sin(theta)
        goal_y = y + lookahead * math.sin(theta) + lateral * math.cos(theta)
        return goal_x, goal_y, goal_yaw

    def _recovery_goal(self, x, y, theta):
        """Last safe pose behind the robot, verified with footprint on the map."""
        min_dist = float(self.get_parameter("min_reverse_distance").value)
        best = None
        best_dist = 0.0
        for sample in self._safe_history:
            dx = sample["x"] - x
            dy = sample["y"] - y
            behind = dx * math.cos(theta) + dy * math.sin(theta)
            dist = math.hypot(dx, dy)
            if behind > -0.05 or dist < min_dist:
                continue
            if dist > best_dist:
                best = sample
                best_dist = dist
        if best is None:
            return None
        if self._map_gray is not None and not self._reverse_path_safe(
            (x, y, theta), (best["x"], best["y"], best["yaw"])
        ):
            return None
        return best["x"], best["y"], best["yaw"]

    def _reverse_path_safe(self, start, goal) -> bool:
        """Sample footprint along reverse chord; current pose may already be unsafe."""
        half_width = robot_half_width(
            float(self.get_parameter("track").value),
            float(self.get_parameter("wheel_length").value),
        )
        length = robot_length(
            float(self.get_parameter("wheelbase").value),
            float(self.get_parameter("wheel_radius").value),
        )
        margin = float(self.get_parameter("safety_margin").value)
        luma = float(self.get_parameter("luma_max_road").value)
        sx, sy, syaw = start
        gx, gy, gyaw = goal
        steps = max(3, int(math.hypot(gx - sx, gy - sy) / 0.05))
        for index in range(1, steps + 1):
            ratio = index / float(steps)
            px = sx + ratio * (gx - sx)
            py = sy + ratio * (gy - sy)
            pyaw = syaw + ratio * math.atan2(
                math.sin(gyaw - syaw), math.cos(gyaw - syaw)
            )
            result = evaluate_pose(
                self._map_gray, (px, py, pyaw), half_width, length, margin, luma
            )
            if not result["footprint_safe"]:
                return False
        return True

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
