#!/usr/bin/env python3
"""Detect the black road in front of the LIMO and publish LaneInfo."""

import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from limo_interfaces.msg import LaneInfo

PLANE_X = 4.88
PLANE_Y = 6.2
DEFAULT_MAP_RELATIVE = (
    "models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
)


def resolve_map_path(map_path, map_package, map_relative_path):
    """Prefer an explicit path; otherwise resolve from a ROS package share."""
    if map_path:
        return map_path
    try:
        share = get_package_share_directory(map_package)
    except Exception:  # noqa: BLE001
        return ""
    relative = map_relative_path or DEFAULT_MAP_RELATIVE
    return os.path.join(share, relative)


def load_road_points(map_path):
    if not map_path:
        return np.zeros((0, 2), dtype=np.float64), None
    image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        return np.zeros((0, 2), dtype=np.float64), None
    height, width = image.shape[:2]
    points = []
    for py in range(0, height, 6):
        for px in range(0, width, 6):
            if image[py, px] < 70:
                u = px / width
                v = py / height
                x = u * PLANE_X - PLANE_X / 2.0
                y = (1.0 - v) * PLANE_Y - PLANE_Y / 2.0
                points.append((x, y))
    road = np.asarray(points, dtype=np.float64) if points else np.zeros((0, 2), dtype=np.float64)
    return road, image


def offset_back_to_road(pose, road_points):
    """Negative offset means turn left. The planner aims left when the offset is negative."""
    if road_points.size == 0:
        return None
    x, y, yaw = pose
    distance = (road_points[:, 0] - x) ** 2 + (road_points[:, 1] - y) ** 2
    target = road_points[int(np.argmin(distance))]
    if math.hypot(target[0] - x, target[1] - y) < 0.25:
        return None
    bearing = math.atan2(target[1] - y, target[0] - x)
    delta = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
    return -0.9 if delta > 0.0 else 0.9


def map_value(gray, x, y):
    if gray is None:
        return 255
    height, width = gray.shape[:2]
    u = (x + PLANE_X / 2.0) / PLANE_X
    v = 1.0 - (y + PLANE_Y / 2.0) / PLANE_Y
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return 190
    px = min(width - 1, max(0, int(u * (width - 1))))
    py = min(height - 1, max(0, int(v * (height - 1))))
    return int(gray[py, px])


def wheel_offset(pose, gray):
    """Steer away when white paint is inside the wheel width. The camera cannot see the tires."""
    if gray is None:
        return None
    x, y, yaw = pose
    front = 0.10
    fx = x + front * math.cos(yaw)
    fy = y + front * math.sin(yaw)

    def side_value(distance, sign):
        # sign +1 is left, -1 is right, in the robot frame.
        sx = fx - sign * distance * math.sin(yaw)
        sy = fy + sign * distance * math.cos(yaw)
        return map_value(gray, sx, sy)

    left_white = any(side_value(distance, 1) > 150 for distance in (0.12, 0.18, 0.22))
    right_white = any(side_value(distance, -1) > 150 for distance in (0.12, 0.18, 0.22))
    # Soft nudge only — a hard ±0.8 made the planner drop the route and freeze on the curb.
    if right_white and not left_white:
        return -0.32
    if left_white and not right_white:
        return 0.32
    return None


class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")
        self.declare_parameter("image_topic", "/rgb/image_raw")
        self.declare_parameter("min_pixels", 80)
        self.declare_parameter("map_path", "")
        self.declare_parameter("map_package", "qcar_large_track")
        self.declare_parameter("map_relative_path", DEFAULT_MAP_RELATIVE)
        self.declare_parameter("ema_alpha", 0.35)
        self.declare_parameter("black_threshold", 55)
        self.bridge = CvBridge()
        self.pose = None
        self._ema_lateral = None
        self._ema_heading = None
        map_file = resolve_map_path(
            str(self.get_parameter("map_path").value),
            str(self.get_parameter("map_package").value),
            str(self.get_parameter("map_relative_path").value),
        )
        self.road_points, self.map_gray = load_road_points(map_file)
        if self.map_gray is None:
            self.get_logger().warn(f"lane map not loaded from '{map_file or '(empty)'}'")
        else:
            self.get_logger().info(f"lane map loaded from {map_file}")
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.publisher = self.create_publisher(LaneInfo, "/perception/lane", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/lane/debug", 10)
        self.get_logger().info(f"lane_detector listening on {topic}")

    def on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
        )
        self.pose = (position.x, position.y, yaw)

    def on_image(self, msg: Image) -> None:
        info = LaneInfo()
        info.header = msg.header
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            result = measure_lane(
                image,
                int(self.get_parameter("min_pixels").value),
                int(self.get_parameter("black_threshold").value),
            )
            vision_source = result["source"]
            vision_heading = float(result["heading_error"])
            vision_lateral = float(result["lateral_offset"])
            if self.pose is not None:
                wheels = wheel_offset(self.pose, self.map_gray)
                if wheels is not None:
                    if vision_source == "black":
                        # Keep camera lane centre; map only biases away from white tires.
                        result["lateral_offset"] = float(
                            np.clip(0.65 * vision_lateral + 0.35 * wheels, -0.45, 0.45)
                        )
                        result["heading_error"] = vision_heading
                        result["confidence"] = max(float(result["confidence"]), 0.75)
                        result["source"] = "black"
                    else:
                        result["lateral_offset"] = wheels
                        result["heading_error"] = 0.0
                        result["confidence"] = 0.65
                        result["source"] = "wheels"
                elif result["source"] == "forbidden":
                    offset = offset_back_to_road(self.pose, self.road_points)
                    if offset is not None:
                        result["lateral_offset"] = float(np.clip(offset, -0.45, 0.45))
                        result["heading_error"] = 0.0
                        result["confidence"] = 0.55
                        result["source"] = "return"
            result = self._smooth(result)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"lane image failed: {exc}")
            self.publisher.publish(info)
            return

        info.lateral_offset = float(result["lateral_offset"])
        info.heading_error = float(result["heading_error"])
        info.curvature = float(result["curvature"])
        info.confidence = float(result["confidence"])
        self.publisher.publish(info)
        if self.debug_pub.get_subscription_count() > 0:
            debug = draw_lane(image, result)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def _smooth(self, result):
        alpha = float(self.get_parameter("ema_alpha").value)
        alpha = float(np.clip(alpha, 0.0, 1.0))
        lateral = float(result["lateral_offset"])
        heading = float(result["heading_error"])
        if self._ema_lateral is None or alpha >= 1.0:
            self._ema_lateral = lateral
            self._ema_heading = heading
        else:
            # Snap faster when the estimate jumps (e.g. leaving a stuck white-edge latch).
            if abs(lateral - self._ema_lateral) > 0.35:
                alpha = max(alpha, 0.75)
            self._ema_lateral = alpha * lateral + (1.0 - alpha) * self._ema_lateral
            self._ema_heading = alpha * heading + (1.0 - alpha) * self._ema_heading
        result["lateral_offset"] = float(self._ema_lateral)
        result["heading_error"] = float(self._ema_heading)
        return result


def measure_lane(image, min_pixels, black_threshold=55):
    """Stay on the black road. White and gray are forbidden."""
    height, width = image.shape[:2]
    top = int(height * 0.45)
    band = image[top:, :]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    black = gray < int(black_threshold)
    kernel = np.ones((3, 3), np.uint8)
    black = cv2.morphologyEx(black.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, kernel).astype(bool)
    lane = lane_under_car(black)
    support = int(np.count_nonzero(lane))
    if support >= min_pixels:
        return black_road(lane, band, width, support)
    # No black road in view: turn toward the darker side instead of driving on white or gray.
    left = float(gray[:, : width // 2].mean())
    right = float(gray[:, width // 2 :].mean())
    turn_left = left <= right
    return {
        "lateral_offset": -0.55 if turn_left else 0.55,
        "heading_error": -0.8 if turn_left else 0.8,
        "curvature": 0.0,
        "confidence": 0.40,
        "points": [],
        "source": "forbidden",
    }


def lane_under_car(black):
    """The black lane the car is on, not the black road on the far side of a white island."""
    count, labels = cv2.connectedComponents(black.astype(np.uint8), connectivity=8)
    if count <= 1:
        return black
    height, width = black.shape
    for y in range(height - 1, int(height * 0.4), -2):
        xs = np.flatnonzero(labels[y] > 0)
        if xs.size == 0:
            continue
        nearest = int(xs[np.argmin(np.abs(xs - width / 2.0))])
        lane = labels == labels[y, nearest]
        eroded = cv2.erode(lane.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1)
        if int(eroded.sum()) >= 80:
            return eroded.astype(bool)
        return lane
    return black


def black_road(black, band, width, support):
    height = black.shape[0]
    white = white_edge(band)
    centers = follow_lane(black, white)
    if not centers:
        return empty()
    near = [x for y, x in centers if y > height * 0.80]
    ahead = [x for y, x in centers if height * 0.12 < y < height * 0.52]
    near_x = float(np.mean(near)) if near else centers[0][1]
    ahead_x = float(np.mean(ahead)) if ahead else near_x
    # Stay on the black strip beside the wheels. Do not chase the bend into the white edge.
    aimed = near_x
    aimed = wheel_clearance(aimed, black, white)
    heading = (ahead_x - near_x) / max(width, 1)
    lateral = (aimed - width / 2.0) / (width / 2.0)
    # Honest confidence from lane support and centreline length.
    confidence = float(np.clip(0.70 + min(support, 400) / 1600.0 + min(len(centers), 20) / 80.0, 0.70, 0.95))
    return {
        "lateral_offset": float(np.clip(lateral, -0.4, 0.4)),
        "heading_error": float(np.clip(heading, -1.0, 1.0)),
        "curvature": float(np.clip(heading, -1.0, 1.0)),
        "confidence": confidence,
        "points": [(int(near_x), height - 1), (int(aimed), int(height * 0.4)), (int(ahead_x), int(height * 0.2))],
        "source": "black",
    }


def wheel_clearance(aimed, black, white):
    """Move the aim off the nearer white edge so the wheels, which the camera cannot see, stay on black."""
    height, width = black.shape
    chosen = None
    for y in range(height - 1, int(height * 0.5), -1):
        run = lane_run(black[y], aimed)
        if run is None:
            continue
        chosen = (y, run)
        break
    if chosen is None:
        return aimed
    y, (left, right) = chosen
    span = max(right - left, 1)
    gap_left = aimed - left
    gap_right = right - aimed
    need = min(0.38 * span, 90.0)
    shift = 0.0
    if gap_left + 6 < gap_right and gap_left < need:
        shift = min(need - gap_left, 0.24 * span)
    elif gap_right + 6 < gap_left and gap_right < need:
        shift = -min(need - gap_right, 0.24 * span)
    aimed = aimed + shift
    return float(min(max(aimed, left + 6), right - 6))


def follow_lane(black, white):
    """Walk up the black strip in front of the car, without jumping across a white gap."""
    height, width = black.shape
    x = width / 2.0
    centers = []
    for y in range(height - 1, -1, -1):
        run = lane_run(black[y], x)
        if run is None:
            continue
        left, right = run
        span = max(right - left, 1)
        inset = min(22, max(8, int(0.10 * span)))
        if left <= 2 or (left > 0 and white[y, left - 1]):
            left = min(left + inset, right)
        if right >= width - 3 or (right < width - 1 and white[y, right + 1]):
            right = max(right - inset, left)
        next_x = 0.5 * (left + right)
        if centers and abs(next_x - x) > 90:
            break
        x = next_x
        centers.append((y, x))
    return centers


def lane_run(row, previous_x):
    xs = np.flatnonzero(row)
    if xs.size < 8:
        return None
    cuts = np.where(np.diff(xs) > 8)[0]
    runs = []
    start = 0
    for cut in list(cuts) + [xs.size - 1]:
        piece = xs[start : cut + 1]
        runs.append((int(piece[0]), int(piece[-1])))
        start = cut + 1

    def distance(run):
        left, right = run
        if left <= previous_x <= right:
            return 0
        return min(abs(previous_x - left), abs(previous_x - right))

    return min(runs, key=distance)


def white_edge(band):
    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
    return (hsv[:, :, 1] < 60) & (hsv[:, :, 2] > 200)


def empty():
    return {
        "lateral_offset": 0.0,
        "heading_error": 0.0,
        "curvature": 0.0,
        "confidence": 0.0,
        "points": [],
        "source": "none",
    }


def draw_lane(image, result):
    debug = image.copy()
    height, width = image.shape[:2]
    top = int(height * 0.45)
    cv2.rectangle(debug, (0, top), (width - 1, height - 1), (0, 220, 0), 2)
    cv2.line(debug, (width // 2, top), (width // 2, height - 1), (255, 180, 0), 1)
    color = {
        "black": (0, 220, 0),
        "forbidden": (0, 0, 255),
        "return": (0, 180, 255),
        "wheels": (0, 0, 255),
    }.get(result["source"], (0, 0, 255))
    points = result["points"]
    for x, y in points:
        cv2.circle(debug, (int(x), top + int(y)), 4, color, -1)
    if points:
        near_x = int(points[-1][0])
        cv2.line(debug, (near_x, top), (near_x, height - 1), color, 2)
    lines = [
        "ALLOWED: black road only",
        "FORBIDDEN: white area and gray outside",
        "FOUND: {}   conf {:.2f}".format(result["source"], result["confidence"]),
        "offset {:+.2f} small margin from the white".format(result["lateral_offset"]),
    ]
    for index, text in enumerate(lines):
        cv2.putText(
            debug, text, (8, 22 + index * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3
        )
        cv2.putText(
            debug, text, (8, 22 + index * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
    return debug


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
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
