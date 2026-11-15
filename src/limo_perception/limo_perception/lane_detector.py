#!/usr/bin/env python3
"""Detect the lane in front of the LIMO and publish LaneInfo."""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from limo_interfaces.msg import LaneInfo

MAP_PATH = (
    "/home/ha054/ros2_ws/src/environment/qcar_large_track/models/"
    "qcar_large_track/materials/textures/SDCS_MapLayout.png"
)
PLANE_X = 4.88
PLANE_Y = 6.2


def load_road_points():
    image = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
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
    if right_white and not left_white:
        return -0.8
    if left_white and not right_white:
        return 0.8
    return None


class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")
        self.declare_parameter("image_topic", "/rgb/image_raw")
        self.declare_parameter("min_pixels", 80)
        self.bridge = CvBridge()
        self.pose = None
        self.road_points, self.map_gray = load_road_points()
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, 10)
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
            result = measure_lane(image, int(self.get_parameter("min_pixels").value))
            if self.pose is not None:
                wheels = wheel_offset(self.pose, self.map_gray)
                if wheels is not None:
                    result["lateral_offset"] = wheels
                    result["heading_error"] = -wheels
                    result["source"] = "wheels"
                elif result["source"] == "forbidden":
                    offset = offset_back_to_road(self.pose, self.road_points)
                    if offset is not None:
                        result["lateral_offset"] = offset
                        result["heading_error"] = -offset
                        result["source"] = "return"
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


def measure_lane(image, min_pixels):
    """Stay on the black road. White and gray are forbidden."""
    height, width = image.shape[:2]
    top = int(height * 0.45)
    band = image[top:, :]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    black = gray < 55
    lane = lane_under_car(black)
    if int(np.count_nonzero(lane)) >= min_pixels:
        return black_road(lane, band, width)
    # No black road in view: turn toward the darker side instead of driving on white or gray.
    left = float(gray[:, : width // 2].mean())
    right = float(gray[:, width // 2 :].mean())
    turn_left = left <= right
    return {
        "lateral_offset": -0.55 if turn_left else 0.55,
        "heading_error": -0.8 if turn_left else 0.8,
        "curvature": 0.0,
        "confidence": 0.8,
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


def black_road(black, band, width):
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
    return {
        "lateral_offset": float(np.clip(lateral, -0.4, 0.4)),
        "heading_error": float(np.clip(heading, -1.0, 1.0)),
        "curvature": float(np.clip(heading, -1.0, 1.0)),
        "confidence": 0.9,
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


def safe_center(black, white):
    """Center of the black road, inset a little wherever it touches white or the image border."""
    height, width = black.shape
    if height == 0 or width == 0:
        return None
    centers = []
    weights = []
    for y in range(height):
        xs = np.flatnonzero(black[y])
        if xs.size < 8:
            continue
        left = int(xs[0])
        right = int(xs[-1])
        span = max(right - left, 1)
        inset = min(28, max(10, int(0.12 * span)))
        if left <= 2 or white[y, max(left - 1, 0)]:
            left = min(left + inset, right)
        if right >= width - 3 or white[y, min(right + 1, width - 1)]:
            right = max(right - inset, left)
        centers.append(0.5 * (left + right))
        weights.append(1.0 + 2.0 * y / max(height - 1, 1))
    if not centers:
        return None
    center = float(np.average(np.asarray(centers), weights=np.asarray(weights)))
    return nudge_away_from_white(center, black, white)


def nudge_away_from_white(center, black, white):
    """Keep a small clearance from the nearer white edge. The wheels sit outside the camera view."""
    height, width = black.shape
    bottom = black[int(height * 0.65) :, :]
    xs = np.flatnonzero(bottom.any(axis=0))
    if xs.size < 8:
        return center
    left = int(xs[0])
    right = int(xs[-1])
    span = max(right - left, 1)
    gap_left = center - left
    gap_right = right - center
    extra = min(36.0, 0.10 * span)
    if gap_left + 8 < gap_right:
        center += extra
    elif gap_right + 8 < gap_left:
        center -= extra
    return float(min(max(center, left + 4), right - 4))


def anticipate_curve(center, black, white):
    """Shift away from a white curve that is closing in ahead, before the wheels reach it."""
    height, width = black.shape
    near = side_bounds(black[int(height * 0.68) :, :])
    ahead = side_bounds(black[int(height * 0.12) : int(height * 0.55), :])
    if near is None or ahead is None:
        return center
    near_left, near_right = near
    ahead_left, ahead_right = ahead
    close_left = ahead_left - near_left
    close_right = near_right - ahead_right
    span = max(near_right - near_left, 1)
    extra = min(70.0, 0.18 * span + 0.45 * abs(close_left - close_right))
    if close_left > close_right + 14:
        center += extra
    elif close_right > close_left + 14:
        center -= extra
    return float(min(max(center, near_left + 4), near_right - 4))


def side_bounds(mask):
    if mask.size == 0:
        return None
    xs = np.flatnonzero(mask.any(axis=0))
    if xs.size < 8:
        return None
    return int(xs[0]), int(xs[-1])


def column_center(mask):
    counts = mask.sum(axis=0).astype(np.float64)
    if float(counts.sum()) < 10.0:
        return None
    positions = np.arange(mask.shape[1], dtype=np.float64)
    return float(np.average(positions, weights=counts))


def yellow_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (15, 70, 70), (40, 255, 255))


def white_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (0, 0, 180), (180, 50, 255))


def fit_mask(mask, width, source, compact=False):
    ys, xs = np.nonzero(mask)
    if xs.size < 8:
        return None
    # Keep a thin centerline: median x in horizontal slices.
    samples_x = []
    samples_y = []
    for y0 in range(0, mask.shape[0], 8):
        row = xs[np.logical_and(ys >= y0, ys < y0 + 8)]
        if row.size < 3:
            continue
        if compact and float(row.max() - row.min()) > 0.35 * width:
            continue
        samples_x.append(float(np.median(row)))
        samples_y.append(float(y0 + 4))
    if len(samples_x) < 4:
        return None
    samples_x = np.asarray(samples_x)
    samples_y = np.asarray(samples_y)
    # Image y grows downward; fit x = a y^2 + b y + c in the band.
    coeff = np.polyfit(samples_y, samples_x, 2)
    near_y = float(mask.shape[0] - 1)
    far_y = float(np.min(samples_y))
    near_x = float(np.polyval(coeff, near_y))
    far_x = float(np.polyval(coeff, far_y))
    lateral = (near_x - width / 2.0) / (width / 2.0)
    heading = (far_x - near_x) / max(width, 1)
    curvature = float(coeff[0]) / max(width, 1)
    confidence = min(0.95, 0.55 + len(samples_x) / 40.0)
    return {
        "lateral_offset": float(np.clip(lateral, -1.0, 1.0)),
        "heading_error": float(np.clip(heading, -1.0, 1.0)),
        "curvature": float(np.clip(curvature, -1.0, 1.0)),
        "confidence": float(confidence),
        "points": [(int(np.polyval(coeff, y)), int(y)) for y in samples_y[::2]],
        "source": source,
    }


def fit_flat_line(mask, band, width, source):
    """Fit a dashed line that crosses the image instead of running toward the horizon."""
    ys, xs = np.nonzero(mask)
    if xs.size < 40:
        return None
    slope, intercept = np.polyfit(xs.astype(np.float64), ys.astype(np.float64), 1)
    best = open_road_column(band)
    # Positive offset: the line we should aim at is on the right.
    # The open road is the opposite of the white island straight ahead.
    lateral = (best - width / 2.0) / (width / 2.0)
    if abs(lateral) < 0.15:
        lateral = -0.85 if best < width / 2.0 else 0.85
    else:
        lateral = float(np.clip(lateral * 2.0, -1.0, 1.0))
    heading = float(np.clip(math.atan2(1.0, -slope) / (math.pi / 2.0), -1.0, 1.0))
    if best < width / 2.0:
        heading = -abs(heading)
    else:
        heading = abs(heading)
    x_samples = np.linspace(float(xs.min()), float(xs.max()), 12)
    points = []
    for x_value in x_samples:
        y_value = int(slope * x_value + intercept)
        if 0 <= y_value < mask.shape[0]:
            points.append((int(x_value), y_value))
    return {
        "lateral_offset": float(np.clip(lateral, -1.0, 1.0)),
        "heading_error": float(np.clip(heading, -1.0, 1.0)),
        "curvature": float(np.clip(slope, -1.0, 1.0)),
        "confidence": 0.9,
        "points": points,
        "source": source,
    }


def fit_edge(band, width):
    """A bright edge blocks the road ahead. Turn toward the wider black road."""
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    bright_rows = np.nonzero((gray > 200).mean(axis=1) > 0.6)[0]
    if bright_rows.size < 4:
        return None
    below = gray[int(bright_rows.max()) + 1 :, :]
    if below.size == 0 or float(below.mean()) > 40.0:
        return None
    best = open_road_column(band)
    turn_left = best <= width / 2.0
    return {
        "lateral_offset": -0.85 if turn_left else 0.85,
        "heading_error": -0.9 if turn_left else 0.9,
        "curvature": 0.0,
        "confidence": 0.8,
        "points": [(best, int(bright_rows.mean()))],
        "source": "edge",
    }


def open_road_column(band):
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
    dark = gray < 60
    flipped = dark[::-1]
    height, width = dark.shape
    depth = np.zeros(width, dtype=np.float64)
    for x_index in range(width):
        column = flipped[:, x_index]
        count = 0
        for value in column:
            if not value:
                break
            count += 1
        depth[x_index] = count
    smooth = np.convolve(depth, np.ones(41) / 41.0, mode="same")
    return int(np.argmax(smooth))


def road_center(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dark = gray < 50
    if int(np.count_nonzero(dark)) < image.shape[1] * 8:
        return None
    weights = dark.sum(axis=0).astype(np.float64)
    if float(weights.sum()) < 1.0:
        return None
    positions = np.arange(image.shape[1], dtype=np.float64)
    return float(np.average(positions, weights=weights))


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
