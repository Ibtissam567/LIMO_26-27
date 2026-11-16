#!/usr/bin/env python3
"""Detect a stop sign and traffic-light colours in the camera image."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from limo_interfaces.msg import TrafficSign, TrafficSignArray


class SignDetector(Node):
    def __init__(self):
        super().__init__("sign_detector")
        self.declare_parameter("image_topic", "/rgb/image_raw")
        self.declare_parameter("min_stop_pixels", 250)
        self.declare_parameter("min_light_pixels", 40)
        self.declare_parameter("stop_confirm_frames", 3)
        self.bridge = CvBridge()
        self._stop_streak = 0
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.stop_pub = self.create_publisher(Bool, "/perception/sign_stop", 10)
        self.sign_pub = self.create_publisher(TrafficSignArray, "/perception/signs", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/signs/debug", 10)
        self.get_logger().info(f"sign_detector listening on {topic}")

    def on_image(self, msg: Image) -> None:
        signs = TrafficSignArray()
        signs.header = msg.header
        stop = Bool()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image conversion failed: {exc}")
            self._stop_streak = 0
            self.stop_pub.publish(stop)
            self.sign_pub.publish(signs)
            return

        needed = max(1, int(self.get_parameter("stop_confirm_frames").value))
        found = detect_signs(
            image,
            int(self.get_parameter("min_stop_pixels").value),
            int(self.get_parameter("min_light_pixels").value),
            stop_streak=self._stop_streak,
            stop_needed=needed,
        )
        saw_stop = any(item["type"] == "STOP" for item in found)
        if saw_stop:
            self._stop_streak += 1
        else:
            self._stop_streak = 0
        stop_latched = self._stop_streak >= needed

        debug = image.copy()
        for item in found:
            if item["type"] == "STOP":
                item = dict(item)
                item["confidence"] = confidence_with_streak(
                    item["confidence"], self._stop_streak, needed
                )
            sign = TrafficSign()
            sign.type = item["type"]
            sign.distance = float(item["distance"])
            sign.confidence = float(item["confidence"])
            signs.signs.append(sign)
            if sign.type == "STOP" and stop_latched:
                stop.data = True
            x, y, w, h = item["box"]
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = sign.type
            if sign.type == "STOP" and not stop_latched:
                label = f"STOP? {self._stop_streak}/{needed}"
            cv2.putText(debug, label, (x, max(16, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.stop_pub.publish(stop)
        self.sign_pub.publish(signs)
        if self.debug_pub.get_subscription_count() > 0:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))


def confidence_with_streak(base, streak, needed):
    progress = min(float(streak) / float(max(needed, 1)), 1.0)
    return float(np.clip(0.5 * base + 0.5 * progress, 0.2, 0.98))


def detect_signs(image, min_stop_pixels, min_light_pixels, stop_streak=0, stop_needed=3):
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    found = []
    # STOP: mid-frame ROI, taller aspect (sign plate), not a compact light blob.
    stop_view = (
        int(height * 0.20),
        int(height * 0.75),
        int(width * 0.15),
        int(width * 0.85),
    )
    stop = largest_blob(
        red_mask(hsv),
        stop_view,
        min_stop_pixels,
        min_ratio=0.85,
        max_ratio=1.6,
        prefer_octagon=True,
    )
    if stop is not None:
        item = sign_from_blob("STOP", stop, width, aspect_score=stop[5])
        found.append(item)

    # Lights: upper ROI, near-circular / compact; reject large STOP-like plates.
    light_view = (0, int(height * 0.55), int(width * 0.20), int(width * 0.80))
    for name, mask in (
        ("FEU_ROUGE", red_mask(hsv)),
        ("FEU_ORANGE", cv2.inRange(hsv, (12, 120, 120), (28, 255, 255))),
        ("FEU_VERT", cv2.inRange(hsv, (40, 80, 80), (90, 255, 255))),
    ):
        blob = largest_blob(
            mask,
            light_view,
            min_light_pixels,
            min_ratio=0.6,
            max_ratio=1.4,
            prefer_octagon=False,
            max_area=min_stop_pixels * 2,
        )
        if blob is not None:
            # Do not re-label a STOP-sized red plate as a red light.
            if name == "FEU_ROUGE" and stop is not None and blobs_overlap(stop, blob):
                continue
            found.append(sign_from_blob(name, blob, width, aspect_score=blob[5]))
    return found


def blobs_overlap(a, b, iou_min=0.25):
    ax, ay, aw, ah = a[:4]
    bx, by, bw, bh = b[:4]
    x0 = max(ax, bx)
    y0 = max(ay, by)
    x1 = min(ax + aw, bx + bw)
    y1 = min(ay + ah, by + bh)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    if inter <= 0:
        return False
    union = aw * ah + bw * bh - inter
    return (inter / max(union, 1)) >= iou_min


def red_mask(hsv):
    low = cv2.inRange(hsv, (0, 90, 70), (12, 255, 255))
    high = cv2.inRange(hsv, (168, 90, 70), (180, 255, 255))
    return cv2.bitwise_or(low, high)


def largest_blob(
    mask,
    view,
    min_pixels,
    min_ratio=0.0,
    max_ratio=10.0,
    prefer_octagon=False,
    max_area=None,
):
    y0, y1, x0, x1 = view
    region = mask[y0:y1, x0:x1]
    region = cv2.morphologyEx(region, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(region)
    best = None
    best_score = -1.0
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        width = int(stats[index, cv2.CC_STAT_WIDTH])
        height = int(stats[index, cv2.CC_STAT_HEIGHT])
        if area < min_pixels or width < 4 or height < 4:
            continue
        if max_area is not None and area > max_area:
            continue
        ratio = height / float(width)
        if ratio < min_ratio or ratio > max_ratio:
            continue
        aspect_score = 1.0 - min(abs(ratio - 1.0), 1.0)
        if prefer_octagon:
            # STOP plates are roughly square; fill ratio helps vs thin noise.
            fill = area / float(max(width * height, 1))
            aspect_score = 0.6 * aspect_score + 0.4 * float(np.clip(fill, 0.0, 1.0))
        else:
            # Traffic lights: prefer compact circular-ish blobs.
            fill = area / float(max(width * height, 1))
            aspect_score = 0.5 * aspect_score + 0.5 * float(np.clip(fill, 0.0, 1.0))
        box = (
            x0 + int(stats[index, cv2.CC_STAT_LEFT]),
            y0 + int(stats[index, cv2.CC_STAT_TOP]),
            width,
            height,
            area,
            float(aspect_score),
        )
        score = area * (0.5 + 0.5 * aspect_score)
        if score > best_score:
            best_score = score
            best = box
    return best


def sign_from_blob(name, blob, image_width, aspect_score=0.5):
    _x, _y, width, height, area = blob[:5]
    size = max(width, height)
    # Rough pinhole distance: a ~0.2 m sign fills the width at about 1 m.
    distance = float(np.clip(0.25 * image_width / max(size, 1), 0.3, 8.0))
    area_term = float(np.clip(area / 1500.0, 0.0, 1.0))
    confidence = float(np.clip(0.35 * area_term + 0.65 * float(aspect_score), 0.25, 0.98))
    return {
        "type": name,
        "distance": distance,
        "confidence": confidence,
        "box": blob[:4],
    }


def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
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
