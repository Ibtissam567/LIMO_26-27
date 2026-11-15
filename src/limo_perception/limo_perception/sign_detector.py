#!/usr/bin/env python3
"""Detect a stop sign and traffic-light colours in the camera image."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from limo_interfaces.msg import TrafficSign, TrafficSignArray


class SignDetector(Node):
    def __init__(self):
        super().__init__("sign_detector")
        self.declare_parameter("image_topic", "/rgb/image_raw")
        self.declare_parameter("min_stop_pixels", 250)
        self.declare_parameter("min_light_pixels", 40)
        self.bridge = CvBridge()
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, 10)
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
            self.stop_pub.publish(stop)
            self.sign_pub.publish(signs)
            return

        found = detect_signs(
            image,
            int(self.get_parameter("min_stop_pixels").value),
            int(self.get_parameter("min_light_pixels").value),
        )
        debug = image.copy()
        for item in found:
            sign = TrafficSign()
            sign.type = item["type"]
            sign.distance = float(item["distance"])
            sign.confidence = float(item["confidence"])
            signs.signs.append(sign)
            if sign.type == "STOP":
                stop.data = True
            x, y, w, h = item["box"]
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(debug, sign.type, (x, max(16, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.stop_pub.publish(stop)
        self.sign_pub.publish(signs)
        if self.debug_pub.get_subscription_count() > 0:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))


def detect_signs(image, min_stop_pixels, min_light_pixels):
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    found = []
    stop_view = (
        int(height * 0.20),
        int(height * 0.75),
        int(width * 0.15),
        int(width * 0.85),
    )
    stop = largest_blob(hsv, red_mask(hsv), stop_view, min_stop_pixels, min_ratio=0.8)
    if stop is not None:
        found.append(sign_from_blob("STOP", stop, width))
    light_view = (0, int(height * 0.55), int(width * 0.20), int(width * 0.80))
    for name, mask in (
        ("FEU_ROUGE", red_mask(hsv)),
        ("FEU_ORANGE", cv2.inRange(hsv, (12, 120, 120), (28, 255, 255))),
        ("FEU_VERT", cv2.inRange(hsv, (40, 80, 80), (90, 255, 255))),
    ):
        blob = largest_blob(hsv, mask, light_view, min_light_pixels, max_ratio=1.8)
        if blob is not None:
            found.append(sign_from_blob(name, blob, width))
    return found


def red_mask(hsv):
    low = cv2.inRange(hsv, (0, 90, 70), (12, 255, 255))
    high = cv2.inRange(hsv, (168, 90, 70), (180, 255, 255))
    return cv2.bitwise_or(low, high)


def largest_blob(hsv, mask, view, min_pixels, min_ratio=0.0, max_ratio=10.0):
    y0, y1, x0, x1 = view
    region = mask[y0:y1, x0:x1]
    region = cv2.morphologyEx(region, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(region)
    best = None
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        width = int(stats[index, cv2.CC_STAT_WIDTH])
        height = int(stats[index, cv2.CC_STAT_HEIGHT])
        if area < min_pixels or width < 4 or height < 4:
            continue
        ratio = height / float(width)
        if ratio < min_ratio or ratio > max_ratio:
            continue
        box = (
            x0 + int(stats[index, cv2.CC_STAT_LEFT]),
            y0 + int(stats[index, cv2.CC_STAT_TOP]),
            width,
            height,
            area,
        )
        if best is None or area > best[4]:
            best = box
    return best


def sign_from_blob(name, blob, image_width):
    _x, _y, width, height, area = blob
    size = max(width, height)
    # Rough pinhole distance: a ~0.2 m sign fills the width at about 1 m.
    distance = float(np.clip(0.25 * image_width / max(size, 1), 0.3, 8.0))
    confidence = float(np.clip(area / 1500.0, 0.4, 0.98))
    return {"type": name, "distance": distance, "confidence": confidence, "box": blob[:4]}


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
