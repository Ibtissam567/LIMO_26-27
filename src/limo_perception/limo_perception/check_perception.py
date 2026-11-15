"""Synthetic checks for the perception helpers. Not a ROS node."""

import cv2
import numpy as np
from sensor_msgs.msg import LaserScan

from limo_perception.lane_detector import measure_lane
from limo_perception.obstacle_detector import cluster_points, front_points
from limo_perception.sign_detector import detect_signs


def main():
    image = np.zeros((200, 320, 3), np.uint8)
    image[:, :] = (40, 40, 40)
    cv2.line(image, (160, 90), (170, 199), (0, 220, 220), 8)
    lane = measure_lane(image, 40)
    if lane["confidence"] <= 0.5:
        raise SystemExit(f"lane confidence too low: {lane}")

    scan = LaserScan()
    scan.angle_min = -0.5
    scan.angle_increment = 0.1
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.ranges = [0.4, 0.41, 0.42, 2.0, 1.8, 1.7, 5.0, 5.0, 5.0, 5.0, 5.0]
    clusters = cluster_points(front_points(scan, 0.7, 2.5), 0.25)
    if not clusters:
        raise SystemExit("no obstacle cluster")

    sign = np.zeros((200, 320, 3), np.uint8)
    cv2.rectangle(sign, (140, 40), (180, 110), (0, 0, 220), -1)
    types = {item["type"] for item in detect_signs(sign, 200, 30)}
    if "STOP" not in types:
        raise SystemExit(f"stop not found: {types}")
    print("PERCEPTION_OK", round(lane["confidence"], 2), len(clusters), sorted(types))


if __name__ == "__main__":
    main()
