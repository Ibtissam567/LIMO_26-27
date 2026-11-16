"""Perception only. Gazebo must already be running."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perception = get_package_share_directory("limo_perception")
    lane_params = os.path.join(perception, "config", "lane_detector.yaml")
    obstacle_params = os.path.join(perception, "config", "obstacle_detector.yaml")
    sign_params = os.path.join(perception, "config", "sign_detector.yaml")
    sim = {"use_sim_time": True}
    return LaunchDescription([
        Node(
            package="limo_perception",
            executable="lane_detector",
            name="lane_detector",
            output="screen",
            parameters=[lane_params, sim],
        ),
        Node(
            package="limo_perception",
            executable="obstacle_detector",
            name="obstacle_detector",
            output="screen",
            parameters=[obstacle_params, sim],
        ),
        Node(
            package="limo_perception",
            executable="sign_detector",
            name="sign_detector",
            output="screen",
            parameters=[sign_params, sim],
        ),
        Node(
            package="limo_perception",
            executable="localization_node",
            name="localization_node",
            output="screen",
            parameters=[sim],
        ),
    ])
