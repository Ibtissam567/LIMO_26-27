"""Perception only. Gazebo must already be running."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim = {"use_sim_time": True}
    return LaunchDescription([
        Node(
            package="limo_perception",
            executable="lane_detector",
            name="lane_detector",
            output="screen",
            parameters=[sim],
        ),
        Node(
            package="limo_perception",
            executable="obstacle_detector",
            name="obstacle_detector",
            output="screen",
            parameters=[sim, {"stop_distance": 0.40, "front_angle": 0.40}],
        ),
        Node(
            package="limo_perception",
            executable="sign_detector",
            name="sign_detector",
            output="screen",
            parameters=[sim],
        ),
        Node(
            package="limo_perception",
            executable="localization_node",
            name="localization_node",
            output="screen",
            parameters=[sim],
        ),
    ])
