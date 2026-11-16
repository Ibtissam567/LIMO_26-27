"""Simulation Room 207 plus suivi de voie. Ne modifie pas le modele du LIMO."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("limo_bringup")
    room207 = get_package_share_directory("room207_limo")
    perception = get_package_share_directory("limo_perception")
    params = os.path.join(bringup, "config", "j1_params.yaml")
    lane_params = os.path.join(perception, "config", "lane_detector.yaml")
    obstacle_params = os.path.join(perception, "config", "obstacle_detector.yaml")
    sign_params = os.path.join(perception, "config", "sign_detector.yaml")
    goal = os.path.join(bringup, "models", "goal.sdf")

    # Evite le telechargement des modeles Gazebo. Ne change pas le rendu.
    model_path = "/usr/share/gazebo-11/models"
    existing = os.environ.get("GAZEBO_MODEL_PATH", "")
    if existing:
        model_path = model_path + ":" + existing

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(room207, "launch", "room207_limo.launch.py")
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        simulation,
        Node(
            package="limo_perception",
            executable="lane_detector",
            name="lane_detector",
            output="screen",
            parameters=[lane_params, {"use_sim_time": True}],
        ),
        Node(
            package="limo_perception",
            executable="obstacle_detector",
            name="obstacle_detector",
            output="screen",
            parameters=[obstacle_params, {"use_sim_time": True}],
        ),
        Node(
            package="limo_perception",
            executable="sign_detector",
            name="sign_detector",
            output="screen",
            parameters=[sign_params, {"use_sim_time": True}],
        ),
        Node(
            package="limo_perception",
            executable="localization_node",
            name="localization_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="limo_planning",
            executable="lane_planner",
            name="lane_planner",
            output="screen",
            parameters=[{"use_sim_time": True, "lookahead": 0.35, "lane_gain": 0.9, "goal_x": -0.725, "goal_y": 2.413}],
        ),
        Node(
            package="limo_planning",
            executable="road_planner",
            name="road_planner",
            output="screen",
            parameters=[{"use_sim_time": True, "goal_x": -0.725, "goal_y": 2.413}],
        ),
        Node(
            package="limo_control",
            executable="behavior_manager",
            name="behavior_manager",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="limo_control",
            executable="polar_controller",
            name="polar_controller",
            output="screen",
            parameters=[params, {"use_sim_time": True, "v_max": 0.05, "w_max": 0.2}],
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-entity", "goal",
                        "-file", goal,
                        "-x", "-0.725",
                        "-y", "2.41",
                        "-z", "0.02",
                        "-Y", "1.5708",
                    ],
                    output="screen",
                )
            ],
        ),
    ])
