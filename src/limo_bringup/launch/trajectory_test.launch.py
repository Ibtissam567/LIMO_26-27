"""Test the polar controller on a fixed path: line, then corner, then curve."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("limo_bringup")
    room207 = get_package_share_directory("room207_limo")
    params = os.path.join(bringup, "config", "j1_params.yaml")
    goal = os.path.join(bringup, "models", "goal.sdf")
    point_a = os.path.join(bringup, "models", "point_a.sdf")
    shape = LaunchConfiguration("shape")

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
        DeclareLaunchArgument("shape", default_value="line"),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        simulation,
        Node(
            package="limo_control",
            executable="polar_controller",
            name="polar_controller",
            output="screen",
            parameters=[params, {
                "use_sim_time": True,
                "v_max": 0.08,
                "w_max": 0.25,
                # Course gain 40 turns the car off a straight line. Use a gentle gain here.
                "k_alpha": 1.2,
                "k_beta": 0.0,
            }],
        ),
        Node(
            package="limo_planning",
            executable="trajectory_tester",
            name="trajectory_tester",
            output="screen",
            parameters=[{"use_sim_time": True, "shape": shape}],
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
                ),
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-entity", "point_a",
                        "-file", point_a,
                        "-x", "-0.725",
                        "-y", "-0.087",
                        "-z", "0.02",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
