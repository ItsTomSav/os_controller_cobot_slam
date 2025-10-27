from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare("worlds_niryo"),
        "worlds",
        "pick_and_place_workplace_cube.world"
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": world_path,
            "verbose": "true"
        }.items()
    )

    return LaunchDescription([
        gazebo_launch
    ])

