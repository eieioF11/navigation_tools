import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    pkg_dir = get_package_share_directory('mpc_path_planning')
    list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_dir, "launch"), "/rviz.launch.py"]
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_dir, "launch"), "/mpc.launch.py"]
            ),
            launch_arguments={
                'config': os.path.join(pkg_dir, "config", "sim_mpc_path_planning_param.yaml"),
            }.items()
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "-2.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "0.0",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "base_link",
            ],
        ),
        Node(
                package="grid_map_publisher",
                executable="grid_map_publisher",
                parameters=[
                    {
                        "map_yaml_filename": os.path.join(
                            get_package_share_directory("grid_map_publisher"), "map", "map.yaml"
                        )
                    }
                ],
                respawn=True,
        )
    ]

    return LaunchDescription(list)