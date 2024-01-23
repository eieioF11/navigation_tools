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
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.0",
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
                "imu",
                "--child-frame-id",
                "base_link",
            ],
        ),
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid',
            namespace='',
            # output="screen",
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[os.path.join(get_package_share_directory('pointcloud_to_grid'), "config", "pointcloud_to_grid_param.yaml")],
            respawn=True,
        )
    ]

    return LaunchDescription(list)