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
    params = [
        DeclareLaunchArgument('config',default_value=os.path.join(pkg_dir, "config", "mpc_path_planning_param.yaml")),
    ]
    list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("data_logger"), "launch"), "/data_logger.launch.py"]
            ),
        ),
        Node(
            package='mpc_path_planning',
            executable='mpc_path_planning',
            namespace='',
            output="screen",
            parameters=[LaunchConfiguration('config')],
            respawn=True,
        ),
        Node(
            package='mpc_path_planning',
            executable='global_path_planning',
            namespace='',
            output="screen",
            parameters=[LaunchConfiguration('config')],
            respawn=True,
        ),
        Node(
            package='mpc_path_planning',
            executable='control',
            namespace='',
            output="screen",
            parameters=[LaunchConfiguration('config')],
            respawn=True,
        ),
        # Node(
        #     package='twist_switcher',
        #     executable='twist_switcher',
        #     namespace='',
        #     output="screen",
        #     parameters=[os.path.join(pkg_dir, "config", "twist_switcher_param.yaml")],
        #     respawn=True,
        # )
    ]

    return LaunchDescription(params + list)