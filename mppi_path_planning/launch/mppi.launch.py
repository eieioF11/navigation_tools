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
    pkg_dir = get_package_share_directory('mppi_path_planning')
    list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("data_logger"), "launch"), "/data_logger.launch.py"]
            ),
        ),
        Node(
            package='mppi_path_planning',
            executable='mppi_path_planning',
            namespace='',
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "mppi_path_planning_param.yaml")],
            # prefix=['valgrind --tool=callgrind'],
            # prefix=['xterm -e valgrind --tool=callgrind'],
            # prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no --verbose'],
            respawn=True,
        ),
        Node(
            package='lrf_to_grid',
            executable='lrf_to_grid',
            namespace='',
            # output="screen",
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[os.path.join(get_package_share_directory("aisl_bringup"), "config", "lrf_to_grid_param.yaml")],
            respawn=True,
        ),
    ]

    return LaunchDescription(list)