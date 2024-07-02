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
import launch_ros.actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mppi_path_planning')
    list = [
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
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
            parameters=[os.path.join(pkg_dir, "config", "sim_mppi_path_planning_param.yaml")],
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
            parameters=[os.path.join(pkg_dir, "config", "sim_lrf_to_grid_param.yaml")],
            respawn=True,
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            parameters=[{'use_sim_time': True}],
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
                "odom",
                "--child-frame-id",
                "map",
            ],
        ),
    ]

    return LaunchDescription(list)