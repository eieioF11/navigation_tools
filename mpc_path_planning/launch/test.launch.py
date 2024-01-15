import os
import sys
from glob import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    share_dir = get_package_share_directory('mpc_path_planning')
    rviz_config_file = os.path.join(share_dir, 'rviz','default.rviz')
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
    )
    map_to_base_link_node = launch_ros.actions.Node(
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
                "map",
                "--child-frame-id",
                "base_link",
            ],
    )
    mpc_path_planning_node = launch_ros.actions.Node(
        package='mpc_path_planning',
        executable='mpc_path_planning',
        namespace='',
        output="screen",
        parameters=[os.path.join(share_dir, "config", "mpc_path_planning_param.yaml")],
        respawn=True,
    )
    return launch.LaunchDescription(
        # [ map_to_base_link_node]
        # [ mpc_path_planning_node, rviz2_node]
        [ mpc_path_planning_node]
    )