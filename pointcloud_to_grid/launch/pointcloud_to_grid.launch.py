import os
import sys
from glob import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    share_dir = get_package_share_directory('pointcloud_to_grid')
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
                "livox_frame",
            ],
    )
    pointcloud_to_grid_node = launch_ros.actions.Node(
        package='pointcloud_to_grid',
        executable='pointcloud_to_grid',
        namespace='',
        output="screen",
        parameters=[os.path.join(share_dir, "config", "pointcloud_to_grid_param.yaml")],
        respawn=True,
    )
    return launch.LaunchDescription(
        [ pointcloud_to_grid_node,map_to_base_link_node]
    )