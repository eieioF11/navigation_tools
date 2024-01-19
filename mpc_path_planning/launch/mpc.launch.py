import os
import sys
from glob import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    share_dir = get_package_share_directory('mpc_path_planning')
    mpc_path_planning_node = launch_ros.actions.Node(
        package='mpc_path_planning',
        executable='mpc_path_planning',
        namespace='',
        output="screen",
        parameters=[os.path.join(share_dir, "config", "mpc_path_planning_param.yaml")],
        respawn=True,
    )

    control_node = launch_ros.actions.Node(
        package='mpc_path_planning',
        executable='control',
        namespace='',
        output="screen",
        parameters=[os.path.join(share_dir, "config", "mpc_path_planning_param.yaml")],
        respawn=True,
    )

    return launch.LaunchDescription(
        [mpc_path_planning_node ,control_node]
    )