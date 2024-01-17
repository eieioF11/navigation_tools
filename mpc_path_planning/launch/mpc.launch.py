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
    rviz_config_file = os.path.join(share_dir, 'rviz','test.rviz')
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
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
        # [lifecycle_manager_node,map_server_node,mpc_path_planning_node,rviz2_node]
        [mpc_path_planning_node]
        # [ rviz2_node, mpc_path_planning_node]
    )