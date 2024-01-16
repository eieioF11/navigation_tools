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
    map_file = os.path.join(share_dir, 'map','map.yaml')
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
    map_server_node= launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output="screen",
        parameters=[{
            "yaml_filename": map_file
        }],
        respawn=True,
    )
    lifecycle_manager_node= launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='',
        output="screen",
        parameters=[{'autostart': True}, {'node_names': ['map_server']}],
        respawn=True,
    )
    return launch.LaunchDescription(
        # [lifecycle_manager_node,map_server_node,mpc_path_planning_node,rviz2_node]
        [ map_to_base_link_node,mpc_path_planning_node,rviz2_node, lifecycle_manager_node,map_server_node]
    )