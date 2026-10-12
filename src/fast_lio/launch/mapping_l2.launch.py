import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_fast_lio = get_package_share_directory('fast_lio')
    config_path = os.path.join(pkg_fast_lio, 'config', 'unitree_l2.yaml')
    rviz_cfg_path = os.path.join(pkg_fast_lio, 'rviz_cfg', 'fastlio.rviz')

    return LaunchDescription([
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg_path]
        )
    ])