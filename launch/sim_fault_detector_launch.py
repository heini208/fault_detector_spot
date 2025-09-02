# launch/my_multi.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('fault_detector_spot')
    config_file = os.path.join(pkg_share, 'config', 'my_tags_sim.yaml')

    return LaunchDescription([
        Node(
            package='fault_detector_spot',
            executable='fault_detector_ui',
            name='fault_detector_ui',
            output='screen'
        ),
        Node(
            package='fault_detector_spot',
            executable='record_manager',
            name='record_manager',
            output='screen'
        ),
        DeclareLaunchArgument('bt', default_value='true'),
        Node(
            package='fault_detector_spot',
            executable='sim_bt_runner',
            name='sim_bt_runner',
            condition=IfCondition(LaunchConfiguration('bt')),
            output='screen'
        ),
    ])
