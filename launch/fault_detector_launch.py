# launch/my_multi.launch.py
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

pkg = get_package_share_directory('fault_detector_spot')


def generate_launch_description():
    pkg = get_package_share_directory('fault_detector_spot')
    tag_config = os.path.join(pkg, 'config', 'my_tags.yaml')

    return LaunchDescription([
        Node(
            package='fault_detector_spot',
            executable='fault_detector_ui',
            name='fault_detector_ui',
            output='screen'
        ),
        Node(
            package='fault_detector_spot',
            executable='bt_runner',
            name='bt_runner',
            output='screen'
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='log',
            remappings=[
                ('image_rect', '/camera/hand/image'),
                ('camera_info', '/camera/hand/camera_info'),
            ],
            parameters=[tag_config]
        ),
        Node(
            package='fault_detector_spot',
            executable='record_manager',
            name='record_manager',
            output='screen'
        ),
    ])