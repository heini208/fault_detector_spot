# launch/my_multi.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            parameters=['/home/marcel/spot_ws/src/fault_detector_spot/config/my_tags.yaml']
        ),
        Node(
            package='fault_detector_spot',
            executable='record_manager',
            name='record_manager',
            output='screen'
        ),
    ])