import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Path to RTAB-Map config (optional)

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config/demo_robot_mapping.rviz'
    )
    return LaunchDescription([
        # RTAB-Map node using Spot depth directly
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'approx_sync': True,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'Reg/Strategy': '0',  # Geometric registration
                'Mem/IncrementalMemory': 'true',
                'RGBD/LinearUpdate': '0.1',
                'RGBD/AngularUpdate': '0.1',
                'RGBD/NeighborLinkRefining': 'true',
            }],
            remappings=[
                ('rgb/image', '/camera/left/image'),
                ('rgb/camera_info', '/camera/left/camera_info'),
                ('depth/image', '/depth_registered/left/image'),
                ('depth/camera_info', '/depth_registered/left/camera_info'),
                ('odom', '/odometry'),
            ]
        ),

        # Optional: RViz to visualize mapping
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', config_rviz],
            output='screen',
        ),
    ])