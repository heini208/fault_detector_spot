from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # RViz config
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config/demo_robot_mapping.rviz'
    )

    # Launch argument for database_path
    declare_db_path = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.ros/slam_map.db'),
        description='Path to the RTAB-Map database (.db)'
    )

    # Localization params (note the differences vs SLAM)
    rtabmap_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_odom': True,
        'subscribe_scan_cloud': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'sync_queue_size': 10,
        'Icp/Iterations': '30',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'Rtabmap/TimeThr': '700',
        'Rtabmap/DetectionRate': '1.0',
        'Vis/MinInliers': '10',
        'Vis/CorType': '1',
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',
        'Mem/IncrementalMemory': 'false',   # don't grow map
        'Mem/InitWMWithAllNodes': 'true',   # preload map nodes
        'localization': 'true',             # enable localization mode
        'database_path': LaunchConfiguration('database_path'),
    }

    remappings = [
        ('scan_cloud', '/merged_cloud'),
        ('rgb/image', '/dummy_rgb'),
        ('rgb/camera_info', '/merged_camera_info'),
        ('odom', '/Spot/odometry'),
    ]

    return LaunchDescription([
        declare_db_path,

        Node(
            package='fault_detector_spot',
            executable='pointcloud_merger',
            name='pointcloud_merger',
            parameters=[{
                'input_topics': [
                    '/Spot/left_flank_depth/point_cloud',
                    '/Spot/left_head_depth/point_cloud',
                    '/Spot/rear_depth/point_cloud',
                    '/Spot/right_flank_depth/point_cloud',
                    '/Spot/right_head_depth/point_cloud',
                ],
                'output_topic': '/merged_cloud',
                'base_frame': 'base_link',
                'remove_underscores': True,
            }],
            output='screen',
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.1,
                'min_height': -0.3,
                'max_height': 0.3,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.0174533,  # 1 degree
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 10.0,
            }],
            remappings=[
                ('cloud_in', '/merged_cloud'),
                ('scan', '/merged_scan'),
            ],
            output='screen',
        ),


    Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[rtabmap_params],
            remappings=remappings,
            output='screen',
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[rtabmap_params],
            remappings=remappings,
            output='screen',
        ),

        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[rtabmap_params],
            remappings=remappings,
            output='screen',
        ),

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
