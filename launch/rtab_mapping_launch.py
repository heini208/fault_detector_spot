import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(get_package_share_directory('rtabmap_demos'), 'rtabmap.db'),
        description='Path to the RTAB-Map database file'
    )

    delete_db_arg = DeclareLaunchArgument(
        'delete_db',
        default_value='true',
        description='Delete existing database on launch'
    )

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config/demo_robot_mapping.rviz'
    )

    # List of cameras
    cameras = [
        ('left', '/camera/left/image', '/depth_registered/left/image', '/camera/left/camera_info', 'rgbd_image_left'),
        ('right', '/camera/right/image', '/depth_registered/right/image', '/camera/right/camera_info',
         'rgbd_image_right'),
        ('frontleft', '/camera/frontleft/image', '/depth_registered/frontleft/image', '/camera/frontleft/camera_info',
         'rgbd_image_frontleft'),
        ('frontright', '/camera/frontright/image', '/depth_registered/frontright/image',
         '/camera/frontright/camera_info', 'rgbd_image_frontright'),
    ]

    # Generate rgbd_sync nodes
    rgbd_sync_nodes = []
    for cam_name, rgb_topic, depth_topic, cam_info_topic, rgbd_out in cameras:
        rgbd_sync_nodes.append(
            Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name=f'rgbd_sync_{cam_name}',
                output='screen',
                parameters=[{'approx_sync': True}],
                remappings=[
                    ('rgb/image', rgb_topic),
                    ('depth/image', depth_topic),
                    ('rgb/camera_info', cam_info_topic),
                    ('rgbd_image', rgbd_out)
                ]
            )
        )

    # RTAB-Map node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'approx_sync': True,
            'subscribe_rgbd': True,
            'rgbd_cameras': 2,
            'Reg/Strategy': '0',  # Geometric
            'Mem/IncrementalMemory': 'true',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/NeighborLinkRefining': 'true',
            'database_path': LaunchConfiguration('db_path'),
            'delete_db_on_start': LaunchConfiguration('delete_db'),
            'topic_queue_size': 20,
            'sync_queue_size': 20,
        }],
        remappings=[
            ('rgbd_image0', 'rgbd_image_left'),
            ('rgbd_image1', 'rgbd_image_right'),
            ('rgbd_image2', 'rgbd_image_frontleft'),
            ('rgbd_image3', 'rgbd_image_frontright'),
            ('odom', '/odometry')
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_rviz],
        output='screen'
    )

    return LaunchDescription(
        [db_path_arg, delete_db_arg] + rgbd_sync_nodes + [rtabmap_node, rviz_node]
    )