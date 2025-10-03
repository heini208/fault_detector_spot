import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
# ===================================================================
# FIX 1: Import the NotSubstitution class
# ===================================================================
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    default_rviz_config_path = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'config',
        'rgbd.rviz'
    )

    # Launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_db_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/.ros/rtabmap_spot.db',
        description='Path to the RTAB-Map database file.')

    declare_localization_arg = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Enable localization mode (vs. mapping mode).')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz.')

    declare_rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz', default_value='true',
        description='Launch RTAB-Map visualization node.')

    # Common parameters for RTAB-Map
    rtabmap_common_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': False,
        'subscribe_rgbd': True,
        'rgbd_cameras': 4,
        'subscribe_odom': True,
        'database_path': LaunchConfiguration('database_path'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'queue_size': 50,

        'odom_topic': '/odometry',
        'Mem/IncrementalMemory': NotSubstitution(LaunchConfiguration('localization')),
        'Mem/InitWMWithAllNodes': LaunchConfiguration('localization'),

        'Vis/MinInliers': '15',
        'Vis/CorType': '1',
        'Rtabmap/LoopThr': '0.15',
        'Rtabmap/TimeThr': '700',
        'Rtabmap/DetectionRate': '1.0',
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'false',
        'Optimizer/Slam2D': 'false',
        'Grid/3D': 'true',
    }

    # Define the four camera prefixes
    camera_prefixes = ['frontleft', 'frontright', 'left', 'right']
    sync_nodes = []

    for i, prefix in enumerate(camera_prefixes):
        # RGBD Sync node for each camera
        sync_nodes.append(
            Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name=f'rgbd_sync_{prefix}',
                parameters=[{
                    'approx_sync': True,
                    'approx_sync_max_interval': 0.3,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'queue_size': 10,
                }],
                remappings=[
                    ('rgb/image', f'/camera/{prefix}/image'),
                    ('depth/image', f'/depth_registered/{prefix}/image'),
                    ('rgb/camera_info', f'/camera/{prefix}/camera_info'),
                    ('rgbd_image', f'/rgbd_image{i}'),
                ],
                output='screen'
            )
        )

    # Core RTAB-Map SLAM node
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[rtabmap_common_params],
        output='screen'
    )

    # Visualization node
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        parameters=[rtabmap_common_params],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        # Launch Arguments
        declare_use_sim_time_arg,
        declare_db_path_arg,
        declare_localization_arg,
        declare_rviz_arg,
        declare_rtabmap_viz_arg,

        # Nodes
        GroupAction(actions=sync_nodes),
        rtabmap_slam_node,
        rtabmap_viz_node,
        rviz_node
    ])