import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    map_db_arg = DeclareLaunchArgument(
        'map_db_path',
        default_value=os.path.expanduser('~/.ros/slam_map.posegraph'),
        description='Path to SLAM Toolbox posegraph file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # Launch configurations
    map_db_path = LaunchConfiguration('map_db_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # Camera topics
    cameras = [
        '/depth_registered/frontleft/points',
        '/depth_registered/frontright/points'
    ]

    pcl_to_scan_nodes = []
    for i, cam_topic in enumerate(cameras):
        pcl_to_scan_nodes.append(
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name=f'pcl_to_scan_{i}',
                remappings=[('cloud_in', cam_topic), ('scan', f'/scan_{i}')],
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.2,
                    'min_height': -0.1,
                    'max_height': 0.3,
                    'angle_min': -3.14,
                    'angle_max': 3.14,
                    'angle_increment': 0.034,
                    'range_min': 0.3,
                    'range_max': 8.0,
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            )
        )

    # Laser scan merger (ros2_laser_scan_merger)
    merger_config_path = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'config', 'merger_params.yaml'
    )

    merge_node = Node(
        package='ros2_laser_scan_merger',
        executable='merge_2_scan',
        name='scan_merger',
        parameters=[merger_config_path],
        output='screen'
    )

    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mode': 'mapping',
            'map_file_name': map_db_path,
            'scan_topic': '/scan',  # virtual merged scan
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'enable_interactive_mode': True,
            'scan_queue_size': 10,
            'map_update_interval': 1.0,
            'map_start_pose': [0.0, 0.0, 0.0]  # x, y, theta
        }],
        output='screen'
    )

    # RViz node
    rviz_config_path = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'rviz', 'visualize_merge_2_scan.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(launch_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        map_db_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        *pcl_to_scan_nodes,
        merge_node,
        slam_node,
        rviz_node
    ])