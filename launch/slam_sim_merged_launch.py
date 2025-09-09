from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Launch arguments
    map_db_arg = DeclareLaunchArgument(
        'map_db_path',
        default_value=os.path.expanduser('~/.ros/slam_map.posegraph'),
        description='Path to Slam Toolbox posegraph file'
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Mode: "mapping" or "localization"'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    map_start_pose_arg = DeclareLaunchArgument(
        'map_start_pose',
        default_value='0.0,0.0,0.0',
        description='Starting pose for localization: x,y,z,qx,qy,qz,qw'
    )

    map_start_pose_array = PythonExpression([
        "[", LaunchConfiguration('map_start_pose'), "]"
    ])

    # Launch configurations
    map_db_path = LaunchConfiguration('map_db_path')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    map_start_pose = LaunchConfiguration('map_start_pose')

    # Merge PointClouds
    merger_node = Node(
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
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Convert merged PointCloud to LaserScan
    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pcl_to_scan',
        remappings=[('cloud_in', '/merged_cloud'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,
            'min_height': -0.3,
            'max_height': 0.5,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.01,
            'range_min': 0.05,
            'range_max': 50.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Slam Toolbox node (async for lifelong mapping)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mode': mode,
            'map_file_name': map_db_path,
            'scan_topic': '/scan',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'enable_interactive_mode': True,
            'scan_queue_size': 3,
            'map_update_interval': 1.0,
            'map_start_pose': map_start_pose_array
        }],
        output='screen'
    )

    # RViz2
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config/demo_robot_mapping.rviz'
    )
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', config_rviz],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        map_db_arg,
        mode_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        map_start_pose_arg,
        merger_node,
        pcl_to_scan_node,
        slam_node,
        rviz_node
    ])
