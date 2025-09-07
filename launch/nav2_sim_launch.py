from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(bringup_dir, 'launch', 'navigation_launch.py')

    params_file = os.path.join(
        get_package_share_directory('fault_detector_spot'),
        'config', 'nav2_sim_params.yaml'
    )

    # Declare use_sim_time argument
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')

    # Include Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # PointCloud to LaserScan node (delayed start to ensure TF exists)
    pcl_to_laserscan = TimerAction(
        period=3.0,  # wait 3 seconds for TF to come up
        actions=[Node(
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
        )]
    )

    return LaunchDescription([
        declare_use_sim_time,
        nav2_launch,
        pcl_to_laserscan
    ])
