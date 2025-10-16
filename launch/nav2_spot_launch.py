import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg = get_package_share_directory('fault_detector_spot')


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )

    map_name_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/.ros/slam_map'),
        description='Path to map file'
    )

    # Substitutions
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_map = LaunchConfiguration('map')

    # Paths
    nav2_params = os.path.join(pkg, 'config', 'nav2_sim_params.yaml')
    # Nav2 bringup (no AMCL)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("params_file", nav2_params),
            ("map", nav2_map),
        ],
    )

    # PointCloud to LaserScan nodes
    cameras = [
        '/depth_registered/back/points',
        '/depth_registered/frontleft/points',
        '/depth_registered/frontright/points',
    ]

    pcl_to_scan_nodes = []
    for i, cam_topic in enumerate(cameras):
        pcl_to_scan_nodes.append(
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name=f'pcl_to_scan_{i}',
                remappings=[('cloud_in', cam_topic), ('scan', '/scan')],
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

    cmd_vel_gate_node = Node(
        package='fault_detector_spot',
        executable='nav2_cmd_vel_gate',  # name of the node script
        name='nav2_cmd_vel_gate',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
                                 use_sim_time_arg,
                                 map_name_arg,
                                 nav2,
                             ] + pcl_to_scan_nodes + [cmd_vel_gate_node])