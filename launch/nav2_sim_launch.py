from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from ros2launch.command import launch

pkg = get_package_share_directory('fault_detector_spot')


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    map_name_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/.ros/slam_map'),
        description='Path to Slam Toolbox posegraph file'
    )

    # Paths
    nav2_map =LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params = os.path.join(pkg, 'config', 'nav2_sim_params.yaml')
    rviz_config = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    initial_pose = LaunchConfiguration("set_initial_pose", default=False)
    spot_initial_pose = Node(
        package="webots_spot",
        executable="set_initial_pose",
        output="screen",
        condition=conditions.IfCondition(initial_pose),
    )

    # Nav2 bringup (without slam_toolbox, only nav stack)
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

    # PointCloud merger
    merger_node = Node(
        package='fault_detector_spot',
        executable='pointcloud_republisher',
        name='pointcloud_republisher',
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

    # PointCloud → LaserScan
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

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        spot_initial_pose,
        nav2,
        merger_node,
        pcl_to_scan_node,
        rviz_node
    ])
