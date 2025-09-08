from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="mapping.yaml",
        description="SLAM toolbox config file (mapping.yaml, localization.yaml, etc.)"
    )

    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true"
    )

    rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz2 with SLAM config"
    )

    # Launch configurations
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # PointCloud merger node
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

    # Convert merged pointcloud to laserscan
    pcl_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pcl_to_scan",
        remappings=[
            ("cloud_in", "/merged_cloud"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.1,
            "min_height": -0.3,
            "max_height": 0.5,
            "angle_min": -3.14,
            "angle_max": 3.14,
            "angle_increment": 0.0058,
            "range_min": 0.1,
            "range_max": 50.0,
            "use_sim_time": use_sim_time,
        }]
    )

    # SLAM Toolbox node (single node, config_file parameter)
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("fault_detector_spot"),
                "config",
                config_file
            ]),
            {"use_sim_time": use_sim_time}
        ]
    )

    # RViz node
    rviz_config = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config/demo_robot_mapping.rviz'
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["--display-config=" + rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        config_file_arg,
        sim_time_arg,
        rviz_arg,
        merger_node,
        pcl_to_scan_node,
        slam_node,
        rviz_node
    ])
