from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

pkg = get_package_share_directory('fault_detector_spot')


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_name_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/.ros/slam_map'),
        description='Path to Slam Toolbox posegraph file'
    )

    # Initial pose arguments
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0', description='Initial X position for AMCL'
    )
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0', description='Initial Y position for AMCL'
    )
    initial_pose_z_arg = DeclareLaunchArgument(
        'initial_pose_z', default_value='0.0', description='Initial Z position for AMCL'
    )
    initial_pose_theta_arg = DeclareLaunchArgument(
        'initial_pose_theta', default_value='0.0', description='Initial yaw (theta) for AMCL'
    )
    set_initial_pose_arg = DeclareLaunchArgument(
        'set_initial_pose', default_value='True', description='Whether to set initial pose in AMCL'
    )

    # Substitutions
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_map = LaunchConfiguration('map')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_theta = LaunchConfiguration('initial_pose_theta')
    set_initial_pose = LaunchConfiguration('set_initial_pose')

    # Paths
    nav2_params = os.path.join(pkg, 'config', 'nav2_sim_params.yaml')
    rviz_config = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    # Nav2 bringup (with AMCL initial pose overrides)
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
            ("amcl/set_initial_pose", set_initial_pose),
            ("amcl/initial_pose/x", initial_pose_x),
            ("amcl/initial_pose/y", initial_pose_y),
            ("amcl/initial_pose/z", initial_pose_z),
            ("amcl/initial_pose/theta", initial_pose_theta),
        ],
    )

    # PointCloud merger
    merger_node = Node(
        package='fault_detector_spot',
        executable='pointcloud_merger',
        name='pointcloud_merger',
        parameters=[{
            'input_topics': [
                '/depth_registered/back/points',
                '/depth_registered/frontleft/points',
                '/depth_registered/frontright/points',
                '/depth_registered/hand/points',
                '/depth_registered/left/points',
                '/depth_registered/right/points',
            ],
            'output_topic': '/merged_cloud',
            'base_frame': 'base_link',
            'remove_underscores': False,
            'use_sim_time': use_sim_time,
        }],
        output='log'
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
        output="log"
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_z_arg,
        initial_pose_theta_arg,
        set_initial_pose_arg,
        nav2,
        merger_node,
        pcl_to_scan_node,
        rviz_node
    ])
