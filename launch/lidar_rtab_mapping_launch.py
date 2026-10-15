import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

pkg = get_package_share_directory("fault_detector_spot")


def make_rtabmap_node(incremental_memory, condition):
    return Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        arguments=[
            "--ros-args", "--log-level",
            ["rtabmap:=", LaunchConfiguration("rtabmap_log_level")],
        ],
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "frame_id": LaunchConfiguration("frame_id"),
            "map_frame_id": LaunchConfiguration("map_frame_id"),
            "odom_frame_id": LaunchConfiguration("odom_frame_id"),
            "database_path": LaunchConfiguration("db_path"),
            "delete_db_on_start": LaunchConfiguration("delete_db"),
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "subscribe_rgbd": False,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            # Resolve odometry at each scan's acquisition time through TF.
            "subscribe_odom": False,
            "pub_loc_pose_only_when_localizing": False,
            "approx_sync": False,
            "qos": 1,
            # Prefer a current scan over reliable retransmission of old scans.
            # This reader remains compatible with the reliable filter output.
            "qos_scan": 2,
            "wait_for_transform": 0.5,
            # DetectionRate gates acquisition stamps; it does not bound DDS
            # backlog while ICP runs. Retain only the newest waiting scan.
            "topic_queue_size": 1,
            "sync_queue_size": 1,
            "Rtabmap/DetectionRate": "2.0",
            "Mem/NotLinkedNodesKept": "false",
            "Mem/STMSize": "30",
            "Mem/IncrementalMemory": incremental_memory,
            "Mem/InitWMWithAllNodes": (
                "true" if incremental_memory == "false" else "false"
            ),
            "RGBD/NeighborLinkRefining": "true",
            "RGBD/ProximityBySpace": "true",
            "RGBD/ProximityMaxGraphDepth": "0",
            "RGBD/ProximityPathMaxNeighbors": (
                "10" if incremental_memory == "false" else "1"
            ),
            "RGBD/ProximityGlobalScanMap": (
                "true" if incremental_memory == "false" else "false"
            ),
            "RGBD/AngularUpdate": "0.05",
            "RGBD/LinearUpdate": "0.05",
            "RGBD/CreateOccupancyGrid": "true",
            # Keep measured height/tilt consistent with TF even with planar ICP.
            "RGBD/ForceOdom3DoF": "false",
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "true",
            "Icp/Strategy": "1",
            "Icp/PointToPlane": "true",
            "Icp/PointToPlaneK": "20",
            "Icp/PointToPlaneRadius": "0",
            "Icp/Iterations": "20",
            "Icp/Epsilon": "0.001",
            "Icp/VoxelSize": "0.10",
            "Icp/MaxCorrespondenceDistance": "1.0",
            "Icp/MaxTranslation": "1.0",
            "Icp/CorrespondenceRatio": "0.20",
            "Icp/OutlierRatio": "0.7",
            "Icp/RangeMin": "0.8",
            "Icp/RangeMax": "12.0",
            "Grid/Sensor": "0",
            "Grid/3D": "true",
            "Grid/RayTracing": "true",
            # New maps use the measured support frame, not Spot's elevated body.
            # Legacy maps explicitly retain their body frame and height cutoff.
            "Grid/MapFrameProjection": "false",
            "Grid/NormalsSegmentation": "false",
            "Grid/MinGroundHeight": "0.0",
            "Grid/MaxGroundHeight": ParameterValue(
                LaunchConfiguration("ground_max_height"), value_type=str,
            ),
            "Grid/MaxObstacleHeight": "1.50",
            "GridGlobal/ProbHit": "0.70",
            "GridGlobal/ProbMiss": "0.46",
            "GridGlobal/OccupancyThr": "0.50",
            "GridGlobal/MaxNodes": "0",
        }],
        remappings=[
            ("scan_cloud", LaunchConfiguration("lidar_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
        ],
        condition=condition,
    )


def generate_launch_description():
    config_rviz = os.path.join(pkg, "config", "mapping.rviz")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )
    db_path_arg = DeclareLaunchArgument(
        "db_path",
        default_value=os.path.expanduser("~/.ros/rtabmap_lidar.db"),
    )
    delete_db_arg = DeclareLaunchArgument(
        "delete_db",
        default_value="false",
    )
    extend_map_arg = DeclareLaunchArgument(
        "extend_map",
        default_value="true",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
    )
    rtabmap_log_level_arg = DeclareLaunchArgument(
        "rtabmap_log_level",
        default_value="warn",
        description="RTAB-Map ROS logger severity; use info for per-scan timing diagnostics",
    )
    raw_lidar_topic_arg = DeclareLaunchArgument(
        "raw_lidar_topic",
        default_value="/velodyne/points",
    )
    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value="/velodyne/points_filtered",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry",
    )
    # A standalone launch may reopen an existing body-frame database. The
    # application passes feet_center/0.05 explicitly for new map metadata.
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="base_link",
    )
    body_frame_arg = DeclareLaunchArgument(
        "body_frame",
        default_value="base_link",
    )
    ground_frame_arg = DeclareLaunchArgument(
        "ground_frame",
        default_value="feet_center",
    )
    ground_max_height_arg = DeclareLaunchArgument(
        "ground_max_height",
        default_value="0.12",
    )
    odom_frame_id_arg = DeclareLaunchArgument(
        "odom_frame_id",
        default_value="odom",
    )
    map_frame_id_arg = DeclareLaunchArgument(
        "map_frame_id",
        default_value="map",
    )

    extend_map = LaunchConfiguration("extend_map")

    lidar_self_filter_node = Node(
        package="fault_detector_spot",
        executable="lidar_self_filter",
        name="lidar_self_filter",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "input_topic": LaunchConfiguration("raw_lidar_topic"),
            "output_topic": LaunchConfiguration("lidar_topic"),
            # The existing arm box is body-relative; it must not move to feet.
            "base_frame": LaunchConfiguration("body_frame"),
            "ground_frame": LaunchConfiguration("ground_frame"),
            "minimum_obstacle_height": 0.05,
            "maximum_obstacle_height": 1.50,
            "obstacle_topic": "/velodyne/points_obstacles",
            "arm_box": [
                0.25,
                0.70,
                -0.23,
                0.23,
                -1.00,
                1.10,
            ],
            "tf_timeout_sec": 0.25,
            "publish_markers": True,
        }],
    )

    rtabmap_mapping_node = make_rtabmap_node(
        incremental_memory="true",
        condition=IfCondition(extend_map),
    )
    rtabmap_localization_node = make_rtabmap_node(
        incremental_memory="false",
        condition=UnlessCondition(extend_map),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", config_rviz],
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        use_sim_time_arg,
        db_path_arg,
        delete_db_arg,
        extend_map_arg,
        rviz_arg,
        rtabmap_log_level_arg,
        raw_lidar_topic_arg,
        lidar_topic_arg,
        odom_topic_arg,
        frame_id_arg,
        body_frame_arg,
        ground_frame_arg,
        ground_max_height_arg,
        odom_frame_id_arg,
        map_frame_id_arg,
        lidar_self_filter_node,
        rtabmap_mapping_node,
        rtabmap_localization_node,
        rviz_node,
    ])
