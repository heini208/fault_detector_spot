import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg = get_package_share_directory("fault_detector_spot")


def make_rtabmap_node(incremental_memory, condition):
    return Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
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
            "approx_sync": False,
            "qos": 1,
            "wait_for_transform": 0.5,
            "topic_queue_size": 20,
            "sync_queue_size": 20,
            "Mem/IncrementalMemory": incremental_memory,
            "Mem/InitWMWithAllNodes": "true" if incremental_memory == "false" else "false",
            "RGBD/NeighborLinkRefining": "true",
            "RGBD/ProximityBySpace": "true",
            "RGBD/ProximityMaxGraphDepth": "0",
            "RGBD/ProximityPathMaxNeighbors": "10" if incremental_memory == "false" else "0",
            "RGBD/ProximityGlobalScanMap": "true" if incremental_memory == "false" else "false",
            "RGBD/AngularUpdate": "0.05",
            "RGBD/LinearUpdate": "0.05",
            "RGBD/CreateOccupancyGrid": "true",
            "Reg/Strategy": "1",
            "Icp/Strategy": "1",
            "Icp/PointToPlane": "true",
            "Icp/Iterations": "20",
            "Icp/VoxelSize": "0.10",
            "Icp/MaxCorrespondenceDistance": "1.0",
            "Icp/CorrespondenceRatio": "0.15",
            "Icp/OutlierRatio": "0.7",
            "Grid/FromDepth": "false",
            "Grid/CellSize": "0.05",
            "Grid/RangeMin": "0.4",
            "Grid/RangeMax": "12.0",
            "Grid/MaxGroundHeight": "0.12",
            "Grid/MaxObstacleHeight": "1.50",
            "Grid/NormalsSegmentation": "true",
            "Grid/RayTracing": "true",
        }],
        remappings=[
            ("scan_cloud", LaunchConfiguration("lidar_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
        ],
        condition=condition,
    )


def generate_launch_description():
    config_rviz = os.path.join(pkg, "config", "mapping.rviz")

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

    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value="/velodyne/points",
    )

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="base_link",
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
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        db_path_arg,
        delete_db_arg,
        extend_map_arg,
        rviz_arg,
        lidar_topic_arg,
        odom_topic_arg,
        frame_id_arg,
        odom_frame_id_arg,
        map_frame_id_arg,
        rtabmap_mapping_node,
        rtabmap_localization_node,
        rviz_node,
    ])