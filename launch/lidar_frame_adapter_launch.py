"""Expose lidar points in their physical sensor frame without starting mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def make_mount_container():
    return ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="lidar_sensor_mount_container",
        namespace="",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_mount_tf")),
        composable_node_descriptions=[ComposableNode(
            package="tf2_ros",
            plugin="tf2_ros::StaticTransformBroadcasterNode",
            name="lidar_sensor_mount",
            namespace="",
            parameters=[
                LaunchConfiguration("calibration_file"),
                {
                    "child_frame_id": LaunchConfiguration("sensor_frame"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        )],
    )


def make_adapter_node():
    return Node(
        package="fault_detector_spot",
        executable="lidar_frame_adapter",
        name="lidar_frame_adapter",
        output="screen",
        parameters=[{
            "sensor_frame": LaunchConfiguration("sensor_frame"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=[
            ("input", LaunchConfiguration("input_topic")),
            ("output", LaunchConfiguration("output_topic")),
        ],
    )


def generate_launch_description():
    calibration = os.path.join(
        get_package_share_directory("fault_detector_spot"),
        "config", "lidar_mount_calibration.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/velodyne/points"),
        DeclareLaunchArgument("output_topic", default_value="/velodyne/points_sensor"),
        DeclareLaunchArgument("sensor_frame", default_value="lidar_sensor"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_mount_tf", default_value="true"),
        DeclareLaunchArgument("calibration_file", default_value=calibration),
        make_mount_container(),
        make_adapter_node(),
    ])
