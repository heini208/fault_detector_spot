import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fault_detector_spot.shared.persistence.runtime_paths import (
    default_map_root,
    default_recording_root,
)


def generate_launch_description():
    pkg = get_package_share_directory("fault_detector_spot")

    tag_config = os.path.join(
        pkg,
        "config",
        "my_tags.yaml",
    )

    tag_sensing_config = os.path.join(
        pkg,
        "config",
        "tag_sensing.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_map_root = LaunchConfiguration("navigation_map_root")
    recording_root = LaunchConfiguration("recording_root")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulated clock published on /clock",
        ),
        DeclareLaunchArgument(
            "navigation_map_root",
            default_value=str(default_map_root()),
            description="Persistent map metadata and RTAB database directory",
        ),
        DeclareLaunchArgument(
            "recording_root",
            default_value=str(default_recording_root()),
            description="Persistent semantic command recording directory",
        ),
        Node(
            package="fault_detector_spot",
            executable="fault_detector_ui",
            name="fault_detector_ui",
            output="screen",
            parameters=[
                tag_sensing_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="fault_detector_spot",
            executable="bt_runner",
            name="bt_runner",
            output="screen",
            parameters=[
                tag_sensing_config,
                {
                    "use_sim_time": use_sim_time,
                    "navigation.map_root": navigation_map_root,
                },
            ],
        ),
        Node(
            package="fault_detector_spot",
            executable="application_api",
            name="application_api",
            output="screen",
            parameters=[
                tag_sensing_config,
                {
                    "use_sim_time": use_sim_time,
                    "navigation.map_root": navigation_map_root,
                },
            ],
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_node",
            output="log",
            remappings=[
                ("image_rect", "/camera/hand/image"),
                ("camera_info", "/camera/hand/camera_info"),
            ],
            parameters=[
                tag_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="fault_detector_spot",
            executable="record_manager",
            name="record_manager",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "recording.root": recording_root,
            }],
        ),
        Node(
            package="fault_detector_spot",
            executable="available_frames_publisher",
            name="available_frames_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
