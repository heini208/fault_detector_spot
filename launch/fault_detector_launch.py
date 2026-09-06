import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

    close_surface_config = os.path.join(
        pkg,
        "config",
        "close_surface.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_map_root = LaunchConfiguration("navigation_map_root")
    recording_root = LaunchConfiguration("recording_root")
    launch_micro_ros_agent = LaunchConfiguration("launch_micro_ros_agent")
    micro_ros_agent_transport = LaunchConfiguration(
        "micro_ros_agent_transport"
    )
    micro_ros_agent_port = LaunchConfiguration("micro_ros_agent_port")
    micro_ros_agent_address = LaunchConfiguration("micro_ros_agent_address")
    micro_ros_agent_verbosity = LaunchConfiguration(
        "micro_ros_agent_verbosity"
    )

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
        DeclareLaunchArgument(
            "launch_micro_ros_agent",
            default_value="true",
            description="Start the micro-ROS Agent for ESP32 sensor mounts",
        ),
        DeclareLaunchArgument(
            "micro_ros_agent_transport",
            default_value="udp4",
            description="micro-ROS Agent transport",
        ),
        DeclareLaunchArgument(
            "micro_ros_agent_port",
            default_value="8888",
            description="UDP port used by the micro-ROS Agent",
        ),
        DeclareLaunchArgument(
            "micro_ros_agent_address",
            default_value="auto",
            description=(
                "LAN IPv4 advertised to sensor mounts, or auto to detect it"
            ),
        ),
        DeclareLaunchArgument(
            "micro_ros_agent_verbosity",
            default_value="4",
            description="micro-ROS Agent log verbosity (0-6)",
        ),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            output="screen",
            arguments=[
                micro_ros_agent_transport,
                "--port",
                micro_ros_agent_port,
                "-v",
                micro_ros_agent_verbosity,
            ],
            condition=IfCondition(launch_micro_ros_agent),
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="fault_detector_spot",
            executable="micro_ros_agent_status",
            name="micro_ros_agent_status",
            output="screen",
            parameters=[{
                "agent.transport": micro_ros_agent_transport,
                "agent.port": micro_ros_agent_port,
                "agent.advertised_address": micro_ros_agent_address,
                "use_sim_time": use_sim_time,
            }],
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
                close_surface_config,
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
            executable="tag_observation_node",
            name="tag_observation",
            output="screen",
            parameters=[
                tag_sensing_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="fault_detector_spot",
            executable="move_close_to_surface_node",
            name="move_close_to_surface",
            output="screen",
            parameters=[
                close_surface_config,
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
